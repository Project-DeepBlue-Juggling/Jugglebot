import asyncio
import collections
import logging
import math
import threading
import time
import qtm_rt
import numpy as np
from typing import Optional, Dict, List, Tuple
import xml.etree.ElementTree as ET
import jugglebot.hardware_config as hw
from jugglebot.bb_calibration import BB_MARKER_COUNT
from jugglebot.qtm_clock_sync import MinLatencyClockSync


#: Bound on the frames received but not yet published (``on_packet`` appends,
#: ``drain_frames`` empties it every publisher tick). 150 frames = 0.5 s of
#: QTM's 300 Hz stream: about 3x the longest stamp gap of the loaded
#: 2026-10-10 sittings (157 ms), so a stall of the node becomes latency and
#: only one longer than 0.5 s loses frames: the OLDEST first, counted in
#: ``overflow_drops``. It also caps the latency a starved node can build up at
#: 0.5 s (a frame older than that is of no use to the ball tracker anyway).
#: Memory: 150 records of ~25 marker tuples, well under 1 MB.
FRAME_QUEUE_MAXLEN = 150

#: One QTM frame as received (logbook 2026-10-10-mocap-frame-queue). Built once
#: in ``on_packet`` from plain floats and tuples; the ROS messages are built by
#: the publisher, once per frame.
#:
#: frame_number  QTM's frame counter (de-duplication and gap counting)
#: qtm_us        QTM's frame timestamp (µs)
#: stamp_ns      that timestamp on the ROS clock through the min-latency sync,
#:               converted ONCE, at receive (None before the sync exists)
#: receive_ns    ROS clock at receive (the same reading the sync was fed)
#: aligned       ``is_aligned`` after this frame's Base body check
#: labelled      [(label, x, y, z, residual)] visible labelled markers
#: unlabelled    [(x, y, z, residual)] visible unlabelled markers
#: bodies        ((name, frame_id, x, y, z, qx, qy, qz, qw), ...) rigid bodies,
#:               name sanitised, z shifted into platform_start for platform bodies
#: bb_markers    BB_MARKER_COUNT rows (x, y, z, residual), NaN when not visible
MocapFrame = collections.namedtuple('MocapFrame', (
    'frame_number', 'qtm_us', 'stamp_ns', 'receive_ns', 'aligned',
    'labelled', 'unlabelled', 'bodies', 'bb_markers'))

_NAN_ROW = (math.nan, math.nan, math.nan, math.nan)
_NAN_BB_ROWS = (_NAN_ROW,) * BB_MARKER_COUNT
_isnan = math.isnan
_sqrt = math.sqrt


def quaternion_from_rotation_list(R) -> Tuple[float, float, float, float]:
    """(qx, qy, qz, qw) of a 9-element COLUMN-major 3x3 rotation matrix
    ``[r11, r21, r31, r12, r22, r32, r13, r23, r33]`` (QTM's 6D layout).

    Plain floats: the same arithmetic, branch for branch, as the numpy
    version it replaced (pinned against it by
    tests/ros/test_mocap_frame_queue.py), at a fraction of the cost per body
    per frame.
    """
    r00, r10, r20, r01, r11, r21, r02, r12, r22 = (float(v) for v in R)
    trace = r00 + r11 + r22
    if trace > 0:
        S = 2.0 * _sqrt(trace + 1.0)
        return ((r21 - r12) / S, (r02 - r20) / S, (r10 - r01) / S, 0.25 * S)
    # np.argmax(diag) picks the first maximum (NaN entries make every branch NaN)
    if r00 >= r11 and r00 >= r22:
        S = 2.0 * _sqrt(1.0 + r00 - r11 - r22)
        return (0.25 * S, (r01 + r10) / S, (r02 + r20) / S, (r21 - r12) / S)
    if r11 >= r22:
        S = 2.0 * _sqrt(1.0 + r11 - r00 - r22)
        return ((r01 + r10) / S, 0.25 * S, (r12 + r21) / S, (r02 - r20) / S)
    S = 2.0 * _sqrt(1.0 + r22 - r00 - r11)
    return ((r02 + r20) / S, (r12 + r21) / S, 0.25 * S, (r10 - r01) / S)

class MocapInterface:
    """
    Class to track mocap data using QTM.
    With the new configuration, all incoming marker data (labelled and unlabelled) is in Jugglebot's base frame.
    Only unlabelled markers and full rigid bodies are stored and processed.
    """

    def __init__(self, host: str = hw.MOCAP_QTM_HOST, port: int = hw.MOCAP_QTM_PORT, logger=None, node=None):
        """
        Initialize the tracker.

        Parameters:
        - host: IP address of the QTM server.
        - port: Port to connect to QTM.
        """
        self.host = host
        self.port = port
        self.logger = logger
        self.node = node

        # mm to move in the z direction from the base to the platform in its lowest pos
        self.base_to_platform_transformation = None

        self.ready_to_publish = False # eg. if we haven't received geometry data yet

        # Initialize data to be stored
        # The LATEST frame's BB markers (x, y, z, residual), NaN rows for the
        # ones QTM cannot see: mocap/status reads it. Each frame's own copy
        # travels in its MocapFrame.
        self.ball_butler_markers = np.full((BB_MARKER_COUNT, 4), np.nan)
        self.body_dict = {}
        self.marker_dict = {}
        # index -> name lookups for the per-frame loops, rebuilt whenever
        # marker_dict / body_dict is REPLACED (_refresh_name_lookups); the
        # dicts are only ever replaced, never mutated in place.
        self._lookup_src = (None, None)
        self._marker_names: List[str] = []
        self._body_names: list = []               # (sanitised name, frame_id, is_base_frame)
        self._bb_marker_indices: List[int] = []   # per BB marker 1..N: its 3D-list index, or -1

        # Bodies whose poses are in the world/base frame (not platform frame).
        # Uses raw QTM names — checked BEFORE space/hyphen sanitization.
        self.base_frame_bodies = {"Base", "Ball Butler", "Ball_Butler", "Catching Cone", "Catching_Cone"}

        # ── Alignment check ────────────────────────────────────────────
        self.is_aligned = False  # Updated every frame from the "Base" rigid body
        self._align_pos_thresh_mm = 2.5   # Overridden by set_alignment_thresholds()
        self._align_rot_thresh_deg = 1.0

        # Performance statistics from the incoming packet header.
        # Rolling window: ~5 seconds at 200 Hz
        self.residuals_unlabelled: collections.deque = collections.deque(maxlen=1000)
        # QTM's 3D-component header: 2D frames lost between the cameras and
        # QTM, and frames out of sync, per thousand over QTM's last 0.5-1 s.
        self.drop_rate = 0
        self.out_of_sync_rate = 0

        # ── QTM ↔ ROS Clock Sync ───────────────────────────────────────
        # QTM packet.timestamp is in microseconds (monotonic from QTM start).
        # Mapping: ros_time_ns = qtm_timestamp_us * 1000 + _qtm_to_ros_offset_ns.
        # Estimated from the MINIMUM receive latency over a sliding window, with
        # a drift term and a slew limit (jugglebot.qtm_clock_sync; logbook
        # 2026-10-10-mocap-min-latency-clock-sync). Until 2026-10-10 this was an
        # EMA of every packet's latency, which put the Jetson's queueing delay
        # into every frame stamp (4-59 ms wander within one sweep under load).
        self._qtm_sync = MinLatencyClockSync()
        self._qtm_to_ros_offset_ns: Optional[int] = None   # mirror of _qtm_sync.offset_ns
        self._qtm_sync_count = 0
        self._qtm_sync_lock = threading.Lock()

        # ── Frame queue (2026-10-10, logbook 2026-10-10-mocap-frame-queue) ──
        # Every frame QTM sends is queued as one MocapFrame, and the publisher
        # drains the queue every tick, one message per frame. Until 2026-10-10
        # on_packet overwrote ONE snapshot that a 5 ms timer published, so
        # every packet that arrived while the process was starved, except the
        # last, was lost (180 of 300 frames/s idle, 70-116 loaded).
        # A frame's markers, bodies and stamp are one record, so a reader can
        # never pair one frame's markers with another frame's time (the
        # 2026-09-20 same-lock rule, now by construction).
        self._frames: collections.deque = collections.deque(maxlen=FRAME_QUEUE_MAXLEN)
        self._latest_frame: Optional[MocapFrame] = None
        # Stream counters, cumulative since start (take_stream_stats).
        # Written by the receive thread under data_lock.
        self._last_frame_number: Optional[int] = None
        self._frames_queued = 0
        self._overflow_drops = 0       # queued, then dropped unpublished (queue full)
        self._qtm_gap_frames = 0       # frame numbers QTM never sent
        self._qtm_gap_events = 0
        self._duplicate_frames = 0     # a packet repeating the last frame number
        self._qtm_restarts = 0         # the frame number went backwards
        self._max_queue_depth = 0      # since the last take_stream_stats
        self._max_drop_rate = 0        # since the last take_stream_stats
        self._max_out_of_sync_rate = 0

        # Packet freshness tracking — lets the publisher detect when QTM stops
        # sending data, regardless of whether _on_qtm_disconnect fires.
        self._last_packet_time: Optional[float] = None

        # Outage-logging state (state-transition logging, matching the
        # SpaceMouse/mocap connection-handling pattern). True while we are in a
        # QTM outage and have already logged its first failure — so the
        # available→unavailable edge logs exactly ONE WARNING and the repeated
        # retry failures within the same outage stay silent (no throttled
        # repeat). Cleared on the unavailable→available edge in connect().
        self._qtm_outage_active = False

        # The qtm_rt library logs `LOG.error(...)` on its own "qtm_rt" logger
        # every failed connect attempt (it swallows the OSError and returns
        # None — verified empirically against an unreachable host). That spam
        # is independent of `self.logger`, so we gate the library logger by
        # connection state: silenced while we're in an outage and retrying,
        # restored to its normal level once connected (when its messages —
        # e.g. "Non handled packet type" — are genuinely diagnostic). Start
        # quiet so a QTM-down-at-startup produces zero qtm_rt ERROR lines.
        self._qtm_rt_logger = logging.getLogger("qtm_rt")
        self._qtm_rt_orig_level = self._qtm_rt_logger.level
        self._set_qtm_lib_quiet(True)

        # Threading lock for data synchronization
        self.data_lock = threading.Lock()

        # Flag to request parameter re-fetch from the asyncio thread
        self._params_need_refresh = False

        # Event loop and thread for asynchronous operations
        self.loop = None
        self.thread = None

        self.start()

    #########################################################################################################
    #                                       Connection Management                                           #
    #########################################################################################################

    def start(self):
        """
        Start the tracker in a separate thread.
        """
        self.thread = threading.Thread(target=self._run_asyncio_loop)
        self.thread.start()

    def _run_asyncio_loop(self):
        """
        Run the asyncio event loop in a separate thread.
        """
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.create_task(self.connect())
            self.loop.run_forever()
        except asyncio.CancelledError:
            pass
        finally:
            self.loop.close()

    _RETRY_DELAYS = [2, 5, 10, 30]  # seconds between reconnection attempts

    # Hard ceiling on a single qtm_rt.connect() attempt. qtm_rt's own
    # `timeout=` argument covers only the QRT protocol handshake, NOT TCP
    # connection establishment: if the host resolves but the QTM port is
    # filtered/down, the kernel sits in SYN-retry for minutes and connect()
    # never reaches the retry/except path — so nothing is ever logged. We
    # wrap the call in asyncio.wait_for() to bound it.
    _CONNECT_TIMEOUT_S = 6.0

    def _log_qtm_outage(self, reason: str):
        """Log the FIRST failure of a QTM outage, then stay silent until QTM
        returns — state-transition logging, matching the connection-handling
        pattern used by the SpaceMouse handler and mocap's own
        _on_qtm_disconnect.

        The available→unavailable edge logs exactly one WARNING; subsequent
        retry failures within the same outage are silent (no throttled repeat
        that keeps spamming a down-for-hours QTM). The unavailable→available
        edge is logged by the "Connected to QTM." INFO in connect(), which also
        clears _qtm_outage_active so the next outage logs its first failure.
        """
        if self._qtm_outage_active:
            return
        self._qtm_outage_active = True
        self.logger.warning(
            f"QTM unavailable at {self.host}:{self.port} ({reason}), "
            f"retrying in background"
        )

    def _set_qtm_lib_quiet(self, quiet: bool):
        """Silence (or restore) the third-party qtm_rt library's own logger.

        While quiet, qtm_rt's per-attempt connect-failure ERRORs (which are
        expected and handled by our retry loop) are suppressed. Restored to
        the original level once connected so genuine streaming-time errors
        still surface.
        """
        self._qtm_rt_logger.setLevel(
            logging.CRITICAL if quiet else self._qtm_rt_orig_level
        )

    def _on_qtm_disconnect(self, exc):
        """Called by qtm_rt when the connection drops."""
        reason = str(exc) if exc else "unknown"
        # Single WARNING for this mid-session drop — this IS the
        # available→unavailable transition log. Mark the outage active so the
        # reconnect loop's connect() failures stay silent (no duplicate "QTM
        # unavailable" line); the "Connected to QTM." INFO clears it on
        # recovery.
        self.logger.warning(f"QTM disconnected ({reason}), reconnecting in background")
        self._qtm_outage_active = True
        self._set_qtm_lib_quiet(True)
        self.connection = None
        # Reset clock sync so stale offsets aren't used during the gap
        with self._qtm_sync_lock:
            self._qtm_sync = MinLatencyClockSync()
            self._qtm_to_ros_offset_ns = None
            self._qtm_sync_count = 0
        # Clear all cached data so downstream sees empty state, not stale data
        # (frames still queued from before the drop are discarded; the frame
        # counter restarts so the reconnect is not counted as a QTM gap).
        with self.data_lock:
            self._frames.clear()
            self._latest_frame = None
            self._last_frame_number = None
            self.ball_butler_markers = np.full((BB_MARKER_COUNT, 4), np.nan)
        self.is_aligned = False
        self._params_need_refresh = False
        if self.loop and self.loop.is_running():
            self.loop.create_task(self.connect())

    async def connect(self):
        """
        Asynchronously connect to QTM and start streaming data.
        Retries with exponential backoff on failure.
        """
        attempt = 0
        while True:
            try:
                # asyncio.wait_for bounds the whole attempt — qtm_rt's own
                # timeout= does not cover TCP connect (see _CONNECT_TIMEOUT_S).
                self.connection = await asyncio.wait_for(
                    qtm_rt.connect(
                        self.host, port=self.port, timeout=5.0,
                        on_disconnect=self._on_qtm_disconnect,
                    ),
                    timeout=self._CONNECT_TIMEOUT_S,
                )
                if self.connection is None:
                    raise ConnectionError("QTM returned None connection")

                # unavailable→available transition. Clear the outage latch so a
                # FUTURE outage logs its first failure again (state-transition
                # logging, not throttled repeats).
                self._qtm_outage_active = False
                self.logger.info("Connected to QTM.")

                # Get 6dof settings from qtm
                xml_6d_string = await self.connection.get_parameters(parameters=["6d"])
                self.body_dict = self.create_body_dict(xml_6d_string)

                # Get 3d settings from qtm
                xml_3d_string = await self.connection.get_parameters(parameters=["3d"])
                self.marker_dict = self.create_marker_dict(xml_3d_string)

                # Start streaming frames with required components.
                await self.start_streaming()
                self._set_qtm_lib_quiet(False)    # restore qtm_rt logging
                return  # streaming started successfully
            except (asyncio.TimeoutError, ConnectionError, OSError) as e:
                delay = self._RETRY_DELAYS[min(attempt, len(self._RETRY_DELAYS) - 1)]
                # str(asyncio.TimeoutError) is "" — fall back to the type name
                # so the line stays informative (e.g. "(TimeoutError)").
                reason = str(e) or type(e).__name__
                # Log only the FIRST failure of this outage; later retry
                # failures stay silent until QTM returns (state-transition
                # logging, not a throttled repeat).
                self._log_qtm_outage(reason)
                await asyncio.sleep(delay)
                attempt += 1
            except Exception as e:
                self.logger.error(f"Unexpected error connecting to QTM: {e}")
                return  # Don't retry on unexpected errors

    async def _refresh_parameters(self):
        """Re-fetch 3D/6DOF parameter dictionaries from QTM.

        Called when streaming data suggests the cached dicts are stale
        (e.g. QTM measurement started after we connected).
        """
        try:
            xml_6d = await self.connection.get_parameters(parameters=["6d"])
            new_body_dict = self.create_body_dict(xml_6d)

            xml_3d = await self.connection.get_parameters(parameters=["3d"])
            new_marker_dict = self.create_marker_dict(xml_3d)

            self.body_dict = new_body_dict
            self.marker_dict = new_marker_dict
            self._params_need_refresh = False

            self.logger.info(
                f"QTM setup refreshed: {len(new_marker_dict)} markers, "
                f"{len(new_body_dict)} bodies"
            )
        except Exception as e:
            self.logger.error(f"Error refreshing QTM parameters: {e}")

    async def start_streaming(self):
        """
        Start streaming frames from QTM.
        """
        try:
            await self.connection.stream_frames(
                components=["3dres", "3dnolabelsres", "6d"],
                on_packet=self.on_packet
            )
        except Exception as e:
            self.logger.error(f"Error starting frame streaming: {e}")

    def is_receiving(self, timeout_s: float = 1.0) -> bool:
        """True if a QTM packet arrived within the last *timeout_s* seconds."""
        return (self._last_packet_time is not None
                and (time.monotonic() - self._last_packet_time) < timeout_s)

    def _refresh_name_lookups(self):
        """Rebuild the index -> name lists when marker_dict / body_dict has been
        replaced (connect, _refresh_parameters). Was get_name_from_index, a
        linear dict scan per marker per frame (O(L²) per packet)."""
        md, bd = self.marker_dict, self.body_dict
        if self._lookup_src[0] is md and self._lookup_src[1] is bd:
            return
        names = [''] * (max(md.values()) + 1 if md else 0)
        for name, i in md.items():
            names[i] = name
        bodies = [''] * (max(bd.values()) + 1 if bd else 0)
        for name, i in bd.items():
            bodies[i] = name
        self._marker_names = names
        self._body_names = [
            (raw.replace(' ', '_').replace('-', '_'),
             "world" if raw in self.base_frame_bodies else "platform_start",
             raw in self.base_frame_bodies)
            for raw in bodies]
        self._bb_marker_indices = [md.get(f"Ball Butler - {i}", -1)
                                   for i in range(1, BB_MARKER_COUNT + 1)]
        self._lookup_src = (md, bd)

    def on_packet(self, packet):
        """
        Callback to process incoming data packets: one MocapFrame per QTM frame
        onto the bounded frame queue (FRAME_QUEUE_MAXLEN).

        Parameters:
        - packet: The data packet received from QTM.
        """
        self._last_packet_time = time.monotonic()

        # ── Update QTM ↔ ROS clock offset every frame ──────────────────
        # Do this before the ready_to_publish check so sync warms up during
        # startup. The frame's stamp is converted here, once.
        receive_ns, stamp_ns = self._update_qtm_clock_sync(packet)

        # ── Extract labelled markers once (reused for staleness check + processing) ──
        markers_residual = packet.get_3d_markers_residual()

        # ── Detect stale parameter dicts and schedule a re-fetch ───────
        # If QTM was started after we connected, marker/body dicts will be empty
        # but the packet will contain data.
        needs_refresh = False
        if markers_residual is not None:
            _, markers_check = markers_residual
            if len(markers_check) > 0 and len(self.marker_dict) == 0:
                needs_refresh = True
            elif len(markers_check) != len(self.marker_dict) and len(self.marker_dict) > 0:
                needs_refresh = True

        sixdof_data = packet.get_6d()
        if sixdof_data is not None:
            _, bodies_check = sixdof_data
            if len(bodies_check) > 0 and len(self.body_dict) == 0:
                needs_refresh = True
            elif len(bodies_check) != len(self.body_dict) and len(self.body_dict) > 0:
                needs_refresh = True

        if needs_refresh and not self._params_need_refresh:
            self._params_need_refresh = True
            self.logger.debug("QTM parameter mismatch detected — scheduling refresh")
            self.loop.create_task(self._refresh_parameters())

        # Check if we are ready to publish data
        if not self.ready_to_publish:
            return

        # De-duplicate by QTM frame number: a packet repeating the last frame
        # is not a new frame. (Only the receive thread writes the number.)
        frame_number = int(packet.framenumber)
        if frame_number == self._last_frame_number:
            with self.data_lock:
                self._duplicate_frames += 1
            return

        self._refresh_name_lookups()

        """ Sometimes QTM erroneously labels single markers as being part of a rigid body.
        Since the balls are single markers, this means they will sometimes appear in the labelled markers section.
        To ensure best ball tracking, we broadcast ALL marker data to the rest of the ROS2 network.
        """
        labelled_markers = []
        bb_rows = _NAN_BB_ROWS
        header = None
        try:
            if markers_residual is not None:
                header, markers = markers_residual
                n_markers = len(markers)

                # Always store Ball Butler marker positions (negligible cost,
                # needed for calibration and useful for always-on visualisation)
                rows = []
                for idx in self._bb_marker_indices:
                    if 0 <= idx < n_markers:
                        m = markers[idx]
                        rows.append((m.x, m.y, m.z, m.residual))
                    else:
                        rows.append(_NAN_ROW)
                bb_rows = tuple(rows)
                if len(bb_rows) != BB_MARKER_COUNT:      # marker_dict not loaded yet
                    bb_rows = _NAN_BB_ROWS

                # Get all labelled markers (including Ball Butler)
                names = self._marker_names
                n_names = len(names)
                for i, marker in enumerate(markers):
                    x, y, z = marker.x, marker.y, marker.z
                    if not (_isnan(x) or _isnan(y) or _isnan(z)):
                        labelled_markers.append(
                            (names[i] if i < n_names else "", x, y, z, marker.residual))

        except Exception as e:
            self.logger.error(f"Error processing labelled markers: {e}")

        # Process 6dof data to know the body positions. Plain tuples; the
        # PoseStamped messages are built by the publisher, once per frame.
        # Keyed by sanitised name: a repeated name keeps the last (as before).
        new_bodies = {}

        if sixdof_data is not None:
            info, bodies = sixdof_data
        else:
            bodies = []

        body_names = self._body_names
        n_body_names = len(body_names)
        for i, body in enumerate(bodies):
            body_name, frame_id, is_base_frame = (
                body_names[i] if i < n_body_names else ("", "platform_start", False))
            pos = body[0]

            # Leave base-frame bodies' Z unchanged; shift others into the platform frame
            z = pos.z
            if not is_base_frame and self.base_to_platform_transformation is not None:
                z = pos.z - self.base_to_platform_transformation

            qx, qy, qz, qw = quaternion_from_rotation_list(body[1].matrix)
            new_bodies[body_name] = (body_name, frame_id, pos.x, pos.y, z, qx, qy, qz, qw)

            # ── Alignment check: "Base" body should be near the global origin ──
            if body_name == "Base":
                pos_dist = _sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                # Rotation angle from identity: 2 * acos(|qw|) (NaN stays NaN)
                rot_angle_deg = math.degrees(2.0 * math.acos(min(max(abs(qw), 0.0), 1.0))) \
                    if not _isnan(qw) else math.nan
                aligned = bool(pos_dist <= self._align_pos_thresh_mm
                              and rot_angle_deg <= self._align_rot_thresh_deg)
                if self.is_aligned and not aligned:
                    if _isnan(pos_dist) or _isnan(rot_angle_deg):
                        misalign_detail = "Base body not visible to QTM"
                    else:
                        misalign_detail = (
                            f"pos {pos_dist:.0f} mm, rot {rot_angle_deg:.1f}° "
                            f"(limits {self._align_pos_thresh_mm:.0f} mm, "
                            f"{self._align_rot_thresh_deg:.0f}°)"
                        )
                    self.logger.warning(f"Mocap base misaligned: {misalign_detail}")
                elif not self.is_aligned and aligned:
                    self.logger.info("Mocap base aligned")
                self.is_aligned = aligned

        # Process unlabelled markers, skipping NaN positions.
        current_unlabelled = []
        markers_no_label_residual = packet.get_3d_markers_no_label_residual()
        if markers_no_label_residual is not None:
            _, markers = markers_no_label_residual
            for marker in markers:
                x, y, z = marker.x, marker.y, marker.z
                if not (_isnan(x) or _isnan(y) or _isnan(z)):
                    current_unlabelled.append((x, y, z, marker.residual))

        frame = MocapFrame(frame_number, packet.timestamp, stamp_ns, receive_ns,
                           self.is_aligned, labelled_markers, current_unlabelled,
                           tuple(new_bodies.values()), bb_rows)

        with self.data_lock:
            if markers_residual is not None:
                self.ball_butler_markers = np.array(bb_rows, dtype=float)
            if header is not None:
                self.drop_rate = header.drop_rate
                self.out_of_sync_rate = header.out_of_sync_rate
                if header.drop_rate > self._max_drop_rate:
                    self._max_drop_rate = header.drop_rate
                if header.out_of_sync_rate > self._max_out_of_sync_rate:
                    self._max_out_of_sync_rate = header.out_of_sync_rate
            # Update unlabelled residual statistics.
            self.residuals_unlabelled.extend(m[3] for m in current_unlabelled)

            last = self._last_frame_number
            if last is not None:
                if frame_number > last + 1:
                    self._qtm_gap_frames += frame_number - last - 1
                    self._qtm_gap_events += 1
                elif frame_number < last:
                    self._qtm_restarts += 1      # QTM restarted its counter: not a gap
            self._last_frame_number = frame_number

            if len(self._frames) == FRAME_QUEUE_MAXLEN:
                self._overflow_drops += 1        # the append below drops the OLDEST
            self._frames.append(frame)
            self._frames_queued += 1
            if len(self._frames) > self._max_queue_depth:
                self._max_queue_depth = len(self._frames)
            self._latest_frame = frame

    def stop(self):
        """
        Stop the tracker and close the connection.
        """
        if self.loop:
            if hasattr(self, 'connection') and self.connection:
                self.loop.call_soon_threadsafe(self.connection.disconnect)
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread:
            self.thread.join()

    #########################################################################################################
    #                                      QTM ↔ ROS Clock Sync                                            #
    #########################################################################################################

    def _update_qtm_clock_sync(self, packet):
        """Feed one packet's (QTM timestamp, ROS receive time) to the clock-sync
        estimator; return ``(receive_ns, stamp_ns)``: the ROS receive time it
        read and the packet's QTM timestamp on the ROS clock through the
        just-updated offset (``(None, None)`` without a node, ``stamp_ns`` None
        on an error). The frame's ONE conversion (logbook
        2026-10-10-mocap-frame-queue).

        ``ros_time_ns ≈ qtm_timestamp_us * 1000 + offset_ns`` where offset_ns is the
        lower envelope of ``ros_receive_ns - qtm_ns`` over a sliding window
        (queueing only ever adds delay), drift-corrected and slew-limited — see
        jugglebot.qtm_clock_sync. The first packet initialises it directly; a QTM
        restart (timestamp backwards / jump > 5 s) or a clock step far outside
        the window's range re-anchors it.
        """
        if self.node is None:
            return None, None

        ros_ns = None
        try:
            qtm_us = int(packet.timestamp)  # QTM camera time in microseconds
            ros_ns = int(self.node.get_clock().now().nanoseconds)  # ROS time in nanoseconds

            with self._qtm_sync_lock:
                event = self._qtm_sync.update(qtm_us, ros_ns)
                self._qtm_to_ros_offset_ns = self._qtm_sync.offset_ns
                self._qtm_sync_count = self._qtm_sync.count
                stamp_ns = (None if self._qtm_to_ros_offset_ns is None
                            else qtm_us * 1000 + self._qtm_to_ros_offset_ns)
            if event == 'restart':
                self.logger.warning("QTM timestamp discontinuity — resetting clock sync")
            elif event in ('reanchor_below', 'reanchor_above'):
                self.logger.warning(
                    f"QTM clock offset moved far outside the sync window ({event}) — "
                    "re-anchored clock sync")
            elif event == 'init':
                self.logger.debug(
                    f"QTM clock sync initialised: offset = {self._qtm_to_ros_offset_ns / 1e9:.6f} s"
                )

            return ros_ns, stamp_ns

        except Exception as e:
            self.logger.error(f"QTM clock sync error: {e}", throttle_duration_sec=5.0)
            return ros_ns, None

    def qtm_timestamp_to_ros_ns(self, qtm_timestamp_us: int) -> Optional[int]:
        """Convert a QTM timestamp (microseconds) to ROS time (nanoseconds).

        Returns None if the clock sync has not yet been initialised.
        """
        with self._qtm_sync_lock:
            if self._qtm_to_ros_offset_ns is None:
                return None
            return int(qtm_timestamp_us) * 1000 + self._qtm_to_ros_offset_ns

    def qtm_timestamp_to_ros_sec(self, qtm_timestamp_us: int) -> Optional[float]:
        """Convert a QTM timestamp (microseconds) to ROS time (seconds as float).

        Returns None if the clock sync has not yet been initialised.
        """
        ns = self.qtm_timestamp_to_ros_ns(qtm_timestamp_us)
        return ns / 1e9 if ns is not None else None

    def latest_frame_ros_ns(self) -> Optional[int]:
        """ROS time (ns) of the most recent frame's own QTM timestamp — the
        stamp its MocapFrame carries (converted once, at receive). None if no
        frame has arrived yet or the clock sync was not established for it.
        """
        frame = self._latest_frame
        return None if frame is None else frame.stamp_ns

    def get_qtm_sync_status(self) -> dict:
        """Return the current QTM clock sync status for diagnostics.

        ``synced`` / ``offset_s`` / ``sample_count`` as before; the estimator's
        diagnostics (last packet's excess latency over the envelope, pending
        slew, window fill, fitted drift, slew-clamp / outlier-clip / re-anchor
        counts) are merged in under their own keys.
        """
        with self._qtm_sync_lock:
            status = {
                "synced": self._qtm_to_ros_offset_ns is not None,
                "offset_s": self._qtm_to_ros_offset_ns / 1e9 if self._qtm_to_ros_offset_ns is not None else None,
                "sample_count": self._qtm_sync_count,
            }
            diag = self._qtm_sync.diagnostics()
        diag.pop('sample_count', None)
        status.update(diag)
        return status

    #########################################################################################################
    #                                  Package Data For Parent Modules                                      #
    #########################################################################################################

    def drain_frames(self) -> List[MocapFrame]:
        """Every queued frame, oldest first, and empty the queue (the
        publisher, once per tick). ``[]`` when nothing new arrived."""
        with self.data_lock:
            if not self._frames:
                return []
            frames = list(self._frames)
            self._frames.clear()
        return frames

    def latest_frame(self) -> Optional[MocapFrame]:
        """The most recent frame received (queued or already drained)."""
        return self._latest_frame

    def take_stream_stats(self) -> dict:
        """The stream counters (cumulative since start) plus the queue depth
        now, its high-water mark and QTM's highest 2D drop / out-of-sync rate
        (per thousand) since the last call — which resets those three. One
        reader: the node's 1 Hz line."""
        with self.data_lock:
            stats = {
                'frames_queued': self._frames_queued,
                'overflow_drops': self._overflow_drops,
                'qtm_gap_frames': self._qtm_gap_frames,
                'qtm_gap_events': self._qtm_gap_events,
                'duplicate_frames': self._duplicate_frames,
                'qtm_restarts': self._qtm_restarts,
                'queue_depth': len(self._frames),
                'max_queue_depth': self._max_queue_depth,
                'drop_rate': self.drop_rate,
                'out_of_sync_rate': self.out_of_sync_rate,
                'max_drop_rate': self._max_drop_rate,
                'max_out_of_sync_rate': self._max_out_of_sync_rate,
            }
            self._max_queue_depth = len(self._frames)
            self._max_drop_rate = self.drop_rate
            self._max_out_of_sync_rate = self.out_of_sync_rate
        return stats

    def get_ball_butler_markers_base_frame(self) -> np.ndarray:
        """
        Get the current ball butler markers (and residuals) in the base frame.

        Returns:
        - A numpy array of ball butler marker data (Nx4).
        """
        with self.data_lock:
            return self.ball_butler_markers.copy() if self.ball_butler_markers.size > 0 else np.empty((0, 4))

    def get_performance_statistics(self) -> Dict[str, Optional[float]]:
        """
        Get the performance statistics.

        Returns:
        - A dictionary with the latest average unlabelled residual, drop rate, and out-of-sync rate.
        """
        with self.data_lock:
            stats = {
                'average_residual_unlabelled': np.mean(self.residuals_unlabelled) if self.residuals_unlabelled else None,
                'drop_rate': self.drop_rate,
                'out_of_sync_rate': self.out_of_sync_rate
            }
            return stats

    #########################################################################################################
    #                                          Helper Methods                                               #
    #########################################################################################################

    def create_body_dict(self, xml_string):
        """ Extract a name to index dictionary from 6dof settings xml """
        xml = ET.fromstring(xml_string)
        
        body_dict = {}
        for index, body in enumerate(xml.findall("*/Body/Name")):
            body_dict[body.text.strip()] = index

        return body_dict

    def create_marker_dict(self, xml_string):
        """ Extract a name to index dictionary from 3d settings xml """
        xml = ET.fromstring(xml_string)
        
        marker_dict = {}
        for index, marker in enumerate(xml.findall("*/Label/Name")):
            marker_dict[marker.text.strip()] = index

        return marker_dict
    
    def get_name_from_index(self, index, name_dict):
        """Linear lookup (off the per-frame path: on_packet uses the lists
        _refresh_name_lookups builds)."""
        for name, i in name_dict.items():
            if i == index:
                return name
        return ""

    def rotation_list_to_quaternion(self, R_list):
        """
        Convert a 9-element list (representing a column-major 3x3 rotation matrix)
        to a quaternion, returned as ``np.array([qx, qy, qz, qw])``.

        R_list = [r11, r21, r31, r12, r22, r32, r13, r23, r33]

        The numpy reference for ``quaternion_from_rotation_list`` (the plain
        float version on_packet uses since 2026-10-10); kept for its test.
        """
        R = np.array(R_list, dtype=np.float64).reshape((3, 3), order='F')
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            S = 2.0 * np.sqrt(trace + 1.0)
            qw = 0.25 * S
            qx = (R[2, 1] - R[1, 2]) / S
            qy = (R[0, 2] - R[2, 0]) / S
            qz = (R[1, 0] - R[0, 1]) / S
        else:
            i = np.argmax(np.diag(R))
            if i == 0:
                S = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                qw = (R[2, 1] - R[1, 2]) / S
                qx = 0.25 * S
                qy = (R[0, 1] + R[1, 0]) / S
                qz = (R[0, 2] + R[2, 0]) / S
            elif i == 1:
                S = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                qw = (R[0, 2] - R[2, 0]) / S
                qx = (R[0, 1] + R[1, 0]) / S
                qy = 0.25 * S
                qz = (R[1, 2] + R[2, 1]) / S
            else:
                S = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                qw = (R[1, 0] - R[0, 1]) / S
                qx = (R[0, 2] + R[2, 0]) / S
                qy = (R[1, 2] + R[2, 1]) / S
                qz = 0.25 * S

        return np.array([qx, qy, qz, qw])

    #########################################################################################################
    #                                        Get Robot Geometry                                             #
    #########################################################################################################

    def set_base_to_platform_offset(self, offset: float):
        """
        Set the offset from the base to the platform.

        Parameters:
        - offset: The offset in mm.
        """
        self.base_to_platform_transformation = offset

    def set_alignment_thresholds(self, pos_mm: float, rot_deg: float):
        """Set thresholds for the base-body alignment check."""
        self._align_pos_thresh_mm = pos_mm
        self._align_rot_thresh_deg = rot_deg


if __name__ == "__main__":
    tracker = MocapInterface()
    if tracker.logger is None:
        logger = logging.getLogger("MocapInterface")
        handler = logging.StreamHandler()
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        tracker.logger = logger

    try:
        while True:
            for frame in tracker.drain_frames():
                print(f"frame {frame.frame_number}: {len(frame.unlabelled)} unlabelled markers")
                print(frame.unlabelled)
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("Keyboard interrupt received. Shutting down.")

    finally:
        tracker.stop()
        print("MocapInterface stopped.")