/**
 * robot-frame.js — world-anchored Stewart model + robot-frame status/move banner.
 *
 * Topics (strings are the contract with mocap_node.py / mocap_status.py):
 *   robot_frame  geometry_msgs/PoseStamped, latched.  Position mm in QTM world,
 *                orientation a pure yaw quaternion.  Moves robotFrameGroup.
 *   mocap/status diagnostic_msgs/DiagnosticStatus (5 Hz).  Keys read:
 *                robot_frame_settled ('1'/'0'), robot_frame_move_count,
 *                robot_frame_last_move_mm, robot_frame_last_move_deg,
 *                robot_frame_last_move_s, robot_frame_origin_mm ('x,y,z'),
 *                robot_frame_yaw_deg, robot_frame_rms_mm, robot_frame_n_matched.
 */
import { robotFrameGroup, robotToThreeScaled } from './viewer.js';
import { setRobotFrameDetail } from './panels.js';

const KEY_SETTLED = 'robot_frame_settled';
const KEY_MOVE_COUNT = 'robot_frame_move_count';
const KEY_LAST_MOVE_MM = 'robot_frame_last_move_mm';
const KEY_LAST_MOVE_DEG = 'robot_frame_last_move_deg';
const KEY_LAST_MOVE_S = 'robot_frame_last_move_s';
const KEY_ORIGIN_MM = 'robot_frame_origin_mm';
const KEY_YAW_DEG = 'robot_frame_yaw_deg';
const KEY_RMS_MM = 'robot_frame_rms_mm';
const KEY_N_MARKERS = 'robot_frame_n_matched';

/**
 * Apply a robot_frame PoseStamped to the Stewart group.
 *
 * Yaw sign derivation: the GUI maps robot (x,y,z) -> Three (x,z,-y), and
 * quaternions as (qx,qy,qz,qw) -> (qx,qz,-qy,qw) (mocap-markers.js, the QTM
 * rigid-body triads).  A robot-frame yaw theta about +Z is (0,0,sin(t/2),cos(t/2));
 * the swap gives (0,sin(t/2),0,cos(t/2)) = rotation +theta about Three +Y.
 * Applying the identical swap here therefore turns the model exactly as a QTM
 * Base body triad with the same yaw turns; no separate sign to get wrong.
 */
export function onRobotFrame(msg) {
    if (!robotFrameGroup) return;
    const pose = (msg && msg.pose) || msg;
    const p = pose && pose.position;
    const q = pose && pose.orientation;
    if (!p || !q) return;
    if (![p.x, p.y, p.z, q.x, q.y, q.z, q.w].every(Number.isFinite)) return;
    const pos = robotToThreeScaled(p.x, p.y, p.z);
    robotFrameGroup.position.set(pos.x, pos.y, pos.z);
    robotFrameGroup.quaternion.set(q.x, q.z, -q.y, q.w);
}

function kvMap(msg) {
    const kv = {};
    for (const v of (msg.values || [])) kv[v.key] = v.value;
    return kv;
}

function num(s) { const v = parseFloat(s); return Number.isFinite(v) ? v : NaN; }

let banner = null;
let bannerText = null;
let lastCount = null;     // null until the first status (a reload does NOT replay an old move)
let bannerTimer = null;
//: the banner auto-hides after this (owner, 2026-10-19: 3 s is enough to read the size);
//: the WARN line in the launch log and mocap/status keep the record.
const ROBOT_FRAME_BANNER_MS = 3000;

function ensureBanner() {
    if (banner) return;
    const pane = document.getElementById('viewer-pane');
    if (!pane) return;
    banner = document.createElement('div');
    banner.id = 'robot-frame-banner';
    banner.className = 'robot-frame-banner';
    banner.hidden = true;
    bannerText = document.createElement('span');
    const close = document.createElement('button');
    close.type = 'button';
    close.className = 'robot-frame-banner-close';
    close.textContent = '×';
    close.title = 'Dismiss';
    close.addEventListener('click', () => { hideBanner(); });
    banner.append(bannerText, close);
    pane.appendChild(banner);
}

function hhmmss(sec) {
    if (!(sec > 0)) return '--:--:--';
    const d = new Date(sec * 1000);
    const z = (n) => String(n).padStart(2, '0');
    return `${z(d.getHours())}:${z(d.getMinutes())}:${z(d.getSeconds())}`;
}

function originXY(kv) {
    const parts = (kv[KEY_ORIGIN_MM] || '').split(',').map(num);
    if (parts.length < 2 || !Number.isFinite(parts[0]) || !Number.isFinite(parts[1])) return '';
    return ` · now (${parts[0].toFixed(1)}, ${parts[1].toFixed(1)}) mm`
        + (Number.isFinite(num(kv[KEY_YAW_DEG])) ? `, ${num(kv[KEY_YAW_DEG]).toFixed(1)}°` : '');
}

function hideBanner() {
    if (bannerTimer) { clearTimeout(bannerTimer); bannerTimer = null; }
    if (banner) banner.hidden = true;
}

/** mocap/status handler: pill detail + move banner. */
export function onMocapStatus(msg) {
    const kv = kvMap(msg);

    // Pill secondary text: "rms 0.87 mm · 7 mk · (23.8, 19.5) mm · -15.5°"
    const rms = num(kv[KEY_RMS_MM]);
    const nmk = num(kv[KEY_N_MARKERS]);
    const org = (kv[KEY_ORIGIN_MM] || '').split(',').map(num);
    const yaw = num(kv[KEY_YAW_DEG]);
    const bits = [];
    if (Number.isFinite(rms)) bits.push(`rms ${rms.toFixed(2)} mm`);
    if (Number.isFinite(nmk)) bits.push(`${nmk} mk`);
    if (org.length >= 2 && Number.isFinite(org[0]) && Number.isFinite(org[1])) {
        bits.push(`(${org[0].toFixed(1)}, ${org[1].toFixed(1)}) mm`);
    }
    if (Number.isFinite(yaw)) bits.push(`${yaw.toFixed(1)}°`);
    // Absent key => treated as settled (a stack without the key must not show SETTLING).
    setRobotFrameDetail({ settled: kv[KEY_SETTLED] !== '0', text: bits.join(' · ') });

    // Move banner: shown when move_count increases, auto-hides after ROBOT_FRAME_BANNER_MS
    // (x closes it sooner). The first status after a reload only seeds the count.
    const count = parseInt(kv[KEY_MOVE_COUNT], 10);
    if (!Number.isFinite(count)) return;
    ensureBanner();
    if (!banner) return;
    const prev = lastCount;
    lastCount = count;
    if (prev !== null && count > prev) {
        const mm = num(kv[KEY_LAST_MOVE_MM]);
        const deg = num(kv[KEY_LAST_MOVE_DEG]);
        bannerText.textContent = `Robot frame MOVED ${Number.isFinite(mm) ? mm.toFixed(1) : '?'} mm / `
            + `${Number.isFinite(deg) ? deg.toFixed(1) : '?'}° at ${hhmmss(num(kv[KEY_LAST_MOVE_S]))}`
            + originXY(kv);
        banner.hidden = false;
        if (bannerTimer) clearTimeout(bannerTimer);
        bannerTimer = setTimeout(hideBanner, ROBOT_FRAME_BANNER_MS);
    }
}
