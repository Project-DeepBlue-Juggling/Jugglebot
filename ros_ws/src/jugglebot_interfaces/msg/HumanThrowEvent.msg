# One revision of the Tracker's Landing for one un-announced (human) Throw flight.
# Published on /human_throws only while the tracker's detect_human_throws is true.
# Identity is exactly (track_id, flight_seq); revision 0 is the first usable Fit,
# later revisions refine it, final = true once the Fit is frozen.

std_msgs/Header header                   # stamp = mocap time of the frame that produced this revision (ROS clock, as /balls)
uint32 track_id                          # BallState.id of the human_throw track
uint32 flight_seq                        # FlightFit segment counter for that track (0 = the promotion flight)
uint32 revision                          # 0 = first usable Fit of this (track_id, flight_seq); +1 per refinement
builtin_interfaces/Time flight_start     # time of the segment's first admitted sample
geometry_msgs/Point landing_position     # mm, mocap WORLD frame, at landing_z_mm (the Catch plane)
geometry_msgs/Vector3 landing_velocity   # mm/s, WORLD frame
builtin_interfaces/Time landing_time     # ROS time of the Catch-plane crossing
float32 landing_z_mm
uint16 n_samples                         # Fit samples surviving the residual gate
float32 rms_mm                           # Fit residual
float32 span_s                           # Fit sample span
float32 fit_age_s                        # header.stamp minus the last admitted sample's time
float32 start_speed_mm_s                 # |v| at the flight's start (the phantom rule's input)
bool final                               # true once the Fit is frozen (ball below the freeze floor)
