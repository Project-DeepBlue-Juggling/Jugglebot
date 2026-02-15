session 1 and 2 - 10 throws to 8 x 8 grid of 800 x 800 mm. Session 1 is first 368 throws, session 2 is rest (note that some throws were skipped due to them being unreachable)
session 3 and 4 - 5 throws to 2 x 2 grid of 200 x 200 mm. Both had correction transformation matrix (from session 1 and 2) applied
session 5 - 4 throws to 5 x 5 grid of 1000 x 1000 mm. No correction transformation matrix applied (post QTM calibration, and post fixing rotation bug from `global_to_bb_frame`
session 6 - same as session 5, but after correcting the forward kinematics (with s, d and l offsets)
session 7 - Timing test. 5 throws to each of 4 'delays'. Min delay = 2.0 sec, max delay = 5.0 sec, with 4 divisions. All aimed at (0, 0, 750) mm
session 8 - same as 7, but after major refactor. Might fix throw delay?