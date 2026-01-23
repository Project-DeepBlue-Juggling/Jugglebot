// TrajFrame.h

#pragma once
struct TrajFrame {
  float t_s;     // time (s) relative to chosen zero (e.g., decel start)
  float pos_cmd; // [rev]
  float vel_ff;  // [rev/s]
  float tor_ff;  // [N·m]
};