// =============================================================================
//  axis_state.cpp — storage for the per-axis state cache
// =============================================================================
#include "axis_state.h"

namespace CanBridge {

// Indices 0..5 = legs (node id == index), index 6 = hand.
AxisState axes[NUM_AXES];

// Ball Butler ODrives on CAN1: index 0 = bb_pitch (node 7), 1 = bb_hand (node 8).
AxisState bb_axes[NUM_BB_AXES];

}  // namespace CanBridge
