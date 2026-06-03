// =============================================================================
//  axis_state.cpp — storage for the per-axis state cache
// =============================================================================
#include "axis_state.h"

namespace CanBridge {

// Indices 0..5 = legs (node id == index), index 6 = hand.
AxisState axes[NUM_AXES];

}  // namespace CanBridge
