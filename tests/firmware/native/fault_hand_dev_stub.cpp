// =============================================================================
//  fault_hand_dev_stub.cpp — minimal stand-in for fault_machine.cpp's
//  fault_hand_dev_prev_reset(), linked ONLY into test_leg_interp.
// =============================================================================
//  leg_interp.cpp's interp_hand_counters_reset() (the `hand7 reset` console
//  verb) calls fault_machine.h's fault_hand_dev_prev_reset() so a reset
//  re-baselines BOTH sides of fault_machine.cpp's `>` exceed-tick comparison
//  atomically (A-N1). test_leg_interp.cpp #includes leg_interp.cpp but NOT
//  fault_machine.cpp (the two TUs are never both #included/linked together —
//  see build.py's ODR-clean rule), so it needs a definition of that symbol
//  from somewhere: this no-op stub.
//
//  test_fault_machine.cpp does NOT link this: it #includes the REAL
//  fault_machine.cpp, which defines fault_hand_dev_prev_reset() itself —
//  linking both there would be a duplicate-definition link error. The masking-
//  window regression this hook fixes is exercised against the REAL
//  implementation in test_fault_machine.cpp (see its "`hand7 reset` re-
//  baselines BOTH sides..." test cases); this stub only keeps
//  test_leg_interp.cpp's own hand7-reset tests (which assert leg_interp.cpp's
//  own counters, not fault_machine.cpp's) linking cleanly.
// =============================================================================

#include "fault_machine.h"

namespace CanBridge {

void fault_hand_dev_prev_reset() {}

}  // namespace CanBridge
