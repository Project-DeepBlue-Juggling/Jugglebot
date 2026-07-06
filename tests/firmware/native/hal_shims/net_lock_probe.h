#pragma once
// =============================================================================
//  net_lock_probe.h — NetLock coverage probe for the native udp_link harness
// =============================================================================
//  Shared, doctest-independent tracker so the fake FreeRTOS recursive mutex
//  (arduino_freertos.h) and the fake QNEthernet (QNEthernet.h) can agree on
//  whether the udp_link NetLock is currently held.
//
//  Model: the only recursive mutex the compiled `udp_link.cpp` ever creates is
//  its single `s_net_mtx` (the NetLock), so a single recursion-depth counter is
//  exactly the net-lock depth. The fake xSemaphoreTakeRecursive/GiveRecursive
//  adjust `depth()`; every fake QNEthernet call (Ethernet.loop + each socket
//  method) calls `require_held()` at entry, which bumps a violation counter when
//  the lock is NOT held. The test asserts `violations() == 0` after driving the
//  service loop — so an unlocked pump/socket call (the regression this probe guards against)
//  is caught as a violation, not a crash.
//
//  Host-only, single-threaded: this validates lock *coverage* (which calls run
//  under the lock), NOT true preemptive reentrancy — that stays an on-hardware
//  gap (see tests/firmware/native/README.md and the firmware README lwIP note).
// =============================================================================

namespace netlockprobe {

// Recursion depth of the (single) udp_link recursive mutex. > 0 ⇒ NetLock held.
inline int& depth() { static int d = 0; return d; }
inline bool held() { return depth() > 0; }

// Count of fake-QNEthernet entries observed while the lock was NOT held.
inline unsigned& violations() { static unsigned v = 0; return v; }

// Called at the entry of every fake QNEthernet call. A well-behaved firmware
// never trips this; a call made outside NetLock records a coverage violation.
inline void require_held() { if (!held()) ++violations(); }

// Reset both counters between test cases.
inline void reset() { depth() = 0; violations() = 0; }

}  // namespace netlockprobe
