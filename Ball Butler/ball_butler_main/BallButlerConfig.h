#pragma once
/**
 * BallButlerConfig.h - Centralised constants for Ball Butler
 *
 * This file collects magic numbers that are referenced across multiple
 * modules so they can be changed in a single place.
 *
 * SECTIONS
 *   1. ODrive axis-state IDs  (protocol constants, unlikely to change)
 *   2. CAN node IDs           
 */

#include <cstdint>

// ============================================================================
// 1. ODrive Axis States
//    These match the ODrive CAN protocol enum (AxisState).
// ============================================================================
namespace ODriveState {
  constexpr uint32_t IDLE              = 1u;
  constexpr uint32_t CLOSED_LOOP       = 8u;
}

// ============================================================================
// 2. CAN Node IDs
//    Each ODrive axis is assigned a unique node ID via the ODrive web GUI.
// ============================================================================
namespace NodeId {
  constexpr uint8_t PITCH = 7;
  constexpr uint8_t HAND  = 8;
}
