/**
 * geometry-config.js — Hardcoded constants from hardware_config.yaml.
 *
 * These are the geometry constants for the Jugglebot Stewart platform.
 * Source of truth: config/hardware_config.yaml
 */

// Stewart platform geometry
export const INITIAL_HEIGHT_MM = 574.3;
export const BASE_RADIUS_MM = 410.0;
export const PLAT_RADIUS_MM = 219.075;
export const LEG_STROKE_MM = 280.0;
export const BALL_JOINT_OFFSET_MM = 25.5;

// Arm / hand structure
export const ARM_RADIUS_MM = 70.0;
export const ARM_HEIGHT_FROM_PLATFORM_MM = 210.25;
export const HAND_STROKE_MM = 355.0;
export const HAND_RADIUS_MM = 35.0;

// Base node positions in the base frame (mm). Z = 0 for all.
// Computed from base_radius=410mm, base_small_angle=20deg.
export const BASE_NODES_MM = [
    [-385.274, -140.228, 0.0],
    [-314.078, -263.543, 0.0],
    [ 314.078, -263.543, 0.0],
    [ 385.274, -140.228, 0.0],
    [  71.196,  403.771, 0.0],
    [ -71.196,  403.771, 0.0],
];

// Initial platform node positions in the platform frame (mm). Z = 0 for all.
// Computed from plat_radius=219.075, plat_small_angle=8.6024, plat_x_axis_offset=154.3012.
export const INIT_PLAT_NODES_MM = [
    [-197.405,   95.000, 0.0],
    [ -16.431, -218.458, 0.0],
    [  16.431, -218.458, 0.0],
    [ 197.405,   95.000, 0.0],
    [ 180.975,  123.458, 0.0],
    [-180.975,  123.458, 0.0],
];

// Initial leg lengths (mm), measured experimentally. Includes ball_joint_offset.
export const INIT_LEG_LENGTHS_MM = [621.389, 618.160, 622.074, 620.653, 620.652, 620.091];

// Init leg lengths WITH the ball joint offset added (derived constant from generate_config.py)
export const INIT_LEG_LENGTHS_WITH_OFFSET_MM = [646.889, 643.660, 647.574, 646.153, 646.152, 645.591];

// Per-leg mm-to-revolutions conversion factors
export const MM_TO_REV = [0.01418332, 0.01419076, 0.01408956, 0.01418684, 0.01426801, 0.01424951];

// Motor position limits
export const LEG_MOTOR_MAX_POS_REVS = 4.2;
export const HAND_MOTOR_MAX_POS_REVS = 11.1;

// Ball Butler position relative to base centre (mm).
// Placeholder — user will provide real coordinates once known.
export const BB_POSITION_MM = [0, -500, 0];

// Hand axis: offset from platform centre along platform local -Y axis (mm)
export const HAND_DEPTH_OFFSET_MM = 100.0;

// ODrive state enum values (from ODrive protocol)
export const ODRIVE_STATE = {
    UNDEFINED: 0,
    IDLE: 1,
    STARTUP_SEQUENCE: 2,
    FULL_CALIBRATION: 3,
    MOTOR_CALIBRATION: 4,
    ENCODER_INDEX_SEARCH: 6,
    ENCODER_OFFSET_CALIBRATION: 7,
    CLOSED_LOOP: 8,
    LOCKIN_SPIN: 9,
    ENCODER_DIR_FIND: 10,
    HOMING: 11,
    ENCODER_HALL_POLARITY: 12,
    ENCODER_HALL_PHASE: 13,
};

// Ball Butler states
export const BB_STATE = {
    BOOT: 0,
    IDLE: 1,
    RELOADING: 2,
    THROWING: 3,
    ERROR: 127,
};

export const BB_STATE_NAMES = {
    0: 'BOOT',
    1: 'IDLE',
    2: 'RELOADING',
    3: 'THROWING',
    127: 'ERROR',
};
