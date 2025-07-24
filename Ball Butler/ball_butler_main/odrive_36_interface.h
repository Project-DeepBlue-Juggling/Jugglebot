#ifndef ODRIVE_36_INTERFACE_H
#define ODRIVE_36_INTERFACE_H

/**
 *  ODrive v3.6 CAN interface helper for Teensy 4.x (FlexCAN_T4 library).
 *  Automatically generates the 11‑bit arbitration ID using the formula:
 *      arbitration_id = (CMD_ID << 5) | node_id
 *  where node_id ∈ [0,31].  The user of this wrapper typically sets node_id = 0
 *  for the yaw axis and 1 for the pitch axis.
 *
 *  Only CAN 2.0B “standard” 11‑bit identifiers are used.
 *
 *  The header contains:
 *    • Enumeration of every command from the official v3.6 CSV spec
 *    • Light‑weight inline helpers for the most common real‑time commands
 *    • Generic send() so any current or future command can be issued
 *    • Optional Heartbeat decode helper
 *
 *  Author: ChatGPT‑o3 — generated 19 Jun 2025
 */

#include <Arduino.h>
#include <FlexCAN_T4.h>

namespace ODriveCAN {

/// All CANSimple command IDs for firmware 0.5x / v3.6 (pulled from can‑protocol.csv)
enum CommandID : uint16_t {
    CMD_CANOPEN_NMT                 = 0x000,
    CMD_HEARTBEAT                   = 0x001,
    CMD_ESTOP                       = 0x002,
    CMD_GET_MOTOR_ERROR             = 0x003,
    CMD_GET_ENCODER_ERROR           = 0x004,
    CMD_GET_SENSORLESS_ERROR        = 0x005,
    CMD_SET_AXIS_NODE_ID            = 0x006,
    CMD_SET_AXIS_REQUESTED_STATE    = 0x007,
    CMD_SET_AXIS_STARTUP_CONFIG     = 0x008,
    CMD_GET_ENCODER_ESTIMATES       = 0x009,
    CMD_GET_ENCODER_COUNT           = 0x00A,
    CMD_SET_CONTROLLER_MODES        = 0x00B,
    CMD_SET_INPUT_POS               = 0x00C,
    CMD_SET_INPUT_VEL               = 0x00D,
    CMD_SET_INPUT_TORQUE            = 0x00E,
    CMD_SET_LIMITS                  = 0x00F,
    CMD_START_ANTICOGGING           = 0x010,
    CMD_SET_TRAJ_VEL_LIMIT          = 0x011,
    CMD_SET_TRAJ_ACCEL_LIMITS       = 0x012,
    CMD_SET_TRAJ_INERTIA            = 0x013,
    CMD_GET_IQ                      = 0x014,
    CMD_GET_SENSORLESS_ESTIMATES    = 0x015,
    CMD_REBOOT                      = 0x016,
    CMD_GET_BUS_VOLT_CURRENT        = 0x017,
    CMD_CLEAR_ERRORS                = 0x018,
    CMD_SET_LINEAR_COUNT            = 0x019,
    CMD_SET_POSITION_GAIN           = 0x01A,
    CMD_SET_VEL_GAINS               = 0x01B,
    CMD_GET_ADC_VOLTAGE             = 0x01C,
    CMD_GET_CONTROLLER_ERROR        = 0x01D,
    // NOTE: 0x700 is a CAN‑Open heartbeat emitted by the master, seldom used here.
};

/// Helper structure for a parsed 8‑byte Heartbeat frame.
struct Heartbeat {
    uint32_t axis_error      = 0;
    uint8_t  current_state   = 0;
    uint8_t  traj_done_flag  = 0;
    uint16_t motor_error     = 0;
    uint16_t encoder_error   = 0;
    uint16_t controller_error= 0;
};

template <typename BUS_T = FlexCAN_T4<CAN1>>
class Interface {
public:
    explicit Interface(BUS_T &bus, uint32_t baud = 1000000)
        : m_bus(bus), m_baud(baud) {}

    void begin() {
        m_bus.begin();
        m_bus.setBaudRate(m_baud);
    }

    /// Generic transmit (data may be nullptr if len == 0)
    bool send(uint8_t node_id, CommandID cmd, const uint8_t *data, uint8_t len) {
        if (len > 8) return false;                // CAN 2.0 data length guard
        CAN_message_t msg{};
        msg.id  = makeID(node_id, cmd);
        msg.len = len;
        if (len && data) memcpy(msg.buf, data, len);
        return m_bus.write(msg);
    }

    /* ------------------------------------------------------------------------
     * High‑level wrappers for time‑critical motion commands
     * ----------------------------------------------------------------------*/

    /// SetPos: position [rad or counts], optional velocity/torque feed‑forwards
    bool setInputPos(uint8_t node_id,
                     float pos,
                     float vel_ff = 0.0f,
                     float torque_ff = 0.0f)
    {
        union { float f; uint32_t u32; } p{pos};
        int16_t vel = static_cast<int16_t>(vel_ff * 1000.0f);      // scale 0.001
        int16_t tq  = static_cast<int16_t>(torque_ff * 1000.0f);   // scale 0.001
        uint8_t payload[8];
        memcpy(&payload[0], &p.u32, 4);
        memcpy(&payload[4], &vel, 2);
        memcpy(&payload[6], &tq,  2);
        return send(node_id, CMD_SET_INPUT_POS, payload, 8);
    }

    /// SetVel: velocity [rad/s], optional torque‑FF
    bool setInputVel(uint8_t node_id, float vel, float torque_ff = 0.0f) {
        union { float f; uint32_t u32; } v{vel};
        union { float f; uint32_t u32; } tq{torque_ff};
        uint8_t payload[8];
        memcpy(&payload[0], &v.u32, 4);
        memcpy(&payload[4], &tq.u32, 4);
        return send(node_id, CMD_SET_INPUT_VEL, payload, 8);
    }

    /// SetTorque: direct torque [Nm] command
    bool setInputTorque(uint8_t node_id, float torque) {
        union { float f; uint32_t u32; } t{torque};
        return send(node_id, CMD_SET_INPUT_TORQUE,
                    reinterpret_cast<uint8_t*>(&t.u32), 4);
    }

    /// Request a new axis state (see enum AxisState in ODrive FW)
    bool setRequestedState(uint8_t node_id, uint32_t state) {
        uint8_t payload[4];
        memcpy(payload, &state, 4);
        return send(node_id, CMD_SET_AXIS_REQUESTED_STATE, payload, 4);
    }

    /// Controller + input modes (bytes 0 and 1)
    bool setControllerModes(uint8_t node_id, uint8_t control_mode, uint8_t input_mode) {
        uint8_t payload[8] = {control_mode, input_mode};
        return send(node_id, CMD_SET_CONTROLLER_MODES, payload, 2);
    }

    /// Velocity / current limits
    bool setLimits(uint8_t node_id, float vel_lim, float current_lim) {
        union { float f; uint32_t u32; } v{vel_lim}, c{current_lim};
        uint8_t payload[8];
        memcpy(&payload[0], &v.u32, 4);
        memcpy(&payload[4], &c.u32, 4);
        return send(node_id, CMD_SET_LIMITS, payload, 8);
    }

    bool clearErrors(uint8_t node_id)            { return send(node_id, CMD_CLEAR_ERRORS, nullptr, 0); }
    bool reboot(uint8_t node_id)                 { return send(node_id, CMD_REBOOT,        nullptr, 0); }
    bool estop(uint8_t node_id)                  { return send(node_id, CMD_ESTOP,         nullptr, 0); }

    /* ------------------------------------------------------------------------
     * Incoming frame helpers
     * ----------------------------------------------------------------------*/

    /// Decode an 8‑byte heartbeat frame into a Heartbeat structure.
    /// Returns false if the supplied message is not a heartbeat.
    static bool decodeHeartbeat(const CAN_message_t &msg, Heartbeat &out) {
        if ((msg.id & 0x1F) > 0x1F) return false;                   // bad node id
        const uint16_t cmd   = msg.id >> 5;
        if (cmd != CMD_HEARTBEAT)  return false;

        if (msg.len < 8) return false;
        out.axis_error      =  (static_cast<uint32_t>(msg.buf[0])      |
                                static_cast<uint32_t>(msg.buf[1])<<8  |
                                static_cast<uint32_t>(msg.buf[2])<<16 |
                                static_cast<uint32_t>(msg.buf[3])<<24);
        out.current_state   =   msg.buf[4];
        out.traj_done_flag  =   msg.buf[5] & 0x01;
        out.motor_error     =  (static_cast<uint16_t>(msg.buf[6])      |
                                static_cast<uint16_t>(msg.buf[7])<<8 );
        // Encoder / controller errs are only present in CAN‑FD frames;
        // leave zeros for CAN‑2.0 heartbeat.
        return true;
    }

private:
    /// Compose the 11‑bit arbitration ID
    static inline uint16_t makeID(uint8_t node_id, CommandID cmd) {
        return static_cast<uint16_t>((static_cast<uint16_t>(cmd) << 5) | (node_id & 0x1F));
    }

    BUS_T   &m_bus;
    uint32_t m_baud;
};

} // namespace ODriveCAN

#endif /* ODRIVE_36_INTERFACE_H */
