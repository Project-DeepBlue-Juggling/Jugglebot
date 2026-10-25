#pragma once

// TWAI (CAN) transport, wave_camera_can_adapter.md s5.1. 1 Mbit/s classic, NORMAL mode,
// accept-all filter, GPIO13 TX / GPIO34 RX.
//
// Two FreeRTOS tasks pinned to core 0 (loop() runs on core 1), RX at a
// higher priority than TX so the receive stamp is never delayed by TX work:
// - can_rx: blocks in twai_receive, stamps esp_timer_get_time() straight
//   after it returns, and forwards {frame, t_rx} to loop() through a queue.
//   No decoding or state lives in the task.
// - can_tx: the only reader of TWAI alerts. Sends one frame at a time,
//   single-shot (no automatic retransmission), and waits for TX_SUCCESS or
//   TX_FAILED (or CAN_TX_DONE_TIMEOUT_MS passes, in which case the parked
//   frame is aborted with a controller stop/start), so a node that is alone
//   on the bus (no ACK) counts a failure per frame instead of retransmitting
//   one frame forever. A frame may carry
//   retries (WAVE_REC_EVENT: 2, i.e. 3 attempts CAN_TX_RETRY_DELAY_MS apart,
//   the unsynced age rewritten before each). Bus-off: initiate recovery, re-arm every
//   CAN_BUS_OFF_RETRY_MS, restart the controller when it parks in STOPPED.
//
// Nothing here prints from a task: `events on` records go through a queue
// that loop() drains (console::out is not reentrant).

#include "pin_io.h"

namespace can_link {

struct RxFrame {
    int64_t  t_rx_us;  // esp_timer_get_time() right after twai_receive returned
    uint32_t id;
    uint8_t  dlc;
    bool     extd;
    bool     rtr;
    uint8_t  data[8];
};

bool begin();  // installs the driver and starts both tasks; false if TWAI failed

// Queue a frame for transmission (never blocks). age_from_us >= 0 asks the TX
// task to write (now - age_from_us) into bytes 0-4 right before each attempt
// (the unsynced WAVE_REC_EVENT age path). Returns false if the queue is full.
bool send(uint32_t id, const uint8_t data[8], uint8_t retries = 0, int64_t age_from_us = -1);

// loop() side: next received frame, if any.
bool pop_rx(RxFrame& out);

// loop() side: print queued `events on` records (CAN lines).
void poll_log();
void set_events(bool on);
bool events();

struct Stats {
    bool     driver_up;
    int      bus_state;  // twai_state_t, -1 if the driver is not installed
    uint32_t tx_err_counter;
    uint32_t rx_err_counter;
    uint32_t rx_frames;
    uint32_t rx_fwd_dropped;   // loop() queue full
    uint32_t rx_queue_full;    // TWAI driver RX queue overflowed (alert)
    uint32_t tx_requests;      // send() calls
    uint32_t tx_queue_full;    // send() refused
    uint32_t tx_attempts;      // twai_transmit calls
    uint32_t tx_ok;
    uint32_t tx_failed;        // TX_FAILED alert or no completion in time
    uint32_t tx_not_running;   // dropped because the controller was not running
    uint32_t tx_retries;
    uint32_t tx_timeouts;      // part of tx_failed: no completion in time (bus never idle)
    uint32_t tx_aborts;        // stuck frames cleared by a controller stop/start
    int      rx_pad_level;     // GPIO34 as read now (1 = recessive; floats 0 with no transceiver)
    uint32_t arb_lost;
    uint32_t bus_errors;
    uint32_t bus_off_events;
    uint32_t recoveries;       // controller restarted after bus-off
    uint32_t err_passive_events;
    uint32_t log_dropped;
};
Stats stats();
const char* bus_state_name(int s);

}  // namespace can_link
