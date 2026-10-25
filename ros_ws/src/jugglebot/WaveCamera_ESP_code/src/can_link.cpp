#include "can_link.h"

#include <driver/twai.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "console.h"
#include "wave_frames.h"

namespace {

struct TxItem {
    uint32_t id;
    uint8_t  data[8];
    uint8_t  retries;
    int64_t  age_from_us;
};

enum class LogDir : uint8_t { Rx, TxOk, TxFail, TxNotRunning };

struct LogItem {
    int64_t  t_us;
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    LogDir   dir;
    uint8_t  attempt;
};

QueueHandle_t s_rx_fwd = nullptr;
QueueHandle_t s_tx_req = nullptr;
QueueHandle_t s_log = nullptr;
TaskHandle_t  s_rx_task = nullptr;
TaskHandle_t  s_tx_task = nullptr;

volatile bool s_driver_up = false;
volatile bool s_running = false;  // controller started and not bus-off / recovering
volatile bool s_events = false;

// Single-writer counters (task or loop as noted); 32-bit loads are atomic.
volatile uint32_t s_rx_frames = 0;       // rx task
volatile uint32_t s_rx_fwd_dropped = 0;  // rx task
volatile uint32_t s_rx_queue_full = 0;   // tx task (alert)
volatile uint32_t s_tx_requests = 0;     // loop
volatile uint32_t s_tx_queue_full = 0;   // loop
volatile uint32_t s_tx_attempts = 0;     // tx task
volatile uint32_t s_tx_ok = 0;
volatile uint32_t s_tx_failed = 0;
volatile uint32_t s_tx_not_running = 0;
volatile uint32_t s_tx_retries = 0;
volatile uint32_t s_tx_timeouts = 0;  // no completion within CAN_TX_DONE_TIMEOUT_MS
volatile uint32_t s_tx_aborts = 0;    // stuck frame cleared by stop/start
volatile uint32_t s_arb_lost = 0;
volatile uint32_t s_bus_errors = 0;
volatile uint32_t s_bus_off_events = 0;
volatile uint32_t s_recoveries = 0;
volatile uint32_t s_err_passive = 0;
volatile uint32_t s_log_dropped = 0;     // rx + tx tasks; display only

int64_t s_next_recovery_us = 0;  // tx task only
bool    s_in_bus_off = false;    // tx task only

constexpr uint32_t ALERTS = TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF |
                            TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ERR_PASS | TWAI_ALERT_ARB_LOST |
                            TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;

void log_push(LogDir dir, uint32_t id, uint8_t dlc, const uint8_t* data, int64_t t_us, uint8_t attempt) {
    if (!s_events || !s_log) return;
    LogItem it;
    it.t_us = t_us;
    it.id = id;
    it.dlc = dlc > 8 ? 8 : dlc;
    memcpy(it.data, data, it.dlc);
    it.dir = dir;
    it.attempt = attempt;
    if (xQueueSend(s_log, &it, 0) != pdTRUE) s_log_dropped = s_log_dropped + 1;
}

void rx_task(void*) {
    for (;;) {
        twai_message_t msg;
        if (twai_receive(&msg, pdMS_TO_TICKS(CAN_RX_BLOCK_MS)) != ESP_OK) continue;
        can_link::RxFrame f;
        f.t_rx_us = esp_timer_get_time();  // first thing after receive returns
        f.id = msg.identifier;
        f.dlc = msg.data_length_code > 8 ? 8 : msg.data_length_code;
        f.extd = msg.extd;
        f.rtr = msg.rtr;
        memset(f.data, 0, sizeof(f.data));
        if (!f.rtr) memcpy(f.data, msg.data, f.dlc);
        s_rx_frames = s_rx_frames + 1;
        if (xQueueSend(s_rx_fwd, &f, 0) != pdTRUE) s_rx_fwd_dropped = s_rx_fwd_dropped + 1;
        log_push(LogDir::Rx, f.id, f.dlc, f.data, f.t_rx_us, 0);
    }
}

// Bus-state housekeeping from alerts plus a status read. Returns the alert
// bits it consumed so a TX wait can look for its completion.
void handle_alerts(uint32_t a) {
    if (a & TWAI_ALERT_ARB_LOST) s_arb_lost = s_arb_lost + 1;
    if (a & TWAI_ALERT_BUS_ERROR) s_bus_errors = s_bus_errors + 1;
    if (a & TWAI_ALERT_ERR_PASS) s_err_passive = s_err_passive + 1;
    if (a & TWAI_ALERT_RX_QUEUE_FULL) s_rx_queue_full = s_rx_queue_full + 1;
    if (a & TWAI_ALERT_BUS_OFF) {
        s_running = false;
        if (!s_in_bus_off) {
            s_in_bus_off = true;
            s_bus_off_events = s_bus_off_events + 1;
        }
        twai_initiate_recovery();
        s_next_recovery_us = esp_timer_get_time() + static_cast<int64_t>(CAN_BUS_OFF_RETRY_MS) * 1000;
    }
    if (a & TWAI_ALERT_BUS_RECOVERED) {
        // Recovery leaves the controller in STOPPED; it needs an explicit start.
        if (twai_start() == ESP_OK) {
            s_recoveries = s_recoveries + 1;
            s_in_bus_off = false;
            s_running = true;
        }
    }
}

void service_bus_state() {
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK) return;
    const int64_t now = esp_timer_get_time();
    switch (st.state) {
        case TWAI_STATE_BUS_OFF:
            s_running = false;
            if (!s_in_bus_off) {  // alert missed: count it here
                s_in_bus_off = true;
                s_bus_off_events = s_bus_off_events + 1;
                s_next_recovery_us = 0;
            }
            if (now >= s_next_recovery_us) {
                twai_initiate_recovery();
                s_next_recovery_us = now + static_cast<int64_t>(CAN_BUS_OFF_RETRY_MS) * 1000;
            }
            break;
        case TWAI_STATE_STOPPED:
            if (twai_start() == ESP_OK) {
                if (s_in_bus_off) s_recoveries = s_recoveries + 1;
                s_in_bus_off = false;
                s_running = true;
            }
            break;
        case TWAI_STATE_RECOVERING:
            s_running = false;
            break;
        case TWAI_STATE_RUNNING:
            s_running = true;
            s_in_bus_off = false;
            break;
    }
}

// A frame that never completes (no idle bus to start on, e.g. RX held
// dominant) would otherwise stay parked in the controller and go out late,
// stale, once the bus recovers. Stopping the controller aborts it and clears
// the driver queue; start re-arms it.
void abort_stuck_tx() {
    twai_status_info_t st;
    if (twai_get_status_info(&st) != ESP_OK || st.state != TWAI_STATE_RUNNING) return;
    if (st.msgs_to_tx == 0) return;
    if (twai_stop() == ESP_OK) {
        s_tx_aborts = s_tx_aborts + 1;
        s_running = twai_start() == ESP_OK;
    }
}

// One attempt: true on TX_SUCCESS.
bool transmit_once(TxItem& it, uint8_t attempt) {
    twai_message_t msg = {};
    msg.identifier = it.id;
    msg.data_length_code = CAN_DLC;
    msg.extd = 0;
    msg.rtr = 0;
    msg.ss = 1;  // single shot: a missing ACK fails this frame instead of retrying forever
    if (it.age_from_us >= 0) {
        const int64_t age = esp_timer_get_time() - it.age_from_us;
        wave::put_u40(it.data, wave::t40_from_age(age));
    }
    memcpy(msg.data, it.data, 8);

    uint32_t a = 0;
    while (twai_read_alerts(&a, 0) == ESP_OK) handle_alerts(a);  // drop stale completions
    s_tx_attempts = s_tx_attempts + 1;
    const int64_t t_tx = esp_timer_get_time();
    if (twai_transmit(&msg, 0) != ESP_OK) {
        s_tx_failed = s_tx_failed + 1;
        log_push(LogDir::TxFail, it.id, 8, it.data, t_tx, attempt);
        return false;
    }
    const int64_t deadline = t_tx + static_cast<int64_t>(CAN_TX_DONE_TIMEOUT_MS) * 1000;
    for (;;) {
        const int64_t left_us = deadline - esp_timer_get_time();
        if (left_us <= 0) break;
        TickType_t wait = pdMS_TO_TICKS((left_us + 999) / 1000);
        if (wait == 0) wait = 1;
        if (twai_read_alerts(&a, wait) != ESP_OK) continue;
        handle_alerts(a);
        if (a & TWAI_ALERT_TX_SUCCESS) {
            s_tx_ok = s_tx_ok + 1;
            log_push(LogDir::TxOk, it.id, 8, it.data, t_tx, attempt);
            return true;
        }
        if (a & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_OFF)) {
            s_tx_failed = s_tx_failed + 1;
            log_push(LogDir::TxFail, it.id, 8, it.data, t_tx, attempt);
            return false;
        }
    }
    s_tx_timeouts = s_tx_timeouts + 1;
    abort_stuck_tx();
    s_tx_failed = s_tx_failed + 1;
    log_push(LogDir::TxFail, it.id, 8, it.data, t_tx, attempt);
    return false;
}

void tx_task(void*) {
    for (;;) {
        uint32_t a = 0;
        while (twai_read_alerts(&a, 0) == ESP_OK) handle_alerts(a);
        service_bus_state();
        TxItem it;
        if (xQueueReceive(s_tx_req, &it, pdMS_TO_TICKS(CAN_STATUS_POLL_MS)) != pdTRUE) continue;
        for (uint8_t attempt = 0; attempt <= it.retries; ++attempt) {
            if (attempt) {
                // Single shot never retransmits after lost arbitration or a
                // bus-down moment; space the retries so they are real ones.
                vTaskDelay(pdMS_TO_TICKS(CAN_TX_RETRY_DELAY_MS));
                s_tx_retries = s_tx_retries + 1;
            }
            service_bus_state();
            if (!s_running) {
                s_tx_not_running = s_tx_not_running + 1;
                log_push(LogDir::TxNotRunning, it.id, 8, it.data, esp_timer_get_time(), attempt);
                continue;
            }
            if (transmit_once(it, attempt)) break;
        }
    }
}

const char* dir_name(LogDir d) {
    switch (d) {
        case LogDir::Rx: return "rx";
        case LogDir::TxOk: return "tx";
        case LogDir::TxFail: return "tx_fail";
        case LogDir::TxNotRunning: return "tx_bus_down";
    }
    return "?";
}

}  // namespace

namespace can_link {

bool begin() {
    s_rx_fwd = xQueueCreate(CAN_RX_FWD_QUEUE_LEN, sizeof(RxFrame));
    s_tx_req = xQueueCreate(CAN_TX_REQ_QUEUE_LEN, sizeof(TxItem));
    s_log = xQueueCreate(CAN_LOG_QUEUE_LEN, sizeof(LogItem));
    if (!s_rx_fwd || !s_tx_req || !s_log) return false;

    // NORMAL mode: we ACK and expect ACKs. NO_ACK / LISTEN_ONLY would make a
    // dead bus look healthy on the bench.
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(static_cast<gpio_num_t>(PIN_CAN_TX),
                                                          static_cast<gpio_num_t>(PIN_CAN_RX), TWAI_MODE_NORMAL);
    g.rx_queue_len = CAN_RX_QUEUE_LEN;
    g.tx_queue_len = CAN_TX_QUEUE_LEN;
    g.alerts_enabled = ALERTS;
    static_assert(CAN_BITRATE_BPS == 1000000, "timing preset below is 1 Mbit/s; change both together");
    twai_timing_config_t t = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g, &t, &f) != ESP_OK) return false;
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }
    s_driver_up = true;
    s_running = true;
    xTaskCreatePinnedToCore(rx_task, "can_rx", CAN_TASK_STACK_BYTES, nullptr, CAN_RX_TASK_PRIORITY, &s_rx_task,
                            CAN_TASK_CORE);
    xTaskCreatePinnedToCore(tx_task, "can_tx", CAN_TASK_STACK_BYTES, nullptr, CAN_TX_TASK_PRIORITY, &s_tx_task,
                            CAN_TASK_CORE);
    return s_rx_task && s_tx_task;
}

bool send(uint32_t id, const uint8_t data[8], uint8_t retries, int64_t age_from_us) {
    s_tx_requests = s_tx_requests + 1;
    if (!s_driver_up) {
        s_tx_queue_full = s_tx_queue_full + 1;
        return false;
    }
    TxItem it;
    it.id = id;
    memcpy(it.data, data, 8);
    it.retries = retries;
    it.age_from_us = age_from_us;
    if (xQueueSend(s_tx_req, &it, 0) != pdTRUE) {
        s_tx_queue_full = s_tx_queue_full + 1;
        return false;
    }
    return true;
}

bool pop_rx(RxFrame& out) { return s_rx_fwd && xQueueReceive(s_rx_fwd, &out, 0) == pdTRUE; }

void poll_log() {
    if (!s_log) return;
    LogItem it;
    int budget = 16;  // bound one loop() pass
    while (budget-- > 0 && xQueueReceive(s_log, &it, 0) == pdTRUE) {
        char hex[8 * 3 + 1];
        size_t w = 0;
        static const char HEX_DIGITS[] = "0123456789ABCDEF";
        for (uint8_t i = 0; i < it.dlc; ++i) {
            hex[w++] = HEX_DIGITS[it.data[i] >> 4];
            hex[w++] = HEX_DIGITS[it.data[i] & 0xF];
            hex[w++] = ' ';
        }
        hex[w ? w - 1 : 0] = '\0';
        console::out_at(it.t_us, "CAN", "%s id=0x%03lX dlc=%u attempt=%u | %s", dir_name(it.dir),
                        (unsigned long)it.id, it.dlc, it.attempt, hex);
    }
}

void set_events(bool on) {
    s_events = on;
    if (!on && s_log) xQueueReset(s_log);
}
bool events() { return s_events; }

Stats stats() {
    Stats s = {};
    s.driver_up = s_driver_up;
    s.bus_state = -1;
    if (s_driver_up) {
        twai_status_info_t st;
        if (twai_get_status_info(&st) == ESP_OK) {
            s.bus_state = static_cast<int>(st.state);
            s.tx_err_counter = st.tx_error_counter;
            s.rx_err_counter = st.rx_error_counter;
        }
    }
    s.rx_frames = s_rx_frames;
    s.rx_fwd_dropped = s_rx_fwd_dropped;
    s.rx_queue_full = s_rx_queue_full;
    s.tx_requests = s_tx_requests;
    s.tx_queue_full = s_tx_queue_full;
    s.tx_attempts = s_tx_attempts;
    s.tx_ok = s_tx_ok;
    s.tx_failed = s_tx_failed;
    s.tx_not_running = s_tx_not_running;
    s.tx_retries = s_tx_retries;
    s.tx_timeouts = s_tx_timeouts;
    s.tx_aborts = s_tx_aborts;
    s.rx_pad_level = pin_io::read_level(PIN_CAN_RX);
    s.arb_lost = s_arb_lost;
    s.bus_errors = s_bus_errors;
    s.bus_off_events = s_bus_off_events;
    s.recoveries = s_recoveries;
    s.err_passive_events = s_err_passive;
    s.log_dropped = s_log_dropped;
    return s;
}

const char* bus_state_name(int s) {
    switch (s) {
        case TWAI_STATE_STOPPED: return "stopped";
        case TWAI_STATE_RUNNING: return "running";
        case TWAI_STATE_BUS_OFF: return "bus_off";
        case TWAI_STATE_RECOVERING: return "recovering";
        default: return "not_installed";
    }
}

}  // namespace can_link
