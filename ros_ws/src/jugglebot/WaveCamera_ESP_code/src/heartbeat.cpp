#include "heartbeat.h"

#include "can_link.h"
#include "record_ctrl.h"
#include "time_slave.h"
#include "wave_frames.h"

namespace {

int64_t  s_next_hb_us = 0;
int64_t  s_next_info_us = 0;
uint8_t  s_page = 0;
uint32_t s_hb_queued = 0;
uint32_t s_hb_refused = 0;
uint32_t s_info_queued = 0;
uint32_t s_slips = 0;

// Per-second snapshots.
int64_t  s_next_rate_us = 0;
uint32_t s_acc_prev = 0, s_rej_prev = 0, s_int_prev = 0, s_miss_prev = 0;
uint8_t  s_acc_ps = 0, s_rej_ps = 0, s_int_ps = 0, s_miss_ps = 0;

// Advance a drift-free schedule; returns true if a slot is due.
bool due(int64_t now, int64_t& next, int64_t period) {
    if (now < next) return false;
    next += period;
    if (now - next > static_cast<int64_t>(SCHEDULE_RESYNC_PERIODS) * period) {
        next = now + period;
        ++s_slips;
    }
    return true;
}

void send_heartbeat(int64_t now) {
    uint8_t d[8];
    wave::encode_heartbeat(record_ctrl::heartbeat(now), d);
    if (can_link::send(wave::ID_HEARTBEAT, d)) ++s_hb_queued;
    else ++s_hb_refused;
}

void send_info(int64_t now) {
    uint8_t d[8];
    switch (s_page) {
        case wave::INFO_PAGE_IDENTITY:
            wave::encode_info_identity(FW_VERSION, WIRE_VERSION, wave::sat_u32(now / 1000000), d);
            break;
        case wave::INFO_PAGE_TICK:
            wave::encode_info_tick(record_ctrl::tick_period_ns(), wave::sat_u16(record_ctrl::tick_jitter_us()),
                                   s_miss_ps, d);
            break;
        case wave::INFO_PAGE_SYNC:
            wave::encode_info_sync(wave::sat_u16(time_slave::offset_spread_us()),
                                   wave::sat_u16(time_slave::track_err_us()), s_acc_ps, s_rej_ps,
                                   wave::sat_u8(time_slave::steps()), d);
            break;
        default: {
            const can_link::Stats st = can_link::stats();
            const int64_t clip = record_ctrl::last_tick_to_clip_us();
            wave::encode_info_link(wave::sat_u16(clip < 0 ? 0 : clip),
                                   wave::sat_u16(static_cast<int64_t>(st.tx_failed) + st.tx_not_running + st.tx_queue_full),
                                   wave::sat_u8(st.bus_off_events), s_int_ps, d);
            break;
        }
    }
    if (can_link::send(wave::ID_INFO, d)) ++s_info_queued;
    s_page = static_cast<uint8_t>((s_page + 1) % INFO_NUM_PAGES);
}

void update_rates(int64_t now) {
    if (!due(now, s_next_rate_us, static_cast<int64_t>(RATE_WINDOW_MS) * 1000)) return;
    const uint32_t acc = time_slave::accepted(), rej = time_slave::rejected(), in = record_ctrl::intents_rx(),
                   miss = record_ctrl::ticks_missed_total();
    s_acc_ps = wave::sat_u8(static_cast<int64_t>(acc - s_acc_prev));
    s_rej_ps = wave::sat_u8(static_cast<int64_t>(rej - s_rej_prev));
    s_int_ps = wave::sat_u8(static_cast<int64_t>(in - s_int_prev));
    s_miss_ps = wave::sat_u8(static_cast<int64_t>(miss - s_miss_prev));
    s_acc_prev = acc;
    s_rej_prev = rej;
    s_int_prev = in;
    s_miss_prev = miss;
}

}  // namespace

namespace heartbeat {

void begin(int64_t now) {
    send_heartbeat(now);  // state BOOT: loop() has not run yet
    s_next_hb_us = now + HEARTBEAT_PERIOD_US;
    // INFO slots sit half a heartbeat period off the heartbeats.
    s_next_info_us = now + HEARTBEAT_PERIOD_US / 2;
    s_next_rate_us = now + static_cast<int64_t>(RATE_WINDOW_MS) * 1000;
}

void poll(int64_t now) {
    update_rates(now);
    if (due(now, s_next_hb_us, HEARTBEAT_PERIOD_US)) send_heartbeat(now);
    if (due(now, s_next_info_us, static_cast<int64_t>(INFO_PAGE_PERIOD_MS) * 1000)) send_info(now);
}

uint32_t heartbeats_queued() { return s_hb_queued; }
uint32_t heartbeats_refused() { return s_hb_refused; }
uint32_t info_queued() { return s_info_queued; }
uint32_t slips() { return s_slips; }
uint8_t  accepted_per_s() { return s_acc_ps; }
uint8_t  rejected_per_s() { return s_rej_ps; }
uint8_t  intent_rx_per_s() { return s_int_ps; }
uint8_t  ticks_missed_last_s() { return s_miss_ps; }

}  // namespace heartbeat
