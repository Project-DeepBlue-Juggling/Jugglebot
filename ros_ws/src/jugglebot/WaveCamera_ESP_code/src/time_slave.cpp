#include "time_slave.h"

#include "console.h"
#include "time_slave_math.h"

namespace {

TimeSlaveEstimator s_est;

}  // namespace

namespace time_slave {

void on_frame(const can_link::RxFrame& f) {
    using R = TimeSlaveEstimator::Result;
    const R r = s_est.on_frame(f.data, f.dlc, f.extd, f.rtr, f.t_rx_us);
    if (r == R::Anchored) {
        console::out("INFO", "time sync anchored: offset_us=%lld", (long long)s_est.predict_offset(f.t_rx_us));
    } else if (r == R::Stepped) {
        console::out("WARN", "time sync step: re-anchored, offset_us=%lld steps=%lu",
                     (long long)s_est.predict_offset(f.t_rx_us), (unsigned long)s_est.steps());
    }
}

bool    synced(int64_t now_us) { return s_est.synced(now_us); }
bool    anchored() { return s_est.anchored(); }
bool    wall_from_local(int64_t t, int64_t& wall) { return s_est.wall_from_local(t, wall); }
int64_t sync_age_us(int64_t now_us) { return s_est.sync_age_us(now_us); }
uint8_t sync_age_100ms(int64_t now_us) {
    return wave::sync_age_100ms(s_est.ever_accepted(), s_est.sync_age_us(now_us));
}
int64_t  offset_us(int64_t now_us) { return s_est.predict_offset(now_us); }
int64_t  offset_spread_us() { return s_est.offset_spread_us(); }
int64_t  track_err_us() { return s_est.track_err_us(); }
uint32_t accepted() { return s_est.accepted(); }
uint32_t rejected() { return s_est.rejected(); }
uint32_t steps() { return s_est.steps(); }

}  // namespace time_slave
