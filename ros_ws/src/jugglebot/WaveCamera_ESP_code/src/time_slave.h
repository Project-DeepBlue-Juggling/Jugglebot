#pragma once

// 0x7DD time slave (wave_camera_can_adapter.md s5.5). Wraps TimeSlaveEstimator
// (time_slave_math.h). The RX task stamps each frame on receipt; loop() feeds
// the stamped frame here, so the estimator and every reader live on core 1
// and no lock is needed.

#include "pin_io.h"

#include "can_link.h"

namespace time_slave {

void on_frame(const can_link::RxFrame& f);

bool    synced(int64_t now_us);
bool    anchored();
bool    wall_from_local(int64_t t_local_us, int64_t& wall_us);  // false before the first anchor
int64_t sync_age_us(int64_t now_us);                            // -1 if never
uint8_t sync_age_100ms(int64_t now_us);                         // heartbeat byte 5

int64_t  offset_us(int64_t now_us);  // predicted wall - local
int64_t  offset_spread_us();
int64_t  track_err_us();
uint32_t accepted();
uint32_t rejected();
uint32_t steps();

}  // namespace time_slave
