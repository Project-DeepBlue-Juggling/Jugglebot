#pragma once

// 0x7DD time-slave estimator (wave_camera_can_adapter.md s5.5): pure, header-only.
//
// Each valid beacon gives offset = wall_us - t_rx_local_us. The bridge stamps
// the beacon before writing it, so a deferred frame is always LATE (smaller
// offset), never early. The estimator therefore tracks the windowed MAXIMUM
// of the offset (the least-delayed frame) over SYNC_WINDOW_MS, fits a
// least-squares line through the last SYNC_HISTORY_WINDOWS window maxima to
// follow crystal skew, and predicts offset(t) = O_k + slope * (t - t_k).
//
// - First anchor: the maximum of the first SYNC_ANCHOR_FRAMES valid frames.
// - A single frame more than SYNC_EARLY_REJECT_US above the prediction is
//   rejected as implausible (counted, not used).
// - Step: a window maximum (over every well-formed frame, rejected or not)
//   more than SYNC_STEP_US below the prediction, or a window in which EVERY
//   frame was rejected as early, for SYNC_STEP_WINDOWS windows in a row ->
//   re-anchor on that window (history cleared, steps++).
// - No accepted beacon for SYNC_STALE_MS: the anchor and history are dropped
//   and the first-anchor path runs again (not counted as a step).
// - A window whose maximum sits more than SYNC_LATE_WINDOW_US below the
//   prediction (every frame in it was delayed) is not added to the line, so
//   delayed frames never pull the offset down. If that persists for
//   SYNC_LATE_REANCHOR_WINDOWS windows it is treated as a step.
//
// Time inputs are injected (int64 us), so this runs in the native tests.

#include <cstddef>
#include <cstdint>

#include "config.h"
#include "wave_frames.h"

class TimeSlaveEstimator {
public:
    enum class Result : uint8_t { Accepted, Anchored, RejectedFormat, RejectedEarly, Stepped };

    // Full frame path: validation (!extd, !rtr, DLC 8, usec < 1e6, sec after
    // 2026-01-01) then on_beacon().
    Result on_frame(const uint8_t* data, uint8_t dlc, bool extd, bool rtr, int64_t t_rx_local_us) {
        wave::TimeSync ts;
        if (!wave::decode_time_sync(data, dlc, extd, rtr, ts)) {
            ++rejected_;
            ++rejected_format_;
            return Result::RejectedFormat;
        }
        return on_beacon(wave::time_sync_wall_us(ts), t_rx_local_us);
    }

    Result on_beacon(int64_t wall_us, int64_t t_local_us) {
        const int64_t off = wall_us - t_local_us;
        if (anchored_ && t_local_us - last_accept_us_ >= static_cast<int64_t>(SYNC_STALE_MS) * 1000) {
            // Beacon silent past SYNC_STALE_MS (bridge anchor stale, cable
            // out): the old line says nothing about the new one, so start
            // over from the first-anchor path. Not a step.
            unanchor();
            ++stale_restarts_;
        }
        if (!anchored_) {
            win_add_raw(off, t_local_us);
            win_add_acc(off, t_local_us);
            accept(t_local_us);
            if (win_acc_n_ >= SYNC_ANCHOR_FRAMES) {
                anchor(win_acc_max_, win_acc_t_);
                spread_us_ = win_acc_max_ - win_acc_min_;
                win_reset();
                return Result::Anchored;
            }
            return Result::Accepted;
        }
        stepped_ = false;
        if (win_raw_n_ && t_local_us - win_start_us_ >= static_cast<int64_t>(SYNC_WINDOW_MS) * 1000) close_window();
        if (!win_raw_n_) win_start_us_ = t_local_us;
        win_add_raw(off, t_local_us);
        if (off > predict_offset(t_local_us) + static_cast<int64_t>(SYNC_EARLY_REJECT_US)) {
            ++rejected_;
            ++rejected_early_;
            return stepped_ ? Result::Stepped : Result::RejectedEarly;
        }
        win_add_acc(off, t_local_us);
        accept(t_local_us);
        return stepped_ ? Result::Stepped : Result::Accepted;
    }

    bool anchored() const { return anchored_; }
    bool ever_accepted() const { return ever_; }

    // Predicted offset (wall - local) at local time t. 0 before anchoring.
    int64_t predict_offset(int64_t t_local_us) const {
        if (hist_n_ == 0) return 0;
        const Point& ref = hist(hist_n_ - 1);
        if (hist_n_ == 1) return ref.off;
        double mx = 0, my = 0;
        for (size_t i = 0; i < hist_n_; ++i) {
            mx += static_cast<double>(hist(i).t - ref.t);
            my += static_cast<double>(hist(i).off - ref.off);
        }
        mx /= hist_n_;
        my /= hist_n_;
        double sxx = 0, sxy = 0;
        for (size_t i = 0; i < hist_n_; ++i) {
            const double dx = static_cast<double>(hist(i).t - ref.t) - mx;
            const double dy = static_cast<double>(hist(i).off - ref.off) - my;
            sxx += dx * dx;
            sxy += dx * dy;
        }
        double slope = sxx > 0 ? sxy / sxx : 0.0;
        const double lim = SYNC_MAX_SLOPE_PPM * 1e-6;
        if (slope > lim) slope = lim;
        if (slope < -lim) slope = -lim;
        const double y = my + slope * (static_cast<double>(t_local_us - ref.t) - mx);
        return ref.off + static_cast<int64_t>(y >= 0 ? y + 0.5 : y - 0.5);
    }

    // Wall time for a local stamp. Returns false (out untouched) before the
    // first anchor.
    bool wall_from_local(int64_t t_local_us, int64_t& wall_us) const {
        if (!anchored_) return false;
        wall_us = t_local_us + predict_offset(t_local_us);
        return true;
    }

    // Age of the last accepted beacon; -1 if none ever.
    int64_t sync_age_us(int64_t now_us) const { return ever_ ? now_us - last_accept_us_ : -1; }
    bool    synced(int64_t now_us) const {
        return anchored_ && ever_ && now_us - last_accept_us_ < static_cast<int64_t>(SYNC_STALE_MS) * 1000;
    }

    uint32_t accepted() const { return accepted_; }
    uint32_t rejected() const { return rejected_; }
    uint32_t rejected_format() const { return rejected_format_; }
    uint32_t rejected_early() const { return rejected_early_; }
    uint32_t steps() const { return steps_; }
    uint32_t windows() const { return windows_; }
    uint32_t stale_restarts() const { return stale_restarts_; }
    int64_t  offset_spread_us() const { return spread_us_; }  // last closed window, max - min accepted
    int64_t  track_err_us() const { return track_err_us_; }   // |last window max - prediction|

private:
    struct Point {
        int64_t t;
        int64_t off;
    };

    const Point& hist(size_t i) const { return hist_[(hist_head_ + SYNC_HISTORY_WINDOWS - hist_n_ + i) % SYNC_HISTORY_WINDOWS]; }
    void push_point(int64_t off, int64_t t) {
        hist_[hist_head_] = Point{t, off};
        hist_head_ = (hist_head_ + 1) % SYNC_HISTORY_WINDOWS;
        if (hist_n_ < SYNC_HISTORY_WINDOWS) ++hist_n_;
    }
    void anchor(int64_t off, int64_t t) {
        hist_n_ = 0;
        hist_head_ = 0;
        push_point(off, t);
        anchored_ = true;
        step_streak_ = 0;
        late_streak_ = 0;
    }
    void unanchor() {
        anchored_ = false;
        hist_n_ = 0;
        hist_head_ = 0;
        step_streak_ = 0;
        late_streak_ = 0;
        win_reset();
    }
    void accept(int64_t t) {
        ++accepted_;
        last_accept_us_ = t;
        ever_ = true;
    }
    void win_reset() {
        win_raw_n_ = win_acc_n_ = 0;
    }
    void win_add_raw(int64_t off, int64_t t) {
        if (!win_raw_n_ || off > win_raw_max_) {
            win_raw_max_ = off;
            win_raw_t_ = t;
        }
        ++win_raw_n_;
    }
    void win_add_acc(int64_t off, int64_t t) {
        if (!win_acc_n_ || off > win_acc_max_) {
            win_acc_max_ = off;
            win_acc_t_ = t;
        }
        if (!win_acc_n_ || off < win_acc_min_) win_acc_min_ = off;
        ++win_acc_n_;
    }
    void reanchor(int64_t off, int64_t t) {
        anchor(off, t);
        ++steps_;
        stepped_ = true;
        last_accept_us_ = t;
    }
    void close_window() {
        ++windows_;
        const int64_t dev_raw = win_raw_max_ - predict_offset(win_raw_t_);
        spread_us_ = win_acc_n_ ? win_acc_max_ - win_acc_min_ : 0;
        // Upward only counts when every frame in the window was rejected as
        // early: a window that merely contains one early outlier is not a step.
        const bool deviant = (dev_raw > static_cast<int64_t>(SYNC_EARLY_REJECT_US) && win_acc_n_ == 0) ||
                             dev_raw < -static_cast<int64_t>(SYNC_STEP_US);
        if (deviant) {
            track_err_us_ = dev_raw < 0 ? -dev_raw : dev_raw;
            late_streak_ = 0;
            if (++step_streak_ >= SYNC_STEP_WINDOWS) reanchor(win_raw_max_, win_raw_t_);
            win_reset();
            return;
        }
        step_streak_ = 0;
        if (win_acc_n_) {
            const int64_t dev = win_acc_max_ - predict_offset(win_acc_t_);
            track_err_us_ = dev < 0 ? -dev : dev;
            if (dev < -static_cast<int64_t>(SYNC_LATE_WINDOW_US)) {
                if (++late_streak_ >= SYNC_LATE_REANCHOR_WINDOWS) reanchor(win_acc_max_, win_acc_t_);
            } else {
                late_streak_ = 0;
                push_point(win_acc_max_, win_acc_t_);
            }
        }
        win_reset();
    }

    bool     anchored_ = false;
    bool     ever_ = false;
    bool     stepped_ = false;
    int64_t  last_accept_us_ = 0;
    Point    hist_[SYNC_HISTORY_WINDOWS] = {};
    size_t   hist_head_ = 0;
    size_t   hist_n_ = 0;
    int64_t  win_start_us_ = 0;
    uint32_t win_raw_n_ = 0;
    int64_t  win_raw_max_ = 0;
    int64_t  win_raw_t_ = 0;
    uint32_t win_acc_n_ = 0;
    int64_t  win_acc_max_ = 0;
    int64_t  win_acc_min_ = 0;
    int64_t  win_acc_t_ = 0;
    uint32_t step_streak_ = 0;
    uint32_t late_streak_ = 0;
    uint32_t accepted_ = 0;
    uint32_t rejected_ = 0;
    uint32_t rejected_format_ = 0;
    uint32_t rejected_early_ = 0;
    uint32_t steps_ = 0;
    uint32_t windows_ = 0;
    uint32_t stale_restarts_ = 0;
    int64_t  spread_us_ = 0;
    int64_t  track_err_us_ = 0;
};
