#pragma once

// Lock-free single-producer / single-consumer ring of edge events.
//
// Producer: the GPIO ISR (all capture pins share one ISR dispatcher on one
// core, so pushes are serialised). Consumer: loop(). Header-only and
// Arduino-free so it runs in the native tests.
//
// When full, the NEW event is dropped and `overflows` is incremented; the
// consumer can report the count. Indices are free-running uint32 counters
// masked on access, so head - tail is the fill level even across wrap.

#include <atomic>
#include <cstddef>
#include <cstdint>

// push() must be inlined into the IRAM ISR (a template member could otherwise
// be emitted into flash).
#ifndef RING_ALWAYS_INLINE
#define RING_ALWAYS_INLINE inline __attribute__((always_inline))
#endif

struct EdgeEvent {
    int64_t t_us;   // esp_timer_get_time() taken inside the ISR
    uint8_t gpio;
    uint8_t level;  // pad level read in the ISR (0/1)
};

template <size_t CAPACITY>
class EventRing {
    static_assert(CAPACITY >= 2 && (CAPACITY & (CAPACITY - 1)) == 0, "capacity must be a power of two");

public:
    // ISR side. Returns false (and counts) if the ring is full.
    RING_ALWAYS_INLINE bool push(const EdgeEvent& ev) {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        const uint32_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail >= CAPACITY) {
            overflows_.store(overflows_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
            return false;
        }
        buf_[head & (CAPACITY - 1)] = ev;
        head_.store(head + 1, std::memory_order_release);
        const uint32_t fill = head + 1 - tail;
        if (fill > high_water_.load(std::memory_order_relaxed)) high_water_.store(fill, std::memory_order_relaxed);
        return true;
    }

    // Consumer side.
    bool pop(EdgeEvent& out) {
        const uint32_t tail = tail_.load(std::memory_order_relaxed);
        const uint32_t head = head_.load(std::memory_order_acquire);
        if (head == tail) return false;
        out = buf_[tail & (CAPACITY - 1)];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    uint32_t size() const {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }
    uint32_t overflows() const { return overflows_.load(std::memory_order_relaxed); }
    uint32_t high_water() const { return high_water_.load(std::memory_order_relaxed); }
    static constexpr size_t capacity() { return CAPACITY; }

private:
    EdgeEvent buf_[CAPACITY] = {};
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
    std::atomic<uint32_t> overflows_{0};
    std::atomic<uint32_t> high_water_{0};
};
