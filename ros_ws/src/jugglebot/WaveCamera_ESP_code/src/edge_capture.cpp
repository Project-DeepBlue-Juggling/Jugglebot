#include "edge_capture.h"

namespace {

edge_capture::EdgeRingT s_ring;  // .bss -> DRAM, reachable from the IRAM ISR

void IRAM_ATTR edge_isr(void* arg) {
    const uint32_t gpio = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(arg));
    const uint32_t level = gpio < 32 ? (GPIO.in >> gpio) & 1u : (GPIO.in1.data >> (gpio - 32)) & 1u;
    EdgeEvent ev;
    ev.t_us = esp_timer_get_time();
    ev.gpio = static_cast<uint8_t>(gpio);
    ev.level = static_cast<uint8_t>(level);
    s_ring.push(ev);
}

}  // namespace

namespace edge_capture {

bool begin() {
    bool ok = true;
    // Level-3 IRAM dispatcher: runs ahead of the UART driver's level-1 ISR
    // and keeps working while flash cache is disabled.
    const esp_err_t svc = gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    if (svc != ESP_OK && svc != ESP_ERR_INVALID_STATE) return false;
    for (uint8_t gpio : CAPTURE_PINS) {
        const PinDrive drive = (gpio == PIN_GPI2) ? PinDrive::OpenDrain : PinDrive::InputPullup;
        ok &= pin_io::configure(gpio, drive);
        ok &= gpio_isr_handler_add(static_cast<gpio_num_t>(gpio), edge_isr,
                                   reinterpret_cast<void*>(static_cast<uintptr_t>(gpio))) == ESP_OK;
    }
    return ok;
}

size_t drain(void (*handler)(const EdgeEvent&), size_t max_events) {
    size_t n = 0;
    EdgeEvent ev;
    while (n < max_events && s_ring.pop(ev)) {
        handler(ev);
        ++n;
    }
    return n;
}

uint32_t overflows() { return s_ring.overflows(); }
uint32_t high_water() { return s_ring.high_water(); }
uint32_t pending() { return s_ring.size(); }

}  // namespace edge_capture
