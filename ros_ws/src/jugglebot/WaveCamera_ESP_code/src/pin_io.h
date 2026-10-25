#pragma once

// The single place where pin direction is set (wave_camera_gpio_explorer.md s5).
//
// There is deliberately no push-pull drive mode: PinDrive has exactly two
// members and both are safe towards the camera. GPIO26 (GPI2, Wave pin 6) is
// the only pad ever made an open-drain output; every other capture pin
// (GPIO32 = Wave pin 3, GPIO27 = Wave pin 4) is an input with the internal
// pull-up. The UART TX helpers below stay compiled, but no pad passes
// is_uart_tx_candidate_pin() while UART_TX_WIRED is false (Pin 5 unwired).
//
// Compile-time guard: every translation unit that includes this header
// (all firmware .cpp files do, before anything else) has pinMode,
// gpio_set_direction and gpio_config poisoned, so any new code that tries
// to set a direction outside pin_io.cpp fails to build.

#include <Arduino.h>
#include <HardwareSerial.h>
#include <driver/gpio.h>
#include <driver/uart.h>
#include <esp_timer.h>
#include <soc/gpio_struct.h>

#include "config.h"

enum class PinDrive : uint8_t {
    InputPullup,  // input, internal pull-up, edge interrupt kept enabled
    OpenDrain,    // input + open-drain output, released (high-Z) on claim
};
constexpr int PIN_DRIVE_MODES = 2;  // must match the enum: no push-pull member

namespace pin_io {

// Set direction/pulls for a capture pin. Returns false (and leaves the pin a
// plain input) if the request is not allowed: non-capture pin, or OpenDrain
// on any pin other than GPI2 (config.h is_output_allowed_pin).
bool configure(uint8_t gpio, PinDrive drive);

// Drive / release an open-drain pin. Refused (returns false) unless the pin
// was claimed with configure(gpio, PinDrive::OpenDrain).
bool od_drive_low(uint8_t gpio);
bool od_release(uint8_t gpio);

// UART2 TX on a TX candidate pin (none while UART_TX_WIRED is false). Call
// prepare_uart_tx() before Serial2.begin() so the pad is already open-drain
// when the UART connects its output, then finish_uart_tx() after begin to re-assert open-drain + pull-up and verify.
// release_uart_tx() before Serial2.end() / re-pinning.
bool prepare_uart_tx(uint8_t gpio);
bool finish_uart_tx(uint8_t gpio);
void release_uart_tx(uint8_t gpio);

// Bench hold: detach the U2TXD signal from a UART-TX pad (matrix back to plain
// GPIO output) and drive the open-drain pad low. Only valid on a pad that is
// currently routed to U2TXD and open-drain. uart_tx_hold_end() releases the
// level first, then re-routes U2TXD and re-asserts open-drain + pull-up.
bool uart_tx_hold_low(uint8_t gpio);
bool uart_tx_hold_end(uint8_t gpio);

// After the UART driver configured `gpio` as its RX pad: restore the pull-up
// and make sure the pad input stays enabled so the edge ISR still sees it.
void finish_uart_rx(uint8_t gpio);

// Pad state helpers (register reads; safe anywhere).
int  read_level(uint8_t gpio);
bool is_open_drain(uint8_t gpio);
bool is_output_enabled(uint8_t gpio);
int  out_signal(uint8_t gpio);  // GPIO matrix output signal index (256 = plain GPIO)

}  // namespace pin_io

#ifndef PIN_IO_IMPL
#pragma GCC poison pinMode gpio_set_direction gpio_config
#endif
