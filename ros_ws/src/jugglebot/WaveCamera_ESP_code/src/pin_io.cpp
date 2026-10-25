#define PIN_IO_IMPL
#include "pin_io.h"

#include <soc/gpio_sig_map.h>
#include <soc/io_mux_reg.h>

static_assert(PIN_DRIVE_MODES == 2, "PinDrive must not grow a push-pull mode");

namespace {

uint64_t s_od_claimed = 0;  // bitmask of pins claimed open-drain

// GPI2 (GPIO26) is the only output pad; UART TX has no wired pad (config.h).
bool od_allowed(uint8_t gpio) { return is_output_allowed_pin(gpio); }

void set_out_level_high(uint8_t gpio) {
    if (gpio < 32) GPIO.out_w1ts = (1u << gpio);
    else GPIO.out1_w1ts.data = (1u << (gpio - 32));
}
void set_out_level_low(uint8_t gpio) {
    if (gpio < 32) GPIO.out_w1tc = (1u << gpio);
    else GPIO.out1_w1tc.data = (1u << (gpio - 32));
}
void output_enable_reg(uint8_t gpio) {
    if (gpio < 32) GPIO.enable_w1ts = (1u << gpio);
    else GPIO.enable1_w1ts.data = (1u << (gpio - 32));
}

}  // namespace

namespace pin_io {

bool configure(uint8_t gpio, PinDrive drive) {
    if (!is_capture_pin(gpio)) return false;
    if (drive == PinDrive::OpenDrain && !od_allowed(gpio)) drive = PinDrive::InputPullup;

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << gpio);
    cfg.pull_up_en = GPIO_PULLUP_ENABLE;  // bare-board bench reads clean highs
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    // Every capture pin keeps its edge interrupt: gpio_config re-applies it.
    cfg.intr_type = GPIO_INTR_ANYEDGE;

    if (drive == PinDrive::OpenDrain) {
        set_out_level_high(gpio);  // released before the output is enabled
        cfg.mode = GPIO_MODE_INPUT_OUTPUT_OD;
    } else {
        cfg.mode = GPIO_MODE_INPUT;
    }
    if (gpio_config(&cfg) != ESP_OK) return false;

    if (drive == PinDrive::OpenDrain) {
        if (!is_open_drain(gpio)) {  // never leave a push-pull output behind
            cfg.mode = GPIO_MODE_INPUT;
            gpio_config(&cfg);
            s_od_claimed &= ~(1ULL << gpio);
            return false;
        }
        s_od_claimed |= (1ULL << gpio);
    } else {
        s_od_claimed &= ~(1ULL << gpio);
    }
    return true;
}

bool od_drive_low(uint8_t gpio) {
    if (!(s_od_claimed & (1ULL << gpio)) || !is_open_drain(gpio)) return false;
    set_out_level_low(gpio);
    return true;
}

bool od_release(uint8_t gpio) {
    if (!(s_od_claimed & (1ULL << gpio))) return false;
    set_out_level_high(gpio);
    return true;
}

bool prepare_uart_tx(uint8_t gpio) {
    if (!is_uart_tx_candidate_pin(gpio)) return false;
    // Open-drain bit and a released GPIO level first; the UART driver only
    // changes the matrix signal and output-enable, never pad_driver.
    set_out_level_high(gpio);
    GPIO.pin[gpio].pad_driver = 1;
    s_od_claimed |= (1ULL << gpio);
    return is_open_drain(gpio);
}

bool finish_uart_tx(uint8_t gpio) {
    if (!is_uart_tx_candidate_pin(gpio)) return false;
    GPIO.pin[gpio].pad_driver = 1;
    gpio_pullup_en(static_cast<gpio_num_t>(gpio));
    gpio_pulldown_dis(static_cast<gpio_num_t>(gpio));
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[gpio]);  // keep the edge ISR fed
    // If RX shares the pad, uart_set_pin(rx) disabled the output; re-enable
    // via the register only (gpio_set_direction would reroute the matrix).
    if (out_signal(gpio) == U2TXD_OUT_IDX) output_enable_reg(gpio);
    return is_open_drain(gpio) && out_signal(gpio) == U2TXD_OUT_IDX;
}

void release_uart_tx(uint8_t gpio) {
    if (!is_uart_tx_candidate_pin(gpio)) return;
    // Released level in the GPIO out register so that, whatever the matrix
    // does during detach, an enabled open-drain pad floats high.
    set_out_level_high(gpio);
    GPIO.pin[gpio].pad_driver = 1;
}

bool uart_tx_hold_low(uint8_t gpio) {
    if (!is_uart_tx_candidate_pin(gpio) || !(s_od_claimed & (1ULL << gpio))) return false;
    if (!is_open_drain(gpio) || out_signal(gpio) != U2TXD_OUT_IDX) return false;
    // Pad stays open-drain throughout; level is low before the matrix switch
    // so the pad never glitches high between UART idle and the hold.
    set_out_level_low(gpio);
    GPIO.func_out_sel_cfg[gpio].oen_sel = 0;
    GPIO.func_out_sel_cfg[gpio].func_sel = SIG_GPIO_OUT_IDX;
    output_enable_reg(gpio);
    return is_open_drain(gpio) && out_signal(gpio) == SIG_GPIO_OUT_IDX && read_level(gpio) == 0;
}

bool uart_tx_hold_end(uint8_t gpio) {
    if (!is_uart_tx_candidate_pin(gpio)) return false;
    set_out_level_high(gpio);  // released (open-drain) while still plain GPIO
    GPIO.pin[gpio].pad_driver = 1;
    GPIO.func_out_sel_cfg[gpio].func_sel = U2TXD_OUT_IDX;  // UART idles high
    return finish_uart_tx(gpio);
}

void finish_uart_rx(uint8_t gpio) {
    if (!is_uart_rx_candidate_pin(gpio)) return;
    gpio_pullup_en(static_cast<gpio_num_t>(gpio));
    gpio_pulldown_dis(static_cast<gpio_num_t>(gpio));
    PIN_INPUT_ENABLE(GPIO_PIN_MUX_REG[gpio]);
    gpio_set_intr_type(static_cast<gpio_num_t>(gpio), GPIO_INTR_ANYEDGE);
    gpio_intr_enable(static_cast<gpio_num_t>(gpio));
}

int read_level(uint8_t gpio) {
    return gpio < 32 ? static_cast<int>((GPIO.in >> gpio) & 1u)
                     : static_cast<int>((GPIO.in1.data >> (gpio - 32)) & 1u);
}

bool is_open_drain(uint8_t gpio) { return GPIO.pin[gpio].pad_driver == 1; }

bool is_output_enabled(uint8_t gpio) {
    return gpio < 32 ? ((GPIO.enable >> gpio) & 1u) : ((GPIO.enable1.data >> (gpio - 32)) & 1u);
}

int out_signal(uint8_t gpio) { return static_cast<int>(GPIO.func_out_sel_cfg[gpio].func_sel); }

}  // namespace pin_io
