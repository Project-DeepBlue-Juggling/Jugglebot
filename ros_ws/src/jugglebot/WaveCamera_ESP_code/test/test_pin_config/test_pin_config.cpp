// Pin table and compile-time pin checks (include/config.h), plus a source
// lint for the 2026-10-23 repin: GPI2 on GPIO26 is the only output pad,
// Wave pin 3 is on GPIO32 (input only), Wave pin 4 on GPIO27, Wave pin 5 and
// GPIO25 unused, and every transmit command refuses while TX is unwired.

#include <unity.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "config.h"

namespace fs = std::filesystem;

void setUp() {}
void tearDown() {}

static void test_pin_constants() {
    TEST_ASSERT_EQUAL_UINT8(26, PIN_GPI2);
    TEST_ASSERT_EQUAL_UINT8(32, PIN_CAM_P3);
    TEST_ASSERT_EQUAL_UINT8(27, PIN_CAM_P4);
    TEST_ASSERT_EQUAL_UINT8(13, PIN_CAN_TX);
    TEST_ASSERT_EQUAL_UINT8(34, PIN_CAN_RX);
    TEST_ASSERT_EQUAL(3, (int)NUM_CAPTURE_PINS);
    TEST_ASSERT_FALSE(is_capture_pin(25));  // old GPI2 pad, now unused
}

static void test_wave_pin_map() {
    TEST_ASSERT_EQUAL_INT(26, gpio_for_wave_pin(6));
    TEST_ASSERT_EQUAL_INT(32, gpio_for_wave_pin(3));
    TEST_ASSERT_EQUAL_INT(27, gpio_for_wave_pin(4));
    TEST_ASSERT_EQUAL_INT(-1, gpio_for_wave_pin(5));
    TEST_ASSERT_EQUAL_INT(6, wave_pin_for_gpio(26));
    TEST_ASSERT_EQUAL_INT(3, wave_pin_for_gpio(32));
    TEST_ASSERT_EQUAL_INT(4, wave_pin_for_gpio(27));
    TEST_ASSERT_EQUAL_INT(-1, wave_pin_for_gpio(25));
    for (size_t i = 0; i < NUM_CAPTURE_PINS; ++i) {
        TEST_ASSERT_EQUAL_INT((int)i, capture_index(CAPTURE_PINS[i]));
        TEST_ASSERT_EQUAL_INT(CAPTURE_PINS[i], gpio_for_wave_pin(CAPTURE_WAVE_PINS[i]));
        TEST_ASSERT_NOT_EQUAL(5, CAPTURE_WAVE_PINS[i]);
    }
}

static void test_output_and_uart_roles() {
    for (int g = 0; g < 256; ++g) {
        const uint8_t p = static_cast<uint8_t>(g);
        TEST_ASSERT_EQUAL(p == 26, is_output_allowed_pin(p));  // GPI2 only
        TEST_ASSERT_FALSE(is_uart_tx_candidate_pin(p));        // Pin 5 unwired
        TEST_ASSERT_EQUAL(p == 32 || p == 27, is_uart_rx_candidate_pin(p));
    }
    TEST_ASSERT_FALSE(UART_TX_WIRED);
    TEST_ASSERT_EQUAL_UINT8(32, UART_DEFAULT_RX_PIN);
}

static void test_esp32_pin_checks() {
    for (uint8_t g : {0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 15, 20, 24, 28, 34, 39, 40})
        TEST_ASSERT_FALSE_MESSAGE(is_safe_esp32_gpio(g), "strap/flash/UART0/input-only/missing pad accepted");
    for (uint8_t g : {4, 13, 14, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33})
        TEST_ASSERT_TRUE(is_safe_esp32_gpio(g));
    TEST_ASSERT_TRUE(is_output_capable_gpio(PIN_GPI2));
    TEST_ASSERT_FALSE(is_output_capable_gpio(34));
    TEST_ASSERT_TRUE(is_ok_can_pin(PIN_CAN_TX, false));
    TEST_ASSERT_TRUE(is_ok_can_pin(PIN_CAN_RX, true));
    TEST_ASSERT_FALSE(is_ok_can_pin(PIN_CAN_RX, false));  // input-only pad cannot be TX
    for (uint8_t g : CAPTURE_PINS) TEST_ASSERT_FALSE(is_ok_can_pin(g, true));
    TEST_ASSERT_TRUE(capture_pins_distinct());
    TEST_ASSERT_TRUE(capture_pins_safe());
}

static fs::path repo_dir() {
    fs::path p = fs::path(__FILE__).parent_path().parent_path().parent_path();
    if (!fs::exists(p / "src")) p = fs::current_path();
    return p;
}

static std::string slurp(const fs::path& p) {
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static void test_no_old_pin_numbers_in_firmware() {
    for (const char* dir : {"src", "include"}) {
        const fs::path d = repo_dir() / dir;
        TEST_ASSERT_TRUE_MESSAGE(fs::exists(d), dir);
        for (const auto& e : fs::directory_iterator(d)) {
            const std::string text = slurp(e.path());
            const std::string name = e.path().filename().string();
            const bool ok = text.find("GPIO25 ->") != std::string::npos ||  // config.h history note only
                            text.find("GPIO25") == std::string::npos;
            TEST_ASSERT_TRUE_MESSAGE(ok, (name + " mentions GPIO25 (old GPI2)").c_str());
            TEST_ASSERT_TRUE_MESSAGE(text.find("26|27|32") == std::string::npos,
                                     (name + " still offers the old uart pin list").c_str());
        }
    }
}

static void test_transmit_commands_refuse_first() {
    const std::string c = slurp(repo_dir() / "src" / "console.cpp");
    // Each transmit entry point checks require_tx_wired() before arm or drive.
    for (const char* fn : {"void cmd_send_text(", "void cmd_tx(", "void cmd_qx("}) {
        const size_t at = c.find(fn);
        TEST_ASSERT_TRUE_MESSAGE(at != std::string::npos, fn);
        const size_t gate = c.find("require_tx_wired()", at);
        TEST_ASSERT_TRUE_MESSAGE(gate < c.find("require_armed()", at) || gate < c.find("s_armed", at), fn);
        TEST_ASSERT_TRUE_MESSAGE(gate < c.find("uart_link::", at), fn);
    }
    const size_t utx = c.find("eq(t[1], \"tx\")");
    TEST_ASSERT_TRUE(utx != std::string::npos);
    TEST_ASSERT_TRUE(c.find("require_tx_wired()", utx) < c.find("uart_link::set_tx", utx));
    TEST_ASSERT_TRUE(c.find("no tx pin wired (Pin 5 not connected)") != std::string::npos);
    const std::string u = slurp(repo_dir() / "src" / "uart_link.cpp");
    TEST_ASSERT_TRUE(u.find("no tx pin wired (Pin 5 not connected)") != std::string::npos);
    const std::string io = slurp(repo_dir() / "src" / "pin_io.cpp");
    TEST_ASSERT_TRUE(io.find("return is_output_allowed_pin(gpio);") != std::string::npos);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_pin_constants);
    RUN_TEST(test_wave_pin_map);
    RUN_TEST(test_output_and_uart_roles);
    RUN_TEST(test_esp32_pin_checks);
    RUN_TEST(test_no_old_pin_numbers_in_firmware);
    RUN_TEST(test_transmit_commands_refuse_first);
    return UNITY_END();
}
