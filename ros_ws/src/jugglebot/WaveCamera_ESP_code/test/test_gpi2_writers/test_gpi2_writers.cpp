// Single-writer lint for GPIO26 (wave_camera_can_adapter.md s5.4): only record_ctrl,
// latency_test and the console's ownership-gated `rec` command may call
// gpi2_drive::request_*. Greps every file under src/.

#include <unity.h>

#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>

namespace fs = std::filesystem;

void setUp() {}
void tearDown() {}

static fs::path src_dir() {
    fs::path p = fs::path(__FILE__).parent_path().parent_path().parent_path() / "src";
    if (!fs::exists(p)) p = fs::current_path() / "src";
    return p;
}

static void test_only_allowed_modules_drive_gpi2() {
    const fs::path dir = src_dir();
    TEST_ASSERT_TRUE_MESSAGE(fs::exists(dir), "src/ not found");
    const std::set<std::string> allowed = {"record_ctrl.cpp", "latency_test.cpp", "console.cpp"};
    std::set<std::string> callers;
    for (const auto& e : fs::directory_iterator(dir)) {
        const std::string name = e.path().filename().string();
        if (e.path().extension() != ".cpp") continue;  // headers only mention it in comments
        std::ifstream f(e.path());
        std::stringstream ss;
        ss << f.rdbuf();
        const std::string text = ss.str();
        const bool calls = text.find("gpi2_drive::request_") != std::string::npos;
        if (!calls) continue;
        callers.insert(name);
        const std::string msg = name + " calls gpi2_drive::request_* but is not an allowed GPI2 writer";
        TEST_ASSERT_TRUE_MESSAGE(allowed.count(name) == 1, msg.c_str());
    }
    // Exactly the allowed set, by name: a writer that vanishes is as suspicious as a new one.
    TEST_ASSERT_TRUE_MESSAGE(callers == allowed, "GPI2 writers differ from {record_ctrl, latency_test, console}");
}

static void test_console_rec_is_ownership_gated() {
    std::ifstream f(src_dir() / "console.cpp");
    std::stringstream ss;
    ss << f.rdbuf();
    const std::string text = ss.str();
    const size_t rec = text.find("void cmd_rec(");
    const size_t lat = text.find("void cmd_lat(");
    TEST_ASSERT_TRUE(rec != std::string::npos && lat != std::string::npos);
    // The ownership check precedes the first drive call inside cmd_rec, and
    // cmd_lat checks before anything else.
    const size_t gate = text.find("record_ctrl::console_owns()", rec);
    const size_t first_drive = text.find("gpi2_drive::request_", rec);
    TEST_ASSERT_TRUE(gate < first_drive);
    TEST_ASSERT_TRUE(text.find("require_console_owns()", lat) < text.find("latency_test::", lat));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_only_allowed_modules_drive_gpi2);
    RUN_TEST(test_console_rec_is_ownership_gated);
    return UNITY_END();
}
