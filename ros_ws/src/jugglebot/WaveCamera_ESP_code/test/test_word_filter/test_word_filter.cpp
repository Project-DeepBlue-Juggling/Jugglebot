// Blocked-word transmit guard (src/word_filter.h).

#include <unity.h>

#include "word_filter.h"

void setUp() {}
void tearDown() {}

static void test_every_blocked_word_is_caught() {
    for (size_t i = 0; i < NUM_BLOCKED_WORDS; ++i) TEST_ASSERT_NOT_NULL(word_filter::find_blocked_word(BLOCKED_WORDS[i]));
}

static void test_case_insensitive_substring() {
    TEST_ASSERT_EQUAL_STRING("format", word_filter::find_blocked_word("format test"));
    TEST_ASSERT_EQUAL_STRING("format", word_filter::find_blocked_word("FoRmAt"));
    TEST_ASSERT_NOT_NULL(word_filter::find_blocked_word("sys_UPGRADE now"));
    TEST_ASSERT_NOT_NULL(word_filter::find_blocked_word("xxFACTORYxx"));
    TEST_ASSERT_NOT_NULL(word_filter::find_blocked_word("alarm"));  // "rm": blunt by design
}

static void test_clean_lines_pass() {
    TEST_ASSERT_NULL(word_filter::find_blocked_word("help"));
    TEST_ASSERT_NULL(word_filter::find_blocked_word("version"));
    TEST_ASSERT_NULL(word_filter::find_blocked_word("?"));
    TEST_ASSERT_NULL(word_filter::find_blocked_word(""));
}

static void test_blocked_char_and_nonprintable() {
    const char* s = "ls | cat";
    TEST_ASSERT_EQUAL_PTR(s + 3, word_filter::find_blocked_char(s));
    for (const char* p = BLOCKED_CHARS; *p; ++p) {
        const char one[2] = {*p, '\0'};
        TEST_ASSERT_NOT_NULL(word_filter::find_blocked_char(one));
    }
    TEST_ASSERT_NOT_NULL(word_filter::find_blocked_char("ab\x01" "c"));
    TEST_ASSERT_NOT_NULL(word_filter::find_blocked_char("tab\there"));
    TEST_ASSERT_NOT_NULL(word_filter::find_blocked_char("caf\xC3\xA9"));
    TEST_ASSERT_NULL(word_filter::find_blocked_char("help ? -v 1,2 (x)"));
    TEST_ASSERT_NULL(word_filter::find_blocked_char(""));  // `>>` stays allowed
}

static void test_new_words() {
    TEST_ASSERT_EQUAL_STRING("reboot", word_filter::find_blocked_word("REBOOT now"));
    TEST_ASSERT_NOT_NULL(word_filter::find_blocked_word("shutdown -h"));
    TEST_ASSERT_NOT_NULL(word_filter::find_blocked_word("kill 1"));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_blocked_char_and_nonprintable);
    RUN_TEST(test_new_words);
    RUN_TEST(test_every_blocked_word_is_caught);
    RUN_TEST(test_case_insensitive_substring);
    RUN_TEST(test_clean_lines_pass);
    return UNITY_END();
}
