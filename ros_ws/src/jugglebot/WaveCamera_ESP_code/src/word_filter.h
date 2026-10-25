#pragma once

// Transmit guard for `>`: case-insensitive substring match against
// BLOCKED_WORDS, plus a byte filter (printable ASCII only, no BLOCKED_CHARS;
// both in include/config.h). Pure, header-only. No override exists.
//
// Substring matching is deliberately blunt: "rm" also blocks "format",
// "alarm", "term". False positives are acceptable; a false negative could
// brick the camera.

#include <cstddef>

#include "config.h"

namespace word_filter {

inline char to_lower_ascii(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }

// Case-insensitive "needle in haystack". needle must be lowercase.
inline bool contains_ci(const char* haystack, const char* needle) {
    if (!*needle) return true;
    for (const char* h = haystack; *h; ++h) {
        const char* a = h;
        const char* b = needle;
        while (*a && *b && to_lower_ascii(*a) == *b) { ++a; ++b; }
        if (!*b) return true;
    }
    return false;
}

// Returns the first refused byte in `line` (outside 0x20..0x7E, or one of
// BLOCKED_CHARS), or nullptr if every byte is allowed. An empty line passes.
inline const char* find_blocked_char(const char* line) {
    for (const char* p = line; *p; ++p) {
        const unsigned char c = static_cast<unsigned char>(*p);
        if (c < 0x20 || c > 0x7E) return p;
        for (const char* b = BLOCKED_CHARS; *b; ++b)
            if (*p == *b) return p;
    }
    return nullptr;
}

// Returns the first blocked word found in `line`, or nullptr if clean.
inline const char* find_blocked_word(const char* line) {
    for (size_t i = 0; i < NUM_BLOCKED_WORDS; ++i)
        if (contains_ci(line, BLOCKED_WORDS[i])) return BLOCKED_WORDS[i];
    return nullptr;
}

}  // namespace word_filter
