#pragma once

// Camera UART line assembler (pure, header-only).
//
// Bytes are fed with their read timestamp. A line ends at '\n' or '\r'
// (a '\n' directly after '\r' is swallowed, so CRLF yields one line). A line
// is flushed as partial when it reaches UART_LINE_MAX_BYTES or when no
// terminator arrives for UART_LINE_IDLE_FLUSH_MS after the last byte.
// Text is escaped for the console: printable ASCII verbatim, others \xNN.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "config.h"

enum class LineEnd : uint8_t { Eol, Overflow, Timeout };

inline const char* line_end_name(LineEnd e) {
    return e == LineEnd::Eol ? "eol" : e == LineEnd::Overflow ? "partial_overflow" : "partial_timeout";
}

struct UartLine {
    char     text[UART_LINE_MAX_BYTES * 4 + 1];  // escaped, NUL-terminated
    char     raw[UART_LINE_MAX_BYTES + 1];       // raw bytes, NUL-terminated (may contain NULs)
    size_t   raw_len;
    int64_t  t_first_us;  // read time of the first byte
    int64_t  t_last_us;   // read time of the last byte (terminator for Eol)
    LineEnd  end;
};

class LineAssembler {
public:
    bool empty() const { return len_ == 0; }

    // Returns true and fills `out` when a line completes.
    bool feed(uint8_t b, int64_t t_us, UartLine& out) {
        const bool after_cr = prev_was_cr_;
        prev_was_cr_ = (b == '\r');
        if (b == '\n' || b == '\r') {
            if (b == '\n' && after_cr && len_ == 0) return false;  // LF of a CRLF pair
            emit(t_us, LineEnd::Eol, out);
            return true;
        }
        if (len_ == 0) t_first_us_ = t_us;
        raw_[len_++] = static_cast<char>(b);
        t_last_us_ = t_us;
        if (len_ >= UART_LINE_MAX_BYTES) {
            emit(t_us, LineEnd::Overflow, out);
            return true;
        }
        return false;
    }

    bool poll_timeout(int64_t now_us, UartLine& out) {
        if (len_ == 0) return false;
        if (now_us - t_last_us_ < static_cast<int64_t>(UART_LINE_IDLE_FLUSH_MS) * 1000) return false;
        emit(t_last_us_, LineEnd::Timeout, out);
        return true;
    }

    static size_t escape(const char* raw, size_t n, char* out, size_t out_len) {
        static const char HEX_DIGITS[] = "0123456789ABCDEF";
        size_t w = 0;
        for (size_t i = 0; i < n; ++i) {
            const uint8_t c = static_cast<uint8_t>(raw[i]);
            if (c >= 0x20 && c <= 0x7E) {
                if (w + 1 >= out_len) break;
                out[w++] = static_cast<char>(c);
            } else {
                if (w + 4 >= out_len) break;
                out[w++] = '\\'; out[w++] = 'x';
                out[w++] = HEX_DIGITS[c >> 4]; out[w++] = HEX_DIGITS[c & 0xF];
            }
        }
        if (out_len) out[w < out_len ? w : out_len - 1] = '\0';
        return w;
    }

private:
    void emit(int64_t t_end_us, LineEnd end, UartLine& out) {
        memcpy(out.raw, raw_, len_);
        out.raw[len_] = '\0';
        out.raw_len = len_;
        escape(raw_, len_, out.text, sizeof(out.text));
        out.t_first_us = len_ ? t_first_us_ : t_end_us;
        out.t_last_us = t_end_us;
        out.end = end;
        len_ = 0;
    }

    char    raw_[UART_LINE_MAX_BYTES];
    size_t  len_ = 0;
    int64_t t_first_us_ = 0;
    int64_t t_last_us_ = 0;
    bool    prev_was_cr_ = false;
};
