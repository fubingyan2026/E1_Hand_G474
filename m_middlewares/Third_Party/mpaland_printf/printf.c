// \author (c) Marco Paland (info@paland.com)
//             2014-2019, PALANDesign Hannover, Germany
//
// \license The MIT License (MIT)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// \brief 微型 printf、sprintf 和 (v)snprintf 实现，专为资源极有限的嵌入式系统优化。
//        线程安全且可重入。
//        替代臃肿的标准库/newlib printf——后者使用 malloc（且可能非线程安全）。
//


#include <stdbool.h>
#include <stdint.h>

#include "printf.h"

// ntoa（整数）转换缓冲区大小，必须足够容纳一个转换后的数值（含前导零），在栈上分配
// 默认：32 字节
#ifndef PRINTF_NTOA_BUFFER_SIZE
#define PRINTF_NTOA_BUFFER_SIZE 32U
#endif

// ftoa（浮点）转换缓冲区大小，必须足够容纳一个转换后的浮点数（含前导零），在栈上分配
// 默认：32 字节
#ifndef PRINTF_FTOA_BUFFER_SIZE
#define PRINTF_FTOA_BUFFER_SIZE 32U
#endif

// 是否支持浮点类型 (%f)
// 默认：启用
#ifndef PRINTF_DISABLE_SUPPORT_FLOAT
#define PRINTF_SUPPORT_FLOAT
#endif

// 是否支持科学计数法浮点 (%e/%g)
// 默认：启用
#ifndef PRINTF_DISABLE_SUPPORT_EXPONENTIAL
#define PRINTF_SUPPORT_EXPONENTIAL
#endif

// 默认浮点小数精度
// 默认：6 位
#ifndef PRINTF_DEFAULT_FLOAT_PRECISION
#define PRINTF_DEFAULT_FLOAT_PRECISION 6U
#endif

// 使用 %f 打印的最大浮点数值，超过此值将使用科学计数法
// 默认：1e9
#ifndef PRINTF_MAX_FLOAT
#define PRINTF_MAX_FLOAT 1e9f
#endif

// 是否支持 ptrdiff_t 类型 (%t)
// ptrdiff_t 通常定义在 <stddef.h> 中，为 long 类型
// 默认：启用
#ifndef PRINTF_DISABLE_SUPPORT_PTRDIFF_T
#define PRINTF_SUPPORT_PTRDIFF_T
#endif

///////////////////////////////////////////////////////////////////////////////

// internal flag definitions
#define FLAGS_ZEROPAD (1U << 0U)
#define FLAGS_LEFT (1U << 1U)
#define FLAGS_PLUS (1U << 2U)
#define FLAGS_SPACE (1U << 3U)
#define FLAGS_HASH (1U << 4U)
#define FLAGS_UPPERCASE (1U << 5U)
#define FLAGS_CHAR (1U << 6U)
#define FLAGS_SHORT (1U << 7U)
#define FLAGS_LONG (1U << 8U)
#define FLAGS_PRECISION (1U << 10U)
#define FLAGS_ADAPT_EXP (1U << 11U)

// import float.h for FLT_MAX
#if defined(PRINTF_SUPPORT_FLOAT)
#include <float.h>
#endif

// output function type
typedef void (*out_fct_type)(char character, void* buffer, size_t idx, size_t maxlen);

// wrapper (used as buffer) for output function type
typedef struct {
    void (*fct)(char character, void* arg);
    void* arg;
} out_fct_wrap_type;

// internal buffer output
static inline void _out_buffer(char character, void* buffer, size_t idx, size_t maxlen)
{
    if (idx < maxlen) {
        ((char*)buffer)[idx] = character;
    }
}

// internal null output
static inline void _out_null(char character, void* buffer, size_t idx, size_t maxlen)
{
    (void)character;
    (void)buffer;
    (void)idx;
    (void)maxlen;
}

// internal _putchar wrapper
static inline void _out_char(char character, void* buffer, size_t idx, size_t maxlen)
{
    (void)buffer;
    (void)idx;
    (void)maxlen;
    if (character) {
        _putchar(character);
    }
}

// internal output function wrapper
static inline void _out_fct(char character, void* buffer, size_t idx, size_t maxlen)
{
    (void)idx;
    (void)maxlen;
    if (character) {
        // buffer is the output fct pointer
        ((out_fct_wrap_type*)buffer)->fct(character, ((out_fct_wrap_type*)buffer)->arg);
    }
}

// internal secure strlen
// \return The length of the string (excluding the terminating 0) limited by 'maxsize'
static inline unsigned int _strnlen_s(const char* str, size_t maxsize)
{
    const char* s;
    for (s = str; *s && maxsize--; ++s)
        ;
    return (unsigned int)(s - str);
}

// internal test if char is a digit (0-9)
// \return true if char is a digit
static inline bool _is_digit(char ch)
{
    return (ch >= '0') && (ch <= '9');
}

// internal ASCII string to unsigned int conversion
static unsigned int _atoi(const char** str)
{
    unsigned int i = 0U;
    while (_is_digit(**str)) {
        i = i * 10U + (unsigned int)(*((*str)++) - '0');
    }
    return i;
}

// output the specified string in reverse, taking care of any zero-padding
static size_t _out_rev(out_fct_type out, char* buffer, size_t idx, size_t maxlen, const char* buf, size_t len, unsigned int width, unsigned int flags)
{
    const size_t start_idx = idx;

    // pad spaces up to given width
    if (!(flags & FLAGS_LEFT) && !(flags & FLAGS_ZEROPAD)) {
        for (size_t i = len; i < width; i++) {
            out(' ', buffer, idx++, maxlen);
        }
    }

    // reverse string
    while (len) {
        out(buf[--len], buffer, idx++, maxlen);
    }

    // append pad spaces up to given width
    if (flags & FLAGS_LEFT) {
        while (idx - start_idx < width) {
            out(' ', buffer, idx++, maxlen);
        }
    }

    return idx;
}

// internal itoa format
static size_t _ntoa_format(out_fct_type out, char* buffer, size_t idx, size_t maxlen, char* buf, size_t len, bool negative, unsigned int base, unsigned int prec, unsigned int width, unsigned int flags)
{
    // reserve one byte for sign character to prevent it being dropped when padding fills the buffer
    const size_t buf_limit = (negative || (flags & (FLAGS_PLUS | FLAGS_SPACE)))
        ? PRINTF_NTOA_BUFFER_SIZE - 1U
        : PRINTF_NTOA_BUFFER_SIZE;

    // pad leading zeros
    if (!(flags & FLAGS_LEFT)) {
        if (width && (flags & FLAGS_ZEROPAD) && (negative || (flags & (FLAGS_PLUS | FLAGS_SPACE)))) {
            width--;
        }
        while ((len < prec) && (len < buf_limit)) {
            buf[len++] = '0';
        }
        while ((flags & FLAGS_ZEROPAD) && (len < width) && (len < buf_limit)) {
            buf[len++] = '0';
        }
    }

    // handle hash
    if (flags & FLAGS_HASH) {
        if (!(flags & FLAGS_PRECISION) && len && ((len == prec) || (len == width))) {
            len--;
            if (len && (base == 16U)) {
                len--;
            }
        }
        if ((base == 16U) && !(flags & FLAGS_UPPERCASE) && (len < PRINTF_NTOA_BUFFER_SIZE)) {
            buf[len++] = 'x';
        } else if ((base == 16U) && (flags & FLAGS_UPPERCASE) && (len < PRINTF_NTOA_BUFFER_SIZE)) {
            buf[len++] = 'X';
        } else if ((base == 2U) && (len < PRINTF_NTOA_BUFFER_SIZE)) {
            buf[len++] = 'b';
        }
        if (len < PRINTF_NTOA_BUFFER_SIZE) {
            buf[len++] = '0';
        }
    }

    if (len < PRINTF_NTOA_BUFFER_SIZE) {
        if (negative) {
            buf[len++] = '-';
        } else if (flags & FLAGS_PLUS) {
            buf[len++] = '+'; // ignore the space if the '+' exists
        } else if (flags & FLAGS_SPACE) {
            buf[len++] = ' ';
        }
    }

    return _out_rev(out, buffer, idx, maxlen, buf, len, width, flags);
}

// internal itoa for 'long' type
static size_t _ntoa_long(out_fct_type out, char* buffer, size_t idx, size_t maxlen, unsigned long value, bool negative, unsigned long base, unsigned int prec, unsigned int width, unsigned int flags)
{
    char buf[PRINTF_NTOA_BUFFER_SIZE];
    size_t len = 0U;

    // no hash for 0 values
    if (!value) {
        flags &= ~FLAGS_HASH;
    }

    // write if precision != 0 and value is != 0
    if (!(flags & FLAGS_PRECISION) || value) {
        do {
            const char digit = (char)(value % base);
            buf[len++] = digit < 10 ? '0' + digit : (flags & FLAGS_UPPERCASE ? 'A' : 'a') + digit - 10;
            value /= base;
        } while (value && (len < PRINTF_NTOA_BUFFER_SIZE));
    }

    return _ntoa_format(out, buffer, idx, maxlen, buf, len, negative, (unsigned int)base, prec, width, flags);
}

// internal itoa for 'long long' type (not supported in this build)

#if defined(PRINTF_SUPPORT_FLOAT)

#if defined(PRINTF_SUPPORT_EXPONENTIAL)
// forward declaration so that _ftoa can switch to exp notation for values > PRINTF_MAX_FLOAT
static size_t _etoa(out_fct_type out, char* buffer, size_t idx, size_t maxlen, float value, unsigned int prec, unsigned int width, unsigned int flags);
#endif

// internal ftoa for fixed decimal floating point
static size_t _ftoa(out_fct_type out, char* buffer, size_t idx, size_t maxlen, float value, unsigned int prec, unsigned int width, unsigned int flags)
{
    char buf[PRINTF_FTOA_BUFFER_SIZE];
    size_t len = 0U;
    float diff = 0.0f;

    // powers of 10
    static const float pow10[] = { 1.0f, 10.0f, 100.0f, 1000.0f, 10000.0f, 100000.0f, 1000000.0f, 10000000.0f, 100000000.0f, 1000000000.0f };

    // test for special values
    if (value != value)
        return _out_rev(out, buffer, idx, maxlen, "nan", 3, width, flags);
    if (value < -FLT_MAX)
        return _out_rev(out, buffer, idx, maxlen, "fni-", 4, width, flags);
    if (value > FLT_MAX)
        return _out_rev(out, buffer, idx, maxlen, (flags & FLAGS_PLUS) ? "fni+" : "fni", (flags & FLAGS_PLUS) ? 4U : 3U, width, flags);

    // test for very large values
    // standard printf behavior is to print EVERY whole number digit -- which could be 100s of characters overflowing your buffers == bad
    if ((value > PRINTF_MAX_FLOAT) || (value < -PRINTF_MAX_FLOAT)) {
#if defined(PRINTF_SUPPORT_EXPONENTIAL)
        return _etoa(out, buffer, idx, maxlen, value, prec, width, flags);
#else
        return 0U;
#endif
    }

    // test for negative
    bool negative = false;
    if (value < 0.0f) {
        negative = true;
        value = 0.0f - value;
    }

    // set default precision, if not set explicitly
    if (!(flags & FLAGS_PRECISION)) {
        prec = PRINTF_DEFAULT_FLOAT_PRECISION;
    }
    // limit precision to 9, cause a prec >= 10 can lead to overflow errors
    while ((len < PRINTF_FTOA_BUFFER_SIZE) && (prec > 9U)) {
        buf[len++] = '0';
        prec--;
    }

    int whole = (int)value;
    float tmp = (value - (float)whole) * pow10[prec];
    unsigned long frac = (unsigned long)tmp;
    diff = tmp - (float)frac;

    if (diff > 0.5f) {
        ++frac;
        // handle rollover, e.g. case 0.99 with prec 1 is 1.0
        if (frac >= pow10[prec]) {
            frac = 0;
            ++whole;
        }
    } else if (diff < 0.5f) {
    } else if ((frac == 0U) || (frac & 1U)) {
        // if halfway, round up if odd OR if last digit is 0
        ++frac;
    }

    if (prec == 0U) {
        diff = value - (float)whole;
        if ((!(diff < 0.5f) || (diff > 0.5f)) && (whole & 1)) {
            // exactly 0.5 and ODD, then round up
            // 1.5 -> 2, but 2.5 -> 2
            ++whole;
        }
    } else {
        unsigned int count = prec;
        // now do fractional part, as an unsigned number
        while (len < PRINTF_FTOA_BUFFER_SIZE) {
            --count;
            buf[len++] = (char)(48U + (frac % 10U));
            if (!(frac /= 10U)) {
                break;
            }
        }
        // add extra 0s
        while ((len < PRINTF_FTOA_BUFFER_SIZE) && (count-- > 0U)) {
            buf[len++] = '0';
        }
        if (len < PRINTF_FTOA_BUFFER_SIZE) {
            // add decimal
            buf[len++] = '.';
        }
    }

    // do whole part, number is reversed
    while (len < PRINTF_FTOA_BUFFER_SIZE) {
        buf[len++] = (char)(48 + (whole % 10));
        if (!(whole /= 10)) {
            break;
        }
    }

    // pad leading zeros
    if (!(flags & FLAGS_LEFT) && (flags & FLAGS_ZEROPAD)) {
        if (width && (negative || (flags & (FLAGS_PLUS | FLAGS_SPACE)))) {
            width--;
        }
        while ((len < width) && (len < PRINTF_FTOA_BUFFER_SIZE)) {
            buf[len++] = '0';
        }
    }

    if (len < PRINTF_FTOA_BUFFER_SIZE) {
        if (negative) {
            buf[len++] = '-';
        } else if (flags & FLAGS_PLUS) {
            buf[len++] = '+'; // ignore the space if the '+' exists
        } else if (flags & FLAGS_SPACE) {
            buf[len++] = ' ';
        }
    }

    return _out_rev(out, buffer, idx, maxlen, buf, len, width, flags);
}

#if defined(PRINTF_SUPPORT_EXPONENTIAL)
// internal ftoa variant for exponential floating-point type, contributed by Martijn Jasperse <m.jasperse@gmail.com>
// adapted for single-precision float
static size_t _etoa(out_fct_type out, char* buffer, size_t idx, size_t maxlen, float value, unsigned int prec, unsigned int width, unsigned int flags)
{
    // check for NaN and special values
    if ((value != value) || (value > FLT_MAX) || (value < -FLT_MAX)) {
        return _ftoa(out, buffer, idx, maxlen, value, prec, width, flags);
    }

    // determine the sign
    const bool negative = value < 0.0f;
    if (negative) {
        value = -value;
    }

    // default precision
    if (!(flags & FLAGS_PRECISION)) {
        prec = PRINTF_DEFAULT_FLOAT_PRECISION;
    }

    // determine the decimal exponent
    // based on the algorithm by David Gay (https://www.ampl.com/netlib/fp/dtoa.c)
    union {
        uint32_t U;
        float F;
    } conv;

    conv.F = value;
    int exp2 = (int)((conv.U >> 23U) & 0xFFU) - 127; // effectively log2
    conv.U = (conv.U & ((1U << 23U) - 1U)) | (127U << 23U); // drop the exponent so conv.F is now in [1,2)
    // now approximate log10 from the log2 integer part and an expansion of ln around 1.5
    int expval = (int)(0.1760912590558f + exp2 * 0.301029995663981f + (conv.F - 1.5f) * 0.289529654602168f);
    // now we want to compute 10^expval but we want to be sure it won't overflow
    exp2 = (int)(expval * 3.321928094887362f + 0.5f);
    const float z = expval * 2.302585092994046f - exp2 * 0.6931471805599453f;
    const float z2 = z * z;
    conv.U = (uint32_t)(exp2 + 127) << 23U;
    // compute exp(z) using continued fractions, see https://en.wikipedia.org/wiki/Exponential_function#Continued_fractions_for_ex
    conv.F *= 1.0f + 2.0f * z / (2.0f - z + (z2 / (6.0f + (z2 / (10.0f + z2 / 14.0f)))));
    // correct for rounding errors
    if (value < conv.F) {
        expval--;
        conv.F /= 10.0f;
    }

    // the exponent format is "%+03d" and largest float value is ~3.4e38, so expval is always within [-38,38]
    // set aside 4 characters for the exponent part
    unsigned int minwidth = 4U;

    // in "%g" mode, "prec" is the number of *significant figures* not decimals
    if (flags & FLAGS_ADAPT_EXP) {
        // do we want to fall-back to "%f" mode?
        if ((value >= 1e-4f) && (value < 1e6f)) {
            if ((int)prec > expval) {
                prec = (unsigned)((int)prec - expval - 1);
            } else {
                prec = 0;
            }
            flags |= FLAGS_PRECISION; // make sure _ftoa respects precision
            // no characters in exponent
            minwidth = 0U;
            expval = 0;
        } else {
            // we use one sigfig for the whole part
            if ((prec > 0) && (flags & FLAGS_PRECISION)) {
                --prec;
            }
        }
    }

    // will everything fit?
    unsigned int fwidth = width;
    if (width > minwidth) {
        // we didn't fall-back so subtract the characters required for the exponent
        fwidth -= minwidth;
    } else {
        // not enough characters, so go back to default sizing
        fwidth = 0U;
    }
    if ((flags & FLAGS_LEFT) && minwidth) {
        // if we're padding on the right, DON'T pad the floating part
        fwidth = 0U;
    }

    // rescale the float value
    if (expval) {
        value /= conv.F;
    }

    // output the floating part
    const size_t start_idx = idx;
    idx = _ftoa(out, buffer, idx, maxlen, negative ? -value : value, prec, fwidth, flags & ~FLAGS_ADAPT_EXP);

    // output the exponent part
    if (minwidth) {
        // output the exponential symbol
        out((flags & FLAGS_UPPERCASE) ? 'E' : 'e', buffer, idx++, maxlen);
        // output the exponent value
        idx = _ntoa_long(out, buffer, idx, maxlen, (expval < 0) ? -expval : expval, expval < 0, 10, 0, minwidth - 1, FLAGS_ZEROPAD | FLAGS_PLUS);
        // might need to right-pad spaces
        if (flags & FLAGS_LEFT) {
            while (idx - start_idx < width)
                out(' ', buffer, idx++, maxlen);
        }
    }
    return idx;
}
#endif // PRINTF_SUPPORT_EXPONENTIAL
#endif // PRINTF_SUPPORT_FLOAT

// internal vsnprintf
static int _vsnprintf(out_fct_type out, char* buffer, const size_t maxlen, const char* format, va_list va)
{
    unsigned int flags, width, precision, n;
    size_t idx = 0U;

    if (!buffer) {
        // use null output function
        out = _out_null;
    }

    while (*format) {
        // format specifier?  %[flags][width][.precision][length]
        if (*format != '%') {
            // no
            out(*format, buffer, idx++, maxlen);
            format++;
            continue;
        } else {
            // yes, evaluate it
            format++;
        }

        // evaluate flags
        flags = 0U;
        do {
            switch (*format) {
            case '0':
                flags |= FLAGS_ZEROPAD;
                format++;
                n = 1U;
                break;
            case '-':
                flags |= FLAGS_LEFT;
                format++;
                n = 1U;
                break;
            case '+':
                flags |= FLAGS_PLUS;
                format++;
                n = 1U;
                break;
            case ' ':
                flags |= FLAGS_SPACE;
                format++;
                n = 1U;
                break;
            case '#':
                flags |= FLAGS_HASH;
                format++;
                n = 1U;
                break;
            default:
                n = 0U;
                break;
            }
        } while (n);

        // evaluate width field
        width = 0U;
        if (_is_digit(*format)) {
            width = _atoi(&format);
        } else if (*format == '*') {
            const int w = va_arg(va, int);
            if (w < 0) {
                flags |= FLAGS_LEFT; // reverse padding
                width = (unsigned int)-w;
            } else {
                width = (unsigned int)w;
            }
            format++;
        }

        // evaluate precision field
        precision = 0U;
        if (*format == '.') {
            flags |= FLAGS_PRECISION;
            format++;
            if (_is_digit(*format)) {
                precision = _atoi(&format);
            } else if (*format == '*') {
                const int prec = (int)va_arg(va, int);
                precision = prec > 0 ? (unsigned int)prec : 0U;
                format++;
            }
        }

        // evaluate length field
        switch (*format) {
        case 'l':
            flags |= FLAGS_LONG;
            format++;
            break;
        case 'h':
            flags |= FLAGS_SHORT;
            format++;
            if (*format == 'h') {
                flags |= FLAGS_CHAR;
                format++;
            }
            break;
#if defined(PRINTF_SUPPORT_PTRDIFF_T)
        case 't':
            flags |= FLAGS_LONG;
            format++;
            break;
#endif
        case 'j':
            flags |= FLAGS_LONG;
            format++;
            break;
        case 'z':
            flags |= FLAGS_LONG;
            format++;
            break;
        default:
            break;
        }

        // evaluate specifier
        switch (*format) {
        case 'd':
        case 'i':
        case 'u':
        case 'x':
        case 'X':
        case 'o':
        case 'b': {
            // set the base
            unsigned int base;
            if (*format == 'x' || *format == 'X') {
                base = 16U;
            } else if (*format == 'o') {
                base = 8U;
            } else if (*format == 'b') {
                base = 2U;
            } else {
                base = 10U;
                flags &= ~FLAGS_HASH; // no hash for dec format
            }
            // uppercase
            if (*format == 'X') {
                flags |= FLAGS_UPPERCASE;
            }

            // no plus or space flag for u, x, X, o, b
            if ((*format != 'i') && (*format != 'd')) {
                flags &= ~(FLAGS_PLUS | FLAGS_SPACE);
            }

            // ignore '0' flag when precision is given
            if (flags & FLAGS_PRECISION) {
                flags &= ~FLAGS_ZEROPAD;
            }

            // convert the integer
            if ((*format == 'i') || (*format == 'd')) {
                // signed
                if (flags & FLAGS_LONG) {
                    const long value = va_arg(va, long);
                    idx = _ntoa_long(out, buffer, idx, maxlen, (unsigned long)(value < 0 ? -(unsigned long)value : (unsigned long)value), value < 0, base, precision, width, flags);
                } else {
                    const int value = (flags & FLAGS_CHAR) ? (char)va_arg(va, int) : (flags & FLAGS_SHORT) ? (short int)va_arg(va, int)
                                                                                                           : va_arg(va, int);
                    idx = _ntoa_long(out, buffer, idx, maxlen, (unsigned int)(value < 0 ? -(unsigned int)value : (unsigned int)value), value < 0, base, precision, width, flags);
                }
            } else {
                // unsigned
                if (flags & FLAGS_LONG) {
                    idx = _ntoa_long(out, buffer, idx, maxlen, va_arg(va, unsigned long), false, base, precision, width, flags);
                } else {
                    const unsigned int value = (flags & FLAGS_CHAR) ? (unsigned char)va_arg(va, unsigned int) : (flags & FLAGS_SHORT) ? (unsigned short int)va_arg(va, unsigned int)
                                                                                                                                      : va_arg(va, unsigned int);
                    idx = _ntoa_long(out, buffer, idx, maxlen, value, false, base, precision, width, flags);
                }
            }
            format++;
            break;
        }
#if defined(PRINTF_SUPPORT_FLOAT)
        case 'f':
        case 'F':
            if (*format == 'F')
                flags |= FLAGS_UPPERCASE;
            idx = _ftoa(out, buffer, idx, maxlen, (float)va_arg(va, double), precision, width, flags);
            format++;
            break;
#if defined(PRINTF_SUPPORT_EXPONENTIAL)
        case 'e':
        case 'E':
        case 'g':
        case 'G':
            if ((*format == 'g') || (*format == 'G'))
                flags |= FLAGS_ADAPT_EXP;
            if ((*format == 'E') || (*format == 'G'))
                flags |= FLAGS_UPPERCASE;
            idx = _etoa(out, buffer, idx, maxlen, (float)va_arg(va, double), precision, width, flags);
            format++;
            break;
#endif // PRINTF_SUPPORT_EXPONENTIAL
#endif // PRINTF_SUPPORT_FLOAT
        case 'c': {
            unsigned int l = 1U;
            // pre padding
            if (!(flags & FLAGS_LEFT)) {
                while (l++ < width) {
                    out(' ', buffer, idx++, maxlen);
                }
            }
            // char output
            out((char)va_arg(va, int), buffer, idx++, maxlen);
            // post padding
            if (flags & FLAGS_LEFT) {
                while (l++ < width) {
                    out(' ', buffer, idx++, maxlen);
                }
            }
            format++;
            break;
        }

        case 's': {
            const char* p = va_arg(va, char*);
            if (!p) {
                p = "(null)";
            }
            unsigned int l = _strnlen_s(p, precision ? precision : (size_t)-1);
            // pre padding
            if (flags & FLAGS_PRECISION) {
                l = (l < precision ? l : precision);
            }
            if (!(flags & FLAGS_LEFT)) {
                while (l++ < width) {
                    out(' ', buffer, idx++, maxlen);
                }
            }
            // string output
            while ((*p != 0) && (!(flags & FLAGS_PRECISION) || precision--)) {
                out(*(p++), buffer, idx++, maxlen);
            }
            // post padding
            if (flags & FLAGS_LEFT) {
                while (l++ < width) {
                    out(' ', buffer, idx++, maxlen);
                }
            }
            format++;
            break;
        }

        case 'p': {
            width = sizeof(void*) * 2U;
            flags |= FLAGS_ZEROPAD | FLAGS_UPPERCASE;
            idx = _ntoa_long(out, buffer, idx, maxlen, (unsigned long)((uintptr_t)va_arg(va, void*)), false, 16U, precision, width, flags);
            format++;
            break;
        }

        case '%':
            out('%', buffer, idx++, maxlen);
            format++;
            break;

        default:
            out(*format, buffer, idx++, maxlen);
            format++;
            break;
        }
    }

    // termination
    out((char)0, buffer, idx < maxlen ? idx : maxlen - 1U, maxlen);

    // return written chars without terminating \0
    return (int)idx;
}

int printf_(const char* format, ...)
{
    va_list va;
    va_start(va, format);
    char buffer[1];
    const int ret = _vsnprintf(_out_char, buffer, (size_t)-1, format, va);
    va_end(va);
    return ret;
}

int sprintf_(char* buffer, const char* format, ...)
{
    va_list va;
    va_start(va, format);
    const int ret = _vsnprintf(_out_buffer, buffer, (size_t)-1, format, va);
    va_end(va);
    return ret;
}

int snprintf_(char* buffer, size_t count, const char* format, ...)
{
    va_list va;
    va_start(va, format);
    const int ret = _vsnprintf(_out_buffer, buffer, count, format, va);
    va_end(va);
    return ret;
}

int vprintf_(const char* format, va_list va)
{
    char buffer[1];
    return _vsnprintf(_out_char, buffer, (size_t)-1, format, va);
}

int vsnprintf_(char* buffer, size_t count, const char* format, va_list va)
{
    return _vsnprintf(_out_buffer, buffer, count, format, va);
}

int fctprintf(void (*out)(char character, void* arg), void* arg, const char* format, ...)
{
    va_list va;
    va_start(va, format);
    const out_fct_wrap_type out_fct_wrap = { out, arg };
    const int ret = _vsnprintf(_out_fct, (char*)(uintptr_t)&out_fct_wrap, (size_t)-1, format, va);
    va_end(va);
    return ret;
}
