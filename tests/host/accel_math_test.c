/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Host-side unit test for src/pointing/rip_accel_math.h (no Zephyr needed).
 *
 * Run from the repository root (test.py runs the same in CI):
 *   cc -std=c11 -Wall -Wextra -Werror -o /tmp/accel_math_test tests/host/accel_math_test.c -lm \
 *     && /tmp/accel_math_test
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../src/pointing/rip_accel_math.h"

static int failures;
static int checks;

#define CHECK(cond, ...)                                                                           \
    do {                                                                                           \
        checks++;                                                                                  \
        if (!(cond)) {                                                                             \
            failures++;                                                                            \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);                                            \
            printf(__VA_ARGS__);                                                                   \
            printf("\n");                                                                          \
        }                                                                                          \
    } while (0)

/* Devicetree defaults (counts per 100 ms) and the Kconfig default window. */
#define LOW 20
#define HIGH 150
#define WINDOW 40

static void test_clamp(void) {
    CHECK(rip_accel_clamp_min_percent(0) == 20, "clamp 0");
    CHECK(rip_accel_clamp_min_percent(-5) == 20, "clamp -5");
    CHECK(rip_accel_clamp_min_percent(19) == 20, "clamp 19");
    CHECK(rip_accel_clamp_min_percent(20) == 20, "clamp 20");
    CHECK(rip_accel_clamp_min_percent(55) == 55, "clamp 55");
    CHECK(rip_accel_clamp_min_percent(100) == 100, "clamp 100");
    CHECK(rip_accel_clamp_min_percent(101) == 100, "clamp 101");
    CHECK(rip_accel_clamp_min_percent(256) == 100, "clamp 256 must not wrap");
    CHECK(rip_accel_clamp_min_percent(4294967295LL) == 100, "clamp UINT32_MAX");
}

static void test_isqrt(void) {
    for (uint64_t n = 0; n < 200000; n++) {
        uint64_t r = rip_accel_isqrt(n);
        if (r * r > n || (r + 1) * (r + 1) <= n) {
            CHECK(0, "isqrt(%llu) = %llu", (unsigned long long)n, (unsigned long long)r);
            break;
        }
    }
    static const uint64_t big[] = {
        4294967295ULL,           4294967296ULL,
        999999999999ULL,         5497558138880000ULL,
        18446744073709551615ULL, 18446744065119617025ULL /* (2^32 - 1)^2 */,
        18446744065119617024ULL,
    };
    for (size_t i = 0; i < sizeof(big) / sizeof(big[0]); i++) {
        uint64_t n = big[i];
        uint64_t r = rip_accel_isqrt(n);
        /* r < 2^32 so r * r cannot overflow; (r + 1)^2 may, so compare via n / (r + 1). */
        bool ok = r * r <= n && n / (r + 1) < r + 1;
        CHECK(ok, "isqrt(%llu) = %llu", (unsigned long long)n, (unsigned long long)r);
    }
    CHECK(rip_accel_isqrt(18446744073709551615ULL) == 4294967295ULL, "isqrt(UINT64_MAX)");
}

static void test_speed(void) {
    /* Counts per 100 ms: a 50-count vector in 40 ms is 125. */
    CHECK(rip_accel_speed(30, 40, WINDOW) == 125, "speed (30, 40): %u",
          rip_accel_speed(30, 40, WINDOW));
    CHECK(rip_accel_speed(-30, 40, WINDOW) == 125, "speed (-30, 40)");
    CHECK(rip_accel_speed(30, -40, WINDOW) == 125, "speed (30, -40)");
    CHECK(rip_accel_speed(3, 4, WINDOW) == 12, "speed (3, 4) floors 12.5: %u",
          rip_accel_speed(3, 4, WINDOW));
    CHECK(rip_accel_speed(0, 0, WINDOW) == 0, "speed zero");
    CHECK(rip_accel_speed(10, 0, 0) == 0, "speed with zero window");
    CHECK(rip_accel_speed(8, 0, WINDOW) == 20 && rip_accel_speed(0, -8, WINDOW) == 20,
          "single axis 8 in 40 ms is 20");

    /* A diagonal counts its hypot, not the sum or the larger axis. */
    CHECK(rip_accel_speed(100, 100, 100) == 141, "diagonal (100, 100): %u",
          rip_accel_speed(100, 100, 100));
    CHECK(rip_accel_speed(100, 0, 100) == 100, "axis (100, 0)");
    for (int32_t x = -60; x <= 60; x += 7) {
        for (int32_t y = -60; y <= 60; y += 5) {
            uint32_t got = rip_accel_speed(x, y, WINDOW);
            uint32_t exact = (uint32_t)floor(hypot(x, y) * 100.0 / WINDOW + 1e-9);
            if (got != exact) {
                CHECK(0, "speed (%d, %d): got %u expected %u", x, y, got, exact);
            }
        }
    }

    /* Huge sums neither overflow nor wrap. */
    CHECK(rip_accel_speed(INT32_MIN, INT32_MIN, 1) > 300000000U, "huge speed: %u",
          rip_accel_speed(INT32_MIN, INT32_MIN, 1));
    uint32_t full = rip_accel_speed(32 * 32767, 32 * -32768, 10);
    double full_exact = floor(hypot(32.0 * 32767, 32.0 * 32768) * 100.0 / 10);
    CHECK(full == (uint32_t)full_exact, "full history speed: %u expected %.0f", full, full_exact);
}

static void test_gain_curve(void) {
    /* Endpoints: min at and below speed_low, 1000 at and above speed_high. */
    CHECK(rip_accel_gain_permille(50, 0, LOW, HIGH) == 500, "min at 0");
    CHECK(rip_accel_gain_permille(50, LOW, LOW, HIGH) == 500, "min at low");
    CHECK(rip_accel_gain_permille(50, HIGH, LOW, HIGH) == 1000, "1000 at high");
    CHECK(rip_accel_gain_permille(50, 100000, LOW, HIGH) == 1000, "1000 above high");
    CHECK(rip_accel_gain_permille(20, 0, LOW, HIGH) == 200, "min 20 at 0");
    /* The smoothstep is halfway at the midpoint. */
    CHECK(rip_accel_gain_permille(50, 85, LOW, HIGH) == 750, "midpoint: %u",
          rip_accel_gain_permille(50, 85, LOW, HIGH));
    /* t = 0.25: smoothstep 0.15625 */
    CHECK(rip_accel_gain_permille(20, 25, 0, 100) == 200 + 800 * 156 / 1000, "quarter: %u",
          rip_accel_gain_permille(20, 25, 0, 100));
    /* Out-of-range minimums are clamped here too. */
    CHECK(rip_accel_gain_permille(5, 0, LOW, HIGH) == 200, "min clamped up");
    CHECK(rip_accel_gain_permille(250, 0, LOW, HIGH) == 1000, "min clamped down");

    /* 100 is off: gain 1 at every speed. */
    for (uint32_t speed = 0; speed <= 400; speed++) {
        if (rip_accel_gain_permille(100, speed, LOW, HIGH) != 1000) {
            CHECK(0, "min 100 at speed %u is not 1000", speed);
            break;
        }
    }

    /* Monotonic in speed, within [min, 1000], and a higher min never lowers it. */
    for (int min = 20; min <= 100; min += 5) {
        uint32_t prev = 0;
        for (uint32_t speed = 0; speed <= 400; speed++) {
            uint32_t g = rip_accel_gain_permille(min, speed, LOW, HIGH);
            uint32_t g_more = rip_accel_gain_permille(min + 5, speed, LOW, HIGH);
            if (g < prev || g < (uint32_t)min * 10 || g > 1000 || g_more < g) {
                CHECK(0, "min %d speed %u: gain %u (prev %u, min+5 %u)", min, speed, g, prev,
                      g_more);
                break;
            }
            prev = g;
        }
    }
    /* Matches the floating-point curve within the integer flooring (t, the
     * smoothstep and the gain each floor; together under 3 permille). */
    for (uint32_t speed = LOW; speed <= HIGH; speed++) {
        double t = (double)(speed - LOW) / (HIGH - LOW);
        double exact = 300.0 + 700.0 * (3 * t * t - 2 * t * t * t);
        uint32_t g = rip_accel_gain_permille(30, speed, LOW, HIGH);
        if (g > exact + 1e-9 || exact - g >= 3.0) {
            CHECK(0, "speed %u: gain %u expected %.2f", speed, g, exact);
        }
    }

    /* speed_high <= speed_low is a step at speed_low. */
    CHECK(rip_accel_gain_permille(40, 30, 30, 30) == 400, "step at low");
    CHECK(rip_accel_gain_permille(40, 31, 30, 30) == 1000, "step above low");
    CHECK(rip_accel_gain_permille(40, 30, 30, 10) == 400, "step, high < low, at low");
    CHECK(rip_accel_gain_permille(40, 31, 30, 10) == 1000, "step, high < low, above low");
    CHECK(rip_accel_gain_permille(40, 20, 30, 10) == 400, "step, high < low, between");
}

static void test_apply_remainder(void) {
    /* N slow 1-count events at gain 0.5 give N/2: slowed, not lost. */
    int32_t rem = 0;
    long total = 0;
    for (int i = 0; i < 1001; i++) {
        total += rip_accel_apply(1, 500, &rem);
    }
    CHECK(labs(total - 500) <= 1, "gain 0.5 x1001: total %ld", total);

    /* Negative values are symmetric. */
    rem = 0;
    total = 0;
    for (int i = 0; i < 1001; i++) {
        total += rip_accel_apply(-1, 500, &rem);
    }
    CHECK(labs(total + 500) <= 1, "gain 0.5 x1001 negative: total %ld", total);
    for (uint32_t gain = 200; gain <= 1000; gain += 37) {
        for (int16_t v = 1; v < 200; v += 3) {
            int32_t rp = 0;
            int32_t rn = 0;
            int16_t p = rip_accel_apply(v, gain, &rp);
            int16_t n = rip_accel_apply((int16_t)-v, gain, &rn);
            if (p != -n || rp != -rn) {
                CHECK(0, "asymmetric at %d gain %u: %d/%d rem %d/%d", v, gain, p, n, rp, rn);
            }
        }
    }

    /* Gain 0.2 on 1-count steps emits every 5th, total exact. */
    rem = 0;
    total = 0;
    for (int i = 0; i < 1000; i++) {
        total += rip_accel_apply(1, 200, &rem);
    }
    CHECK(total == 200 && rem == 0, "gain 0.2 x1000: total %ld rem %d", total, rem);

    /* Back-and-forth motion cancels in the carried fraction. */
    rem = 0;
    total = 0;
    for (int i = 0; i < 100; i++) {
        total += rip_accel_apply(i % 2 ? -3 : 3, 300, &rem);
    }
    CHECK(total == 0 && rem == 0, "alternating: total %ld rem %d", total, rem);

    /* Full gain with no remainder passes values through. */
    rem = 0;
    CHECK(rip_accel_apply(123, 1000, &rem) == 123 && rem == 0, "gain 1 passthrough");
    CHECK(rip_accel_apply(-123, 1000, &rem) == -123 && rem == 0, "gain 1 negative passthrough");

    /* int16 extremes stay in range; the remainder stays below one count. */
    rem = 999;
    CHECK(rip_accel_apply(INT16_MAX, 1000, &rem) == INT16_MAX && rem == 999,
          "max with remainder: rem %d", rem);
    rem = -999;
    CHECK(rip_accel_apply(INT16_MIN, 1000, &rem) == INT16_MIN && rem == -999,
          "min with remainder: rem %d", rem);
    srand(1);
    rem = 0;
    for (int i = 0; i < 100000; i++) {
        int16_t v = (int16_t)((rand() % 65536) - 32768);
        uint32_t gain = 200 + (uint32_t)(rand() % 801);
        int16_t out = rip_accel_apply(v, gain, &rem);
        if (labs((long)rem) >= 1000 || labs((long)out) > labs((long)v) + 1) {
            CHECK(0, "random %d gain %u: out %d rem %d", v, gain, out, rem);
            break;
        }
    }
}

static void test_window_sums(void) {
    struct rip_accel_state s = {0};
    int32_t sx;
    int32_t sy;
    rip_accel_window_sums(&s, 1000, WINDOW, &sx, &sy);
    CHECK(sx == 0 && sy == 0, "empty history");

    rip_accel_record(&s, 1000, true, 10);
    rip_accel_record(&s, 1010, true, 10);
    rip_accel_record(&s, 1030, false, -5);
    rip_accel_window_sums(&s, 1035, WINDOW, &sx, &sy);
    CHECK(sx == 20 && sy == -5, "all in window: %d %d", sx, sy);
    /* Exactly window_ms old is outside. */
    rip_accel_window_sums(&s, 1040, WINDOW, &sx, &sy);
    CHECK(sx == 10 && sy == -5, "oldest expired at 40 ms: %d %d", sx, sy);
    rip_accel_window_sums(&s, 1039, WINDOW, &sx, &sy);
    CHECK(sx == 20 && sy == -5, "oldest kept at 39 ms: %d %d", sx, sy);
    rip_accel_window_sums(&s, 1100, WINDOW, &sx, &sy);
    CHECK(sx == 0 && sy == 0, "all expired: %d %d", sx, sy);
    CHECK(rip_accel_speed(20, -5, WINDOW) == 51, "speed from those sums: %u",
          rip_accel_speed(20, -5, WINDOW));

    /* Only the newest RIP_ACCEL_HISTORY_LEN samples are kept. */
    s = (struct rip_accel_state){0};
    for (int i = 0; i < RIP_ACCEL_HISTORY_LEN + 10; i++) {
        rip_accel_record(&s, 5000, i % 2 == 0, (int16_t)(i < 10 ? 100 : 1));
    }
    rip_accel_window_sums(&s, 5000, WINDOW, &sx, &sy);
    CHECK(s.count == RIP_ACCEL_HISTORY_LEN && sx == RIP_ACCEL_HISTORY_LEN / 2 &&
              sy == RIP_ACCEL_HISTORY_LEN / 2,
          "ring wraps: count %u sums %d %d", s.count, sx, sy);

    /* The uint32 millisecond counter wrapping keeps working. */
    s = (struct rip_accel_state){0};
    rip_accel_record(&s, 0xFFFFFFF0u, true, 7);
    rip_accel_record(&s, 0x00000005u, false, 3);
    rip_accel_window_sums(&s, 0x00000008u, WINDOW, &sx, &sy);
    CHECK(sx == 7 && sy == 3, "time wrap: %d %d", sx, sy);
}

/* Feed `count` events of one axis spaced `spacing_ms` apart; return the total. */
static long feed(struct rip_accel_state *s, bool is_x, int16_t value, int count,
                 uint32_t spacing_ms, int min_percent, uint32_t *now) {
    long total = 0;
    for (int i = 0; i < count; i++, *now += spacing_ms) {
        total += rip_accel_apply_event(s, is_x, value, *now, WINDOW, min_percent, LOW, HIGH);
    }
    return total;
}

static void test_apply_event(void) {
    /* min 100 is the identity for any input and leaves no remainder. */
    struct rip_accel_state s = {0};
    uint32_t now = 1000;
    static const int16_t values[] = {1, -1, 7, -300, 32767, -32768, 2, 50};
    for (int round = 0; round < 20; round++) {
        for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); i++) {
            bool is_x = (i + round) % 2 == 0;
            int16_t out = rip_accel_apply_event(&s, is_x, values[i], now++, WINDOW, 100, LOW, HIGH);
            if (out != values[i] || s.remainder_x != 0 || s.remainder_y != 0) {
                CHECK(0, "min 100 changed %d to %d", values[i], out);
            }
        }
    }

    /* Slow 1-count events (alone in each window) get the minimum gain: N/2 at 50%. */
    s = (struct rip_accel_state){0};
    now = 1000;
    long total = feed(&s, true, 1, 1000, 100, 50, &now);
    CHECK(labs(total - 500) <= 1, "slow x at 50%%: total %ld", total);
    total = feed(&s, false, -1, 1000, 100, 50, &now);
    CHECK(labs(total + 500) <= 1, "slow -y at 50%%: total %ld", total);

    /* A steady 1 count every 6 ms is 16 counts per 100 ms: still the minimum. */
    s = (struct rip_accel_state){0};
    now = 1000;
    total = feed(&s, true, 1, 600, 6, 40, &now);
    CHECK(labs(total - 240) <= 1, "1 count / 6 ms at 40%%: total %ld", total);

    /* Fast motion keeps gain 1: 30 counts every 6 ms is 500 per 100 ms. */
    s = (struct rip_accel_state){0};
    now = 1000;
    (void)feed(&s, true, 30, 10, 6, 40, &now); /* ramp in */
    total = feed(&s, true, 30, 100, 6, 40, &now);
    CHECK(total == 3000, "fast stays 1:1: total %ld", total);
    /* A single large report alone is fast too. */
    s = (struct rip_accel_state){0};
    CHECK(rip_accel_apply_event(&s, true, -100, 5000, WINDOW, 20, LOW, HIGH) == -100,
          "single fast report");

    /* Jitter that goes back and forth stays slow even with large counts. */
    s = (struct rip_accel_state){0};
    now = 1000;
    total = 0;
    for (int i = 0; i < 200; i++, now += 3) {
        total += labs(rip_accel_apply_event(&s, true, i % 2 ? -6 : 6, now, WINDOW, 50, LOW, HIGH));
    }
    CHECK(total <= 200 * 3 + 2, "jitter got the minimum gain: |total| %ld", total);

    /* Diagonal uses hypot (speed_high 30 here): X 6 alone is 15 per 100 ms,
     * the minimum gain; Y 6 at the same time makes hypot 8.49, 21 per 100 ms,
     * just above speed_low. The larger axis alone (15) would keep the minimum
     * and the sum of both axes (30) would give gain 1. */
    s = (struct rip_accel_state){0};
    int16_t ox = rip_accel_apply_event(&s, true, 6, 1000, WINDOW, 50, LOW, 30);
    CHECK(ox == 3 && s.remainder_x == 0, "diagonal x at min: %d rem %d", ox, s.remainder_x);
    int16_t oy = rip_accel_apply_event(&s, false, 6, 1000, WINDOW, 50, LOW, 30);
    uint32_t g = rip_accel_gain_permille(50, 21, LOW, 30);
    CHECK(rip_accel_speed(6, 6, WINDOW) == 21 && g == 514 && oy == 3 && s.remainder_y == 84,
          "diagonal y: speed %u gain %u out %d rem %d", rip_accel_speed(6, 6, WINDOW), g, oy,
          s.remainder_y);

    /* Both axes of a diagonal are judged by the combined speed: a 45 degree
     * move fast enough overall keeps gain 1 on both axes, although either
     * axis alone (40 per 100 ms) would be below speed_high = 50. */
    CHECK(rip_accel_speed(16, -16, WINDOW) >= 50 && rip_accel_speed(16, 0, WINDOW) < 50,
          "diagonal speed setup");
    s = (struct rip_accel_state){0};
    now = 1000;
    long tx = 0;
    long ty = 0;
    for (int i = 0; i < 50; i++, now += 10) {
        tx += rip_accel_apply_event(&s, true, 4, now, WINDOW, 50, LOW, 50);
        ty += rip_accel_apply_event(&s, false, -4, now, WINDOW, 50, LOW, 50);
    }
    CHECK(tx >= 4 * 46 && ty <= -4 * 46 && tx == -ty, "diagonal fast: %ld %ld", tx, ty);

    /* Zero events are skipped: no output, no history slot, remainder kept. */
    s = (struct rip_accel_state){0};
    (void)rip_accel_apply_event(&s, true, 1, 1000, WINDOW, 50, LOW, HIGH);
    int32_t kept = s.remainder_x;
    CHECK(rip_accel_apply_event(&s, true, 0, 1001, WINDOW, 50, LOW, HIGH) == 0 && s.count == 1 &&
              s.remainder_x == kept,
          "zero skipped: count %u rem %d", s.count, s.remainder_x);

    /* Remainders are per axis. */
    s = (struct rip_accel_state){0};
    (void)rip_accel_apply_event(&s, true, 1, 1000, WINDOW, 50, LOW, HIGH);
    (void)rip_accel_apply_event(&s, false, -1, 1200, WINDOW, 50, LOW, HIGH);
    CHECK(s.remainder_x == 500 && s.remainder_y == -500, "per-axis remainders: %d %d",
          s.remainder_x, s.remainder_y);
}

int main(void) {
    test_clamp();
    test_isqrt();
    test_speed();
    test_gain_curve();
    test_apply_remainder();
    test_window_sums();
    test_apply_event();

    if (failures) {
        printf("accel_math_test: %d of %d checks FAILED\n", failures, checks);
        return 1;
    }
    printf("accel_math_test: all %d checks passed\n", checks);
    return 0;
}
