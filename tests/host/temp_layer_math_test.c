/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Host-side unit test for src/pointing/rip_temp_layer_math.h (no Zephyr needed).
 *
 * Run from the repository root (test.py runs the same in CI):
 *   cc -std=c11 -Wall -Wextra -Werror -o /tmp/temp_layer_math_test \
 *     tests/host/temp_layer_math_test.c && /tmp/temp_layer_math_test
 */

#include <stdio.h>
#include <stdlib.h>

#include "../../src/pointing/rip_temp_layer_math.h"

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

static void test_clamp(void) {
    CHECK(rip_temp_layer_clamp_distance(-5) == 0, "negative clamps to 0");
    CHECK(rip_temp_layer_clamp_distance(0) == 0, "0 stays");
    CHECK(rip_temp_layer_clamp_distance(8) == 8, "8 stays");
    CHECK(rip_temp_layer_clamp_distance(100) == 100, "100 stays");
    CHECK(rip_temp_layer_clamp_distance(101) == 100, "101 clamps to 100");
    CHECK(rip_temp_layer_clamp_distance(300) == 100, "300 clamps to 100");
    CHECK(rip_temp_layer_clamp_distance(UINT32_MAX) == 100, "UINT32_MAX clamps to 100");
}

/* |dx| + |dy| of one burst: signs do not cancel. */
static void test_sum(void) {
    struct rip_temp_layer_distance d = {0};
    uint32_t t = 1000;
    CHECK(!rip_temp_layer_distance_add(&d, 3, t, 8), "3 < 8");
    CHECK(!rip_temp_layer_distance_add(&d, -4, t + 6, 8), "7 < 8");
    CHECK(d.total == 7, "total %u expected 7", d.total);
    CHECK(rip_temp_layer_distance_add(&d, 1, t + 12, 8), "8 reaches 8");
    /* Not reset by reaching it: the caller resets on activation. */
    CHECK(rip_temp_layer_distance_add(&d, -2, t + 18, 8), "still reached");
    CHECK(d.total == 10, "total %u expected 10", d.total);
    rip_temp_layer_distance_reset(&d);
    CHECK(d.total == 0, "reset clears");
    CHECK(!rip_temp_layer_distance_add(&d, 5, t + 24, 8), "counts again from 0");
}

/* Back-and-forth vibration adds up too, but only within one burst. */
static void test_gap(void) {
    struct rip_temp_layer_distance d = {0};
    uint32_t t = 5000;
    CHECK(!rip_temp_layer_distance_add(&d, 5, t, 8), "5 < 8");
    /* Exactly the gap: still the same burst. */
    CHECK(rip_temp_layer_distance_add(&d, -5, t + RIP_TEMP_LAYER_DISTANCE_GAP_MS, 8),
          "10 after a 150 ms pause");
    rip_temp_layer_distance_reset(&d);
    t += 1000;
    CHECK(!rip_temp_layer_distance_add(&d, 5, t, 8), "5 < 8");
    /* One ms more starts a new burst. */
    CHECK(!rip_temp_layer_distance_add(&d, 5, t + RIP_TEMP_LAYER_DISTANCE_GAP_MS + 1, 8),
          "a 151 ms pause starts over");
    CHECK(d.total == 5, "total %u expected 5", d.total);
    /* Slow but steady motion (one count every 100 ms) still adds up. */
    rip_temp_layer_distance_reset(&d);
    t += 1000;
    int reached_at = -1;
    for (int i = 0; i < 20 && reached_at < 0; i++) {
        if (rip_temp_layer_distance_add(&d, 1, t + (uint32_t)i * 100, 8)) {
            reached_at = i;
        }
    }
    CHECK(reached_at == 7, "steady 1 per 100 ms reached at event %d, expected 7", reached_at);
    /* Sparse jitter (one count every 200 ms) never does. */
    rip_temp_layer_distance_reset(&d);
    t += 10000;
    bool reached = false;
    for (int i = 0; i < 50; i++) {
        reached |= rip_temp_layer_distance_add(&d, i % 2 ? 1 : -1, t + (uint32_t)i * 200, 8);
    }
    CHECK(!reached, "isolated 1-count jitter every 200 ms never reaches 8");
}

/* The millisecond clock is the uptime cut to 32 bits: it may wrap. */
static void test_wrap(void) {
    struct rip_temp_layer_distance d = {0};
    uint32_t t = UINT32_MAX - 10;
    CHECK(!rip_temp_layer_distance_add(&d, 4, t, 8), "4 < 8 before the wrap");
    CHECK(rip_temp_layer_distance_add(&d, 4, t + 20, 8), "8 across the wrap (20 ms later)");
}

/* 0 is reached by any event, as the caller never counts with 0. Large values
 * neither overflow nor go negative. */
static void test_limits(void) {
    struct rip_temp_layer_distance d = {0};
    CHECK(rip_temp_layer_distance_add(&d, 0, 100, 0), "0 always reached");
    rip_temp_layer_distance_reset(&d);
    CHECK(rip_temp_layer_distance_add(&d, INT16_MIN, 100, 100), "INT16_MIN reaches 100");
    CHECK(d.total == 32768, "total %u expected 32768", d.total);
    d.total = UINT32_MAX - 3;
    CHECK(rip_temp_layer_distance_add(&d, 10, 100, 100), "saturated stays reached");
    CHECK(d.total == UINT32_MAX, "total saturates at UINT32_MAX");
}

int main(void) {
    test_clamp();
    test_sum();
    test_gap();
    test_wrap();
    test_limits();

    if (failures) {
        printf("temp_layer_math_test: %d of %d checks FAILED\n", failures, checks);
        return 1;
    }
    printf("temp_layer_math_test: all %d checks passed\n", checks);
    return 0;
}
