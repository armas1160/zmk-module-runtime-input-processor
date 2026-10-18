/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Host-side unit test for src/pointing/rip_aim_math.h (no Zephyr needed).
 *
 * Run from the repository root (test.py runs the same in CI):
 *   cc -std=c11 -Wall -Wextra -Werror -o /tmp/aim_math_test tests/host/aim_math_test.c \
 *     && /tmp/aim_math_test
 */

#include <stdio.h>
#include <stdlib.h>

#include "../../src/pointing/rip_aim_math.h"

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

/* Devicetree defaults and the Kconfig default release interval. */
#define TAU 50
#define PERCENT 40
#define INTERVAL 15

/* Release on the timer until idle; returns the counts output, or a huge
 * number if it did not settle within max_ticks. */
static long drain(struct rip_aim_axis *a, uint32_t *now, int max_ticks) {
    long out = 0;
    for (int i = 0; i < max_ticks; i++) {
        if (rip_aim_idle(a)) {
            return out;
        }
        *now += INTERVAL;
        out += rip_aim_tick(a, *now, TAU);
    }
    return rip_aim_idle(a) ? out : 1000000000L;
}

static void test_clamp(void) {
    CHECK(rip_aim_clamp_percent(-5) == 10, "clamp -5");
    CHECK(rip_aim_clamp_percent(0) == 10, "clamp 0");
    CHECK(rip_aim_clamp_percent(9) == 10, "clamp 9");
    CHECK(rip_aim_clamp_percent(40) == 40, "clamp 40");
    CHECK(rip_aim_clamp_percent(100) == 100, "clamp 100");
    CHECK(rip_aim_clamp_percent(256) == 100, "clamp 256 must not wrap");
    CHECK(rip_aim_clamp_percent(4294967295LL) == 100, "clamp UINT32_MAX");

    CHECK(rip_aim_clamp_smoothing_ms(-1) == 0, "smoothing clamp -1");
    CHECK(rip_aim_clamp_smoothing_ms(0) == 0, "smoothing clamp 0");
    CHECK(rip_aim_clamp_smoothing_ms(50) == 50, "smoothing clamp 50");
    CHECK(rip_aim_clamp_smoothing_ms(200) == 200, "smoothing clamp 200");
    CHECK(rip_aim_clamp_smoothing_ms(201) == 200, "smoothing clamp 201");
    CHECK(rip_aim_clamp_smoothing_ms(456) == 200, "smoothing clamp 456 must not wrap");
    CHECK(rip_aim_clamp_smoothing_ms(4294967295LL) == 200, "smoothing clamp UINT32_MAX");
}

static void test_rest(void) {
    struct rip_aim_axis a = {0};
    CHECK(rip_aim_apply_event(&a, 0, 1000, PERCENT, TAU) == 0 && rip_aim_idle(&a) && a.frac == 0,
          "zero input stays idle");

    /* The first event from rest releases nothing, even after a long pause. */
    a = (struct rip_aim_axis){.last_ms = 5};
    CHECK(rip_aim_apply_event(&a, 10, 100000, PERCENT, TAU) == 0 && a.pending == 4000 &&
              a.last_ms == 100000,
          "first event from rest: pending %d last %u", a.pending, a.last_ms);

    /* The next one 6 ms later releases 6 / 56 of what is pending. */
    int16_t out = rip_aim_apply_event(&a, 10, 100006, PERCENT, TAU);
    CHECK(out == 0 && a.frac == 857 && a.pending == 8000 - 857, "second event: out %d frac %d",
          out, a.frac);
}

static void test_passthrough(void) {
    /* tau 0 and 100 % is the identity. */
    struct rip_aim_axis a = {0};
    uint32_t now = 1000;
    for (int v = -300; v <= 300; v += 7, now += 6) {
        int16_t out = rip_aim_apply_event(&a, (int16_t)v, now, 100, 0);
        if (out != v || !rip_aim_idle(&a) || a.frac != 0) {
            CHECK(0, "identity: %d -> %d", v, out);
            return;
        }
    }
    CHECK(1, "identity");

    /* tau 0 is no smoothing at all: each event is output at once, slowed
     * only, with nothing left pending for the timer (no divide by zero). */
    a = (struct rip_aim_axis){0};
    now = 1000;
    long in = 0;
    long out_total = 0;
    for (int v = -50; v <= 60; v += 3, now += 6) {
        int16_t out = rip_aim_apply_event(&a, (int16_t)v, now, PERCENT, 0);
        in += v;
        out_total += out;
        if (!rip_aim_idle(&a)) {
            CHECK(0, "tau 0 left %d pending after %d", a.pending, v);
            return;
        }
    }
    long expected = in * PERCENT * 10;
    long got = out_total * 1000 + a.frac;
    CHECK(got == expected, "tau 0 output %ld milli expected %ld", got, expected);
    CHECK(rip_aim_tick(&a, now + 100, 0) == 0 && rip_aim_idle(&a), "tau 0 tick after input");
}

/* Everything added comes out (in milli-counts, with the fraction in frac). */
static void test_conservation(void) {
    srand(1234);
    for (int run = 0; run < 200; run++) {
        struct rip_aim_axis a = {0};
        uint32_t now = (uint32_t)rand();
        int percent = 10 + rand() % 91;
        long added = 0;
        long out = 0;
        int events = 1 + rand() % 200;
        for (int i = 0; i < events; i++) {
            now += (uint32_t)(rand() % 25);
            int16_t v = (int16_t)(rand() % 41 - 20);
            added += (long)v * percent * 10;
            out += rip_aim_apply_event(&a, v, now, percent, TAU);
        }
        long drained = drain(&a, &now, 200);
        out += drained;
        if (drained >= 1000000000L || out * 1000 + a.frac != added || a.frac <= -1000 ||
            a.frac >= 1000) {
            CHECK(0, "run %d: added %ld out %ld frac %d pending %d", run, added, out, a.frac,
                  a.pending);
            return;
        }
    }
    CHECK(1, "conservation");
}

static void test_symmetry(void) {
    struct rip_aim_axis p = {0};
    struct rip_aim_axis n = {0};
    uint32_t now = 1000;
    for (int i = 0; i < 60; i++, now += 6) {
        int16_t v = (int16_t)((i * 7) % 5);
        int16_t op = rip_aim_apply_event(&p, v, now, PERCENT, TAU);
        int16_t on = rip_aim_apply_event(&n, (int16_t)-v, now, PERCENT, TAU);
        if (op != -on || p.pending != -n.pending || p.frac != -n.frac) {
            CHECK(0, "mirror at %d: %d %d", i, op, on);
            return;
        }
    }
    CHECK(1, "mirror");
}

/* Slow steady motion trails by about speed * tau and is all there after the
 * thumb stops. */
static void test_steady_motion(void) {
    struct rip_aim_axis a = {0};
    uint32_t now = 1000;
    long out = 0;
    for (int i = 0; i < 50; i++, now += 6) {
        out += rip_aim_apply_event(&a, 2, now, 100, TAU);
    }
    /* 100 counts in, speed 1/3 count per ms: about 19 counts behind. */
    CHECK(out >= 100 - 25 && out <= 100 - 10, "lag: out %ld of 100", out);
    long rest = drain(&a, &now, 40);
    CHECK(out + rest == 100, "settles to 100: %ld + %ld", out, rest);
}

/* A 10 Hz back-and-forth (the size of a thumb tremor) is damped to under a
 * third, before the slowdown. */
static void test_tremor(void) {
    for (int pass = 0; pass < 2; pass++) {
        int percent = pass == 0 ? 100 : PERCENT;
        struct rip_aim_axis a = {0};
        uint32_t now = 1000;
        long pos = 0;
        long lo = 0;
        long hi = 0;
        /* 8 events of +1 then 8 of -1, every 6 ms: 96 ms period, 8 counts peak to peak. */
        for (int i = 0; i < 400; i++, now += 6) {
            int16_t v = (i / 8) % 2 == 0 ? 1 : -1;
            pos += rip_aim_apply_event(&a, v, now, percent, TAU);
            if (i == 160) {
                lo = hi = pos;
            } else if (i > 160) {
                lo = pos < lo ? pos : lo;
                hi = pos > hi ? pos : hi;
            }
        }
        long limit = pass == 0 ? 3 : 2;
        CHECK(hi - lo <= limit, "tremor at %d%%: %ld peak to peak (raw 8)", percent, hi - lo);
    }
}

static void test_release_steps(void) {
    /* Only time passing releases: dt 0 releases nothing. */
    struct rip_aim_axis a = {.pending = 5000, .last_ms = 100};
    CHECK(rip_aim_tick(&a, 100, TAU) == 0 && a.pending == 5000, "dt 0: pending %d", a.pending);

    /* A long pause releases at most RIP_AIM_DT_MAX_MS worth. */
    a = (struct rip_aim_axis){.pending = 8000, .last_ms = 100};
    int16_t out = rip_aim_tick(&a, 100100, TAU);
    CHECK(out == 3 && a.frac == 0 && a.pending == 5000, "capped step: out %d pending %d", out,
          a.pending);

    /* Small leftovers still finish: at least 1 milli-count, then all at the snap. */
    a = (struct rip_aim_axis){.pending = 60, .last_ms = 0};
    (void)rip_aim_tick(&a, 1, 500);
    CHECK(a.pending == 59, "minimum progress: %d", a.pending);
    a = (struct rip_aim_axis){.pending = -50, .frac = -950, .last_ms = 0};
    out = rip_aim_tick(&a, 1, TAU);
    CHECK(out == -1 && a.pending == 0 && a.frac == 0, "snap: out %d frac %d", out, a.frac);

    /* Draining is monotonic and quick. */
    a = (struct rip_aim_axis){0};
    uint32_t now = 1000;
    (void)rip_aim_apply_event(&a, 40, now, 100, TAU);
    int ticks = 0;
    int32_t prev = a.pending;
    long total = 0;
    while (!rip_aim_idle(&a) && ticks < 100) {
        now += INTERVAL;
        total += rip_aim_tick(&a, now, TAU);
        if (a.pending > prev || a.pending < 0) {
            CHECK(0, "drain not monotonic at tick %d: %d -> %d", ticks, prev, a.pending);
            return;
        }
        prev = a.pending;
        ticks++;
    }
    CHECK(total == 40 && ticks <= 40, "drain: %ld counts in %d ticks", total, ticks);
}

static void test_int16_limit(void) {
    struct rip_aim_axis a = {0};
    uint32_t now = 1000;
    long out = 0;
    for (int i = 0; i < 4; i++, now += 6) {
        int16_t o = rip_aim_apply_event(&a, INT16_MAX, now, 100, 0);
        CHECK(o == INT16_MAX, "clamped output %d", o);
        out += o;
    }
    CHECK(a.pending == 0 && out == 4L * INT16_MAX, "nothing left over");

    /* Over the int16 range, the excess waits in pending instead of being dropped. */
    a = (struct rip_aim_axis){.pending = 40000 * 1000, .last_ms = 0};
    int16_t o1 = rip_aim_release(&a, 10, 0);
    int16_t o2 = rip_aim_release(&a, 10, 0);
    CHECK(o1 == INT16_MAX && o2 == 40000 - INT16_MAX && a.pending == 0 && a.frac == 0,
          "excess carried: %d %d", o1, o2);

    /* Pending saturates instead of overflowing. */
    a = (struct rip_aim_axis){.pending = RIP_AIM_PENDING_MAX - 10};
    (void)rip_aim_apply_event(&a, INT16_MAX, now, 100, 1000000);
    CHECK(a.pending <= RIP_AIM_PENDING_MAX && a.pending > 0, "saturated: %d", a.pending);
}

int main(void) {
    test_clamp();
    test_rest();
    test_passthrough();
    test_conservation();
    test_symmetry();
    test_steady_motion();
    test_tremor();
    test_release_steps();
    test_int16_limit();

    if (failures) {
        printf("aim_math_test: %d of %d checks FAILED\n", failures, checks);
        return 1;
    }
    printf("aim_math_test: all %d checks passed\n", checks);
    return 0;
}
