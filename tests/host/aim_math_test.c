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

/* --- Normal smoothing (rip_smooth_*) ------------------------------------ */

/* The event and release-work logic of the runtime before normal smoothing,
 * kept here as the reference that smoothing 0 must reproduce exactly. The
 * work releases an axis that has had no event for INTERVAL ms and drops the
 * rest below one count once nothing is held back after release. */
static int16_t ref_event(struct rip_aim_axis *a, int16_t v, uint32_t now, bool held) {
    if (!held && rip_aim_idle(a)) {
        return v;
    }
    if (held) {
        return rip_aim_apply_event(a, v, now, PERCENT, TAU);
    }
    int32_t sum = (int32_t)v + rip_aim_tick(a, now, TAU);
    if (rip_aim_idle(a)) {
        a->frac = 0;
    }
    return (int16_t)(sum > INT16_MAX ? INT16_MAX : (sum < INT16_MIN ? INT16_MIN : sum));
}

static int16_t ref_work(struct rip_aim_axis *a, uint32_t now, bool held) {
    int16_t out = 0;
    if (!rip_aim_idle(a) && now - a->last_ms >= INTERVAL) {
        out = rip_aim_tick(a, now, TAU);
    }
    if (rip_aim_idle(a) && !held) {
        a->frac = 0;
    }
    return out;
}

/* The same through the new helpers, with normal smoothing ntau. */
static int16_t new_event(struct rip_aim_axis *a, int16_t v, uint32_t now, bool held,
                         uint32_t ntau) {
    if (rip_smooth_passthrough(a, held, ntau)) {
        return v;
    }
    return rip_smooth_event(a, v, now, held, PERCENT, TAU, ntau);
}

static int16_t new_work(struct rip_aim_axis *a, uint32_t now, bool held, uint32_t ntau) {
    int16_t out = 0;
    if (!rip_aim_idle(a) && now - a->last_ms >= INTERVAL) {
        out = rip_aim_tick(a, now, rip_smooth_tau(held, TAU, ntau));
    }
    if (rip_aim_idle(a) && !held) {
        a->frac = 0;
    }
    return out;
}

static void test_smooth_tau(void) {
    CHECK(rip_smooth_tau(true, 50, 80) == 50, "held uses aim tau");
    CHECK(rip_smooth_tau(true, 50, 0) == 50, "held, normal off");
    CHECK(rip_smooth_tau(false, 50, 80) == 80, "not held uses normal tau");
    CHECK(rip_smooth_tau(false, 50, 0) == 50, "normal off drains at aim tau");

    struct rip_aim_axis a = {0};
    CHECK(rip_smooth_passthrough(&a, false, 0), "idle, not held, off: passthrough");
    CHECK(!rip_smooth_passthrough(&a, true, 0), "held: no passthrough");
    CHECK(!rip_smooth_passthrough(&a, false, 30), "normal smoothing: no passthrough");
    a.pending = 1;
    CHECK(!rip_smooth_passthrough(&a, false, 0), "held back: no passthrough");
}

/* Smoothing 0 gives exactly the old outputs and states, for any mix of aim
 * presses, events, pauses and release work. */
static void test_smooth_off_matches_old(void) {
    srand(4321);
    for (int run = 0; run < 300; run++) {
        struct rip_aim_axis r = {0};
        struct rip_aim_axis n = {0};
        uint32_t now = (uint32_t)rand();
        bool held = false;
        for (int i = 0; i < 300; i++) {
            int op = rand() % 10;
            int16_t ro = 0;
            int16_t no = 0;
            if (op == 0) {
                held = !held;
            } else if (op <= 2) {
                now += (uint32_t)(rand() % 40);
                ro = ref_work(&r, now, held);
                no = new_work(&n, now, held, 0);
            } else {
                now += (uint32_t)(rand() % 12);
                int16_t v = (int16_t)(rand() % 9 == 0 ? rand() % 2001 - 1000 : rand() % 41 - 20);
                ro = ref_event(&r, v, now, held);
                no = new_event(&n, v, now, held, 0);
            }
            if (ro != no || r.pending != n.pending || r.frac != n.frac || r.last_ms != n.last_ms) {
                CHECK(0, "run %d op %d: old %d/%d/%d new %d/%d/%d", run, i, ro, r.pending, r.frac,
                      no, n.pending, n.frac);
                return;
            }
        }
    }
    CHECK(1, "smoothing 0 matches the old path");
}

/* Normal smoothing holds motion back like aim, at full speed, and lets it
 * all out once the thumb stops. */
static void test_smooth_normal(void) {
    struct rip_aim_axis a = {0};
    uint32_t now = 1000;
    CHECK(new_event(&a, 10, now, false, 80) == 0 && a.pending == 10000,
          "first event from rest is held back: pending %d", a.pending);
    long out = 0;
    for (int i = 0; i < 200 && !rip_aim_idle(&a); i++) {
        now += INTERVAL;
        out += new_work(&a, now, false, 80);
    }
    CHECK(out == 10 && rip_aim_idle(&a), "all 10 counts come out: %ld", out);

    /* Steady motion at full speed trails by about speed * tau. */
    a = (struct rip_aim_axis){0};
    out = 0;
    for (int i = 0; i < 50; i++, now += 6) {
        out += new_event(&a, 2, now, false, TAU);
    }
    CHECK(out >= 100 - 25 && out <= 100 - 10, "normal lag: out %ld of 100", out);
}

/* Switching between aim and normal smoothing neither drops nor repeats what
 * is held back: everything added (aim input slowed, normal input at full
 * speed) comes out once both have drained. */
static void test_smooth_switch_conserves(void) {
    srand(777);
    for (int run = 0; run < 300; run++) {
        struct rip_aim_axis a = {0};
        uint32_t now = (uint32_t)rand();
        uint32_t ntau = 1 + (uint32_t)(rand() % RIP_AIM_MAX_SMOOTHING_MS);
        bool held = rand() % 2;
        long added = 0;
        long out = 0;
        for (int i = 0; i < 200; i++) {
            if (rand() % 15 == 0) {
                held = !held;
            }
            now += (uint32_t)(rand() % 20);
            if (rand() % 4 == 0) {
                if (!rip_aim_idle(&a) && now - a.last_ms >= INTERVAL) {
                    out += rip_aim_tick(&a, now, rip_smooth_tau(held, TAU, ntau));
                }
                continue;
            }
            int16_t v = (int16_t)(rand() % 41 - 20);
            added += (long)v * (held ? PERCENT : 100) * 10;
            out += rip_smooth_event(&a, v, now, held, PERCENT, TAU, ntau);
        }
        held = false;
        for (int i = 0; i < 400 && !rip_aim_idle(&a); i++) {
            now += INTERVAL;
            out += rip_aim_tick(&a, now, rip_smooth_tau(held, TAU, ntau));
        }
        if (!rip_aim_idle(&a) || out * 1000 + a.frac != added) {
            CHECK(0, "run %d: added %ld out %ld frac %d pending %d", run, added, out, a.frac,
                  a.pending);
            return;
        }
    }
    CHECK(1, "aim <-> normal switching conserves");
}

/* Turning normal smoothing off while it holds motion back: later events pass
 * at once and what was held back still drains (at aim's pace); nothing is
 * repeated and at most the last sub-count rest is dropped. */
static void test_smooth_turned_off(void) {
    struct rip_aim_axis a = {0};
    uint32_t now = 1000;
    long added = 0;
    long out = 0;
    for (int i = 0; i < 10; i++, now += 6) {
        added += 7 * 1000;
        out += rip_smooth_event(&a, 7, now, false, PERCENT, TAU, 120);
    }
    CHECK(!rip_aim_idle(&a), "normal smoothing holds some back: pending %d", a.pending);
    for (int i = 0; i < 10; i++, now += 6) {
        added += 3 * 1000;
        out += new_event(&a, 3, now, false, 0);
    }
    for (int i = 0; i < 200 && !rip_aim_idle(&a); i++) {
        now += INTERVAL;
        out += new_work(&a, now, false, 0);
    }
    long diff = added - out * 1000;
    CHECK(rip_aim_idle(&a) && diff >= 0 && diff < 1000, "turned off: added %ld out %ld", added,
          out * 1000);
}

/* --- Per-key aim speed (rip_aim_keys_*) --------------------------------- */

static void test_aim_key_percent(void) {
    CHECK(rip_aim_key_percent(0) == 0, "0 stays default");
    CHECK(rip_aim_key_percent(1) == 10, "1 clamps to 10");
    CHECK(rip_aim_key_percent(9) == 10, "9 clamps to 10");
    CHECK(rip_aim_key_percent(10) == 10, "10");
    CHECK(rip_aim_key_percent(55) == 55, "55");
    CHECK(rip_aim_key_percent(100) == 100, "100");
    CHECK(rip_aim_key_percent(101) == 100, "101 clamps to 100");
    CHECK(rip_aim_key_percent(4294967295u) == 100, "UINT32_MAX clamps to 100");
    CHECK(rip_aim_effective_percent(0, 40) == 40, "default uses the processor's speed");
    CHECK(rip_aim_effective_percent(25, 40) == 25, "a key's own speed wins");
}

/* The newest held key decides; releasing it falls back to the newest one
 * still held, whatever order they are released in. */
static void test_aim_keys_order(void) {
    struct rip_aim_keys k = {0};
    CHECK(rip_aim_keys_percent(&k) == 0, "nothing held: default");
    CHECK(rip_aim_keys_press(&k, 13, 0) && rip_aim_keys_percent(&k) == 0, "F (default)");
    CHECK(rip_aim_keys_press(&k, 3, 20) && rip_aim_keys_percent(&k) == 20, "R at 20");
    CHECK(rip_aim_keys_press(&k, 23, 70) && rip_aim_keys_percent(&k) == 70, "V at 70");
    /* Release the middle one: the newest (V) still decides. */
    CHECK(rip_aim_keys_release(&k, 3) && rip_aim_keys_percent(&k) == 70, "R up: still 70");
    /* Release the newest: back to F, the newest still held. */
    CHECK(rip_aim_keys_release(&k, 23) && rip_aim_keys_percent(&k) == 0, "V up: back to default");
    CHECK(rip_aim_keys_press(&k, 3, 5) && rip_aim_keys_percent(&k) == 10, "R again at 5 -> 10");
    CHECK(rip_aim_keys_release(&k, 13) && rip_aim_keys_percent(&k) == 10, "F up: R stays");
    CHECK(rip_aim_keys_release(&k, 3) && k.count == 0 && rip_aim_keys_percent(&k) == 0,
          "all up: default");

    /* A release that did not press is ignored and changes nothing. */
    CHECK(rip_aim_keys_press(&k, 13, 30), "F at 30");
    CHECK(!rip_aim_keys_release(&k, 14) && k.count == 1 && rip_aim_keys_percent(&k) == 30,
          "stray release ignored");
    CHECK(rip_aim_keys_release(&k, 13) && !rip_aim_keys_release(&k, 13) && k.count == 0,
          "second release of the same key ignored");

    /* Full: the press is refused, so its release is ignored too. */
    k = (struct rip_aim_keys){0};
    for (uint32_t p = 0; p < RIP_AIM_KEYS_MAX; p++) {
        (void)rip_aim_keys_press(&k, p, 10 + p);
    }
    CHECK(!rip_aim_keys_press(&k, 99, 50) && rip_aim_keys_percent(&k) == 10 + RIP_AIM_KEYS_MAX - 1,
          "full: refused");
    CHECK(!rip_aim_keys_release(&k, 99) && k.count == RIP_AIM_KEYS_MAX, "refused key's release");

    /* The same position twice (e.g. two layers' bindings): the newest goes first. */
    k = (struct rip_aim_keys){0};
    (void)rip_aim_keys_press(&k, 13, 20);
    (void)rip_aim_keys_press(&k, 13, 60);
    CHECK(rip_aim_keys_release(&k, 13) && rip_aim_keys_percent(&k) == 20, "duplicate: newest out");
}

/* Changing the speed while aiming (another key pressed or released) only
 * scales what comes in from then on: nothing held back jumps or is lost. */
static void test_aim_speed_change_conserves(void) {
    srand(2468);
    for (int run = 0; run < 300; run++) {
        struct rip_aim_axis a = {0};
        uint32_t now = (uint32_t)rand();
        uint32_t ntau = (uint32_t)(rand() % 3) * 40;
        long added = 0;
        long out = 0;
        int percent = 40;
        for (int i = 0; i < 200; i++) {
            if (rand() % 8 == 0) {
                percent =
                    rip_aim_effective_percent(rip_aim_key_percent((uint32_t)(rand() % 110)), 40);
            }
            now += (uint32_t)(rand() % 20);
            if (rand() % 4 == 0) {
                if (!rip_aim_idle(&a) && now - a.last_ms >= INTERVAL) {
                    out += rip_aim_tick(&a, now, rip_smooth_tau(true, TAU, ntau));
                }
                continue;
            }
            int16_t v = (int16_t)(rand() % 41 - 20);
            added += (long)v * percent * 10;
            out += rip_smooth_event(&a, v, now, true, percent, TAU, ntau);
        }
        for (int i = 0; i < 400 && !rip_aim_idle(&a); i++) {
            now += INTERVAL;
            out += rip_aim_tick(&a, now, TAU);
        }
        if (!rip_aim_idle(&a) || out * 1000 + a.frac != added) {
            CHECK(0, "run %d: added %ld out %ld frac %d", run, added, out, a.frac);
            return;
        }
    }
    CHECK(1, "speed changes while aiming conserve");
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
    test_smooth_tau();
    test_smooth_off_matches_old();
    test_smooth_normal();
    test_smooth_switch_conserves();
    test_smooth_turned_off();
    test_aim_key_percent();
    test_aim_keys_order();
    test_aim_speed_change_conserves();

    if (failures) {
        printf("aim_math_test: %d of %d checks FAILED\n", failures, checks);
        return 1;
    }
    printf("aim_math_test: all %d checks passed\n", checks);
    return 0;
}
