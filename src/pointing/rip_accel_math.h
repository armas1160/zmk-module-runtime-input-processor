/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Pure math for the runtime input processor's speed-dependent pointer
 * acceleration ("precision at low speed").
 *
 * Kept free of Zephyr dependencies so it can be unit-tested on the host
 * (see tests/host/accel_math_test.c). Integer only, no floats.
 *
 * Speed: the signed X and Y input of the last window_ms is summed per axis
 * from a short history of recent events, and
 *   speed = hypot(sum_x, sum_y) * 100 / window_ms
 * in input counts per 100 ms. Summing signed values first means sensor
 * jitter that goes back and forth does not count as speed, and a diagonal
 * counts its true length rather than the sum of both axes.
 *
 * Gain (permille): min_percent * 10 at or below speed_low, 1000 (unchanged)
 * at or above speed_high, and a smoothstep in between:
 *   t = clamp((speed - low) / (high - low), 0, 1)
 *   g = min + (1 - min) * (3 t^2 - 2 t^3)
 * When speed_high <= speed_low the curve is a step at speed_low.
 *
 * Output: value' = (value * g + remainder) / 1000 per axis, truncated toward
 * zero with the signed rest carried to that axis' next event, so slow 1-count
 * motion below gain 1 is slowed down, never lost.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* accel-min-percent range: 100 is off (gain 1 at every speed). */
#define RIP_ACCEL_MIN_PERCENT 20
#define RIP_ACCEL_OFF_PERCENT 100

/* Fixed-point scale of the gain and of the remainders (permille). */
#define RIP_ACCEL_GAIN_ONE 1000

/* Recent events kept for the speed estimate. A window longer than this many
 * events (both axes counted) sees only the newest ones. */
#define RIP_ACCEL_HISTORY_LEN 32

struct rip_accel_sample {
    uint32_t time_ms; /* k_uptime_get() truncated; compared by difference */
    int16_t value;
    bool is_x;
};

/* Per-processor speed history and per-axis remainders; zero-initialize. */
struct rip_accel_state {
    struct rip_accel_sample history[RIP_ACCEL_HISTORY_LEN];
    uint8_t head;        /* next slot to write */
    uint8_t count;       /* valid samples, up to RIP_ACCEL_HISTORY_LEN */
    int32_t remainder_x; /* permille, always within (-1000, 1000) */
    int32_t remainder_y;
};

/* Clamp a requested minimum gain to [RIP_ACCEL_MIN_PERCENT, RIP_ACCEL_OFF_PERCENT].
 * Takes int64_t so both unsigned (RPC) and signed (devicetree) inputs keep
 * their meaning. */
static inline uint8_t rip_accel_clamp_min_percent(int64_t percent) {
    if (percent < RIP_ACCEL_MIN_PERCENT) {
        return RIP_ACCEL_MIN_PERCENT;
    }
    if (percent > RIP_ACCEL_OFF_PERCENT) {
        return RIP_ACCEL_OFF_PERCENT;
    }
    return (uint8_t)percent;
}

/* floor(sqrt(n)), bit by bit (at most 32 rounds). */
static inline uint64_t rip_accel_isqrt(uint64_t n) {
    uint64_t root = 0;
    uint64_t bit = (uint64_t)1 << 62;
    while (bit > n) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (n >= root + bit) {
            n -= root + bit;
            root = (root >> 1) + bit;
        } else {
            root >>= 1;
        }
        bit >>= 2;
    }
    return root;
}

/* hypot(sum_x, sum_y) * 100 / window_ms, in counts per 100 ms (floored). */
static inline uint32_t rip_accel_speed(int32_t sum_x, int32_t sum_y, uint32_t window_ms) {
    if (window_ms == 0) {
        return 0;
    }
    uint64_t ax = (uint64_t)(sum_x < 0 ? -(int64_t)sum_x : (int64_t)sum_x);
    uint64_t ay = (uint64_t)(sum_y < 0 ? -(int64_t)sum_y : (int64_t)sum_y);
    uint64_t squares = ax * ax + ay * ay;
    /* sqrt(squares * 100^2) keeps the hypot's fraction; fall back for huge sums. */
    uint64_t length_x100 = squares <= UINT64_MAX / 10000 ? rip_accel_isqrt(squares * 10000)
                                                         : rip_accel_isqrt(squares) * 100;
    uint64_t speed = length_x100 / window_ms;
    return speed > UINT32_MAX ? UINT32_MAX : (uint32_t)speed;
}

/* Gain in permille for a speed; see the curve at the top of this file. */
static inline uint32_t rip_accel_gain_permille(int64_t min_percent, uint32_t speed,
                                               uint32_t speed_low, uint32_t speed_high) {
    uint32_t min_gain = (uint32_t)rip_accel_clamp_min_percent(min_percent) * 10;
    uint32_t t; /* 0..1000 */
    if (speed <= speed_low) {
        t = 0;
    } else if (speed >= speed_high) { /* also the step when high <= low */
        t = RIP_ACCEL_GAIN_ONE;
    } else {
        t = (uint32_t)((uint64_t)(speed - speed_low) * RIP_ACCEL_GAIN_ONE /
                       (speed_high - speed_low));
    }
    /* smoothstep 3t^2 - 2t^3 with t and the result in 0..1000 */
    uint32_t s = (uint32_t)((uint64_t)t * t * (3000 - 2 * t) / 1000000);
    return min_gain + (RIP_ACCEL_GAIN_ONE - min_gain) * s / RIP_ACCEL_GAIN_ONE;
}

/* (value * gain + remainder) / 1000, truncated toward zero; the signed rest
 * stays in remainder. Clamped to int16 (dropping the rest when clamping). */
static inline int16_t rip_accel_apply(int16_t value, uint32_t gain_permille, int32_t *remainder) {
    int64_t numerator = (int64_t)value * gain_permille + *remainder;
    int64_t out = numerator / RIP_ACCEL_GAIN_ONE;
    int64_t rest = numerator - out * RIP_ACCEL_GAIN_ONE;
    if (out > INT16_MAX) {
        out = INT16_MAX;
        rest = 0;
    } else if (out < INT16_MIN) {
        out = INT16_MIN;
        rest = 0;
    }
    *remainder = (int32_t)rest;
    return (int16_t)out;
}

static inline void rip_accel_record(struct rip_accel_state *state, uint32_t now_ms, bool is_x,
                                    int16_t value) {
    state->history[state->head] =
        (struct rip_accel_sample){.time_ms = now_ms, .value = value, .is_x = is_x};
    state->head = (uint8_t)((state->head + 1) % RIP_ACCEL_HISTORY_LEN);
    if (state->count < RIP_ACCEL_HISTORY_LEN) {
        state->count++;
    }
}

/* Sum the signed X and Y samples recorded less than window_ms before now_ms,
 * newest first (the history is in time order, so stop at the first older one). */
static inline void rip_accel_window_sums(const struct rip_accel_state *state, uint32_t now_ms,
                                         uint32_t window_ms, int32_t *sum_x, int32_t *sum_y) {
    int32_t sx = 0;
    int32_t sy = 0;
    uint8_t idx = state->head;
    for (uint8_t i = 0; i < state->count; i++) {
        idx = (uint8_t)((idx + RIP_ACCEL_HISTORY_LEN - 1) % RIP_ACCEL_HISTORY_LEN);
        const struct rip_accel_sample *sample = &state->history[idx];
        if ((uint32_t)(now_ms - sample->time_ms) >= window_ms) {
            break;
        }
        if (sample->is_x) {
            sx += sample->value;
        } else {
            sy += sample->value;
        }
    }
    *sum_x = sx;
    *sum_y = sy;
}

/*
 * Accelerate one single-axis event and return that axis' output value.
 *
 * The event is recorded first, so the speed includes it. Zero events carry no
 * motion and are skipped, so they take no history slot. min_percent 100 gives
 * gain 1000 at every speed, i.e. the identity (the processor skips the call
 * entirely then).
 */
static inline int16_t rip_accel_apply_event(struct rip_accel_state *state, bool is_x, int16_t value,
                                            uint32_t now_ms, uint32_t window_ms,
                                            int64_t min_percent, uint32_t speed_low,
                                            uint32_t speed_high) {
    if (value == 0) {
        return 0;
    }
    rip_accel_record(state, now_ms, is_x, value);

    int32_t sum_x;
    int32_t sum_y;
    rip_accel_window_sums(state, now_ms, window_ms, &sum_x, &sum_y);
    uint32_t gain = rip_accel_gain_permille(min_percent, rip_accel_speed(sum_x, sum_y, window_ms),
                                            speed_low, speed_high);
    return rip_accel_apply(value, gain, is_x ? &state->remainder_x : &state->remainder_y);
}
