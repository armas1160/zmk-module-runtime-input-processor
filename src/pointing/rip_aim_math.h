/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Pure math for the runtime input processor's aim mode: while an aim key is
 * held, cursor output is slowed down and low-pass filtered, so the small fast
 * back-and-forth of a trembling thumb cancels out while deliberate motion
 * still arrives, a little later. Normal smoothing (end of this file) runs the
 * same filter at full speed while no aim key is held.
 *
 * Kept free of Zephyr dependencies so it can be unit-tested on the host
 * (see tests/host/aim_math_test.c). Integer only, no floats.
 *
 * Units are milli-counts (1/1000 of an output count). Per output axis,
 * `pending` holds input received but not released yet and `frac` the released
 * part below one whole count. Input adds value * percent / 100 to pending;
 * each step of dt ms releases
 *   d = pending * dt / (tau + dt)
 * which is a first-order low-pass (backward Euler, stable for any dt) with
 * time constant tau: motion faster than about 1 / (2 pi tau) (3 Hz at 50 ms)
 * is damped, physiological tremor (8-12 Hz) to about a third.
 *
 * Nothing is lost: a step releases at least 1 milli-count, and the rest at
 * once when it is within RIP_AIM_SNAP_MILLI, so pending always reaches zero.
 * Released milli-counts become whole counts with the signed rest carried in
 * frac, so once pending is zero, everything added has been output except a
 * fraction below one count. Because frac is signed, the output only moves
 * when the filtered position gets a whole count away from it: a filtered
 * back-and-forth under 2 counts peak to peak leaves the cursor still.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* aim-scale-percent range. */
#define RIP_AIM_MIN_PERCENT 10
#define RIP_AIM_MAX_PERCENT 100

/* aim-smoothing-ms range. 0 releases everything at once (no smoothing, only
 * the slowdown); the top keeps the cursor lag at a usable fraction of a
 * second and the value in one byte. */
#define RIP_AIM_MAX_SMOOTHING_MS 200

#define RIP_AIM_MILLI 1000

/* Pending at or below this many milli-counts is released whole. */
#define RIP_AIM_SNAP_MILLI 50

/* Longest step: after a pause the first step releases at most this much
 * time's worth instead of everything at once. */
#define RIP_AIM_DT_MAX_MS 30

/* Pending is clamped here so additions can never overflow. */
#define RIP_AIM_PENDING_MAX (INT32_MAX / 4)

/* One output axis; zero-initialize. */
struct rip_aim_axis {
    int32_t pending;  /* milli-counts not released yet */
    int32_t frac;     /* released milli-counts not output yet, within (-1000, 1000) */
    uint32_t last_ms; /* time of the last step (k_uptime_get() truncated) */
};

/* Clamp a requested slowdown to [RIP_AIM_MIN_PERCENT, RIP_AIM_MAX_PERCENT]. */
static inline uint8_t rip_aim_clamp_percent(int64_t percent) {
    if (percent < RIP_AIM_MIN_PERCENT) {
        return RIP_AIM_MIN_PERCENT;
    }
    if (percent > RIP_AIM_MAX_PERCENT) {
        return RIP_AIM_MAX_PERCENT;
    }
    return (uint8_t)percent;
}

/* Clamp a requested smoothing time constant to [0, RIP_AIM_MAX_SMOOTHING_MS]. */
static inline uint8_t rip_aim_clamp_smoothing_ms(int64_t ms) {
    if (ms < 0) {
        return 0;
    }
    if (ms > RIP_AIM_MAX_SMOOTHING_MS) {
        return RIP_AIM_MAX_SMOOTHING_MS;
    }
    return (uint8_t)ms;
}

static inline bool rip_aim_idle(const struct rip_aim_axis *axis) { return axis->pending == 0; }

/*
 * Release one step of dt_ms and return the whole counts to output. Whole
 * counts beyond the int16 range stay in pending for the next step.
 */
static inline int16_t rip_aim_release(struct rip_aim_axis *axis, uint32_t dt_ms, uint32_t tau_ms) {
    int64_t pending = axis->pending;
    int64_t d;
    if (pending == 0) {
        d = 0;
    } else if (tau_ms == 0 || (pending <= RIP_AIM_SNAP_MILLI && pending >= -RIP_AIM_SNAP_MILLI)) {
        d = pending;
    } else {
        d = pending * dt_ms / ((int64_t)tau_ms + dt_ms);
        if (d == 0 && dt_ms > 0) {
            d = pending > 0 ? 1 : -1;
        }
    }

    int64_t acc = (int64_t)axis->frac + d;
    int64_t out = acc / RIP_AIM_MILLI;
    int64_t rest = acc - out * RIP_AIM_MILLI;
    if (out > INT16_MAX) {
        d -= (out - INT16_MAX) * RIP_AIM_MILLI;
        out = INT16_MAX;
    } else if (out < INT16_MIN) {
        d -= (out - INT16_MIN) * RIP_AIM_MILLI;
        out = INT16_MIN;
    }
    axis->pending = (int32_t)(pending - d);
    axis->frac = (int32_t)rest;
    return (int16_t)out;
}

/* Release what is due since this axis' last step (capped at
 * RIP_AIM_DT_MAX_MS) and return the whole counts to output. */
static inline int16_t rip_aim_tick(struct rip_aim_axis *axis, uint32_t now_ms, uint32_t tau_ms) {
    uint32_t dt = now_ms - axis->last_ms;
    if (dt > RIP_AIM_DT_MAX_MS) {
        dt = RIP_AIM_DT_MAX_MS;
    }
    axis->last_ms = now_ms;
    return rip_aim_release(axis, dt, tau_ms);
}

/*
 * Aim one single-axis event: add value * percent / 100, then release what is
 * due and return the whole counts to output. Motion that starts from rest
 * releases nothing on its first event, so the filter starts from the cursor's
 * resting position instead of from a stale timestamp.
 */
static inline int16_t rip_aim_apply_event(struct rip_aim_axis *axis, int16_t value,
                                          uint32_t now_ms, int64_t percent, uint32_t tau_ms) {
    if (axis->pending == 0) {
        axis->last_ms = now_ms;
    }
    int64_t pending = (int64_t)axis->pending +
                      (int64_t)value * rip_aim_clamp_percent(percent) * (RIP_AIM_MILLI / 100);
    if (pending > RIP_AIM_PENDING_MAX) {
        pending = RIP_AIM_PENDING_MAX;
    } else if (pending < -RIP_AIM_PENDING_MAX) {
        pending = -RIP_AIM_PENDING_MAX;
    }
    axis->pending = (int32_t)pending;
    return rip_aim_tick(axis, now_ms, tau_ms);
}

/*
 * Normal smoothing: the same filter while no aim key is held, at full speed
 * and with its own time constant (smoothing-ms, 0 = off). Both modes share one
 * pending/frac per axis, so switching between them neither drops nor repeats
 * what is held back; it is only released at the new mode's pace. With normal
 * smoothing off, an event passes unchanged plus whatever aim still holds back,
 * which drains at aim's pace, exactly as before normal smoothing existed.
 */

/* Time constant that releases what is held back: aim's while an aim key is
 * held, the normal one otherwise, and aim's again (draining what aim left)
 * while normal smoothing is off. */
static inline uint32_t rip_smooth_tau(bool aim_held, uint32_t aim_tau_ms, uint32_t normal_tau_ms) {
    return (aim_held || normal_tau_ms == 0) ? aim_tau_ms : normal_tau_ms;
}

/* True when an event passes untouched: no aim key held, normal smoothing off
 * and nothing held back. */
static inline bool rip_smooth_passthrough(const struct rip_aim_axis *axis, bool aim_held,
                                          uint32_t normal_tau_ms) {
    return !aim_held && normal_tau_ms == 0 && rip_aim_idle(axis);
}

/*
 * Filter one single-axis event that is not a passthrough one and return the
 * whole counts to output: slowed and smoothed while aim is held, smoothed at
 * full speed with normal smoothing, otherwise the event plus what aim still
 * holds back (the sub-count rest is dropped once nothing is held back).
 */
static inline int16_t rip_smooth_event(struct rip_aim_axis *axis, int16_t value, uint32_t now_ms,
                                       bool aim_held, int64_t aim_percent, uint32_t aim_tau_ms,
                                       uint32_t normal_tau_ms) {
    if (aim_held) {
        return rip_aim_apply_event(axis, value, now_ms, aim_percent, aim_tau_ms);
    }
    if (normal_tau_ms > 0) {
        return rip_aim_apply_event(axis, value, now_ms, RIP_AIM_MAX_PERCENT, normal_tau_ms);
    }
    int32_t sum = (int32_t)value + rip_aim_tick(axis, now_ms, aim_tau_ms);
    if (rip_aim_idle(axis)) {
        axis->frac = 0;
    }
    if (sum > INT16_MAX) {
        return INT16_MAX;
    }
    if (sum < INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)sum;
}
