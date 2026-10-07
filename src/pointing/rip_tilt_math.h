/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Pure math for the runtime input processor's rotation + tilt compensation.
 *
 * Kept free of Zephyr dependencies so it can be unit-tested on the host
 * (see tests/host/tilt_math_test.c).
 *
 * Coordinate frame: the event codes' frame, x to the right and y down (HID).
 *
 * Rotation r keeps the module's historical sign convention:
 *   x' = x cos r - y sin r
 *   y' = x sin r + y cos r
 * so +90 degrees maps right to down, i.e. clockwise on screen.
 *
 * Tilt compensation boosts motion along one axis by k = 1 / cos(tilt). That
 * axis is given as a clock hand angle alpha, clockwise from 12 o'clock
 * (0 = up/down, +30 = 1-7 o'clock, +90 = left/right, -30 = 11-5 o'clock).
 * Its unit vector in the x-right / y-down frame is d = (sin alpha, -cos alpha)
 * and the tilt matrix is M_tilt = I + (k - 1) * d * d^T.
 *
 * The applied transform is M = M_tilt * R(r), stored in fixed point x1000.
 */

#pragma once

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* Never approach 90 degrees: there the sensor sees nothing and k diverges.
 * 60 degrees gives k = 2.0. */
#define RIP_TILT_MAX_DEGREES 60

/* Fixed-point scale of the transform matrix and of the remainders. */
#define RIP_TILT_FIXED_ONE 1000

#define RIP_TILT_PI 3.14159265358979323846

/* out_x = xx * x + xy * y; out_y = yx * x + yy * y (all x1000). */
struct rip_tilt_matrix {
    int32_t xx;
    int32_t xy;
    int32_t yx;
    int32_t yy;
};

/* Clamp a requested tilt to [0, RIP_TILT_MAX_DEGREES]. Takes int64_t so both
 * unsigned (RPC) and signed (devicetree) inputs keep their meaning. */
static inline uint8_t rip_tilt_clamp_degrees(int64_t degrees) {
    if (degrees < 0) {
        return 0;
    }
    if (degrees > RIP_TILT_MAX_DEGREES) {
        return RIP_TILT_MAX_DEGREES;
    }
    return (uint8_t)degrees;
}

/* The tilt axis is 180-degree periodic; normalize into (-90, 90]. */
static inline int16_t rip_tilt_normalize_axis_degrees(int32_t degrees) {
    int32_t axis = degrees % 180; /* (-180, 180) */
    if (axis > 90) {
        axis -= 180;
    } else if (axis <= -90) {
        axis += 180;
    }
    return (int16_t)axis;
}

/* Round half away from zero to the nearest x1000 fixed-point value. */
static inline int32_t rip_tilt_round_fixed(double value) {
    double scaled = value * RIP_TILT_FIXED_ONE;
    return scaled >= 0 ? (int32_t)(scaled + 0.5) : -(int32_t)(-scaled + 0.5);
}

static inline struct rip_tilt_matrix
rip_tilt_compute_matrix(int32_t rotation_degrees, int64_t tilt_degrees, int32_t tilt_axis_degrees) {
    /* Reduce first so multiples of 90 degrees stay exact after rounding. */
    double r = (double)(rotation_degrees % 360) * RIP_TILT_PI / 180.0;
    double t = (double)rip_tilt_clamp_degrees(tilt_degrees) * RIP_TILT_PI / 180.0;
    double a = (double)rip_tilt_normalize_axis_degrees(tilt_axis_degrees) * RIP_TILT_PI / 180.0;

    double cr = cos(r);
    double sr = sin(r);
    double gain = 1.0 / cos(t) - 1.0; /* k - 1 */
    double dx = sin(a);
    double dy = -cos(a);

    /* M_tilt = I + (k - 1) d d^T */
    double txx = 1.0 + gain * dx * dx;
    double txy = gain * dx * dy;
    double tyy = 1.0 + gain * dy * dy;

    /* M = M_tilt * R, with R = [[cr, -sr], [sr, cr]] */
    struct rip_tilt_matrix m = {
        .xx = rip_tilt_round_fixed(txx * cr + txy * sr),
        .xy = rip_tilt_round_fixed(-txx * sr + txy * cr),
        .yx = rip_tilt_round_fixed(txy * cr + tyy * sr),
        .yy = rip_tilt_round_fixed(-txy * sr + tyy * cr),
    };
    return m;
}

static inline bool rip_tilt_matrix_is_identity(const struct rip_tilt_matrix *m) {
    return m->xx == RIP_TILT_FIXED_ONE && m->xy == 0 && m->yx == 0 && m->yy == RIP_TILT_FIXED_ONE;
}

static inline bool rip_tilt_matrix_equal(const struct rip_tilt_matrix *a,
                                         const struct rip_tilt_matrix *b) {
    return a->xx == b->xx && a->xy == b->xy && a->yx == b->yx && a->yy == b->yy;
}

/*
 * One output axis: out = (remainder + m_a * a + m_b * b) / 1000, carrying the
 * signed sub-count remainder (x1000 units, |remainder| < 1000) to the next
 * call so slow movement is not lost to integer division. The result is
 * clamped to the int16 range; the remainder is dropped when clamping.
 */
static inline int16_t rip_tilt_apply_axis(int32_t m_a, int32_t m_b, int16_t a, int16_t b,
                                          int32_t *remainder) {
    int64_t numerator = (int64_t)*remainder + (int64_t)m_a * a + (int64_t)m_b * b;
    int64_t out = numerator / RIP_TILT_FIXED_ONE;
    int64_t rest = numerator - out * RIP_TILT_FIXED_ONE;

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
