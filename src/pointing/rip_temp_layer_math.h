/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/*
 * Pure math for the temp layer's ("auto mouse") activation distance: the
 * layer only activates once the motion of one burst adds up to this many
 * output counts (|dx| + |dy| after scaling, about cursor pixels), so a
 * vibration or a hand brushing the ball does not switch layers. A pause of
 * more than RIP_TEMP_LAYER_DISTANCE_GAP_MS ends the burst and starts the
 * count over; the caller also starts it over on a key press and when the
 * layer activates or deactivates.
 *
 * Kept free of Zephyr dependencies so it can be unit-tested on the host
 * (see tests/host/temp_layer_math_test.c).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* temp-layer-activation-distance range; 0 activates on any motion. */
#define RIP_TEMP_LAYER_MAX_ACTIVATION_DISTANCE 100

/* A pause longer than this (ms) between motion events starts the count over. */
#define RIP_TEMP_LAYER_DISTANCE_GAP_MS 150

struct rip_temp_layer_distance {
    uint32_t total;   // output counts since the burst started
    uint32_t last_ms; // time of the last counted event
};

static inline uint8_t rip_temp_layer_clamp_distance(int64_t distance) {
    if (distance < 0) {
        return 0;
    }
    if (distance > RIP_TEMP_LAYER_MAX_ACTIVATION_DISTANCE) {
        return RIP_TEMP_LAYER_MAX_ACTIVATION_DISTANCE;
    }
    return (uint8_t)distance;
}

static inline void rip_temp_layer_distance_reset(struct rip_temp_layer_distance *d) {
    d->total = 0;
}

/* Count one output event (after scaling) at now_ms. Returns true once the
 * burst has reached distance. The count keeps going until the caller resets
 * it, so a burst that reaches the distance while a key press still blocks
 * activation activates with its next event once that delay has passed. */
static inline bool rip_temp_layer_distance_add(struct rip_temp_layer_distance *d, int32_t value,
                                               uint32_t now_ms, uint8_t distance) {
    if (now_ms - d->last_ms > RIP_TEMP_LAYER_DISTANCE_GAP_MS) {
        d->total = 0;
    }
    d->last_ms = now_ms;
    uint32_t magnitude = value < 0 ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
    // Saturates: the count may run on while a key press blocks activation
    d->total = UINT32_MAX - d->total < magnitude ? UINT32_MAX : d->total + magnitude;
    return d->total >= distance;
}
