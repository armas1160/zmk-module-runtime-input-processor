/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zmk/event_manager.h>

/* A flick fired a gesture while gesture test mode was on (nothing was typed;
 * Studio shows it). */
struct zmk_input_processor_gesture_fired {
    const struct device *dev;
    uint8_t direction; // enum zmk_input_processor_gesture_direction
    uint32_t keycode;  // what it is assigned (0 = nothing)
};

ZMK_EVENT_DECLARE(zmk_input_processor_gesture_fired);
