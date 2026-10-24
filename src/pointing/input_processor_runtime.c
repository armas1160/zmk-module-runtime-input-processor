/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_runtime

#include <drivers/input_processor.h>
#include <math.h>
#include <zephyr/device.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/dlist.h>

#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
#include <cormoran/zmk/custom_settings.h>
#endif

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/endpoints.h>
#include <zmk/events/input_processor_state_changed.h>
#include <zmk/events/input_processor_inertia_state_changed.h>
#include <zmk/events/input_processor_gesture_fired.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/events/layer_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/hid.h>
#include <zmk/keymap.h>
#include <zmk/keys.h>

#include "rip_accel_math.h"
#include "rip_aim_math.h"
#include "rip_gesture_math.h"
#include "rip_temp_layer_math.h"
#include "rip_tilt_math.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct runtime_processor_config {
    const char *name;
    uint8_t type;
    size_t x_codes_len;
    size_t y_codes_len;
    const uint16_t *x_codes;
    const uint16_t *y_codes;
    uint32_t initial_scale_multiplier;
    uint32_t initial_scale_divisor;
    int32_t initial_rotation_degrees;
    // Tilt compensation defaults from DT (clamped/normalized when applied)
    int32_t initial_tilt_degrees;
    int32_t initial_tilt_axis_degrees;
    // Pointer acceleration defaults from DT (speeds in counts per 100 ms)
    int32_t initial_accel_min_percent;
    uint32_t accel_speed_low;
    uint32_t accel_speed_high;
    // Aim mode defaults from DT, used while an aim behavior is held (see
    // rip_aim_math.h). Kept raw and clamped when applied.
    int32_t initial_aim_scale_percent;
    int32_t initial_aim_smoothing_ms;
    // Normal smoothing default from DT: the same filter at full speed while no
    // aim key is held (0 = off). Kept raw and clamped when applied.
    int32_t initial_smoothing_ms;
    // Input dropped after a mouse button press or release (0 = off)
    uint32_t click_freeze_ms;
    // Gesture defaults from DT: what a flick fires (ZMK encoded keycodes as
    // &kp's parameter, 0 = nothing; left, right, up, down), how far it must
    // go and how long the ball must be still before the next. Kept raw and
    // clamped when applied (see rip_gesture_math.h)
    uint32_t initial_gesture_keycodes[RIP_GESTURE_DIRS];
    int32_t initial_gesture_distance;
    int32_t initial_gesture_still_ms;
#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
    // Custom-settings key of the gesture settings ("<processor-label>_gesture")
    const char *gesture_settings_key;
#endif
    // Temp-layer behavior references for efficient comparison
    const struct device *temp_layer_transparent_behavior;
    const struct device *temp_layer_kp_behavior;
    size_t temp_layer_keep_keycodes_len;
    const uint16_t *temp_layer_keep_keycodes;
    // Temp-layer default settings from DT
    bool initial_temp_layer_enabled;
    // Input this processor does not handle still delays temp-layer deactivation
    bool temp_layer_extend_on_other_input;
    uint8_t initial_temp_layer_layer;
    uint16_t initial_temp_layer_activation_delay_ms;
    uint16_t initial_temp_layer_deactivation_delay_ms;
    // Kept raw and clamped when applied (see rip_temp_layer_math.h)
    uint32_t initial_temp_layer_activation_distance;
    // Active layers bitmask from DT
    uint32_t initial_active_layers;
    // Axis snap default settings from DT
    uint8_t initial_axis_snap_mode;
    uint16_t initial_axis_snap_threshold;
    uint16_t initial_axis_snap_timeout_ms;
    // Code mapping default settings from DT
    bool initial_xy_to_scroll_enabled;
    bool initial_xy_swap_enabled;
    // Axis reverse default settings from DT
    bool initial_x_invert;
    bool initial_y_invert;
    // Inertia defaults from DT.
    bool initial_inertia_enabled;
    uint16_t initial_inertia_window_ms;
    uint16_t initial_inertia_interval_ms;
    uint16_t initial_inertia_threshold;
    uint32_t initial_inertia_decay_percent;
    uint16_t initial_inertia_normal_max_output;
    uint16_t initial_inertia_fast_threshold;
    uint16_t initial_inertia_fast_output_percent;
};

#define INERTIA_BUCKETS CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_INERTIA_BUCKETS
#define INERTIA_WINDOW_SLICES CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_INERTIA_WINDOW_SLICES

/* Pointer acceleration speed window (see rip_accel_math.h). */
#define ACCEL_WINDOW_MS CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_ACCEL_WINDOW_MS

/* Aim mode: how often what the filter holds back is released once input
 * stops (see rip_aim_math.h). */
#define AIM_INTERVAL_MS CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_AIM_INTERVAL_MS

/* Gestures: a flick covers the gesture distance within this window, and the
 * shortcut it fires is held this long (see rip_gesture_math.h). */
#define GESTURE_WINDOW_MS CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_GESTURE_WINDOW_MS
#define GESTURE_TAP_MS CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_GESTURE_TAP_MS

BUILD_ASSERT(GESTURE_WINDOW_MS <= RIP_GESTURE_MAX_WINDOW_MS,
             "Gesture window is longer than the gesture motion ring");

/* How long a gesture test request lasts; Studio renews it every few seconds
 * while its page is open and in front, so a closed or crashed client does not
 * leave the shortcuts off for long. */
#define GESTURE_TEST_MS 20000

/* A tap & hold key's tap that comes this soon after its hold used the ball is
 * dropped (the hold-tap releases the hold and presses the tap in one go). */
#define BALL_USED_TAP_MS 20

BUILD_ASSERT(INERTIA_BUCKETS >= INERTIA_WINDOW_SLICES + 2,
             "Inertia bucket count must exceed window slices by at least two");
BUILD_ASSERT(CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_INERTIA_REVERSE_EXIT_MIN <=
                 CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_INERTIA_REVERSE_EXIT_MAX,
             "Inertia reverse exit minimum must not exceed maximum");

struct inertia_trigger_state {
    /* Each bucket holds Q16 input spread uniformly over its time slice. */
    uint64_t bucket_q16[INERTIA_BUCKETS];
    int64_t bucket_number[INERTIA_BUCKETS];
    int64_t last_report_ms;
    bool has_report;
    int8_t direction;
};

/* Ignore isolated sensor jitter, but keep reversing much easier than entry. */
#define INERTIA_REVERSE_EXIT_MIN CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_INERTIA_REVERSE_EXIT_MIN
#define INERTIA_REVERSE_EXIT_MAX CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_INERTIA_REVERSE_EXIT_MAX
#define INERTIA_SPEED_FRACTION_BITS 16

struct runtime_processor_data {
    const struct device *dev;
#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
    struct k_work_delayable save_work;
#endif
    // Current active values (may be temporary from behavior)
    uint32_t scale_multiplier;
    uint32_t scale_divisor;
    int32_t rotation_degrees;
    uint8_t tilt_degrees;      // [0, RIP_TILT_MAX_DEGREES]
    int16_t tilt_axis_degrees; // (-90, 90], clockwise from up/down
    uint8_t accel_min_percent; // [RIP_ACCEL_MIN_PERCENT, 100], 100 = off
    uint8_t aim_scale_percent; // [RIP_AIM_MIN_PERCENT, RIP_AIM_MAX_PERCENT]
    uint8_t aim_smoothing_ms;  // [0, RIP_AIM_MAX_SMOOTHING_MS], 0 = no smoothing
    uint8_t smoothing_ms;      // [0, RIP_AIM_MAX_SMOOTHING_MS] when aim is not held, 0 = off

    // Persistent values (saved to settings, not affected by behavior)
    uint32_t persistent_scale_multiplier;
    uint32_t persistent_scale_divisor;
    int32_t persistent_rotation_degrees;
    uint8_t persistent_tilt_degrees;
    int16_t persistent_tilt_axis_degrees;
    uint8_t persistent_accel_min_percent;
    uint8_t persistent_aim_scale_percent;
    uint8_t persistent_aim_smoothing_ms;
    uint8_t persistent_smoothing_ms;

    // Precomputed rotation + tilt transform (x1000, see rip_tilt_math.h)
    struct rip_tilt_matrix transform;
    bool transform_identity;
    // Per-axis remainders (x1000) and cross terms carried between events
    struct rip_tilt_state transform_state;

    // Recent input for the acceleration speed estimate and per-axis
    // remainders (permille, see rip_accel_math.h)
    struct rip_accel_state accel_state;

    // Aim mode and normal smoothing: how many aim keys are held, the filter
    // per output axis (X, Y, shared by both modes) and the work that releases
    // what is left once input stops. aim_lock guards aim_axis between the
    // input thread and the work.
    uint8_t aim_hold_count;
    // Speed of the newest held aim key (10..100), 0 = aim_scale_percent.
    // Set by the aim behavior, not persisted
    uint8_t aim_key_percent;
    struct rip_aim_axis aim_axis[2];
    struct k_spinlock aim_lock;
    struct k_work_delayable aim_work;
    // Motion while an aim key is held (sensor counts since the newest aim
    // press), to tell whether a tap & hold key's hold used the ball
    atomic_t aim_hold_motion;

    // Gestures (see rip_gesture_math.h): what each direction fires, how far a
    // flick must go and how long the ball must be still before the next, and
    // their persistent baseline
    uint32_t gesture_keycodes[RIP_GESTURE_DIRS];
    uint16_t gesture_distance;
    uint16_t gesture_still_ms;
    uint32_t persistent_gesture_keycodes[RIP_GESTURE_DIRS];
    uint16_t persistent_gesture_distance;
    uint16_t persistent_gesture_still_ms;
    // Gesture keys held, the detector, when the last one went up (the cursor
    // stays put while draining, until the ball stops), the direction waiting
    // for the fire work and test mode (until this uptime: Studio is told
    // instead of the shortcut being sent). gesture_lock guards them between
    // the input thread, the behaviors, the RPC and the work.
    uint8_t gesture_hold_count;
    struct rip_gesture gesture;
    uint32_t gesture_up_ms;
    bool gesture_draining;
    // Flicks waiting for the fire work, oldest first (a few, in case the work
    // queue is busy)
    int8_t gesture_pending[4];
    uint8_t gesture_pending_count;
    int64_t gesture_test_until;
    // The input device whose motion the detector sees; other processors only
    // leave motion from this device to it
    const struct device *gesture_input_dev;
    struct k_spinlock gesture_lock;
    // The shortcut being tapped (system work queue only)
    uint32_t gesture_tapped_keycode;
    struct k_work gesture_fire_work;
    struct k_work_delayable gesture_release_work;
    // When an aim or gesture hold that used the ball ended (0 = none) and the
    // key position it was held from (set by its behavior right after): the tap
    // of that tap & hold key is dropped. Behavior context only
    int64_t ball_used_at;
    uint32_t ball_used_position;

    // Input is dropped until this uptime after a mouse button changed
    // (0 = not frozen). Only touched from the input thread.
    int64_t click_freeze_until;

    // Temp-layer layer settings
    bool temp_layer_enabled;
    uint8_t temp_layer_layer;
    uint16_t temp_layer_activation_delay_ms;
    uint16_t temp_layer_deactivation_delay_ms;
    uint8_t temp_layer_activation_distance; // [0, RIP_TEMP_LAYER_MAX_ACTIVATION_DISTANCE]

    // Persistent temp-layer settings
    bool persistent_temp_layer_enabled;
    uint8_t persistent_temp_layer_layer;
    uint16_t persistent_temp_layer_activation_delay_ms;
    uint16_t persistent_temp_layer_deactivation_delay_ms;
    uint8_t persistent_temp_layer_activation_distance;

    // Active layers bitmask (0 = all layers)
    uint32_t active_layers;
    uint32_t persistent_active_layers;

    // Axis snap settings
    uint8_t axis_snap_mode;
    uint16_t axis_snap_threshold;
    uint16_t axis_snap_timeout_ms;

    // Persistent axis snap settings
    uint8_t persistent_axis_snap_mode;
    uint16_t persistent_axis_snap_threshold;
    uint16_t persistent_axis_snap_timeout_ms;

    // Axis snap runtime state
    int16_t axis_snap_cross_axis_accum;     // Accumulated movement on cross axis
    int64_t axis_snap_last_decay_timestamp; // Last time accumulator was decayed

    // Code mapping settings
    bool xy_to_scroll_enabled;
    bool xy_swap_enabled;

    // Persistent code mapping settings
    bool persistent_xy_to_scroll_enabled;
    bool persistent_xy_swap_enabled;

    // Axis reverse settings
    bool x_invert;
    bool y_invert;

    // Persistent axis reverse settings
    bool persistent_x_invert;
    bool persistent_y_invert;

    // Inertia settings and their persistent baseline.
    bool inertia_enabled;
    uint16_t inertia_window_ms;
    uint16_t inertia_interval_ms;
    uint16_t inertia_threshold;
    uint8_t inertia_decay_percent;
    uint16_t inertia_normal_max_output;
    uint16_t inertia_fast_threshold;
    uint16_t inertia_fast_output_percent;
    bool inertia_notifications_enabled;
    bool persistent_inertia_enabled;
    uint16_t persistent_inertia_window_ms;
    uint16_t persistent_inertia_interval_ms;
    uint16_t persistent_inertia_threshold;
    uint8_t persistent_inertia_decay_percent;
    uint16_t persistent_inertia_normal_max_output;
    uint16_t persistent_inertia_fast_threshold;
    uint16_t persistent_inertia_fast_output_percent;

    // Raw input drives inertia; a parallel post-scale window keeps the
    // user-facing entry threshold in scroll units.
    struct inertia_trigger_state inertia_trigger[2];
    struct inertia_trigger_state inertia_scaled_trigger[2];
    struct inertia_trigger_state inertia_measurement;
    struct inertia_trigger_state inertia_scaled_measurement;
    bool inertia_active;
    bool inertia_fast_active;
    uint8_t inertia_axis;
    int8_t inertia_direction;
    uint16_t inertia_output_code;
    uint64_t inertia_speed_q16;
    uint64_t inertia_output_remainder;
    uint32_t inertia_scale_remainder;
    uint8_t inertia_fast_remainder;
    bool inertia_received_input;
    uint32_t inertia_reverse_amount;
    int64_t inertia_reverse_start;
    struct k_work_delayable inertia_work;

    // Temp-layer runtime state
    struct k_work_delayable temp_layer_activation_work;
    struct k_work_delayable temp_layer_deactivation_work;
    bool temp_layer_layer_active;
    bool temp_layer_keep_active; // Set by behavior to prevent deactivation
    int64_t last_input_timestamp;
    int64_t last_keypress_timestamp;
    // Output motion counted toward the activation distance (input thread;
    // the work handlers and key listeners only reset it)
    struct rip_temp_layer_distance temp_layer_distance;
};

/* Recompute the combined rotation + tilt matrix. Call whenever rotation, tilt
 * or tilt axis changes. A changed matrix drops the carried remainders so no
 * fraction or cross term from the old transform leaks out. */
static void update_rotation_values(struct runtime_processor_data *data) {
    struct rip_tilt_matrix transform = rip_tilt_compute_matrix(
        data->rotation_degrees, data->tilt_degrees, data->tilt_axis_degrees);

    if (!rip_tilt_matrix_equal(&transform, &data->transform)) {
        data->transform = transform;
        data->transform_state = (struct rip_tilt_state){0};
    }
    data->transform_identity = rip_tilt_matrix_is_identity(&transform);

    LOG_DBG("Rotation %d, tilt %u at axis %d degrees: [[%d, %d], [%d, %d]]", data->rotation_degrees,
            data->tilt_degrees, data->tilt_axis_degrees, transform.xx, transform.xy, transform.yx,
            transform.yy);
}

// Temp-layer layer work handlers
static void temp_layer_activation_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct runtime_processor_data *data =
        CONTAINER_OF(dwork, struct runtime_processor_data, temp_layer_activation_work);

    if (!data->temp_layer_enabled || data->temp_layer_layer_active) {
        return;
    }

    // Activate the temp-layer layer
    int ret = zmk_keymap_layer_activate(data->temp_layer_layer, false);
    if (ret == 0) {
        data->temp_layer_layer_active = true;
        rip_temp_layer_distance_reset(&data->temp_layer_distance);
        LOG_INF("Temp-layer layer %d activated", data->temp_layer_layer);
    } else {
        LOG_ERR("Failed to activate temp-layer layer %d: %d", data->temp_layer_layer, ret);
    }
}

static void temp_layer_deactivation_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct runtime_processor_data *data =
        CONTAINER_OF(dwork, struct runtime_processor_data, temp_layer_deactivation_work);

    // A held aim or gesture key keeps the layer; its release re-arms this work
    if (!data->temp_layer_layer_active || data->temp_layer_keep_active ||
        data->aim_hold_count > 0 || data->gesture_hold_count > 0) {
        return;
    }

    // Deactivate the temp-layer layer
    int ret = zmk_keymap_layer_deactivate(data->temp_layer_layer, false);
    if (ret == 0) {
        data->temp_layer_layer_active = false;
        rip_temp_layer_distance_reset(&data->temp_layer_distance);
        LOG_INF("Temp-layer layer %d deactivated", data->temp_layer_layer);
    } else {
        LOG_ERR("Failed to deactivate temp-layer layer %d: %d", data->temp_layer_layer, ret);
    }
}

static int code_idx(uint16_t code, const uint16_t *list, size_t len) {
    for (int i = 0; i < len; i++) {
        if (list[i] == code) {
            return i;
        }
    }
    return -ENODEV;
}

static bool is_processor_active_for_current_layers(uint32_t active_layers_mask) {
    // If mask is 0, processor is active for all layers
    if (active_layers_mask == 0) {
        return true;
    }

    // Check only the layers that are set in the bitmask
    // This is more efficient than checking all layers
    uint32_t remaining_mask = active_layers_mask;
    int layer_idx = 0;

    while (remaining_mask != 0 && layer_idx < ZMK_KEYMAP_LAYERS_LEN) {
        // Check if this bit is set
        if (remaining_mask & 1) {
            zmk_keymap_layer_id_t layer_id = zmk_keymap_layer_index_to_id(layer_idx);

            if (layer_id != ZMK_KEYMAP_LAYER_ID_INVAL && zmk_keymap_layer_active(layer_id)) {
                return true;
            }
        }

        remaining_mask >>= 1;
        layer_idx++;
    }

    return false;
}

static int scale_val(struct input_event *event, uint32_t mul, uint32_t div,
                     struct zmk_input_processor_state *state) {
    if (mul == 0 || div == 0) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int16_t value_mul = event->value * (int16_t)mul;

    if (state && state->remainder) {
        value_mul += *state->remainder;
    }

    int16_t scaled = value_mul / (int16_t)div;

    if (state && state->remainder) {
        *state->remainder = value_mul - (scaled * (int16_t)div);
    }

    LOG_DBG("scaled %d with %d/%d to %d", event->value, mul, div, scaled);

    event->value = scaled;
    return 0;
}

static uint32_t input_value_magnitude(int16_t value) {
    return value < 0 ? (uint32_t)(-(int32_t)value) : (uint32_t)value;
}

static uint32_t saturating_add_amount(uint32_t current, uint32_t amount) {
    return UINT32_MAX - current < amount ? UINT32_MAX : current + amount;
}

static void inertia_clear_windows(struct runtime_processor_data *data) {
    memset(data->inertia_trigger, 0, sizeof(data->inertia_trigger));
    memset(data->inertia_scaled_trigger, 0, sizeof(data->inertia_scaled_trigger));
    memset(&data->inertia_measurement, 0, sizeof(data->inertia_measurement));
    memset(&data->inertia_scaled_measurement, 0, sizeof(data->inertia_scaled_measurement));
    data->inertia_speed_q16 = 0;
    data->inertia_output_remainder = 0;
    data->inertia_scale_remainder = 0;
    data->inertia_fast_remainder = 0;
    data->inertia_fast_active = false;
    data->inertia_received_input = false;
    data->inertia_reverse_amount = 0;
    data->inertia_reverse_start = 0;
}

static void inertia_stop(struct runtime_processor_data *data,
                         enum zmk_input_processor_inertia_stop_reason reason) {
    bool was_active = data->inertia_active;
    data->inertia_active = false;
    inertia_clear_windows(data);
    k_work_cancel_delayable(&data->inertia_work);
    if (was_active && data->inertia_notifications_enabled) {
        raise_zmk_input_processor_inertia_state_changed(
            (struct zmk_input_processor_inertia_state_changed){.dev = data->dev,
                                                               .state = ZMK_INPUT_PROCESSOR_INERTIA_STOPPED,
                                                               .stop_reason = reason});
        raise_zmk_input_processor_inertia_state_changed(
            (struct zmk_input_processor_inertia_state_changed){.dev = data->dev,
                                                               .state = ZMK_INPUT_PROCESSOR_INERTIA_FAST_INPUT_STOPPED});
    }
}

static uint32_t inertia_bucket_ms(uint32_t window_ms) {
    return (window_ms + INERTIA_WINDOW_SLICES - 1) / INERTIA_WINDOW_SLICES;
}

static void inertia_add_report(struct inertia_trigger_state *track, uint32_t amount,
                               int64_t now, uint32_t window_ms) {
    uint32_t bucket_ms = inertia_bucket_ms(window_ms);
    int64_t start = track->has_report ? track->last_report_ms : now;
    if (start > now) {
        start = now;
    }
    if (start == now) {
        int64_t number = now / bucket_ms;
        size_t slot = number % INERTIA_BUCKETS;
        if (track->bucket_number[slot] != number) {
            track->bucket_number[slot] = number;
            track->bucket_q16[slot] = 0;
        }
        track->bucket_q16[slot] += (uint64_t)amount << INERTIA_SPEED_FRACTION_BITS;
        track->last_report_ms = now;
        track->has_report = true;
        return;
    }
    uint64_t duration = MAX(now - start, 1);
    /* A report after a long pause only contributes its portion within the window. */
    int64_t cursor = MAX(start, now - window_ms);
    while (cursor < now) {
        int64_t number = cursor / bucket_ms;
        int64_t end = MIN(now, (number + 1) * bucket_ms);
        size_t slot = number % INERTIA_BUCKETS;
        if (track->bucket_number[slot] != number) {
            track->bucket_number[slot] = number;
            track->bucket_q16[slot] = 0;
        }
        track->bucket_q16[slot] += ((uint64_t)amount << INERTIA_SPEED_FRACTION_BITS) *
                                   (end - cursor) / duration;
        cursor = end;
    }
    track->last_report_ms = now;
    track->has_report = true;
}

static uint64_t inertia_window_total_q16(const struct inertia_trigger_state *track,
                                         int64_t now, uint32_t window_ms) {
    uint32_t bucket_ms = inertia_bucket_ms(window_ms);
    int64_t start = now - window_ms;
    uint64_t total = 0;
    for (size_t slot = 0; slot < INERTIA_BUCKETS; slot++) {
        int64_t bucket_start = track->bucket_number[slot] * bucket_ms;
        int64_t bucket_end = bucket_start + bucket_ms;
        if (track->bucket_q16[slot] == 0 || bucket_end <= start || bucket_start > now) {
            continue;
        }
        if (bucket_start < start) {
            total += track->bucket_q16[slot] * (bucket_end - start) / bucket_ms;
        } else {
            total += track->bucket_q16[slot];
        }
    }
    return total;
}

static uint32_t inertia_raw_threshold(const struct runtime_processor_data *data) {
    if (data->scale_multiplier == 0 || data->scale_divisor == 0) {
        return data->inertia_threshold;
    }
    uint64_t raw = ((uint64_t)data->inertia_threshold * data->scale_divisor +
                    data->scale_multiplier - 1) /
                   data->scale_multiplier;
    return MIN(raw, UINT32_MAX);
}

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_TEST)
int zmk_input_processor_runtime_test_inertia_sliding_window(void) {
    struct inertia_trigger_state track = {0};
    inertia_add_report(&track, 10, 100, 40);
    inertia_add_report(&track, 20, 120, 40);
    if (inertia_window_total_q16(&track, 120, 40) != (30ULL << INERTIA_SPEED_FRACTION_BITS)) {
        return -EINVAL;
    }
    if (inertia_window_total_q16(&track, 160, 40) != 0) {
        return -EINVAL;
    }
    /* An 80 ms report contributes half its value to a 40 ms window. */
    inertia_add_report(&track, 40, 200, 40);
    if (inertia_window_total_q16(&track, 200, 40) != (20ULL << INERTIA_SPEED_FRACTION_BITS)) {
        return -EINVAL;
    }
    return 0;
}
#endif

/* Finish one output interval. Speed and the rolling window use input before
 * scaling; apply scale only when producing synthetic output. */
static int16_t inertia_finish_interval(struct runtime_processor_data *data, int64_t now) {
    if (!data->inertia_active || data->inertia_window_ms == 0 ||
        data->inertia_interval_ms == 0) {
        return 0;
    }
    if (!is_processor_active_for_current_layers(data->active_layers)) {
        inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_LAYER_INACTIVE);
        return 0;
    }

    uint64_t denominator = (uint64_t)data->inertia_window_ms <<
                           INERTIA_SPEED_FRACTION_BITS;
    /* Physical reports already pass through to the endpoint. Estimate their
     * current rate from the latest bucket, rather than the whole window, so
     * inertia can take over promptly when physical motion slows or stops.
     * Convert the bucket to window units before subtracting the retained peak. */
    uint32_t bucket_ms = inertia_bucket_ms(data->inertia_window_ms);
    const struct inertia_trigger_state *measurement = &data->inertia_measurement;
    uint64_t physical_q16 = 0;
    if (measurement->has_report && now - measurement->last_report_ms < bucket_ms) {
        int64_t bucket_number = measurement->last_report_ms / bucket_ms;
        size_t slot = bucket_number % INERTIA_BUCKETS;
        /* Distributed reports end just before last_report_ms; at an exact
         * boundary their latest populated bucket is the preceding one. */
        if (bucket_number > 0 &&
            (measurement->bucket_number[slot] != bucket_number ||
             measurement->bucket_q16[slot] == 0)) {
            bucket_number--;
            slot = bucket_number % INERTIA_BUCKETS;
        }
        if (measurement->bucket_number[slot] == bucket_number) {
            physical_q16 = measurement->bucket_q16[slot] * data->inertia_window_ms / bucket_ms;
        }
    }
    /* Fast mode raises the total target, not just the supplemental output:
     * physical + inertia = retained peak * fast percentage. */
    uint64_t target_q16 = data->inertia_speed_q16;
    if (data->inertia_fast_active) {
        uint64_t fast_numerator = target_q16 * data->inertia_fast_output_percent +
                                  data->inertia_fast_remainder;
        target_q16 = fast_numerator / 100U;
        data->inertia_fast_remainder = fast_numerator % 100U;
    }
    uint64_t supplemental_q16 = target_q16 > physical_q16 ? target_q16 - physical_q16 : 0;
    uint64_t numerator = supplemental_q16 * data->inertia_interval_ms +
                         data->inertia_output_remainder;
    uint64_t raw_amount = numerator / denominator;
    data->inertia_output_remainder = numerator % denominator;
    uint32_t multiplier = data->scale_multiplier > 0 && data->scale_divisor > 0
                              ? data->scale_multiplier
                              : 1;
    uint32_t divisor = data->scale_multiplier > 0 && data->scale_divisor > 0
                           ? data->scale_divisor
                           : 1;
    uint64_t scaled_numerator = raw_amount * multiplier + data->inertia_scale_remainder;
    uint64_t scaled_amount = scaled_numerator / divisor;
    data->inertia_scale_remainder = scaled_numerator % divisor;
    scaled_amount = MIN(scaled_amount, (uint64_t)INT16_MAX);
    if (!data->inertia_fast_active && data->inertia_normal_max_output > 0) {
        scaled_amount = MIN(scaled_amount, (uint64_t)data->inertia_normal_max_output);
    }
    if (scaled_amount > INT16_MAX) {
        scaled_amount = INT16_MAX;
    }
    int16_t output =
        data->inertia_direction < 0 ? -(int16_t)scaled_amount : (int16_t)scaled_amount;

    /* Repeated multiplication by the retained fraction gives exponential
     * decay. Skip the interval that received input so its full speed is used. */
    bool received_input = data->inertia_received_input;
    if (!received_input) {
        data->inertia_speed_q16 =
            data->inertia_speed_q16 * (100U - data->inertia_decay_percent) / 100U;
    }
    data->inertia_received_input = false;

    /* Count the raw input still needed for one scaled output. If even the
     * full remaining decay tail cannot reach it, no more scroll can appear. */
    if (!received_input && data->inertia_decay_percent != 0 &&
        data->inertia_speed_q16 <= UINT32_MAX) {
        uint64_t raw_needed =
            ((uint64_t)divisor - data->inertia_scale_remainder + multiplier - 1) / multiplier;
        uint64_t missing_q16 = raw_needed * denominator - data->inertia_output_remainder;
        uint32_t output_percent = data->inertia_fast_active ? data->inertia_fast_output_percent : 100U;
        uint64_t future_numerator = data->inertia_speed_q16 * data->inertia_interval_ms *
                                    output_percent;
        if (missing_q16 > UINT64_MAX / data->inertia_decay_percent ||
            future_numerator < missing_q16 * data->inertia_decay_percent) {
            inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTLED);
        }
    }

    return output;
}

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_TEST)
int zmk_input_processor_runtime_test_inertia_latest_bucket(void) {
    struct runtime_processor_data data = {
        .inertia_active = true,
        .inertia_window_ms = 100,
        .inertia_interval_ms = 20,
        .inertia_direction = 1,
        .inertia_speed_q16 = 100ULL << INERTIA_SPEED_FRACTION_BITS,
        .scale_multiplier = 1,
        .scale_divisor = 1,
    };
    uint32_t bucket_ms = inertia_bucket_ms(data.inertia_window_ms);
    int64_t now = 1000 * bucket_ms;
    /* Older input remains in the window, but only the latest bucket offsets
     * inertia. Its rate is 40 window counts, leaving 60 * 20/100 = 12. */
    inertia_add_report(&data.inertia_measurement, 100, now - bucket_ms, 100);
    inertia_add_report(&data.inertia_measurement, 1, now, 100);
    size_t slot = (now / bucket_ms) % INERTIA_BUCKETS;
    data.inertia_measurement.bucket_number[slot] = now / bucket_ms;
    data.inertia_measurement.bucket_q16[slot] =
        (40ULL * bucket_ms << INERTIA_SPEED_FRACTION_BITS) / 100;
    if (inertia_finish_interval(&data, now) != 12) {
        return -EINVAL;
    }
    /* A report ending exactly on a bucket boundary populated the preceding
     * bucket, which is still the latest measurement for this tick. */
    size_t previous_slot = (slot + INERTIA_BUCKETS - 1) % INERTIA_BUCKETS;
    data.inertia_measurement.bucket_number[previous_slot] = now / bucket_ms - 1;
    data.inertia_measurement.bucket_q16[previous_slot] = data.inertia_measurement.bucket_q16[slot];
    data.inertia_measurement.bucket_q16[slot] = 0;
    if (inertia_finish_interval(&data, now) != 12) {
        return -EINVAL;
    }
    data.inertia_measurement.bucket_q16[slot] = data.inertia_measurement.bucket_q16[previous_slot];
    data.inertia_direction = -1;
    if (inertia_finish_interval(&data, now) != -12) {
        return -EINVAL;
    }
    /* Equal/faster physical input never produces opposite inertia. */
    data.inertia_measurement.bucket_q16[slot] =
        (100ULL * bucket_ms << INERTIA_SPEED_FRACTION_BITS) / 100;
    if (inertia_finish_interval(&data, now) != 0) {
        return -EINVAL;
    }
    data.inertia_measurement.bucket_q16[slot] *= 2;
    if (inertia_finish_interval(&data, now) != 0) {
        return -EINVAL;
    }
    /* A new empty bucket hands output back to inertia immediately, even
     * though those physical reports remain in the full measurement window. */
    if (inertia_finish_interval(&data, now + bucket_ms) != -20) {
        return -EINVAL;
    }
    data.inertia_direction = 1;
    data.inertia_fast_active = true;
    data.inertia_fast_output_percent = 200;
    data.inertia_measurement.bucket_q16[slot] =
        (40ULL * bucket_ms << INERTIA_SPEED_FRACTION_BITS) / 100;
    if (inertia_finish_interval(&data, now) != 32) {
        return -EINVAL;
    }
    /* At the doubled target, physical input leaves no supplemental output. */
    data.inertia_measurement.bucket_q16[slot] =
        (200ULL * bucket_ms << INERTIA_SPEED_FRACTION_BITS) / 100;
    if (inertia_finish_interval(&data, now) != 0) {
        return -EINVAL;
    }
    data.inertia_measurement.bucket_q16[slot] *= 2;
    if (inertia_finish_interval(&data, now) != 0) {
        return -EINVAL;
    }
    data.inertia_measurement.bucket_q16[slot] =
        (40ULL * bucket_ms << INERTIA_SPEED_FRACTION_BITS) / 100;
    data.inertia_direction = -1;
    if (inertia_finish_interval(&data, now) != -32 ||
        inertia_finish_interval(&data, now + bucket_ms) != -40) {
        return -EINVAL;
    }
    data.inertia_direction = 1;
    data.inertia_fast_active = false;
    data.scale_divisor = 60;
    int total = 0;
    for (int i = 0; i < 5; i++) {
        total += inertia_finish_interval(&data, now);
    }
    if (total != 1) {
        return -EINVAL;
    }
    return 0;
}
#endif

/* Inertia is already in its final, processor-transformed coordinate
 * system. Do not inject it back into the source input device: that would send
 * it through every input processor again, allowing a later processor to turn
 * a scroll event back into pointer movement (or vice versa). */
static void inertia_emit(uint16_t code, int16_t value) {
    /* A processor may be present in builds that omit the pointing endpoint. */
#if IS_ENABLED(CONFIG_ZMK_POINTING)
    switch (code) {
    case INPUT_REL_X:
        zmk_hid_mouse_movement_set(value, 0);
        break;
    case INPUT_REL_Y:
        zmk_hid_mouse_movement_set(0, value);
        break;
    case INPUT_REL_HWHEEL:
        zmk_hid_mouse_scroll_set(value, 0);
        break;
    case INPUT_REL_WHEEL:
        zmk_hid_mouse_scroll_set(0, value);
        break;
    default:
        LOG_WRN("Inertia ignored unsupported output code %u", code);
        return;
    }

    int ret = zmk_endpoint_send_mouse_report();
    if (ret < 0) {
        LOG_WRN("Inertia report failed: %d", ret);
    }
    zmk_hid_mouse_scroll_set(0, 0);
    zmk_hid_mouse_movement_set(0, 0);
#else
    ARG_UNUSED(code);
    ARG_UNUSED(value);
#endif
}

static void inertia_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct runtime_processor_data *data =
        CONTAINER_OF(dwork, struct runtime_processor_data, inertia_work);

    if (!data->inertia_active || !data->inertia_enabled ||
        data->inertia_window_ms == 0 || data->inertia_interval_ms == 0) {
        return;
    }

    int16_t output = inertia_finish_interval(data, k_uptime_get());
    if (output != 0) {
        inertia_emit(data->inertia_output_code, output);
        LOG_DBG("Inertia emitted %d every %u ms", output, data->inertia_interval_ms);
    }

    if (data->inertia_active) {
        k_work_reschedule(&data->inertia_work, K_MSEC(data->inertia_interval_ms));
    }
}

static void inertia_start(struct runtime_processor_data *data, uint8_t axis, int8_t direction,
                          uint16_t output_code, int64_t now) {
    uint64_t initial_amount_q16 = inertia_window_total_q16(
        &data->inertia_trigger[axis], now, data->inertia_window_ms);
    data->inertia_measurement = data->inertia_trigger[axis];
    data->inertia_scaled_measurement = data->inertia_scaled_trigger[axis];
    data->inertia_fast_active = data->inertia_fast_threshold > 0 &&
        inertia_window_total_q16(&data->inertia_scaled_measurement, now,
                                 data->inertia_window_ms) >=
            (uint64_t)data->inertia_fast_threshold << INERTIA_SPEED_FRACTION_BITS;
    data->inertia_active = true;
    if (data->inertia_notifications_enabled) {
        raise_zmk_input_processor_inertia_state_changed(
            (struct zmk_input_processor_inertia_state_changed){
                .dev = data->dev, .state = ZMK_INPUT_PROCESSOR_INERTIA_STARTED});
        raise_zmk_input_processor_inertia_state_changed(
            (struct zmk_input_processor_inertia_state_changed){.dev = data->dev,
                                                               .state = data->inertia_fast_active
                                                                           ? ZMK_INPUT_PROCESSOR_INERTIA_FAST_INPUT_STARTED
                                                                           : ZMK_INPUT_PROCESSOR_INERTIA_FAST_INPUT_STOPPED});
    }
    data->inertia_axis = axis;
    data->inertia_direction = direction;
    data->inertia_output_code = output_code;
    memset(data->inertia_trigger, 0, sizeof(data->inertia_trigger));
    memset(data->inertia_scaled_trigger, 0, sizeof(data->inertia_scaled_trigger));
    data->inertia_speed_q16 = initial_amount_q16;
    data->inertia_output_remainder = 0;
    data->inertia_scale_remainder = 0;
    data->inertia_fast_remainder = 0;
    data->inertia_received_input = true;
    data->inertia_reverse_amount = 0;
    data->inertia_reverse_start = 0;
    k_work_reschedule(&data->inertia_work, K_MSEC(data->inertia_interval_ms));
    LOG_DBG("Inertia started: axis=%u direction=%d threshold=%u window=%u interval=%u",
            axis, direction, data->inertia_threshold, data->inertia_window_ms,
            data->inertia_interval_ms);
}

static void inertia_handle_physical_event(struct runtime_processor_data *data, uint8_t axis,
                                          const struct input_event *event, int16_t raw_value) {
    if (!IS_ENABLED(CONFIG_ZMK_POINTING) || event->type != INPUT_EV_REL || raw_value == 0 ||
        !data->inertia_enabled ||
        data->inertia_window_ms == 0 || data->inertia_interval_ms == 0) {
        return;
    }

    const uint32_t magnitude = input_value_magnitude(raw_value);
    const int8_t direction = raw_value < 0 ? -1 : 1;

    if (data->inertia_active) {
        if (axis != data->inertia_axis) {
            return;
        }
        if (direction != data->inertia_direction) {
            int64_t now = k_uptime_get();
            if (data->inertia_reverse_amount == 0 ||
                now - data->inertia_reverse_start >= data->inertia_window_ms) {
                data->inertia_reverse_start = now;
                data->inertia_reverse_amount = 0;
            }
            data->inertia_reverse_amount =
                saturating_add_amount(data->inertia_reverse_amount, magnitude);
            uint32_t reverse_threshold = CLAMP(inertia_raw_threshold(data) / 2,
                                               INERTIA_REVERSE_EXIT_MIN,
                                               INERTIA_REVERSE_EXIT_MAX);
            if (data->inertia_reverse_amount >= reverse_threshold) {
                LOG_DBG("Inertia stopped by reverse input %d", raw_value);
                inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_REVERSE_INPUT);
            }
            return;
        }

        data->inertia_reverse_amount = 0;
        data->inertia_reverse_start = 0;
        data->inertia_output_code = event->code;
        int64_t now = k_uptime_get();
        struct inertia_trigger_state *measurement = &data->inertia_measurement;
        inertia_add_report(measurement, magnitude, now, data->inertia_window_ms);
        if (event->value != 0 && (event->value < 0 ? -1 : 1) == direction) {
            inertia_add_report(&data->inertia_scaled_measurement,
                               input_value_magnitude(event->value), now,
                               data->inertia_window_ms);
            if (!data->inertia_fast_active && data->inertia_fast_threshold > 0 &&
                inertia_window_total_q16(&data->inertia_scaled_measurement, now,
                                         data->inertia_window_ms) >=
                    (uint64_t)data->inertia_fast_threshold << INERTIA_SPEED_FRACTION_BITS) {
                data->inertia_fast_active = true;
                if (data->inertia_notifications_enabled) {
                    raise_zmk_input_processor_inertia_state_changed(
                        (struct zmk_input_processor_inertia_state_changed){
                            .dev = data->dev,
                            .state = ZMK_INPUT_PROCESSOR_INERTIA_FAST_INPUT_STARTED});
                }
            }
        }
        uint64_t measured_speed_q16 =
            inertia_window_total_q16(measurement, now, data->inertia_window_ms);
        if (measured_speed_q16 > data->inertia_speed_q16) {
            data->inertia_speed_q16 = measured_speed_q16;
        }
        data->inertia_received_input = true;
        return;
    }

    struct inertia_trigger_state *trigger = &data->inertia_trigger[axis];
    struct inertia_trigger_state *scaled_trigger = &data->inertia_scaled_trigger[axis];
    int64_t now = k_uptime_get();
    if (trigger->direction != direction) {
        memset(trigger, 0, sizeof(*trigger));
        memset(scaled_trigger, 0, sizeof(*scaled_trigger));
        trigger->direction = direction;
    }
    inertia_add_report(trigger, magnitude, now, data->inertia_window_ms);
    if (event->value == 0 || (event->value < 0 ? -1 : 1) != direction) {
        return;
    }
    inertia_add_report(scaled_trigger, input_value_magnitude(event->value), now,
                       data->inertia_window_ms);

    if (inertia_window_total_q16(scaled_trigger, now, data->inertia_window_ms) >=
        (uint64_t)data->inertia_threshold << INERTIA_SPEED_FRACTION_BITS) {
        inertia_start(data, axis, direction, event->code, now);
    }
}

/* Aim output released by the work is already in final cursor coordinates, so
 * like inertia it goes straight to the HID report (see inertia_emit). The work
 * only steps an axis that has had no event for AIM_INTERVAL_MS, so with a
 * sensor that reports both axes it stays out of the way while the ball moves.
 * It can still land between a listener's report and its send during a pause,
 * which costs that one report of motion, as with inertia. */
static void aim_emit(int16_t x, int16_t y) {
#if IS_ENABLED(CONFIG_ZMK_POINTING)
    zmk_hid_mouse_movement_set(x, y);
    int ret = zmk_endpoint_send_mouse_report();
    if (ret < 0) {
        LOG_WRN("Aim report failed: %d", ret);
    }
    zmk_hid_mouse_movement_set(0, 0);
#else
    ARG_UNUSED(x);
    ARG_UNUSED(y);
#endif
}

/* Call with aim_lock held. */
static bool aim_busy(const struct runtime_processor_data *data) {
    return !rip_aim_idle(&data->aim_axis[0]) || !rip_aim_idle(&data->aim_axis[1]);
}

static void aim_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct runtime_processor_data *data =
        CONTAINER_OF(dwork, struct runtime_processor_data, aim_work);

    k_spinlock_key_t key = k_spin_lock(&data->aim_lock);
    uint32_t now = (uint32_t)k_uptime_get();
    bool held = data->aim_hold_count > 0;
    uint32_t tau = rip_smooth_tau(held, data->aim_smoothing_ms, data->smoothing_ms);
    int16_t out[2] = {0, 0};
    for (int i = 0; i < 2; i++) {
        // An axis still getting events releases there; step only quiet ones
        struct rip_aim_axis *state = &data->aim_axis[i];
        if (!rip_aim_idle(state) && now - state->last_ms >= AIM_INTERVAL_MS) {
            out[i] = rip_aim_tick(state, now, tau);
        }
    }
    int16_t x = out[0];
    int16_t y = out[1];
    bool busy = aim_busy(data);
    if (!busy && !held) {
        // Done after release: a fraction below one count is not worth a late move
        data->aim_axis[0].frac = 0;
        data->aim_axis[1].frac = 0;
    }
    k_spin_unlock(&data->aim_lock, key);

    if (x != 0 || y != 0) {
        aim_emit(x, y);
        // With normal smoothing the cursor keeps catching up after the last
        // input: that is still use, so the temp layer (and its clicks) stays
        // until the cursor has stopped. Smoothing 0 keeps the old timing.
        if (!held && data->smoothing_ms > 0 && data->temp_layer_enabled &&
            data->temp_layer_layer_active && !data->temp_layer_keep_active) {
            k_work_reschedule(&data->temp_layer_deactivation_work,
                              K_MSEC(data->temp_layer_deactivation_delay_ms));
        }
    }
    if (busy) {
        k_work_reschedule(&data->aim_work, K_MSEC(AIM_INTERVAL_MS));
    }
}

/* Aim mode and normal smoothing on one cursor event, after scaling (see
 * rip_aim_math.h). While an aim key is held the event is slowed down and
 * filtered; otherwise normal smoothing filters it at full speed. With normal
 * smoothing off, what the filter still holds after an aim release is added to
 * the passing events until it is gone, and events pass untouched otherwise.
 * The work runs every AIM_INTERVAL_MS while anything is held back and
 * releases it on the axes that have gone quiet. */
static void aim_handle_event(struct runtime_processor_data *data, struct input_event *event) {
    int axis = event->code == INPUT_REL_X ? 0 : (event->code == INPUT_REL_Y ? 1 : -1);
    if (event->type != INPUT_EV_REL || axis < 0) {
        return;
    }

    k_spinlock_key_t key = k_spin_lock(&data->aim_lock);
    struct rip_aim_axis *state = &data->aim_axis[axis];
    bool held = data->aim_hold_count > 0;
    if (rip_smooth_passthrough(state, held, data->smoothing_ms)) {
        k_spin_unlock(&data->aim_lock, key);
        return;
    }

    uint32_t now = (uint32_t)k_uptime_get();
    int16_t value = (int16_t)CLAMP(event->value, INT16_MIN, INT16_MAX);
    uint8_t aim_percent = rip_aim_effective_percent(data->aim_key_percent, data->aim_scale_percent);
    event->value = rip_smooth_event(state, value, now, held, aim_percent, data->aim_smoothing_ms,
                                    data->smoothing_ms);
    bool busy = aim_busy(data);
    k_spin_unlock(&data->aim_lock, key);

    if (busy) {
        // Keeps an already scheduled run: it must not be pushed back forever
        k_work_schedule(&data->aim_work, K_MSEC(AIM_INTERVAL_MS));
    }
}

/* Input of this processor's type that it does not handle, typically motion
 * that a processor earlier in the listener already turned into scroll on a
 * layer reached from the temp layer. With temp-layer-extend-on-other-input it
 * still counts as use, so the temp layer does not time out underneath (and
 * its keys are still there when scrolling ends). It never activates the layer.
 * Zero values count too: a scroll processor scaling 1/180 turns most motion
 * into 0 with the rest carried, so slow scrolling would otherwise not count. */
static void temp_layer_extend_on_other_input(const struct runtime_processor_config *cfg,
                                             struct runtime_processor_data *data,
                                             const struct input_event *event) {
    ARG_UNUSED(event);
    if (!cfg->temp_layer_extend_on_other_input ||
        !data->temp_layer_enabled || !data->temp_layer_layer_active ||
        data->temp_layer_keep_active || data->aim_hold_count > 0 ||
        data->gesture_hold_count > 0) {
        return;
    }
    k_work_reschedule(&data->temp_layer_deactivation_work,
                      K_MSEC(data->temp_layer_deactivation_delay_ms));
}

/* Temp-layer activation with an activation distance (see
 * rip_temp_layer_math.h): one event's output after acceleration and scaling,
 * before aim and smoothing, so it is about cursor pixels, AIM's slowdown does
 * not make the layer harder to reach and smoothing does not delay it. The
 * motion that aim, smoothing and inertia release later goes straight to the
 * HID report and never counts. The layer activates once the burst has reached
 * the distance and no key was pressed within the activation delay. */
static void temp_layer_count_distance(struct runtime_processor_data *data, int32_t value) {
    if (!data->temp_layer_enabled || data->temp_layer_activation_distance == 0 ||
        data->temp_layer_layer_active || value == 0) {
        return;
    }

    int64_t now = k_uptime_get();
    if (!rip_temp_layer_distance_add(&data->temp_layer_distance, value, (uint32_t)now,
                                     data->temp_layer_activation_distance)) {
        return;
    }
    if (data->last_keypress_timestamp == 0 ||
        (now - data->last_keypress_timestamp) >= data->temp_layer_activation_delay_ms) {
        k_work_reschedule(&data->temp_layer_activation_work, K_NO_WAIT);
    }
}

/*
 * Gestures (see rip_gesture_math.h). The processor the gesture behaviors act
 * on (gesture_dev, found when they start) watches every motion event it gets:
 * while a gesture key is held, and after the release until the ball stops,
 * the motion moves nothing, and a flick fires its direction's shortcut once.
 * Any other processor leaves that motion alone meanwhile, so a held gesture
 * key works the same on a layer where it would turn the motion into scroll.
 */
static const struct device *gesture_dev;

BUILD_ASSERT(RIP_TILT_FIXED_ONE == RIP_GESTURE_MILLI,
             "Gesture milli-counts use the scale of the rotation matrix");
BUILD_ASSERT(RIP_GESTURE_DIRS == ZMK_INPUT_PROCESSOR_GESTURE_DIRS,
             "Gesture directions differ from the public API");

/* A keycode a gesture may fire: 0 (nothing) or a key or consumer usage, as
 * &kp's parameter (implicit modifiers in the top byte). */
static bool gesture_keycode_valid(uint32_t keycode) {
    if (keycode == 0) {
        return true;
    }
    uint16_t page = ZMK_HID_USAGE_PAGE(keycode);
    return (page == 0 || page == HID_USAGE_KEY || page == HID_USAGE_CONSUMER) &&
           ZMK_HID_USAGE_ID(keycode) != 0;
}

/* One event in the cursor frame, in milli-counts: the processor's rotation
 * and tilt (without the remainders carried between events, which only round),
 * inversion and swap, so "right" is where the cursor would have gone. */
static void gesture_cursor_delta(const struct runtime_processor_data *data, bool is_x,
                                 int32_t value, int32_t *dx, int32_t *dy) {
    // Times the x1000 matrix stays well inside int32
    int32_t v = CLAMP(value, INT16_MIN, INT16_MAX);
    int32_t x;
    int32_t y;
    if (data->transform_identity) {
        x = is_x ? v * RIP_GESTURE_MILLI : 0;
        y = is_x ? 0 : v * RIP_GESTURE_MILLI;
    } else {
        x = (is_x ? data->transform.xx : data->transform.xy) * v;
        y = (is_x ? data->transform.yx : data->transform.yy) * v;
    }
    if (data->x_invert) {
        x = -x;
    }
    if (data->y_invert) {
        y = -y;
    }
    if (data->xy_swap_enabled && !data->xy_to_scroll_enabled) {
        int32_t t = x;
        x = y;
        y = t;
    }
    *dx = x;
    *dy = y;
}

/* Whether gesture keys hold the ball at now: one is held, or the last one
 * went up and the ball has not been still since. Call with gesture_lock held,
 * before the event being handled is added. */
static bool gesture_engaged(struct runtime_processor_data *data, uint32_t now) {
    if (data->gesture_hold_count > 0) {
        return true;
    }
    if (data->gesture_draining &&
        !rip_gesture_draining(&data->gesture, data->gesture_up_ms, now)) {
        data->gesture_draining = false;
    }
    return data->gesture_draining;
}

/* A motion event of gesture_dev, held or not, so the detector always knows
 * whether the ball is still. Returns true while gesture keys hold the ball:
 * the event then moves nothing. A flick hands its direction to the fire
 * work. Runs on the input thread. */
static bool gesture_handle_event(struct runtime_processor_data *data, struct input_event *event,
                                 bool is_x) {
    uint32_t now = (uint32_t)k_uptime_get();
    int32_t dx;
    int32_t dy;
    gesture_cursor_delta(data, is_x, event->value, &dx, &dy);
    const struct rip_gesture_params params = {
        .distance = data->gesture_distance,
        .still_ms = data->gesture_still_ms,
        .window_ms = GESTURE_WINDOW_MS,
    };

    k_spinlock_key_t key = k_spin_lock(&data->gesture_lock);
    data->gesture_input_dev = event->dev;
    bool engaged = gesture_engaged(data, now);
    int dir = rip_gesture_event(&data->gesture, &params, data->gesture_hold_count > 0, dx, dy,
                                event->sync, now);
    if (dir != RIP_GESTURE_NONE) {
        if (data->gesture_pending_count < ARRAY_SIZE(data->gesture_pending)) {
            data->gesture_pending[data->gesture_pending_count++] = (int8_t)dir;
        } else {
            LOG_WRN("Gesture %d dropped: the fire work is behind", dir);
        }
    }
    k_spin_unlock(&data->gesture_lock, key);

    if (dir != RIP_GESTURE_NONE) {
        k_work_submit(&data->gesture_fire_work);
    }
    if (engaged) {
        event->value = 0;
    }
    return engaged;
}

/* For processors other than gesture_dev: whether its gesture keys hold the
 * ball this event comes from, so the motion is left for gesture_dev. Motion
 * of another device (a second pointing device, a scroll listener) is never
 * gesture_dev's and is handled as usual. */
static bool gesture_holds_ball(const struct input_event *event) {
    struct runtime_processor_data *owner = gesture_dev->data;
    uint32_t now = (uint32_t)k_uptime_get();
    k_spinlock_key_t key = k_spin_lock(&owner->gesture_lock);
    bool engaged = event->dev == owner->gesture_input_dev && gesture_engaged(owner, now);
    k_spin_unlock(&owner->gesture_lock, key);
    return engaged;
}

/* Motion of gesture_dev's ball that another processor uses up (scroll on its
 * layer) never reaches gesture_dev, which only needs to know that the ball
 * moves. */
static void gesture_note_motion(const struct input_event *event) {
    struct runtime_processor_data *owner = gesture_dev->data;
    uint32_t now = (uint32_t)k_uptime_get();
    uint32_t moved = input_value_magnitude((int16_t)CLAMP(event->value, INT16_MIN, INT16_MAX)) *
                     RIP_GESTURE_MILLI;
    k_spinlock_key_t key = k_spin_lock(&owner->gesture_lock);
    if (event->dev == owner->gesture_input_dev) {
        rip_gesture_note_motion(&owner->gesture, moved, now);
    }
    k_spin_unlock(&owner->gesture_lock, key);
}

/* Gesture shortcuts are raised with timestamp 0: a gesture is not typing, so
 * it must not count as the last key tapped for a hold-tap's
 * require-prior-idle-ms (which would make the next press of a gesture tap
 * key a tap at once and type its letter). ZMK keeps the newest timestamp, so
 * 0 never replaces it. */
#define GESTURE_KEY_TIMESTAMP 0

/* The shortcut a gesture is tapping goes up (system work queue). */
static void gesture_release_tap(struct runtime_processor_data *data) {
    if (data->gesture_tapped_keycode == 0) {
        return;
    }
    raise_zmk_keycode_state_changed_from_encoded(data->gesture_tapped_keycode, false,
                                                 GESTURE_KEY_TIMESTAMP);
    data->gesture_tapped_keycode = 0;
}

static void gesture_release_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct runtime_processor_data *data =
        CONTAINER_OF(dwork, struct runtime_processor_data, gesture_release_work);
    gesture_release_tap(data);
}

/* A flick fired: tap its shortcut (down, GESTURE_TAP_MS, up) on the system
 * work queue like a key, never held, so the host does not repeat it. In test
 * mode Studio is told instead and nothing is typed. */
static void gesture_fire_work_handler(struct k_work *work) {
    struct runtime_processor_data *data =
        CONTAINER_OF(work, struct runtime_processor_data, gesture_fire_work);

    for (;;) {
        int64_t now = k_uptime_get();
        k_spinlock_key_t key = k_spin_lock(&data->gesture_lock);
        if (data->gesture_pending_count == 0) {
            k_spin_unlock(&data->gesture_lock, key);
            return;
        }
        int dir = data->gesture_pending[0];
        data->gesture_pending_count--;
        memmove(&data->gesture_pending[0], &data->gesture_pending[1],
                data->gesture_pending_count * sizeof(data->gesture_pending[0]));
        uint32_t keycode = dir >= 0 && dir < RIP_GESTURE_DIRS ? data->gesture_keycodes[dir] : 0;
        bool test = data->gesture_test_until > now;
        k_spin_unlock(&data->gesture_lock, key);
        if (dir < 0 || dir >= RIP_GESTURE_DIRS) {
            continue;
        }

        LOG_INF("Gesture %d fired: 0x%08x%s", dir, (unsigned int)keycode,
                test ? " (test, not sent)" : "");
        if (test) {
            raise_zmk_input_processor_gesture_fired((struct zmk_input_processor_gesture_fired){
                .dev = data->dev, .direction = (uint8_t)dir, .keycode = keycode});
            continue;
        }
        if (keycode == 0) {
            continue;
        }
        // One tap at a time: one still down goes up first
        k_work_cancel_delayable(&data->gesture_release_work);
        gesture_release_tap(data);
        raise_zmk_keycode_state_changed_from_encoded(keycode, true, GESTURE_KEY_TIMESTAMP);
        data->gesture_tapped_keycode = keycode;
        k_work_schedule(&data->gesture_release_work, K_MSEC(GESTURE_TAP_MS));
    }
}

static int runtime_processor_handle_event(const struct device *dev, struct input_event *event,
                                          uint32_t param1, uint32_t param2,
                                          struct zmk_input_processor_state *state) {
    const struct runtime_processor_config *cfg = dev->config;
    struct runtime_processor_data *data = dev->data;

    if (event->type != cfg->type) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    int x_idx = code_idx(event->code, cfg->x_codes, cfg->x_codes_len);
    int y_idx = code_idx(event->code, cfg->y_codes, cfg->y_codes_len);

    if (x_idx < 0 && y_idx < 0) {
        temp_layer_extend_on_other_input(cfg, data, event);
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // Gestures before everything else, on every layer: while gesture keys
    // hold the ball its motion moves nothing (see gesture_handle_event)
    if (gesture_dev == dev) {
        if (gesture_handle_event(data, event, x_idx >= 0)) {
            return ZMK_INPUT_PROC_CONTINUE;
        }
    } else if (gesture_dev != NULL && gesture_holds_ball(event)) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // Check if processor should be active for current layers
    if (!is_processor_active_for_current_layers(data->active_layers)) {
        if (data->inertia_active) {
            inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_LAYER_INACTIVE);
        }
        if (data->accel_state.count != 0) {
            data->accel_state = (struct rip_accel_state){0};
        }
        return ZMK_INPUT_PROC_CONTINUE;
    }

    // Motion this processor uses for something else than gesture_dev's cursor
    // (scroll on its layer) still tells gesture_dev that the ball moves
    if (gesture_dev != NULL && gesture_dev != dev) {
        gesture_note_motion(event);
    }

    // An aim hold that moves the ball is in use: the tap of its tap & hold
    // key types nothing (see zmk_input_processor_runtime_take_ball_used)
    if (data->aim_hold_count > 0 && event->value != 0) {
        atomic_add(&data->aim_hold_motion, (atomic_val_t)input_value_magnitude(event->value));
    }

    // Drop input for a moment after a mouse button press or release, so a
    // thumb that shifts while a finger clicks does not move the cursor off
    // its target (see click_freeze_handle_button).
    if (data->click_freeze_until != 0) {
        if (k_uptime_get() < data->click_freeze_until) {
            event->value = 0;
            return ZMK_INPUT_PROC_CONTINUE;
        }
        data->click_freeze_until = 0;
    }

    bool is_x = (x_idx >= 0);
    int16_t value = event->value;

    // Apply code mapping (XY swap and XY-to-scroll)
    // These mappings are mutually exclusive - XY-to-scroll takes precedence
    if (data->xy_to_scroll_enabled) {
        // Map X/Y to horizontal/vertical scroll
        // X (0x00) -> HWHEEL (0x06), Y (0x01) -> WHEEL (0x08)
        if (is_x) {
            event->code = INPUT_REL_HWHEEL;
            LOG_DBG("XY-to-scroll: mapped X to HWHEEL");
        } else {
            event->code = INPUT_REL_WHEEL;
            LOG_DBG("XY-to-scroll: mapped Y to WHEEL");
        }
    } else if (data->xy_swap_enabled) {
        // Swap X and Y axes
        // X (0x00) -> Y (0x01), Y (0x01) -> X (0x00)
        if (is_x) {
            event->code = INPUT_REL_Y;
            LOG_DBG("XY-swap: swapped X to Y");
        } else {
            event->code = INPUT_REL_X;
            LOG_DBG("XY-swap: swapped Y to X");
        }
    }

    // Handle temp-layer layer activation (with an activation distance it is
    // checked after scaling instead, see temp_layer_count_distance)
    if (data->temp_layer_enabled && event->value != 0) {
        int64_t now = k_uptime_get();
        data->last_input_timestamp = now;

        // Check if we should activate the layer
        if (!data->temp_layer_layer_active && data->temp_layer_activation_distance == 0) {
            // Only activate if no key press within activation delay window
            if (data->last_keypress_timestamp == 0 ||
                (now - data->last_keypress_timestamp) >= data->temp_layer_activation_delay_ms) {
                // Schedule activation
                k_work_reschedule(&data->temp_layer_activation_work, K_NO_WAIT);
            }
        }
    }

    // Apply rotation and tilt compensation: M = M_tilt * R(rotation), x1000
    // fixed point. Each event is applied on its own axis at once; its cross
    // term is carried to the other axis' next event (see rip_tilt_math.h), so
    // drivers that report only the moving axis lose nothing. Skipped entirely
    // when M is the identity (rotation 0 and tilt 0).
    if (!data->transform_identity) {
        event->value = rip_tilt_apply_event(&data->transform, is_x, value, k_uptime_get(),
                                            &data->transform_state);
    }

    // Apply axis inversion after rotation
    if ((is_x && data->x_invert) || (!is_x && data->y_invert)) {
        event->value = -event->value;
    }
    value = event->value;

    // Apply axis snapping if configured
    if (data->axis_snap_mode != ZMK_INPUT_PROCESSOR_AXIS_SNAP_MODE_NONE && event->value != 0) {
        int64_t now = k_uptime_get();
        bool is_snapped_axis =
            (data->axis_snap_mode == ZMK_INPUT_PROCESSOR_AXIS_SNAP_MODE_X && is_x) ||
            (data->axis_snap_mode == ZMK_INPUT_PROCESSOR_AXIS_SNAP_MODE_Y && !is_x);
        bool is_cross_axis = !is_snapped_axis;

        // Decay accumulator over time
        if (data->axis_snap_timeout_ms > 0 && data->axis_snap_last_decay_timestamp > 0) {
            int64_t elapsed = now - data->axis_snap_last_decay_timestamp;
            if (elapsed > 0) {
                // Decay rate: threshold per timeout period
                // Decay every 50ms
                int64_t decay_periods = elapsed / 50;
                if (decay_periods > 0) {
                    int16_t decay_per_50ms =
                        data->axis_snap_threshold / (data->axis_snap_timeout_ms / 50);
                    if (decay_per_50ms < 1) {
                        decay_per_50ms = 1; // Minimum decay of 1
                    }

                    // Capped instead of truncated to int16: after a long pause (about 18 s
                    // at 90 per period) the product wrapped to a small or negative decay
                    // and left the cross axis unlocked.
                    int total_decay = (int)MIN(decay_per_50ms * decay_periods, INT16_MAX);

                    // Decay towards zero
                    if (data->axis_snap_cross_axis_accum > 0) {
                        data->axis_snap_cross_axis_accum -= total_decay;
                        if (data->axis_snap_cross_axis_accum < 0) {
                            data->axis_snap_cross_axis_accum = 0;
                        }
                    } else if (data->axis_snap_cross_axis_accum < 0) {
                        data->axis_snap_cross_axis_accum += total_decay;
                        if (data->axis_snap_cross_axis_accum > 0) {
                            data->axis_snap_cross_axis_accum = 0;
                        }
                    }

                    data->axis_snap_last_decay_timestamp = now;
                    LOG_DBG("Axis snap: decayed accum to %d (decay=%d)",
                            data->axis_snap_cross_axis_accum, total_decay);
                }
            }
        }

        bool should_unlock = false;

        if (is_cross_axis) {
            int16_t current_abs_accum = data->axis_snap_cross_axis_accum < 0
                                            ? -data->axis_snap_cross_axis_accum
                                            : data->axis_snap_cross_axis_accum;
            bool is_unsnapped = current_abs_accum >= data->axis_snap_threshold;

            if (is_unsnapped) {
                // Just increase accumulator when already unsnapped
                data->axis_snap_cross_axis_accum = current_abs_accum + (value > 0 ? value : -value);
            } else {
                // Accumulate normally when snapped (no abs)
                data->axis_snap_cross_axis_accum += value;
            }
            // Reset decay timer on movement
            data->axis_snap_last_decay_timestamp = now;

            // Check if threshold exceeded (check absolute value)
            int16_t abs_accum = data->axis_snap_cross_axis_accum < 0
                                    ? -data->axis_snap_cross_axis_accum
                                    : data->axis_snap_cross_axis_accum;

            if (abs_accum >= data->axis_snap_threshold) {
                LOG_DBG("Axis snap: unlocked (threshold=%d exceeded with accum=%d)",
                        data->axis_snap_threshold, data->axis_snap_cross_axis_accum);
                // cap the accumulator to twice the threshold so that it decays
                // under threshold within timeout
                if (abs_accum > data->axis_snap_threshold * 2) {
                    data->axis_snap_cross_axis_accum =
                        (data->axis_snap_cross_axis_accum > 0 ? data->axis_snap_threshold
                                                              : -data->axis_snap_threshold) *
                        2;
                }
            } else {
                // Suppress cross-axis movement while locked
                event->value = 0;
                LOG_DBG("Axis snap: suppressing cross-axis movement (accum=%d, "
                        "threshold=%d)",
                        data->axis_snap_cross_axis_accum, data->axis_snap_threshold);
            }
        }

        // Update value after snap processing
        value = event->value;
    }

    // Inertia measures speed before scaling. Keep the scaled report too, so
    // its entry threshold remains in the units seen by the user.
    int16_t inertia_raw_value = event->value;

    // Speed-dependent pointer acceleration (see rip_accel_math.h): slow motion
    // gets accel_min_percent, fast motion stays 1:1. Runs before scaling and
    // after inertia_raw_value, so inertia still measures the raw input.
    // Skipped entirely when off (100%).
    if (data->accel_min_percent < RIP_ACCEL_OFF_PERCENT) {
        event->value = rip_accel_apply_event(
            &data->accel_state, is_x, event->value, (uint32_t)k_uptime_get(), ACCEL_WINDOW_MS,
            data->accel_min_percent, cfg->accel_speed_low, cfg->accel_speed_high);
    }

    // Apply scaling
    if (data->scale_multiplier > 0 && data->scale_divisor > 0) {
        scale_val(event, data->scale_multiplier, data->scale_divisor, state);
        value = event->value;
    }

    temp_layer_count_distance(data, event->value);

    // No inertia while aiming: it would measure the speed before the slowdown
    // and glide past the target (aim press also stops a running glide)
    if (data->aim_hold_count == 0) {
        inertia_handle_physical_event(data, is_x ? 0 : 1, event, inertia_raw_value);
    }

    // Aim mode (slowed and smoothed while an aim key is held) and normal
    // smoothing on the final cursor values
    aim_handle_event(data, event);

    // Schedule deactivation after input stops (held aim keeps the layer)
    if (data->temp_layer_enabled && data->temp_layer_layer_active &&
        !data->temp_layer_keep_active && data->aim_hold_count == 0) {
        k_work_reschedule(&data->temp_layer_deactivation_work,
                          K_MSEC(data->temp_layer_deactivation_delay_ms));
    }

    return ZMK_INPUT_PROC_CONTINUE;
}

static struct zmk_input_processor_driver_api runtime_processor_driver_api = {
    .handle_event = runtime_processor_handle_event,
};

#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)

/*
 * Persistence backend: one zmk-feature-custom-settings entry per processor
 * (subsystem "cormoran_rip", key = the processor's compile-time
 * processor-label), storing a version-byte-prefixed raw memcpy of the 15
 * persisted fields. See docs/design/custom-settings-storage.md.
 *
 * This id MUST match a registered ZMK_RPC_CUSTOM_SUBSYSTEM identifier: the
 * generic custom-settings ListSettings handler resolves every entry's
 * custom_subsystem_id to a registered subsystem index and silently drops any
 * entry whose id does not resolve (see setting_to_proto in
 * zmk-feature-custom-settings). We therefore reuse this module's own Studio RPC
 * subsystem id "cormoran_rip" (see src/studio/custom_handler.c) rather than a
 * separate namespace, so these entries appear in the generic list.
 *
 * Confidentiality is RPC_PUBLIC: the value (the packed blob) is exposed over the
 * generic custom-settings RPC's list/get alongside the key/meta. This module's
 * own Studio RPC remains the intended editing surface (there is deliberately no
 * zmk_custom_setting_changed listener re-applying generic writes to the device -
 * see the load path below), so the generic surface is for visibility/inspection.
 */
#define RIP_SETTINGS_SUBSYSTEM_ID "cormoran_rip"
#define RIP_SETTINGS_BLOB_VERSION 8

/*
 * The persisted-on-flash v1 struct for one processor's original 15 settings
 * fields. Keep this exact layout so existing saved settings remain readable.
 * Stored as a raw memcpy (see pack/unpack below) rather than a hand-serialized
 * byte stream: it is only ever written and read back by this same firmware
 * image, so the in-memory layout is a valid wire format, and the leading
 * version byte + exact-size check guard against a struct-layout change. New
 * versions retain an explicit reader for every compatible old version and use
 * devicetree defaults for fields that an old record lacks. This is the same
 * field set the module previously persisted;
 * there is no on-flash backward-compat requirement with the older
 * settings_save_one blob that predates custom-settings storage.
 */
struct rip_persist_v1 {
    uint32_t scale_multiplier;
    uint32_t scale_divisor;
    int32_t rotation_degrees;
    bool temp_layer_enabled;
    uint8_t temp_layer_layer;
    uint16_t temp_layer_activation_delay_ms;
    uint16_t temp_layer_deactivation_delay_ms;
    uint32_t active_layers;
    uint8_t axis_snap_mode;
    uint16_t axis_snap_threshold;
    uint16_t axis_snap_timeout_ms;
    bool xy_to_scroll_enabled;
    bool xy_swap_enabled;
    bool x_invert;
    bool y_invert;
};

/* v2 remains readable for devices already flashed with the first PR revision. */
struct rip_persist_v2 {
    struct rip_persist_v1 v1;
    bool inertia_enabled;
    uint16_t inertia_window_ms;
    uint16_t inertia_interval_ms;
    uint16_t inertia_threshold;
    uint8_t inertia_decay_percent;
    uint16_t inertia_fast_threshold;
    uint16_t inertia_fast_output_percent;
};

struct rip_persist_v3 {
    struct rip_persist_v2 v2;
    uint16_t inertia_normal_max_output;
};

/* v4 adds tilt compensation. 1 + sizeof(v4) is 61 bytes on 32-bit ARM
 * (v3 is 56 bytes; +1 tilt, +1 padding, +2 axis). */
struct rip_persist_v4 {
    struct rip_persist_v3 v3;
    uint8_t tilt_degrees;
    int16_t tilt_axis_degrees;
};

/* v5 adds pointer acceleration. Naturally aligned it would pad to 64 bytes
 * (a 65-byte blob, over the 64-byte value), so it is packed: 1 + sizeof(v5)
 * is 62 bytes on 32-bit ARM. Only ever copied whole with memcpy. */
struct rip_persist_v5 {
    struct rip_persist_v4 v4;
    uint8_t accel_min_percent;
} __packed;

/*
 * v6 adds aim mode. Appended to v5 the blob would be exactly 64 bytes, the
 * whole value (see the BUILD_ASSERT below), with nothing left for the next
 * field. So v6 is one flat packed struct that holds every field once with no
 * padding: 1 + sizeof(v6) is 52 bytes, leaving 12. The nested v1-v5 layouts
 * above are kept for reading old records. Only ever copied whole with memcpy.
 */
struct rip_persist_v6 {
    uint32_t scale_multiplier;
    uint32_t scale_divisor;
    int32_t rotation_degrees;
    uint32_t active_layers;
    bool temp_layer_enabled;
    uint8_t temp_layer_layer;
    uint16_t temp_layer_activation_delay_ms;
    uint16_t temp_layer_deactivation_delay_ms;
    uint8_t axis_snap_mode;
    uint16_t axis_snap_threshold;
    uint16_t axis_snap_timeout_ms;
    bool xy_to_scroll_enabled;
    bool xy_swap_enabled;
    bool x_invert;
    bool y_invert;
    bool inertia_enabled;
    uint16_t inertia_window_ms;
    uint16_t inertia_interval_ms;
    uint16_t inertia_threshold;
    uint8_t inertia_decay_percent;
    uint16_t inertia_fast_threshold;
    uint16_t inertia_fast_output_percent;
    uint16_t inertia_normal_max_output;
    uint8_t tilt_degrees;
    int16_t tilt_axis_degrees;
    uint8_t accel_min_percent;
    uint8_t aim_scale_percent;
    uint8_t aim_smoothing_ms;
} __packed;

/* v7 adds normal smoothing: v6 and one byte, so a v6 record is the start of
 * a v7 one. 1 + sizeof(v7) is 53 bytes, leaving 11. */
struct rip_persist_v7 {
    struct rip_persist_v6 v6;
    uint8_t smoothing_ms;
} __packed;

/* v8 adds the temp layer's activation distance: v7 and one byte, so a v7
 * record is the start of a v8 one. 1 + sizeof(v8) is 54 bytes, leaving 10. */
struct rip_persist_v8 {
    struct rip_persist_v7 v7;
    uint8_t temp_layer_activation_distance;
} __packed;

#define RIP_SETTINGS_BLOB_SIZE (1 + sizeof(struct rip_persist_v8))

/*
 * Gesture settings live in a second entry per processor
 * ("<processor-label>_gesture"), so the blob above keeps its v8 layout (and
 * older firmware still reads it) and the gesture keycodes have room of their
 * own. Same scheme: a version byte, then the packed struct.
 */
#define RIP_GESTURE_SETTINGS_KEY_SUFFIX "_gesture"
#define RIP_GESTURE_SETTINGS_VERSION 1

struct rip_gesture_persist_v1 {
    uint32_t keycodes[RIP_GESTURE_DIRS]; // left, right, up, down; 0 = nothing
    uint16_t distance;
    uint16_t still_ms;
} __packed;

#define RIP_GESTURE_SETTINGS_SIZE (1 + sizeof(struct rip_gesture_persist_v1))

BUILD_ASSERT(RIP_GESTURE_SETTINGS_SIZE <= CONFIG_ZMK_CUSTOM_SETTINGS_VALUE_MAX_SIZE,
             "gesture settings blob exceeds CONFIG_ZMK_CUSTOM_SETTINGS_VALUE_MAX_SIZE");

/* The entry below is a plain ZMK_CUSTOM_SETTING_DEFINE BYTES setting, whose
 * value holds at most CONFIG_ZMK_CUSTOM_SETTINGS_VALUE_MAX_SIZE bytes; the
 * reads and writes go through struct zmk_custom_setting_value, sized the same.
 * custom-settings pins that Kconfig to 64 while its Studio protobuf schema is
 * enabled (it is also the single-frame RPC value bound), so it cannot simply
 * be raised. Fail the build rather than truncate. A blob that outgrows it needs
 * ZMK_CUSTOM_SETTING_DEFINE_SIZED with CONFIG_ZMK_CUSTOM_SETTINGS_LARGE_VALUE_MAX_SIZE
 * raised to match, read with zmk_custom_setting_read_into and written with
 * zmk_custom_setting_write_bytes. */
BUILD_ASSERT(RIP_SETTINGS_BLOB_SIZE <= CONFIG_ZMK_CUSTOM_SETTINGS_VALUE_MAX_SIZE,
             "runtime input processor settings blob exceeds "
             "CONFIG_ZMK_CUSTOM_SETTINGS_VALUE_MAX_SIZE; switch to "
             "ZMK_CUSTOM_SETTING_DEFINE_SIZED");

/* A genuinely empty BYTES default (size 0, "nothing persisted yet") - wrapped
 * in outer parens (not just braces) so the top-level comma inside the
 * initializer does not get parsed as a macro-argument separator when this is
 * passed into ZMK_CUSTOM_SETTING_DEFINE below. */
#define RIP_SETTINGS_EMPTY_BYTES_DEFAULT                                                           \
    ((struct zmk_custom_setting_value){.type = ZMK_CUSTOM_SETTING_VALUE_TYPE_BYTES, .size = 0})

/* Writes [version][raw struct] into buf (must be >= RIP_SETTINGS_BLOB_SIZE)
 * and returns the number of bytes written. Version byte is written separately
 * from the struct memcpy so no wrapper-struct padding enters the layout. */
static size_t pack_processor_settings(const struct runtime_processor_data *data, uint8_t *buf) {
    struct rip_persist_v7 v7 = {
        .v6 = {
            .scale_multiplier = data->persistent_scale_multiplier,
            .scale_divisor = data->persistent_scale_divisor,
            .rotation_degrees = data->persistent_rotation_degrees,
            .active_layers = data->persistent_active_layers,
            .temp_layer_enabled = data->persistent_temp_layer_enabled,
            .temp_layer_layer = data->persistent_temp_layer_layer,
            .temp_layer_activation_delay_ms = data->persistent_temp_layer_activation_delay_ms,
            .temp_layer_deactivation_delay_ms = data->persistent_temp_layer_deactivation_delay_ms,
            .axis_snap_mode = data->persistent_axis_snap_mode,
            .axis_snap_threshold = data->persistent_axis_snap_threshold,
            .axis_snap_timeout_ms = data->persistent_axis_snap_timeout_ms,
            .xy_to_scroll_enabled = data->persistent_xy_to_scroll_enabled,
            .xy_swap_enabled = data->persistent_xy_swap_enabled,
            .x_invert = data->persistent_x_invert,
            .y_invert = data->persistent_y_invert,
            .inertia_enabled = data->persistent_inertia_enabled,
            .inertia_window_ms = data->persistent_inertia_window_ms,
            .inertia_interval_ms = data->persistent_inertia_interval_ms,
            .inertia_threshold = data->persistent_inertia_threshold,
            .inertia_decay_percent = data->persistent_inertia_decay_percent,
            .inertia_fast_threshold = data->persistent_inertia_fast_threshold,
            .inertia_fast_output_percent = data->persistent_inertia_fast_output_percent,
            .inertia_normal_max_output = data->persistent_inertia_normal_max_output,
            .tilt_degrees = data->persistent_tilt_degrees,
            .tilt_axis_degrees = data->persistent_tilt_axis_degrees,
            .accel_min_percent = data->persistent_accel_min_percent,
            .aim_scale_percent = data->persistent_aim_scale_percent,
            .aim_smoothing_ms = data->persistent_aim_smoothing_ms,
        },
        .smoothing_ms = data->persistent_smoothing_ms,
    };
    struct rip_persist_v8 settings = {
        .v7 = v7,
        .temp_layer_activation_distance = data->persistent_temp_layer_activation_distance,
    };
    buf[0] = RIP_SETTINGS_BLOB_VERSION;
    memcpy(&buf[1], &settings, sizeof(settings));
    return RIP_SETTINGS_BLOB_SIZE;
}

/* Writes [version][raw struct] of the gesture baseline into buf (must be >=
 * RIP_GESTURE_SETTINGS_SIZE) and returns the number of bytes written. */
static size_t pack_gesture_settings(const struct runtime_processor_data *data, uint8_t *buf) {
    struct rip_gesture_persist_v1 settings = {
        .distance = data->persistent_gesture_distance,
        .still_ms = data->persistent_gesture_still_ms,
    };
    memcpy(settings.keycodes, data->persistent_gesture_keycodes, sizeof(settings.keycodes));
    buf[0] = RIP_GESTURE_SETTINGS_VERSION;
    memcpy(&buf[1], &settings, sizeof(settings));
    return RIP_GESTURE_SETTINGS_SIZE;
}

/* Apply a saved gesture entry to the live values and the baseline. */
static int unpack_and_apply_gesture_settings(struct runtime_processor_data *data,
                                             const uint8_t *buf, size_t len) {
    if (len != RIP_GESTURE_SETTINGS_SIZE || buf[0] != RIP_GESTURE_SETTINGS_VERSION) {
        return -EINVAL;
    }
    struct rip_gesture_persist_v1 saved;
    memcpy(&saved, &buf[1], sizeof(saved));
    for (int i = 0; i < RIP_GESTURE_DIRS; i++) {
        if (!gesture_keycode_valid(saved.keycodes[i])) {
            return -EINVAL;
        }
    }
    if (saved.distance != rip_gesture_clamp_distance(saved.distance) ||
        saved.still_ms != rip_gesture_clamp_still_ms(saved.still_ms)) {
        return -EINVAL;
    }

    k_spinlock_key_t key = k_spin_lock(&data->gesture_lock);
    memcpy(data->gesture_keycodes, saved.keycodes, sizeof(data->gesture_keycodes));
    data->gesture_distance = saved.distance;
    data->gesture_still_ms = saved.still_ms;
    k_spin_unlock(&data->gesture_lock, key);
    memcpy(data->persistent_gesture_keycodes, saved.keycodes,
           sizeof(data->persistent_gesture_keycodes));
    data->persistent_gesture_distance = saved.distance;
    data->persistent_gesture_still_ms = saved.still_ms;
    return 0;
}

/* v1 has no inertia fields; start from the current devicetree defaults.
 * v1-v3 have no tilt fields, v1-v4 no acceleration field, v1-v5 no aim fields,
 * v1-v6 no normal smoothing and v1-v7 no activation distance; those take the
 * devicetree defaults. */
static int unpack_and_apply_processor_settings(struct runtime_processor_data *data,
                                               const uint8_t *buf, size_t len) {
    const struct runtime_processor_config *cfg = data->dev->config;
    struct rip_persist_v2 settings = {0};
    uint16_t normal_max_output = data->persistent_inertia_normal_max_output;
    uint8_t tilt_degrees = rip_tilt_clamp_degrees(cfg->initial_tilt_degrees);
    int16_t tilt_axis_degrees = rip_tilt_normalize_axis_degrees(cfg->initial_tilt_axis_degrees);
    uint8_t accel_min_percent = rip_accel_clamp_min_percent(cfg->initial_accel_min_percent);
    uint8_t aim_scale_percent = rip_aim_clamp_percent(cfg->initial_aim_scale_percent);
    uint8_t aim_smoothing_ms = rip_aim_clamp_smoothing_ms(cfg->initial_aim_smoothing_ms);
    uint8_t smoothing_ms = rip_aim_clamp_smoothing_ms(cfg->initial_smoothing_ms);
    uint8_t activation_distance =
        rip_temp_layer_clamp_distance(cfg->initial_temp_layer_activation_distance);
    if (len == 1 + sizeof(struct rip_persist_v1) && buf[0] == 1) {
        memcpy(&settings.v1, &buf[1], sizeof(settings.v1));
        settings.inertia_enabled = data->persistent_inertia_enabled;
        settings.inertia_window_ms = data->persistent_inertia_window_ms;
        settings.inertia_interval_ms = data->persistent_inertia_interval_ms;
        settings.inertia_threshold = data->persistent_inertia_threshold;
        settings.inertia_decay_percent = data->persistent_inertia_decay_percent;
        settings.inertia_fast_threshold = data->persistent_inertia_fast_threshold;
        settings.inertia_fast_output_percent = data->persistent_inertia_fast_output_percent;
    } else if (len == 1 + sizeof(settings) && buf[0] == 2) {
        memcpy(&settings, &buf[1], sizeof(settings));
    } else if (len == 1 + sizeof(struct rip_persist_v3) && buf[0] == 3) {
        struct rip_persist_v3 saved;
        memcpy(&saved, &buf[1], sizeof(saved));
        settings = saved.v2;
        normal_max_output = saved.inertia_normal_max_output;
    } else if (len == 1 + sizeof(struct rip_persist_v4) && buf[0] == 4) {
        struct rip_persist_v4 saved;
        memcpy(&saved, &buf[1], sizeof(saved));
        settings = saved.v3.v2;
        normal_max_output = saved.v3.inertia_normal_max_output;
        tilt_degrees = saved.tilt_degrees;
        tilt_axis_degrees = saved.tilt_axis_degrees;
    } else if (len == 1 + sizeof(struct rip_persist_v5) && buf[0] == 5) {
        struct rip_persist_v5 saved;
        memcpy(&saved, &buf[1], sizeof(saved));
        settings = saved.v4.v3.v2;
        normal_max_output = saved.v4.v3.inertia_normal_max_output;
        tilt_degrees = saved.v4.tilt_degrees;
        tilt_axis_degrees = saved.v4.tilt_axis_degrees;
        accel_min_percent = saved.accel_min_percent;
    } else if ((len == 1 + sizeof(struct rip_persist_v6) && buf[0] == 6) ||
               (len == 1 + sizeof(struct rip_persist_v7) && buf[0] == 7) ||
               (len == RIP_SETTINGS_BLOB_SIZE && buf[0] == RIP_SETTINGS_BLOB_VERSION)) {
        // v6 is the start of v7 and v7 the start of v8; v7 and v8 have the
        // normal smoothing byte, only v8 the activation distance
        struct rip_persist_v8 saved8;
        memcpy(&saved8, &buf[1], len - 1);
        if (buf[0] >= 7) {
            smoothing_ms = saved8.v7.smoothing_ms;
        }
        if (buf[0] == RIP_SETTINGS_BLOB_VERSION) {
            activation_distance = saved8.temp_layer_activation_distance;
        }
        struct rip_persist_v6 saved = saved8.v7.v6;
        settings.v1 = (struct rip_persist_v1){
            .scale_multiplier = saved.scale_multiplier,
            .scale_divisor = saved.scale_divisor,
            .rotation_degrees = saved.rotation_degrees,
            .temp_layer_enabled = saved.temp_layer_enabled,
            .temp_layer_layer = saved.temp_layer_layer,
            .temp_layer_activation_delay_ms = saved.temp_layer_activation_delay_ms,
            .temp_layer_deactivation_delay_ms = saved.temp_layer_deactivation_delay_ms,
            .active_layers = saved.active_layers,
            .axis_snap_mode = saved.axis_snap_mode,
            .axis_snap_threshold = saved.axis_snap_threshold,
            .axis_snap_timeout_ms = saved.axis_snap_timeout_ms,
            .xy_to_scroll_enabled = saved.xy_to_scroll_enabled,
            .xy_swap_enabled = saved.xy_swap_enabled,
            .x_invert = saved.x_invert,
            .y_invert = saved.y_invert,
        };
        settings.inertia_enabled = saved.inertia_enabled;
        settings.inertia_window_ms = saved.inertia_window_ms;
        settings.inertia_interval_ms = saved.inertia_interval_ms;
        settings.inertia_threshold = saved.inertia_threshold;
        settings.inertia_decay_percent = saved.inertia_decay_percent;
        settings.inertia_fast_threshold = saved.inertia_fast_threshold;
        settings.inertia_fast_output_percent = saved.inertia_fast_output_percent;
        normal_max_output = saved.inertia_normal_max_output;
        tilt_degrees = saved.tilt_degrees;
        tilt_axis_degrees = saved.tilt_axis_degrees;
        accel_min_percent = saved.accel_min_percent;
        aim_scale_percent = saved.aim_scale_percent;
        aim_smoothing_ms = saved.aim_smoothing_ms;
    } else {
        return -EINVAL;
    }
    if (buf[0] != 1) {
        if (settings.inertia_window_ms == 0 || settings.inertia_window_ms > 60000 ||
            settings.inertia_interval_ms == 0 || settings.inertia_interval_ms > 60000 ||
            settings.inertia_threshold == 0 || settings.inertia_decay_percent > 100 ||
            settings.inertia_fast_output_percent < 100 ||
            settings.inertia_fast_output_percent > 1000 || normal_max_output > INT16_MAX) {
            return -EINVAL;
        }
    }
    if (tilt_degrees > RIP_TILT_MAX_DEGREES || tilt_axis_degrees <= -90 || tilt_axis_degrees > 90) {
        return -EINVAL;
    }
    if (accel_min_percent < RIP_ACCEL_MIN_PERCENT || accel_min_percent > RIP_ACCEL_OFF_PERCENT) {
        return -EINVAL;
    }
    if (aim_scale_percent < RIP_AIM_MIN_PERCENT || aim_scale_percent > RIP_AIM_MAX_PERCENT ||
        aim_smoothing_ms > RIP_AIM_MAX_SMOOTHING_MS || smoothing_ms > RIP_AIM_MAX_SMOOTHING_MS) {
        return -EINVAL;
    }
    if (activation_distance > RIP_TEMP_LAYER_MAX_ACTIVATION_DISTANCE) {
        return -EINVAL;
    }

    const struct rip_persist_v1 *v1 = &settings.v1;
    data->persistent_scale_multiplier = data->scale_multiplier = v1->scale_multiplier;
    data->persistent_scale_divisor = data->scale_divisor = v1->scale_divisor;
    data->persistent_rotation_degrees = data->rotation_degrees = v1->rotation_degrees;
    data->persistent_tilt_degrees = data->tilt_degrees = tilt_degrees;
    data->persistent_tilt_axis_degrees = data->tilt_axis_degrees = tilt_axis_degrees;
    data->persistent_accel_min_percent = data->accel_min_percent = accel_min_percent;
    data->accel_state = (struct rip_accel_state){0};
    data->persistent_aim_scale_percent = data->aim_scale_percent = aim_scale_percent;
    data->persistent_aim_smoothing_ms = data->aim_smoothing_ms = aim_smoothing_ms;
    data->persistent_smoothing_ms = data->smoothing_ms = smoothing_ms;
    data->persistent_temp_layer_enabled = data->temp_layer_enabled = v1->temp_layer_enabled;
    data->persistent_temp_layer_layer = data->temp_layer_layer = v1->temp_layer_layer;
    data->persistent_temp_layer_activation_delay_ms = data->temp_layer_activation_delay_ms =
        v1->temp_layer_activation_delay_ms;
    data->persistent_temp_layer_deactivation_delay_ms = data->temp_layer_deactivation_delay_ms =
        v1->temp_layer_deactivation_delay_ms;
    data->persistent_temp_layer_activation_distance = data->temp_layer_activation_distance =
        activation_distance;
    data->persistent_active_layers = data->active_layers = v1->active_layers;
    data->persistent_axis_snap_mode = data->axis_snap_mode = v1->axis_snap_mode;
    data->persistent_axis_snap_threshold = data->axis_snap_threshold = v1->axis_snap_threshold;
    data->persistent_axis_snap_timeout_ms = data->axis_snap_timeout_ms = v1->axis_snap_timeout_ms;
    data->persistent_xy_to_scroll_enabled = data->xy_to_scroll_enabled = v1->xy_to_scroll_enabled;
    data->persistent_xy_swap_enabled = data->xy_swap_enabled = v1->xy_swap_enabled;
    data->persistent_x_invert = data->x_invert = v1->x_invert;
    data->persistent_y_invert = data->y_invert = v1->y_invert;
    data->persistent_inertia_enabled = data->inertia_enabled = settings.inertia_enabled;
    data->persistent_inertia_window_ms = data->inertia_window_ms = settings.inertia_window_ms;
    data->persistent_inertia_interval_ms = data->inertia_interval_ms = settings.inertia_interval_ms;
    data->persistent_inertia_threshold = data->inertia_threshold = settings.inertia_threshold;
    data->persistent_inertia_decay_percent = data->inertia_decay_percent =
        settings.inertia_decay_percent;
    data->persistent_inertia_fast_threshold = data->inertia_fast_threshold =
        settings.inertia_fast_threshold;
    data->persistent_inertia_fast_output_percent = data->inertia_fast_output_percent =
        settings.inertia_fast_output_percent;
    data->persistent_inertia_normal_max_output = data->inertia_normal_max_output =
        normal_max_output;
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);
    update_rotation_values(data);
    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_TEST)
/* The processor's baseline in the nested v5 layout; v1-v4 are its prefixes.
 * The packed blob starts with v6 (the start of v7). */
static struct rip_persist_v5 test_legacy_v5(const struct runtime_processor_data *data) {
    uint8_t current[RIP_SETTINGS_BLOB_SIZE];
    pack_processor_settings(data, current);
    struct rip_persist_v6 v6;
    memcpy(&v6, &current[1], sizeof(v6));
    struct rip_persist_v5 v5 = {
        .v4 = {
            .v3 = {
                .v2 = {
                    .v1 = {
                        .scale_multiplier = v6.scale_multiplier,
                        .scale_divisor = v6.scale_divisor,
                        .rotation_degrees = v6.rotation_degrees,
                        .temp_layer_enabled = v6.temp_layer_enabled,
                        .temp_layer_layer = v6.temp_layer_layer,
                        .temp_layer_activation_delay_ms = v6.temp_layer_activation_delay_ms,
                        .temp_layer_deactivation_delay_ms = v6.temp_layer_deactivation_delay_ms,
                        .active_layers = v6.active_layers,
                        .axis_snap_mode = v6.axis_snap_mode,
                        .axis_snap_threshold = v6.axis_snap_threshold,
                        .axis_snap_timeout_ms = v6.axis_snap_timeout_ms,
                        .xy_to_scroll_enabled = v6.xy_to_scroll_enabled,
                        .xy_swap_enabled = v6.xy_swap_enabled,
                        .x_invert = v6.x_invert,
                        .y_invert = v6.y_invert,
                    },
                    .inertia_enabled = v6.inertia_enabled,
                    .inertia_window_ms = v6.inertia_window_ms,
                    .inertia_interval_ms = v6.inertia_interval_ms,
                    .inertia_threshold = v6.inertia_threshold,
                    .inertia_decay_percent = v6.inertia_decay_percent,
                    .inertia_fast_threshold = v6.inertia_fast_threshold,
                    .inertia_fast_output_percent = v6.inertia_fast_output_percent,
                },
                .inertia_normal_max_output = v6.inertia_normal_max_output,
            },
            .tilt_degrees = v6.tilt_degrees,
            .tilt_axis_degrees = v6.tilt_axis_degrees,
        },
        .accel_min_percent = v6.accel_min_percent,
    };
    return v5;
}

int zmk_input_processor_runtime_test_apply_legacy_v1(const struct device *dev) {
    if (!dev) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    struct rip_persist_v1 settings = test_legacy_v5(data).v4.v3.v2.v1;
    settings.rotation_degrees = 42;
    uint8_t blob[1 + sizeof(settings)] = {1};
    memcpy(&blob[1], &settings, sizeof(settings));
    return unpack_and_apply_processor_settings(data, blob, sizeof(blob));
}

int zmk_input_processor_runtime_test_apply_legacy_v2(const struct device *dev) {
    if (!dev) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    struct rip_persist_v2 settings = test_legacy_v5(data).v4.v3.v2;
    settings.v1.rotation_degrees = 43;
    uint8_t blob[1 + sizeof(settings)] = {2};
    memcpy(&blob[1], &settings, sizeof(settings));
    return unpack_and_apply_processor_settings(data, blob, sizeof(blob));
}

int zmk_input_processor_runtime_test_apply_legacy_v3(const struct device *dev) {
    if (!dev) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    struct rip_persist_v3 settings = test_legacy_v5(data).v4.v3;
    settings.v2.v1.rotation_degrees = 44;
    uint8_t blob[1 + sizeof(settings)] = {3};
    memcpy(&blob[1], &settings, sizeof(settings));
    return unpack_and_apply_processor_settings(data, blob, sizeof(blob));
}

int zmk_input_processor_runtime_test_apply_legacy_v4(const struct device *dev) {
    if (!dev) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    struct rip_persist_v4 settings = test_legacy_v5(data).v4;
    settings.v3.v2.v1.rotation_degrees = 45;
    uint8_t blob[1 + sizeof(settings)] = {4};
    memcpy(&blob[1], &settings, sizeof(settings));
    return unpack_and_apply_processor_settings(data, blob, sizeof(blob));
}

int zmk_input_processor_runtime_test_apply_legacy_v6(const struct device *dev) {
    if (!dev) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    uint8_t current[RIP_SETTINGS_BLOB_SIZE];
    pack_processor_settings(data, current);
    struct rip_persist_v7 settings;
    memcpy(&settings, &current[1], sizeof(settings));
    settings.v6.rotation_degrees = 47;
    uint8_t blob[1 + sizeof(settings.v6)] = {6};
    memcpy(&blob[1], &settings.v6, sizeof(settings.v6));
    return unpack_and_apply_processor_settings(data, blob, sizeof(blob));
}

int zmk_input_processor_runtime_test_apply_legacy_v7(const struct device *dev) {
    if (!dev) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    uint8_t current[RIP_SETTINGS_BLOB_SIZE];
    pack_processor_settings(data, current);
    struct rip_persist_v8 settings;
    memcpy(&settings, &current[1], sizeof(settings));
    settings.v7.v6.rotation_degrees = 48;
    uint8_t blob[1 + sizeof(settings.v7)] = {7};
    memcpy(&blob[1], &settings.v7, sizeof(settings.v7));
    return unpack_and_apply_processor_settings(data, blob, sizeof(blob));
}

int zmk_input_processor_runtime_test_apply_legacy_v5(const struct device *dev) {
    if (!dev) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    struct rip_persist_v5 settings = test_legacy_v5(data);
    settings.v4.v3.v2.v1.rotation_degrees = 46;
    uint8_t blob[1 + sizeof(settings)] = {5};
    memcpy(&blob[1], &settings, sizeof(settings));
    return unpack_and_apply_processor_settings(data, blob, sizeof(blob));
}
#endif

/* Pack the processor's current baseline (both entries) and flush it to flash
 * immediately. */
static int save_processor_settings_now(const struct device *dev) {
    struct runtime_processor_data *data = dev->data;
    const struct runtime_processor_config *cfg = dev->config;

    struct zmk_custom_setting_value value = {.type = ZMK_CUSTOM_SETTING_VALUE_TYPE_BYTES};
    value.size = pack_processor_settings(data, value.bytes_value);

    int ret = zmk_custom_setting_write_by_key(RIP_SETTINGS_SUBSYSTEM_ID, cfg->name, &value,
                                              ZMK_CUSTOM_SETTING_WRITE_MODE_PERSIST);
    if (ret < 0) {
        LOG_ERR("Failed to save settings for %s: %d", cfg->name, ret);
    } else {
        LOG_INF("Saved settings for %s", cfg->name);
    }

    struct zmk_custom_setting_value gesture = {.type = ZMK_CUSTOM_SETTING_VALUE_TYPE_BYTES};
    gesture.size = pack_gesture_settings(data, gesture.bytes_value);
    int gesture_ret = zmk_custom_setting_write_by_key(
        RIP_SETTINGS_SUBSYSTEM_ID, cfg->gesture_settings_key, &gesture,
        ZMK_CUSTOM_SETTING_WRITE_MODE_PERSIST);
    if (gesture_ret < 0) {
        LOG_ERR("Failed to save gesture settings for %s: %d", cfg->name, gesture_ret);
    }
    return ret < 0 ? ret : gesture_ret;
}

static void save_processor_settings_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct runtime_processor_data *data =
        CONTAINER_OF(dwork, struct runtime_processor_data, save_work);
    save_processor_settings_now(data->dev);
}

static int schedule_save_processor_settings(const struct device *dev) {
    struct runtime_processor_data *data = dev->data;
    return k_work_reschedule(&data->save_work, K_MSEC(CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE));
}

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_TEST)
/* Test-only: force a pending debounced save to run now, instead of waiting
 * out CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE. */
void zmk_input_processor_runtime_test_flush_save(const struct device *dev) {
    struct runtime_processor_data *data = dev->data;
    struct k_work_sync sync;
    k_work_flush_delayable(&data->save_work, &sync);
}
#endif

#endif // CONFIG_ZMK_CUSTOM_SETTINGS

/* Gesture settings (live value and baseline) from the devicetree. */
static void load_gesture_defaults(struct runtime_processor_data *data) {
    const struct runtime_processor_config *cfg = data->dev->config;
    uint32_t keycodes[RIP_GESTURE_DIRS];
    for (int i = 0; i < RIP_GESTURE_DIRS; i++) {
        keycodes[i] = gesture_keycode_valid(cfg->initial_gesture_keycodes[i])
                          ? cfg->initial_gesture_keycodes[i]
                          : 0;
    }
    uint16_t distance = rip_gesture_clamp_distance(cfg->initial_gesture_distance);
    uint16_t still_ms = rip_gesture_clamp_still_ms(cfg->initial_gesture_still_ms);

    k_spinlock_key_t key = k_spin_lock(&data->gesture_lock);
    memcpy(data->gesture_keycodes, keycodes, sizeof(data->gesture_keycodes));
    data->gesture_distance = distance;
    data->gesture_still_ms = still_ms;
    k_spin_unlock(&data->gesture_lock, key);
    memcpy(data->persistent_gesture_keycodes, keycodes, sizeof(data->persistent_gesture_keycodes));
    data->persistent_gesture_distance = distance;
    data->persistent_gesture_still_ms = still_ms;
}

static int runtime_processor_init(const struct device *dev) {
    const struct runtime_processor_config *cfg = dev->config;
    struct runtime_processor_data *data = dev->data;

    // Initialize with default values
    data->scale_multiplier = cfg->initial_scale_multiplier;
    data->scale_divisor = cfg->initial_scale_divisor;
    data->rotation_degrees = cfg->initial_rotation_degrees;
    data->tilt_degrees = rip_tilt_clamp_degrees(cfg->initial_tilt_degrees);
    data->tilt_axis_degrees = rip_tilt_normalize_axis_degrees(cfg->initial_tilt_axis_degrees);
    data->accel_min_percent = rip_accel_clamp_min_percent(cfg->initial_accel_min_percent);
    data->aim_scale_percent = rip_aim_clamp_percent(cfg->initial_aim_scale_percent);
    data->aim_smoothing_ms = rip_aim_clamp_smoothing_ms(cfg->initial_aim_smoothing_ms);
    data->smoothing_ms = rip_aim_clamp_smoothing_ms(cfg->initial_smoothing_ms);

    // Initialize persistent values same as current
    data->persistent_scale_multiplier = cfg->initial_scale_multiplier;
    data->persistent_scale_divisor = cfg->initial_scale_divisor;
    data->persistent_rotation_degrees = cfg->initial_rotation_degrees;
    data->persistent_tilt_degrees = data->tilt_degrees;
    data->persistent_tilt_axis_degrees = data->tilt_axis_degrees;
    data->persistent_accel_min_percent = data->accel_min_percent;
    data->persistent_aim_scale_percent = data->aim_scale_percent;
    data->persistent_aim_smoothing_ms = data->aim_smoothing_ms;
    data->persistent_smoothing_ms = data->smoothing_ms;
    data->accel_state = (struct rip_accel_state){0};

    // Initialize temp-layer settings from DT defaults
    data->temp_layer_enabled = cfg->initial_temp_layer_enabled;
    data->temp_layer_layer = cfg->initial_temp_layer_layer;
    data->temp_layer_activation_delay_ms = cfg->initial_temp_layer_activation_delay_ms;
    data->temp_layer_deactivation_delay_ms = cfg->initial_temp_layer_deactivation_delay_ms;
    data->persistent_temp_layer_enabled = cfg->initial_temp_layer_enabled;
    data->persistent_temp_layer_layer = cfg->initial_temp_layer_layer;
    data->persistent_temp_layer_activation_delay_ms = cfg->initial_temp_layer_activation_delay_ms;
    data->persistent_temp_layer_deactivation_delay_ms =
        cfg->initial_temp_layer_deactivation_delay_ms;
    data->temp_layer_activation_distance =
        rip_temp_layer_clamp_distance(cfg->initial_temp_layer_activation_distance);
    data->persistent_temp_layer_activation_distance = data->temp_layer_activation_distance;

    // Initialize temp-layer runtime state
    data->temp_layer_layer_active = false;
    data->temp_layer_keep_active = false;
    data->last_input_timestamp = 0;
    data->last_keypress_timestamp = 0;
    data->temp_layer_distance = (struct rip_temp_layer_distance){0};

    // Initialize active layers from DT defaults
    data->active_layers = cfg->initial_active_layers;
    data->persistent_active_layers = cfg->initial_active_layers;

    // Initialize axis snap settings from DT defaults
    data->axis_snap_mode = cfg->initial_axis_snap_mode;
    data->axis_snap_threshold = cfg->initial_axis_snap_threshold;
    data->axis_snap_timeout_ms = cfg->initial_axis_snap_timeout_ms;
    data->persistent_axis_snap_mode = cfg->initial_axis_snap_mode;
    data->persistent_axis_snap_threshold = cfg->initial_axis_snap_threshold;
    data->persistent_axis_snap_timeout_ms = cfg->initial_axis_snap_timeout_ms;

    // Initialize axis snap runtime state
    data->axis_snap_cross_axis_accum = 0;
    data->axis_snap_last_decay_timestamp = 0;

    // Initialize code mapping settings from DT defaults
    data->xy_to_scroll_enabled = cfg->initial_xy_to_scroll_enabled;
    data->xy_swap_enabled = cfg->initial_xy_swap_enabled;
    data->persistent_xy_to_scroll_enabled = cfg->initial_xy_to_scroll_enabled;
    data->persistent_xy_swap_enabled = cfg->initial_xy_swap_enabled;
    // Initialize axis invert settings from DT defaults
    data->x_invert = cfg->initial_x_invert;
    data->y_invert = cfg->initial_y_invert;
    data->persistent_x_invert = cfg->initial_x_invert;
    data->persistent_y_invert = cfg->initial_y_invert;
    data->inertia_enabled = cfg->initial_inertia_enabled;
    data->persistent_inertia_enabled = cfg->initial_inertia_enabled;
    data->inertia_window_ms = cfg->initial_inertia_window_ms;
    data->inertia_interval_ms = cfg->initial_inertia_interval_ms;
    data->inertia_threshold = cfg->initial_inertia_threshold;
    data->inertia_decay_percent = MIN(cfg->initial_inertia_decay_percent, 100U);
    data->inertia_normal_max_output = cfg->initial_inertia_normal_max_output;
    data->inertia_fast_threshold = cfg->initial_inertia_fast_threshold;
    data->inertia_fast_output_percent = cfg->initial_inertia_fast_output_percent;
    data->persistent_inertia_window_ms = cfg->initial_inertia_window_ms;
    data->persistent_inertia_interval_ms = cfg->initial_inertia_interval_ms;
    data->persistent_inertia_threshold = cfg->initial_inertia_threshold;
    data->persistent_inertia_decay_percent = data->inertia_decay_percent;
    data->persistent_inertia_normal_max_output = data->inertia_normal_max_output;
    data->persistent_inertia_fast_threshold = data->inertia_fast_threshold;
    data->persistent_inertia_fast_output_percent = data->inertia_fast_output_percent;
    data->inertia_active = false;
    inertia_clear_windows(data);

    update_rotation_values(data);

    data->dev = dev;
#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
    k_work_init_delayable(&data->save_work, save_processor_settings_work_handler);
#endif
    // Initialize temp-layer work queues
    k_work_init_delayable(&data->temp_layer_activation_work, temp_layer_activation_work_handler);
    k_work_init_delayable(&data->temp_layer_deactivation_work,
                          temp_layer_deactivation_work_handler);
    k_work_init_delayable(&data->inertia_work, inertia_work_handler);

    // Aim mode and click freeze start idle
    data->aim_hold_count = 0;
    data->aim_key_percent = 0;
    data->aim_axis[0] = (struct rip_aim_axis){0};
    data->aim_axis[1] = (struct rip_aim_axis){0};
    data->click_freeze_until = 0;
    k_work_init_delayable(&data->aim_work, aim_work_handler);
    atomic_set(&data->aim_hold_motion, 0);

    // Gestures start with the devicetree settings, no key held
    load_gesture_defaults(data);
    data->gesture_hold_count = 0;
    data->gesture = (struct rip_gesture){.last_dir = RIP_GESTURE_NONE};
    data->gesture_draining = false;
    data->gesture_pending_count = 0;
    data->gesture_test_until = 0;
    data->gesture_input_dev = NULL;
    data->gesture_tapped_keycode = 0;
    data->ball_used_at = 0;
    data->ball_used_position = UINT32_MAX;
    k_work_init(&data->gesture_fire_work, gesture_fire_work_handler);
    k_work_init_delayable(&data->gesture_release_work, gesture_release_work_handler);

    LOG_INF("Runtime processor '%s' initialized", cfg->name);

    return 0;
}

// Forward declaration (defined later in this file) so the change notification
// below can report the correct processor id.
int zmk_input_processor_runtime_get_id(const struct device *dev);

// Helper to raise state changed event
static void raise_state_changed_event(const struct device *dev) {
    const char *name;
    struct zmk_input_processor_runtime_config config;

    int ret = zmk_input_processor_runtime_get_config(dev, &name, &config);
    if (ret < 0) {
        return;
    }

    // BUGFIX: the original omitted .id, so every change notification carried
    // id=0 and the Studio web UI overwrote whichever processor is id 0
    // (e.g. editing "left" clobbered "right"). Report the real id instead.
    int id = zmk_input_processor_runtime_get_id(dev);
    if (id < 0) {
        return;
    }

    raise_zmk_input_processor_state_changed((struct zmk_input_processor_state_changed){
        .id = (uint8_t)id, .name = name, .config = config});
}

// Human-readable suffix for the setter LOG lines (leading space + parens so the
// format strings stay unchanged).
static const char *write_mode_label(enum zmk_input_processor_runtime_write_mode mode) {
    switch (mode) {
    case ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST:
        return " (persistent)";
    case ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY:
        return " (memory)";
    default:
        return " (temporary)";
    }
}

/*
 * Common tail for every setter. The caller has already updated data->X (the
 * live value) and, for non-temporary modes, data->persistent_X (the baseline).
 * Propagate that write per its mode - mirroring zmk-feature-custom-settings:
 *   TEMPORARY: live value only; no notification, no save.
 *   MEMORY:    baseline staged in RAM; notify, but do not touch flash.
 *   PERSIST:   baseline staged in RAM; notify and flush to flash.
 * Without CONFIG_ZMK_CUSTOM_SETTINGS there is no persistence and the module
 * raised no state-changed notification before, so this is a no-op.
 */
static int commit_write(const struct device *dev,
                        enum zmk_input_processor_runtime_write_mode mode) {
#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
    if (mode == ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        return 0;
    }
    raise_state_changed_event(dev);
    if (mode == ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST) {
        return schedule_save_processor_settings(dev);
    }
    return 0;
#else
    ARG_UNUSED(dev);
    ARG_UNUSED(mode);
    return 0;
#endif
}

// Restore every setting (live value + baseline) to its devicetree default. Used
// by reset (which then persists) and by discard when nothing is saved on flash.
static void load_processor_defaults(const struct device *dev) {
    const struct runtime_processor_config *cfg = dev->config;
    struct runtime_processor_data *data = dev->data;

    data->scale_multiplier = cfg->initial_scale_multiplier;
    data->scale_divisor = cfg->initial_scale_divisor;
    data->rotation_degrees = cfg->initial_rotation_degrees;
    data->tilt_degrees = rip_tilt_clamp_degrees(cfg->initial_tilt_degrees);
    data->tilt_axis_degrees = rip_tilt_normalize_axis_degrees(cfg->initial_tilt_axis_degrees);
    data->accel_min_percent = rip_accel_clamp_min_percent(cfg->initial_accel_min_percent);
    data->aim_scale_percent = rip_aim_clamp_percent(cfg->initial_aim_scale_percent);
    data->aim_smoothing_ms = rip_aim_clamp_smoothing_ms(cfg->initial_aim_smoothing_ms);
    data->smoothing_ms = rip_aim_clamp_smoothing_ms(cfg->initial_smoothing_ms);
    data->persistent_scale_multiplier = cfg->initial_scale_multiplier;
    data->persistent_scale_divisor = cfg->initial_scale_divisor;
    data->persistent_rotation_degrees = cfg->initial_rotation_degrees;
    data->persistent_tilt_degrees = data->tilt_degrees;
    data->persistent_tilt_axis_degrees = data->tilt_axis_degrees;
    data->persistent_accel_min_percent = data->accel_min_percent;
    data->persistent_aim_scale_percent = data->aim_scale_percent;
    data->persistent_aim_smoothing_ms = data->aim_smoothing_ms;
    data->persistent_smoothing_ms = data->smoothing_ms;
    data->accel_state = (struct rip_accel_state){0};

    data->temp_layer_enabled = cfg->initial_temp_layer_enabled;
    data->temp_layer_layer = cfg->initial_temp_layer_layer;
    data->temp_layer_activation_delay_ms = cfg->initial_temp_layer_activation_delay_ms;
    data->temp_layer_deactivation_delay_ms = cfg->initial_temp_layer_deactivation_delay_ms;
    data->persistent_temp_layer_enabled = cfg->initial_temp_layer_enabled;
    data->persistent_temp_layer_layer = cfg->initial_temp_layer_layer;
    data->persistent_temp_layer_activation_delay_ms = cfg->initial_temp_layer_activation_delay_ms;
    data->persistent_temp_layer_deactivation_delay_ms =
        cfg->initial_temp_layer_deactivation_delay_ms;
    data->temp_layer_activation_distance =
        rip_temp_layer_clamp_distance(cfg->initial_temp_layer_activation_distance);
    data->persistent_temp_layer_activation_distance = data->temp_layer_activation_distance;

    data->active_layers = cfg->initial_active_layers;
    data->persistent_active_layers = cfg->initial_active_layers;

    // Deactivate temp-layer layer if active
    if (data->temp_layer_layer_active) {
        zmk_keymap_layer_deactivate(data->temp_layer_layer, false);
        data->temp_layer_layer_active = false;
        rip_temp_layer_distance_reset(&data->temp_layer_distance);
    }

    // Reset axis snap settings to defaults
    data->axis_snap_mode = cfg->initial_axis_snap_mode;
    data->axis_snap_threshold = cfg->initial_axis_snap_threshold;
    data->axis_snap_timeout_ms = cfg->initial_axis_snap_timeout_ms;
    data->persistent_axis_snap_mode = cfg->initial_axis_snap_mode;
    data->persistent_axis_snap_threshold = cfg->initial_axis_snap_threshold;
    data->persistent_axis_snap_timeout_ms = cfg->initial_axis_snap_timeout_ms;
    data->axis_snap_cross_axis_accum = 0;

    // Reset code mapping settings to defaults
    data->xy_to_scroll_enabled = cfg->initial_xy_to_scroll_enabled;
    data->xy_swap_enabled = cfg->initial_xy_swap_enabled;
    data->persistent_xy_to_scroll_enabled = cfg->initial_xy_to_scroll_enabled;
    data->persistent_xy_swap_enabled = cfg->initial_xy_swap_enabled;

    // Reset axis invert settings to defaults
    data->x_invert = cfg->initial_x_invert;
    data->y_invert = cfg->initial_y_invert;
    data->persistent_x_invert = cfg->initial_x_invert;
    data->persistent_y_invert = cfg->initial_y_invert;
    data->inertia_enabled = cfg->initial_inertia_enabled;
    data->persistent_inertia_enabled = cfg->initial_inertia_enabled;
    data->inertia_window_ms = cfg->initial_inertia_window_ms;
    data->inertia_interval_ms = cfg->initial_inertia_interval_ms;
    data->inertia_threshold = cfg->initial_inertia_threshold;
    data->inertia_decay_percent = MIN(cfg->initial_inertia_decay_percent, 100U);
    data->inertia_normal_max_output = cfg->initial_inertia_normal_max_output;
    data->inertia_fast_threshold = cfg->initial_inertia_fast_threshold;
    data->inertia_fast_output_percent = cfg->initial_inertia_fast_output_percent;
    data->persistent_inertia_window_ms = cfg->initial_inertia_window_ms;
    data->persistent_inertia_interval_ms = cfg->initial_inertia_interval_ms;
    data->persistent_inertia_threshold = cfg->initial_inertia_threshold;
    data->persistent_inertia_decay_percent = data->inertia_decay_percent;
    data->persistent_inertia_normal_max_output = data->inertia_normal_max_output;
    data->persistent_inertia_fast_threshold = data->inertia_fast_threshold;
    data->persistent_inertia_fast_output_percent = data->inertia_fast_output_percent;
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);

    load_gesture_defaults(data);

    update_rotation_values(data);
}

// Public API for runtime configuration
int zmk_input_processor_runtime_set_scaling(const struct device *dev, uint32_t multiplier,
                                            uint32_t divisor,
                                            enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;

    if ((multiplier > 0 && multiplier != data->scale_multiplier) ||
        (divisor > 0 && divisor != data->scale_divisor)) {
        inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);
    }

    if (multiplier > 0) {
        data->scale_multiplier = multiplier;
        if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
            data->persistent_scale_multiplier = multiplier;
        }
    }
    if (divisor > 0) {
        data->scale_divisor = divisor;
        if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
            data->persistent_scale_divisor = divisor;
        }
    }

    LOG_INF("Set scaling to %d/%d%s", data->scale_multiplier, data->scale_divisor,
            write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_rotation(const struct device *dev, int32_t degrees,
                                             enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->rotation_degrees = degrees;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_rotation_degrees = degrees;
    }
    update_rotation_values(data);

    LOG_INF("Set rotation to %d degrees%s", degrees, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_tilt(const struct device *dev, uint32_t degrees,
                                         enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    uint8_t tilt = rip_tilt_clamp_degrees(degrees);
    data->tilt_degrees = tilt;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_tilt_degrees = tilt;
    }
    update_rotation_values(data);

    LOG_INF("Set tilt to %u degrees%s", tilt, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_tilt_axis(const struct device *dev, int32_t degrees,
                                              enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    int16_t axis = rip_tilt_normalize_axis_degrees(degrees);
    data->tilt_axis_degrees = axis;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_tilt_axis_degrees = axis;
    }
    update_rotation_values(data);

    LOG_INF("Set tilt axis to %d degrees%s", axis, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_accel(const struct device *dev, uint32_t percent,
                                          enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    uint8_t accel = rip_accel_clamp_min_percent(percent);
    data->accel_min_percent = accel;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_accel_min_percent = accel;
    }
    // Speed history and remainders from the old curve must not leak out
    data->accel_state = (struct rip_accel_state){0};

    LOG_INF("Set accel minimum to %u%%%s", accel, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_aim(const struct device *dev, uint32_t scale_percent,
                                        uint32_t smoothing_ms,
                                        enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    uint8_t scale = rip_aim_clamp_percent(scale_percent);
    uint8_t smoothing = rip_aim_clamp_smoothing_ms(smoothing_ms);
    // Single bytes, so the input thread and the aim work never see a torn
    // value. A held aim key uses them from its next step on; what the filter
    // already holds back is still released, at the new pace
    data->aim_scale_percent = scale;
    data->aim_smoothing_ms = smoothing;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_aim_scale_percent = scale;
        data->persistent_aim_smoothing_ms = smoothing;
    }

    LOG_INF("Set aim to %u%%, smoothing %u ms%s", scale, smoothing, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_smoothing(const struct device *dev, uint32_t ms,
                                              enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    uint8_t smoothing = rip_aim_clamp_smoothing_ms(ms);
    // One byte, read by the input thread and the release work: what is held
    // back is kept and released at the new pace (or with the next events and
    // aim's pace once set to 0), so nothing jumps or is lost
    data->smoothing_ms = smoothing;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_smoothing_ms = smoothing;
    }

    LOG_INF("Set smoothing to %u ms%s", smoothing, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_reset(const struct device *dev) {
    if (!dev) {
        return -EINVAL;
    }

    const struct runtime_processor_config *cfg = dev->config;

    load_processor_defaults(dev);

    LOG_INF("Reset processor '%s' to defaults", cfg->name);

    return commit_write(dev, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
}

void zmk_input_processor_runtime_restore_persistent(const struct device *dev) {
    if (!dev) {
        return;
    }

    struct runtime_processor_data *data = dev->data;

    // Restore persistent values (used after temporary behavior changes)
    data->scale_multiplier = data->persistent_scale_multiplier;
    data->scale_divisor = data->persistent_scale_divisor;
    data->rotation_degrees = data->persistent_rotation_degrees;
    data->tilt_degrees = data->persistent_tilt_degrees;
    data->tilt_axis_degrees = data->persistent_tilt_axis_degrees;
    update_rotation_values(data);
    data->accel_min_percent = data->persistent_accel_min_percent;
    data->accel_state = (struct rip_accel_state){0};
    data->aim_scale_percent = data->persistent_aim_scale_percent;
    data->aim_smoothing_ms = data->persistent_aim_smoothing_ms;
    data->smoothing_ms = data->persistent_smoothing_ms;
    data->temp_layer_activation_distance = data->persistent_temp_layer_activation_distance;

    // Restore axis snap settings
    data->axis_snap_mode = data->persistent_axis_snap_mode;
    data->axis_snap_threshold = data->persistent_axis_snap_threshold;
    data->axis_snap_timeout_ms = data->persistent_axis_snap_timeout_ms;
    // Reset snap state when restoring
    data->axis_snap_cross_axis_accum = 0;
    data->axis_snap_last_decay_timestamp = 0;

    // Restore code mapping settings
    data->xy_to_scroll_enabled = data->persistent_xy_to_scroll_enabled;
    data->xy_swap_enabled = data->persistent_xy_swap_enabled;

    // Restore axis invert settings
    data->x_invert = data->persistent_x_invert;
    data->y_invert = data->persistent_y_invert;

    // Restore inertia settings and stop a temporary active mode.
    data->inertia_enabled = data->persistent_inertia_enabled;
    data->inertia_window_ms = data->persistent_inertia_window_ms;
    data->inertia_interval_ms = data->persistent_inertia_interval_ms;
    data->inertia_threshold = data->persistent_inertia_threshold;
    data->inertia_decay_percent = data->persistent_inertia_decay_percent;
    data->inertia_normal_max_output = data->persistent_inertia_normal_max_output;
    data->inertia_fast_threshold = data->persistent_inertia_fast_threshold;
    data->inertia_fast_output_percent = data->persistent_inertia_fast_output_percent;
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);

    // Restore gesture settings
    k_spinlock_key_t key = k_spin_lock(&data->gesture_lock);
    memcpy(data->gesture_keycodes, data->persistent_gesture_keycodes,
           sizeof(data->gesture_keycodes));
    data->gesture_distance = data->persistent_gesture_distance;
    data->gesture_still_ms = data->persistent_gesture_still_ms;
    k_spin_unlock(&data->gesture_lock, key);

    LOG_DBG("Restored persistent values");
}

int zmk_input_processor_runtime_get_config(const struct device *dev, const char **name,
                                           struct zmk_input_processor_runtime_config *config) {
    if (!dev) {
        return -EINVAL;
    }

    const struct runtime_processor_config *cfg = dev->config;
    struct runtime_processor_data *data = dev->data;

    if (name) {
        *name = cfg->name;
    }
    if (config) {
        config->scale_multiplier = data->persistent_scale_multiplier;
        config->scale_divisor = data->persistent_scale_divisor;
        config->rotation_degrees = data->persistent_rotation_degrees;
        config->tilt_degrees = data->persistent_tilt_degrees;
        config->tilt_axis_degrees = data->persistent_tilt_axis_degrees;
        config->accel_min_percent = data->persistent_accel_min_percent;
        config->aim_scale_percent = data->persistent_aim_scale_percent;
        config->aim_smoothing_ms = data->persistent_aim_smoothing_ms;
        config->smoothing_ms = data->persistent_smoothing_ms;
        config->temp_layer_enabled = data->persistent_temp_layer_enabled;
        config->temp_layer_layer = data->persistent_temp_layer_layer;
        config->temp_layer_activation_delay_ms = data->persistent_temp_layer_activation_delay_ms;
        config->temp_layer_deactivation_delay_ms =
            data->persistent_temp_layer_deactivation_delay_ms;
        config->temp_layer_activation_distance = data->persistent_temp_layer_activation_distance;
        config->active_layers = data->persistent_active_layers;
        config->axis_snap_mode = data->persistent_axis_snap_mode;
        config->axis_snap_threshold = data->persistent_axis_snap_threshold;
        config->axis_snap_timeout_ms = data->persistent_axis_snap_timeout_ms;
        config->xy_to_scroll_enabled = data->persistent_xy_to_scroll_enabled;
        config->xy_swap_enabled = data->persistent_xy_swap_enabled;
        config->x_invert = data->persistent_x_invert;
        config->y_invert = data->persistent_y_invert;
        config->inertia_enabled = data->persistent_inertia_enabled;
        config->inertia_window_ms = data->persistent_inertia_window_ms;
        config->inertia_interval_ms = data->persistent_inertia_interval_ms;
        config->inertia_threshold = data->persistent_inertia_threshold;
        config->inertia_decay_percent = data->persistent_inertia_decay_percent;
        config->inertia_normal_max_output = data->persistent_inertia_normal_max_output;
        config->inertia_fast_threshold = data->persistent_inertia_fast_threshold;
        config->inertia_fast_output_percent = data->persistent_inertia_fast_output_percent;
        config->inertia_notifications_enabled = data->inertia_notifications_enabled;
        config->inertia_active = data->inertia_active;
        memcpy(config->gesture_keycodes, data->persistent_gesture_keycodes,
               sizeof(config->gesture_keycodes));
        config->gesture_distance = data->persistent_gesture_distance;
        config->gesture_still_ms = data->persistent_gesture_still_ms;
        config->gesture_supported = dev == gesture_dev;
    }

    return 0;
}

/* The (int32_t) casts on angle properties: edtlib emits negative DT ints as
 * unsigned literals. */
#define RUNTIME_PROCESSOR_INST(n)                                                                  \
    static const uint16_t runtime_x_codes_##n[] = DT_INST_PROP(n, x_codes);                        \
    static const uint16_t runtime_y_codes_##n[] = DT_INST_PROP(n, y_codes);                        \
    BUILD_ASSERT(ARRAY_SIZE(runtime_x_codes_##n) == ARRAY_SIZE(runtime_y_codes_##n),               \
                 "X and Y codes need to be the same size");                                        \
    BUILD_ASSERT(DT_INST_PROP_OR(n, inertia_normal_max_output, 0) <= INT16_MAX,                  \
                 "normal inertia output limit exceeds signed report range");                    \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(n, temp_layer_keep_keycodes),                                \
                (static const uint16_t runtime_temp_layer_keep_keycodes_##n[] =                    \
                     DT_INST_PROP(n, temp_layer_keep_keycodes);),                                  \
                ())                                                                                \
    BUILD_ASSERT(sizeof(DT_INST_PROP(n, processor_label)) <=                                       \
                     CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_NAME_MAX_LEN,                              \
                 "processor_label " DT_INST_PROP(                                                  \
                     n, processor_label) " property +1 exceeds maximum "                           \
                                         "length " STRINGIFY(                                      \
                                             CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_NAME_MAX_LEN));    \
    static const struct runtime_processor_config runtime_config_##n = {                            \
        .name = DT_INST_PROP(n, processor_label),                                                  \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .x_codes_len = DT_INST_PROP_LEN(n, x_codes),                                               \
        .y_codes_len = DT_INST_PROP_LEN(n, y_codes),                                               \
        .x_codes = runtime_x_codes_##n,                                                            \
        .y_codes = runtime_y_codes_##n,                                                            \
        .initial_scale_multiplier = DT_INST_PROP_OR(n, scale_multiplier, 1),                       \
        .initial_scale_divisor = DT_INST_PROP_OR(n, scale_divisor, 1),                             \
        .initial_rotation_degrees = (int32_t)DT_INST_PROP_OR(n, rotation_degrees, 0),              \
        .initial_tilt_degrees = (int32_t)DT_INST_PROP_OR(n, tilt_degrees, 0),                      \
        .initial_tilt_axis_degrees = (int32_t)DT_INST_PROP_OR(n, tilt_axis_degrees, 0),            \
        .initial_accel_min_percent = (int32_t)DT_INST_PROP_OR(n, accel_min_percent, 100),          \
        .accel_speed_low = DT_INST_PROP_OR(n, accel_speed_low, 20),                                \
        .accel_speed_high = DT_INST_PROP_OR(n, accel_speed_high, 150),                             \
        .initial_aim_scale_percent = (int32_t)DT_INST_PROP_OR(n, aim_scale_percent, 40),           \
        .initial_aim_smoothing_ms = (int32_t)DT_INST_PROP_OR(n, aim_smoothing_ms, 50),             \
        .initial_smoothing_ms = (int32_t)DT_INST_PROP_OR(n, smoothing_ms, 0),                      \
        .click_freeze_ms = DT_INST_PROP_OR(n, click_freeze_ms, 0),                                 \
        .initial_gesture_keycodes = {(uint32_t)DT_INST_PROP_OR(n, gesture_left, 0),                \
                                     (uint32_t)DT_INST_PROP_OR(n, gesture_right, 0),               \
                                     (uint32_t)DT_INST_PROP_OR(n, gesture_up, 0),                  \
                                     (uint32_t)DT_INST_PROP_OR(n, gesture_down, 0)},               \
        .initial_gesture_distance = (int32_t)DT_INST_PROP_OR(n, gesture_distance, 330),            \
        .initial_gesture_still_ms = (int32_t)DT_INST_PROP_OR(n, gesture_still_ms, 150),            \
        IF_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS,                                                     \
                   (.gesture_settings_key =                                                        \
                        DT_INST_PROP(n, processor_label) RIP_GESTURE_SETTINGS_KEY_SUFFIX, ))       \
        .temp_layer_transparent_behavior = COND_CODE_1(                                            \
            DT_INST_NODE_HAS_PROP(n, temp_layer_transparent_behavior),                             \
            (DEVICE_DT_GET(DT_INST_PHANDLE(n, temp_layer_transparent_behavior))), (NULL)),         \
        .temp_layer_kp_behavior =                                                                  \
            COND_CODE_1(DT_INST_NODE_HAS_PROP(n, temp_layer_kp_behavior),                          \
                        (DEVICE_DT_GET(DT_INST_PHANDLE(n, temp_layer_kp_behavior))), (NULL)),      \
        .temp_layer_keep_keycodes_len =                                                            \
            COND_CODE_1(DT_INST_NODE_HAS_PROP(n, temp_layer_keep_keycodes),                        \
                        (DT_INST_PROP_LEN(n, temp_layer_keep_keycodes)), (0)),                     \
        .temp_layer_keep_keycodes =                                                                \
            COND_CODE_1(DT_INST_NODE_HAS_PROP(n, temp_layer_keep_keycodes),                        \
                        (runtime_temp_layer_keep_keycodes_##n), (NULL)),                           \
        .initial_temp_layer_enabled = DT_INST_PROP(n, temp_layer_enabled),                         \
        .temp_layer_extend_on_other_input = DT_INST_PROP(n, temp_layer_extend_on_other_input),     \
        .initial_temp_layer_layer = DT_INST_PROP_OR(n, temp_layer, 0),                             \
        .initial_temp_layer_activation_delay_ms =                                                  \
            DT_INST_PROP_OR(n, temp_layer_activation_delay_ms, 100),                               \
        .initial_temp_layer_deactivation_delay_ms =                                                \
            DT_INST_PROP_OR(n, temp_layer_deactivation_delay_ms, 500),                             \
        .initial_temp_layer_activation_distance =                                                  \
            DT_INST_PROP_OR(n, temp_layer_activation_distance, 0),                                 \
        .initial_active_layers = DT_INST_PROP_OR(n, active_layers, 0),                             \
        .initial_axis_snap_mode = DT_INST_PROP_OR(n, axis_snap_mode, 0),                           \
        .initial_axis_snap_threshold = DT_INST_PROP_OR(n, axis_snap_threshold, 100),               \
        .initial_axis_snap_timeout_ms = DT_INST_PROP_OR(n, axis_snap_timeout_ms, 1000),            \
        .initial_xy_to_scroll_enabled = DT_INST_PROP(n, xy_to_scroll_enabled),                     \
        .initial_xy_swap_enabled = DT_INST_PROP(n, xy_swap_enabled),                               \
        .initial_x_invert = DT_INST_PROP(n, x_invert),                                             \
        .initial_y_invert = DT_INST_PROP(n, y_invert),                                             \
        .initial_inertia_enabled = DT_INST_PROP(n, inertia_enabled),                            \
        .initial_inertia_window_ms = DT_INST_PROP_OR(n, inertia_window_ms, 200),             \
        .initial_inertia_interval_ms = DT_INST_PROP_OR(n, inertia_interval_ms, 20),          \
        .initial_inertia_threshold = DT_INST_PROP_OR(n, inertia_threshold, 12),              \
        .initial_inertia_decay_percent =                                                        \
            DT_INST_PROP_OR(n, inertia_decay_percent, 8),                                       \
        .initial_inertia_normal_max_output = DT_INST_PROP_OR(n, inertia_normal_max_output, 0), \
        .initial_inertia_fast_threshold = DT_INST_PROP_OR(n, inertia_fast_threshold, 20),       \
        .initial_inertia_fast_output_percent =                                                 \
            DT_INST_PROP_OR(n, inertia_fast_output_percent, 200),                                       \
    };                                                                                             \
    static struct runtime_processor_data runtime_data_##n;                                         \
    DEVICE_DT_INST_DEFINE(n, &runtime_processor_init, NULL, &runtime_data_##n,                     \
                          &runtime_config_##n, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,   \
                          &runtime_processor_driver_api);

DT_INST_FOREACH_STATUS_OKAY(RUNTIME_PROCESSOR_INST)

#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
/* One custom-settings entry per devicetree instance, keyed by its
 * compile-time processor-label literal (no runtime key buffer needed,
 * unlike settings whose key must be built from a runtime-resolved id). See
 * the RIP_SETTINGS_* definitions above. */
#define RUNTIME_PROCESSOR_SETTINGS_INST(n)                                                         \
    ZMK_CUSTOM_SETTING_DEFINE(                                                                     \
        runtime_processor_setting_##n, RIP_SETTINGS_SUBSYSTEM_ID,                                  \
        DT_INST_PROP(n, processor_label), ZMK_CUSTOM_SETTING_VALUE_TYPE_BYTES,                     \
        RIP_SETTINGS_EMPTY_BYTES_DEFAULT, ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,           \
        ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,            \
        ZMK_CUSTOM_SETTING_NO_CONSTRAINT);

DT_INST_FOREACH_STATUS_OKAY(RUNTIME_PROCESSOR_SETTINGS_INST)

/* The gesture entry of each instance (see RIP_GESTURE_SETTINGS_KEY_SUFFIX). */
#define RUNTIME_PROCESSOR_GESTURE_SETTINGS_INST(n)                                                 \
    ZMK_CUSTOM_SETTING_DEFINE(                                                                     \
        runtime_processor_gesture_setting_##n, RIP_SETTINGS_SUBSYSTEM_ID,                          \
        DT_INST_PROP(n, processor_label) RIP_GESTURE_SETTINGS_KEY_SUFFIX,                          \
        ZMK_CUSTOM_SETTING_VALUE_TYPE_BYTES, RIP_SETTINGS_EMPTY_BYTES_DEFAULT,                     \
        ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,     \
        ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE, ZMK_CUSTOM_SETTING_NO_CONSTRAINT);

DT_INST_FOREACH_STATUS_OKAY(RUNTIME_PROCESSOR_GESTURE_SETTINGS_INST)
#endif

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)
#define DEVICE_ADDR(idx) DEVICE_DT_GET(DT_DRV_INST(idx)),

static struct device *runtime_processors[] = {DT_INST_FOREACH_STATUS_OKAY(DEVICE_ADDR)};

static const size_t runtime_processors_count = sizeof(runtime_processors) / sizeof(struct device *);

#else

static struct device *runtime_processors[] = {};
static const size_t runtime_processors_count = 0;

#endif

int zmk_input_processor_runtime_foreach(int (*callback)(const struct device *dev, void *user_data),
                                        void *user_data) {
    for (size_t i = 0; i < runtime_processors_count; i++) {
        int ret = callback(runtime_processors[i], user_data);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

const struct device *zmk_input_processor_runtime_find_by_name(const char *name) {
    for (size_t i = 0; i < runtime_processors_count; i++) {
        const struct device *dev = runtime_processors[i];
        const struct runtime_processor_config *cfg = dev->config;
        if (strcmp(cfg->name, name) == 0) {
            return dev;
        }
    }

    return NULL;
}

const struct device *zmk_input_processor_runtime_find_by_id(uint8_t id) {
    if (id < runtime_processors_count) {
        return runtime_processors[id];
    }
    return NULL;
}

int zmk_input_processor_runtime_get_id(const struct device *dev) {
    for (size_t i = 0; i < runtime_processors_count; i++) {
        if (runtime_processors[i] == dev) {
            return i;
        }
    }
    return -1;
}

#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)

/*
 * Boot-apply: reads each processor's persisted blob (if any) back from
 * custom-settings and applies it. custom-settings' settings_load() runs from
 * ZMK main() after all SYS_INIT levels, so the persisted value is not yet
 * readable at an ordinary SYS_INIT hook. The apply is instead driven by the
 * zmk_custom_settings_initialized event (see the listener below), which
 * custom-settings raises exactly once, right after that boot settings_load
 * pass completes - i.e. once every persisted value is readable. No
 * zmk_custom_setting_changed listener is registered (see
 * docs/design/custom-settings-storage.md, "Design A"): the device is the
 * runtime source of truth and this module's own Studio RPC handlers already
 * mutate data->* directly, so a re-apply-on-change listener would risk
 * clobbering behavior-set temporary values and a write->changed->apply
 * write recursion.
 */
/* Apply the saved gesture entry, if any (devicetree defaults stay otherwise).
 * Returns true when one was applied. */
static bool apply_persisted_gesture_settings(const struct device *dev) {
    const struct runtime_processor_config *cfg = dev->config;
    struct runtime_processor_data *data = dev->data;

    struct zmk_custom_setting_value value;
    if (zmk_custom_setting_read_by_key(RIP_SETTINGS_SUBSYSTEM_ID, cfg->gesture_settings_key,
                                       &value) < 0 ||
        value.type != ZMK_CUSTOM_SETTING_VALUE_TYPE_BYTES || value.size == 0) {
        return false;
    }
    if (unpack_and_apply_gesture_settings(data, value.bytes_value, value.size) < 0) {
        LOG_WRN("Ignoring invalid persisted gesture settings for %s (size=%u)", cfg->name,
                (unsigned int)value.size);
        return false;
    }
    LOG_INF("Loaded gesture settings for %s: 0x%08x 0x%08x 0x%08x 0x%08x, %u counts, %u ms",
            cfg->name, (unsigned int)data->persistent_gesture_keycodes[0],
            (unsigned int)data->persistent_gesture_keycodes[1],
            (unsigned int)data->persistent_gesture_keycodes[2],
            (unsigned int)data->persistent_gesture_keycodes[3],
            (unsigned int)data->persistent_gesture_distance,
            (unsigned int)data->persistent_gesture_still_ms);
    return true;
}

static int apply_persisted_settings_cb(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);

    const struct runtime_processor_config *cfg = dev->config;
    struct runtime_processor_data *data = dev->data;

    // Its own entry: applied even when the main one is missing or invalid
    apply_persisted_gesture_settings(dev);

    struct zmk_custom_setting_value value;
    int ret = zmk_custom_setting_read_by_key(RIP_SETTINGS_SUBSYSTEM_ID, cfg->name, &value);
    if (ret < 0) {
        LOG_WRN("No persisted settings entry for %s: %d", cfg->name, ret);
        return 0;
    }

    if (value.type != ZMK_CUSTOM_SETTING_VALUE_TYPE_BYTES || value.size == 0) {
        // Nothing persisted yet - keep devicetree defaults.
        return 0;
    }

    if (unpack_and_apply_processor_settings(data, value.bytes_value, value.size) < 0) {
        LOG_WRN("Ignoring invalid persisted settings for %s (size=%u)", cfg->name,
                (unsigned int)value.size);
        return 0;
    }

    LOG_INF("Loaded settings for %s: scale=%d/%d, rotation=%d, tilt=%u@%d, accel=%u%%, "
            "aim=%u%%/%ums, smoothing=%ums, temp_layer=%d, active_layers=0x%08x, axis_snap=%d",
            cfg->name, data->persistent_scale_multiplier, data->persistent_scale_divisor,
            data->persistent_rotation_degrees, data->persistent_tilt_degrees,
            data->persistent_tilt_axis_degrees, data->persistent_accel_min_percent,
            data->persistent_aim_scale_percent, data->persistent_aim_smoothing_ms,
            data->persistent_smoothing_ms, data->persistent_temp_layer_enabled,
            data->persistent_active_layers, data->persistent_axis_snap_mode);
    return 0;
}

/* custom-settings raises zmk_custom_settings_initialized exactly once, after
 * the boot settings_load has populated every persisted value - the point at
 * which apply_persisted_settings_cb can read effective (persisted) values
 * rather than compile-time defaults. The event is dispatched synchronously
 * from custom-settings' boot settings-commit, which does not hold
 * custom_settings_lock, so reading settings back here is safe. */
static int settings_initialized_listener(const zmk_event_t *eh) {
    if (as_zmk_custom_settings_initialized(eh) == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    zmk_input_processor_runtime_foreach(apply_persisted_settings_cb, NULL);
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(runtime_processor_settings_initialized_listener, settings_initialized_listener);
ZMK_SUBSCRIPTION(runtime_processor_settings_initialized_listener, zmk_custom_settings_initialized);

static int save_all_cb(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    return save_processor_settings_now(dev);
}

/* Discard one processor's unsaved (memory) changes: cancel any pending
 * debounced flush and reload the last-saved values from flash, falling back to
 * devicetree defaults when nothing is saved. Mirrors custom-settings' discard. */
static int discard_processor_settings_cb(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    struct runtime_processor_data *data = dev->data;
    const struct runtime_processor_config *cfg = dev->config;

    // Reverting to the on-flash state; drop any pending debounced flush.
    k_work_cancel_delayable(&data->save_work);

    struct zmk_custom_setting_value value;
    int ret = zmk_custom_setting_read_by_key(RIP_SETTINGS_SUBSYSTEM_ID, cfg->name, &value);
    if (ret == 0 && value.type == ZMK_CUSTOM_SETTING_VALUE_TYPE_BYTES && value.size > 0 &&
        unpack_and_apply_processor_settings(data, value.bytes_value, value.size) == 0) {
        LOG_INF("Discarded unsaved changes for %s (reloaded from flash)", cfg->name);
    } else {
        load_processor_defaults(dev);
        LOG_INF("Discarded unsaved changes for %s (no saved value, using defaults)", cfg->name);
    }
    // The gesture entry on its own: the saved one, else the defaults
    if (!apply_persisted_gesture_settings(dev)) {
        load_gesture_defaults(data);
    }
    raise_state_changed_event(dev);
    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_TEST)
/* Test-only: synchronously re-run the boot-apply logic (bypassing the
 * SYS_INIT delay) so a test can simulate "reload after reboot" by writing a
 * persisted value, forcing custom-settings to reload it from the fake
 * settings backend (settings_load_subtree), and then calling this to verify
 * the exact apply path the real boot work handler uses. */
void zmk_input_processor_runtime_test_apply_persisted_settings(void) {
    zmk_input_processor_runtime_foreach(apply_persisted_settings_cb, NULL);
}
#endif

#endif // CONFIG_ZMK_CUSTOM_SETTINGS

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_TEST)
int zmk_input_processor_runtime_test_inertia_tick(const struct device *dev, int16_t *value) {
    if (!dev || !value) {
        return -EINVAL;
    }
    *value = inertia_finish_interval(dev->data, k_uptime_get());
    return 0;
}

int zmk_input_processor_runtime_test_inertia_tick_at(const struct device *dev, int64_t now,
                                                     int16_t *value) {
    if (!dev || !value) {
        return -EINVAL;
    }
    *value = inertia_finish_interval(dev->data, now);
    return 0;
}

bool zmk_input_processor_runtime_test_inertia_active(const struct device *dev) {
    return dev && ((struct runtime_processor_data *)dev->data)->inertia_active;
}

int zmk_input_processor_runtime_test_inertia_output_code(const struct device *dev,
                                                             uint16_t *code) {
    if (!dev || !code) {
        return -EINVAL;
    }

    const struct runtime_processor_data *data = dev->data;
    if (!data->inertia_active) {
        return -ENOENT;
    }

    *code = data->inertia_output_code;
    return 0;
}
#endif

int zmk_input_processor_runtime_save_all(void) {
#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
    LOG_INF("Saving all processor settings to flash");
    return zmk_input_processor_runtime_foreach(save_all_cb, NULL);
#else
    return 0;
#endif
}

int zmk_input_processor_runtime_discard_all(void) {
#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
    LOG_INF("Discarding all unsaved processor settings");
    return zmk_input_processor_runtime_foreach(discard_processor_settings_cb, NULL);
#else
    return 0;
#endif
}

static int reset_all_cb(const struct device *dev, void *user_data) {
    ARG_UNUSED(user_data);
    return zmk_input_processor_runtime_reset(dev);
}

int zmk_input_processor_runtime_reset_all(void) {
    LOG_INF("Resetting all processor settings to defaults");
    return zmk_input_processor_runtime_foreach(reset_all_cb, NULL);
}

// Event listener for keycode changes (for timestamp tracking)
static int keycode_state_changed_listener(const zmk_event_t *eh) {
    struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Only handle key presses
    if (!ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Update last keypress timestamp for all processors
    int64_t now = k_uptime_get();
    for (size_t i = 0; i < runtime_processors_count; i++) {
        const struct device *dev = runtime_processors[i];
        struct runtime_processor_data *data = dev->data;
        data->last_keypress_timestamp = now;
        // Motion counted toward the activation distance starts over
        rip_temp_layer_distance_reset(&data->temp_layer_distance);
    }

    return ZMK_EV_EVENT_BUBBLE;
}

// Event listener for position changes (for temp-layer deactivation logic)
static int position_state_changed_listener(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Only handle key presses
    if (!ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    // Check temp-layer deactivation for all processors
    for (size_t i = 0; i < runtime_processors_count; i++) {
        const struct device *dev = runtime_processors[i];
        const struct runtime_processor_config *cfg = dev->config;
        struct runtime_processor_data *data = dev->data;

        // Check if temp-layer layer should be deactivated
        if (!data->temp_layer_enabled || !data->temp_layer_layer_active ||
            data->temp_layer_keep_active) {
            continue;
        }

        // Check if the temp-layer layer has a non-transparent binding for this
        // position
        zmk_keymap_layer_id_t temp_layer_layer_id = data->temp_layer_layer;
        const struct zmk_behavior_binding *temp_layer_binding =
            zmk_keymap_get_layer_binding_at_idx(temp_layer_layer_id, ev->position);

        // If temp-layer layer has non-transparent binding, don't deactivate
        // Use device pointer comparison if transparent behavior is configured
        bool is_transparent = false;
        if (temp_layer_binding) {
            if (cfg->temp_layer_transparent_behavior) {
                // Efficient device pointer comparison
                const struct device *binding_dev =
                    zmk_behavior_get_binding(temp_layer_binding->behavior_dev);
                is_transparent = (binding_dev == cfg->temp_layer_transparent_behavior);
            } else {
                // Fallback to string comparison if not configured
                is_transparent = (strcmp(temp_layer_binding->behavior_dev, "trans") == 0 ||
                                  strcmp(temp_layer_binding->behavior_dev, "TRANS") == 0);
            }

            if (!is_transparent) {
                LOG_DBG("Temp-layer layer has non-transparent binding at position "
                        "%d, not deactivating",
                        ev->position);
                continue;
            }
        }

        // Temp-layer binding is transparent, check the resolved binding
        // Find the highest active layer's non-transparent binding
        const struct zmk_behavior_binding *resolved_binding = NULL;

        for (int layer_idx = ZMK_KEYMAP_LAYERS_LEN - 1; layer_idx >= 0; layer_idx--) {
            zmk_keymap_layer_id_t layer_id = zmk_keymap_layer_index_to_id(layer_idx);

            if (layer_id == ZMK_KEYMAP_LAYER_ID_INVAL) {
                continue;
            }

            if (!zmk_keymap_layer_active(layer_id)) {
                continue;
            }

            const struct zmk_behavior_binding *binding =
                zmk_keymap_get_layer_binding_at_idx(layer_id, ev->position);

            if (binding) {
                bool binding_is_transparent = false;
                if (cfg->temp_layer_transparent_behavior) {
                    const struct device *binding_dev =
                        zmk_behavior_get_binding(binding->behavior_dev);
                    binding_is_transparent = (binding_dev == cfg->temp_layer_transparent_behavior);
                } else {
                    binding_is_transparent = (strcmp(binding->behavior_dev, "trans") == 0 ||
                                              strcmp(binding->behavior_dev, "TRANS") == 0);
                }

                if (!binding_is_transparent) {
                    resolved_binding = binding;
                    break;
                }
            }
        }

        // If resolved binding is &kp with a modifier keycode, don't deactivate
        if (resolved_binding) {
            bool is_kp = false;
            if (cfg->temp_layer_kp_behavior) {
                const struct device *binding_dev =
                    zmk_behavior_get_binding(resolved_binding->behavior_dev);
                is_kp = (binding_dev == cfg->temp_layer_kp_behavior);
            } else {
                is_kp = (strcmp(resolved_binding->behavior_dev, "kp") == 0 ||
                         strcmp(resolved_binding->behavior_dev, "KEY_PRESS") == 0);
            }

            if (is_kp) {
                // The param1 contains the keycode for &kp behavior
                uint32_t keycode_encoded = resolved_binding->param1;
                uint16_t usage_page = ZMK_HID_USAGE_PAGE(keycode_encoded);
                uint16_t usage_id = ZMK_HID_USAGE_ID(keycode_encoded);

                if (!usage_page) {
                    usage_page = HID_USAGE_KEY;
                }

                // Check if it's in the keep-keycodes list if configured
                bool should_keep = false;
                if (cfg->temp_layer_keep_keycodes_len > 0) {
                    for (size_t j = 0; j < cfg->temp_layer_keep_keycodes_len; j++) {
                        if (cfg->temp_layer_keep_keycodes[j] == usage_id) {
                            should_keep = true;
                            break;
                        }
                    }
                } else {
                    // Fallback to is_mod check if keycodes not configured
                    should_keep = is_mod(usage_page, usage_id);
                }

                if (should_keep) {
                    LOG_DBG("Resolved binding is keep keycode, not deactivating "
                            "temp-layer layer");
                    continue;
                }
            }
        }

        // Deactivate the temp-layer layer
        LOG_DBG("Deactivating temp-layer layer %d due to key press at position %d",
                data->temp_layer_layer, ev->position);
        k_work_cancel_delayable(&data->temp_layer_deactivation_work);
        int ret = zmk_keymap_layer_deactivate(data->temp_layer_layer, false);
        if (ret == 0) {
            data->temp_layer_layer_active = false;
            rip_temp_layer_distance_reset(&data->temp_layer_distance);
            LOG_INF("Temp-layer layer %d deactivated by key press", data->temp_layer_layer);
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(runtime_processor_keycode_listener, keycode_state_changed_listener);
ZMK_SUBSCRIPTION(runtime_processor_keycode_listener, zmk_keycode_state_changed);

ZMK_LISTENER(runtime_processor_position_listener, position_state_changed_listener);
ZMK_SUBSCRIPTION(runtime_processor_position_listener, zmk_position_state_changed);

static int runtime_processor_layer_state_changed_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (ev == NULL || ev->state) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    for (size_t i = 0; i < runtime_processors_count; i++) {
        struct runtime_processor_data *data = runtime_processors[i]->data;
        if (data->inertia_active && !is_processor_active_for_current_layers(data->active_layers)) {
            LOG_DBG("Inertia stopped after active layer deactivation");
            inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_LAYER_INACTIVE);
        }
        if (data->accel_state.count != 0 &&
            !is_processor_active_for_current_layers(data->active_layers)) {
            data->accel_state = (struct rip_accel_state){0};
        }
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(runtime_processor_layer_listener, runtime_processor_layer_state_changed_listener);
ZMK_SUBSCRIPTION(runtime_processor_layer_listener, zmk_layer_state_changed);

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_MOUSE_KEY_PRESS) &&                                             \
    (!IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL))
/*
 * Mouse button presses and releases: &mkp reports them as key events on its
 * own input device. Whatever aim still holds back is forgotten, so the cursor
 * does not drift on after the click. Processors with click-freeze-ms also
 * drop input for that long and stop a running inertia glide, so the cursor
 * stays where the click landed. Runs on the input thread, like
 * runtime_processor_handle_event.
 */
static void click_freeze_handle_button(const struct input_event *evt) {
    if (evt->type != INPUT_EV_KEY) {
        return;
    }

    int64_t now = k_uptime_get();
    for (size_t i = 0; i < runtime_processors_count; i++) {
        const struct device *dev = runtime_processors[i];
        const struct runtime_processor_config *cfg = dev->config;
        struct runtime_processor_data *data = dev->data;

        k_spinlock_key_t key = k_spin_lock(&data->aim_lock);
        bool had_aim = aim_busy(data);
        data->aim_axis[0] = (struct rip_aim_axis){0};
        data->aim_axis[1] = (struct rip_aim_axis){0};
        k_spin_unlock(&data->aim_lock, key);
        if (had_aim) {
            k_work_cancel_delayable(&data->aim_work);
        }

        if (cfg->click_freeze_ms == 0) {
            continue;
        }
        data->click_freeze_until = now + cfg->click_freeze_ms;
        if (data->inertia_active) {
            inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_UNSPECIFIED);
        }

        LOG_DBG("Button %u %s: '%s' frozen for %u ms", evt->code, evt->value ? "down" : "up",
                cfg->name, cfg->click_freeze_ms);
    }
}

#define CLICK_FREEZE_CALLBACK(node_id)                                                             \
    static void click_freeze_cb_##node_id(struct input_event *evt, void *user_data) {              \
        ARG_UNUSED(user_data);                                                                     \
        click_freeze_handle_button(evt);                                                           \
    }                                                                                              \
    INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(node_id), click_freeze_cb_##node_id, NULL);

DT_FOREACH_STATUS_OKAY(zmk_behavior_mouse_key_press, CLICK_FREEZE_CALLBACK)
#endif

// Temp-layer layer configuration API
int zmk_input_processor_runtime_set_temp_layer(const struct device *dev, bool enabled,
                                               uint8_t layer, uint32_t activation_delay_ms,
                                               uint32_t deactivation_delay_ms,
                                               enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;

    data->temp_layer_enabled = enabled;
    data->temp_layer_layer = layer;
    data->temp_layer_activation_delay_ms = activation_delay_ms;
    data->temp_layer_deactivation_delay_ms = deactivation_delay_ms;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_temp_layer_enabled = enabled;
        data->persistent_temp_layer_layer = layer;
        data->persistent_temp_layer_activation_delay_ms = activation_delay_ms;
        data->persistent_temp_layer_deactivation_delay_ms = deactivation_delay_ms;
    }

    LOG_INF("Temp-layer layer config: enabled=%d, layer=%d, act_delay=%d, "
            "deact_delay=%d%s",
            enabled, layer, activation_delay_ms, deactivation_delay_ms, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_temp_layer_enabled(
    const struct device *dev, bool enabled, enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->temp_layer_enabled = enabled;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_temp_layer_enabled = enabled;
    }

    LOG_INF("Temp-layer enabled: %d%s", enabled, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_temp_layer_layer(
    const struct device *dev, uint8_t layer, enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->temp_layer_layer = layer;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_temp_layer_layer = layer;
    }

    LOG_INF("Temp-layer layer: %d%s", layer, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_temp_layer_activation_delay(
    const struct device *dev, uint32_t activation_delay_ms,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->temp_layer_activation_delay_ms = activation_delay_ms;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_temp_layer_activation_delay_ms = activation_delay_ms;
    }

    LOG_INF("Temp-layer activation delay: %dms%s", activation_delay_ms, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_temp_layer_deactivation_delay(
    const struct device *dev, uint32_t deactivation_delay_ms,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->temp_layer_deactivation_delay_ms = deactivation_delay_ms;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_temp_layer_deactivation_delay_ms = deactivation_delay_ms;
    }

    LOG_INF("Temp-layer deactivation delay: %dms%s", deactivation_delay_ms, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_temp_layer_activation_distance(
    const struct device *dev, uint32_t distance, enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    // One byte, read by the input thread: the next event counts against it
    uint8_t value = rip_temp_layer_clamp_distance(distance);
    data->temp_layer_activation_distance = value;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_temp_layer_activation_distance = value;
    }

    LOG_INF("Temp-layer activation distance: %u%s", value, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_active_layers(
    const struct device *dev, uint32_t layers, enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->active_layers = layers;
    if (data->inertia_active && !is_processor_active_for_current_layers(layers)) {
        inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_LAYER_INACTIVE);
    }

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_active_layers = layers;
    }

    LOG_INF("Active layers: 0x%08x%s", layers, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_axis_snap_mode(
    const struct device *dev, uint8_t mode,
    enum zmk_input_processor_runtime_write_mode write_mode) {
    if (!dev) {
        return -EINVAL;
    }

    if (mode > ZMK_INPUT_PROCESSOR_AXIS_SNAP_MODE_Y) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->axis_snap_mode = mode;

    // Reset snap state when mode changes
    data->axis_snap_cross_axis_accum = 0;

    if (write_mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_axis_snap_mode = mode;
    }

    LOG_INF("Axis snap mode: %d%s", mode, write_mode_label(write_mode));

    return commit_write(dev, write_mode);
}

int zmk_input_processor_runtime_set_axis_snap_threshold(
    const struct device *dev, uint16_t threshold,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->axis_snap_threshold = threshold;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_axis_snap_threshold = threshold;
    }

    LOG_INF("Axis snap threshold: %d%s", threshold, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_axis_snap_timeout(
    const struct device *dev, uint16_t timeout_ms,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->axis_snap_timeout_ms = timeout_ms;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_axis_snap_timeout_ms = timeout_ms;
    }

    LOG_INF("Axis snap timeout: %d ms%s", timeout_ms, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_axis_snap(
    const struct device *dev, uint8_t mode, uint16_t threshold, uint16_t timeout_ms,
    enum zmk_input_processor_runtime_write_mode write_mode) {
    if (!dev) {
        return -EINVAL;
    }

    if (mode > ZMK_INPUT_PROCESSOR_AXIS_SNAP_MODE_Y) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->axis_snap_mode = mode;
    data->axis_snap_threshold = threshold;
    data->axis_snap_timeout_ms = timeout_ms;

    // Reset snap state when configuration changes
    data->axis_snap_cross_axis_accum = 0;

    if (write_mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_axis_snap_mode = mode;
        data->persistent_axis_snap_threshold = threshold;
        data->persistent_axis_snap_timeout_ms = timeout_ms;
    }

    LOG_INF("Axis snap config: mode=%d, threshold=%d, timeout=%d ms%s", mode, threshold, timeout_ms,
            write_mode_label(write_mode));

    return commit_write(dev, write_mode);
}

int zmk_input_processor_runtime_set_x_invert(const struct device *dev, bool invert,
                                             enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->x_invert = invert;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_x_invert = invert;
    }

    LOG_INF("X axis invert: %s%s", invert ? "true" : "false", write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_y_invert(const struct device *dev, bool invert,
                                             enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->y_invert = invert;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_y_invert = invert;
    }

    LOG_INF("Y axis invert: %s%s", invert ? "true" : "false", write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_inertia_window(
    const struct device *dev, uint16_t window_ms,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }
    if (window_ms == 0 || window_ms > 60000) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->inertia_window_ms = window_ms;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_inertia_window_ms = window_ms;
    }
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);

    LOG_INF("Inertia window: %u ms%s", window_ms, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_inertia_interval(
    const struct device *dev, uint16_t interval_ms,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }
    if (interval_ms == 0 || interval_ms > 60000) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->inertia_interval_ms = interval_ms;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_inertia_interval_ms = interval_ms;
    }
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);

    LOG_INF("Inertia output interval: %u ms%s", interval_ms, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_inertia_threshold(
    const struct device *dev, uint16_t threshold,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev || threshold == 0) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->inertia_threshold = threshold;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_inertia_threshold = threshold;
    }
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);

    LOG_INF("Inertia threshold: %u%s", threshold, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_inertia_decay(
    const struct device *dev, uint8_t decay_percent,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev || decay_percent > 100) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->inertia_decay_percent = decay_percent;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_inertia_decay_percent = decay_percent;
    }
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);

    LOG_INF("Inertia decay: %u%% per interval%s", decay_percent, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_inertia_normal_max_output(
    const struct device *dev, uint16_t max_output,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev || max_output > INT16_MAX) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    data->inertia_normal_max_output = max_output;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_inertia_normal_max_output = max_output;
    }
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);
    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_inertia_enabled(
    const struct device *dev, bool enabled,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    data->inertia_enabled = enabled;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_inertia_enabled = enabled;
    }
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);
    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_inertia_fast_threshold(
    const struct device *dev, uint16_t threshold,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    data->inertia_fast_threshold = threshold;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_inertia_fast_threshold = threshold;
    }
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);
    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_inertia_fast_output_percent(
    const struct device *dev, uint16_t percent,
    enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev || percent < 100 || percent > 1000) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    data->inertia_fast_output_percent = percent;
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_inertia_fast_output_percent = percent;
    }
    inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED);
    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_inertia_notifications(const struct device *dev, bool enabled) {
    if (!dev) {
        return -EINVAL;
    }
    struct runtime_processor_data *data = dev->data;
    data->inertia_notifications_enabled = enabled;
    if (enabled) {
        raise_zmk_input_processor_inertia_state_changed(
            (struct zmk_input_processor_inertia_state_changed){.dev = dev,
                                                                .state = data->inertia_active
                                                                             ? ZMK_INPUT_PROCESSOR_INERTIA_STARTED
                                                                             : ZMK_INPUT_PROCESSOR_INERTIA_STOPPED});
    }
    raise_state_changed_event(dev);
    return 0;
}

void zmk_input_processor_runtime_temp_layer_keep_active(const struct device *dev,
                                                        bool keep_active) {
    if (!dev) {
        return;
    }

    struct runtime_processor_data *data = dev->data;
    data->temp_layer_keep_active = keep_active;

    LOG_DBG("Temp-layer keep_active set to %d", keep_active);

    // If releasing keep_active and layer is still active, deactivate
    // immediately
    if (!keep_active && data->temp_layer_enabled && data->temp_layer_layer_active) {
        k_work_reschedule(&data->temp_layer_deactivation_work, K_NO_WAIT);
    }
}

void zmk_input_processor_runtime_set_aim_key_scale(const struct device *dev, uint32_t percent) {
    if (!dev) {
        return;
    }

    struct runtime_processor_data *data = dev->data;
    // Only scales input from now on: what the filter holds back stays as is
    k_spinlock_key_t key = k_spin_lock(&data->aim_lock);
    data->aim_key_percent = rip_aim_key_percent(percent);
    k_spin_unlock(&data->aim_lock, key);
}

void zmk_input_processor_runtime_aim(const struct device *dev, bool hold) {
    if (!dev) {
        return;
    }

    struct runtime_processor_data *data = dev->data;

    k_spinlock_key_t key = k_spin_lock(&data->aim_lock);
    if (hold) {
        if (data->aim_hold_count < UINT8_MAX) {
            data->aim_hold_count++;
        }
    } else if (data->aim_hold_count > 0) {
        data->aim_hold_count--;
    }
    uint8_t count = data->aim_hold_count;
    bool busy = aim_busy(data);
    k_spin_unlock(&data->aim_lock, key);

    LOG_DBG("Aim %s, held by %u", hold ? "pressed" : "released", count);

    // Whether this hold moved the ball, for the tap of its tap & hold key
    if (hold) {
        atomic_set(&data->aim_hold_motion, 0);
    } else {
        bool used = atomic_get(&data->aim_hold_motion) >=
                    (atomic_val_t)(RIP_GESTURE_USED_MILLI / RIP_GESTURE_MILLI);
        data->ball_used_at = used ? k_uptime_get() : 0;
        data->ball_used_position = UINT32_MAX; // the behavior names its key next
    }

    // A glide that is still running would carry the cursor past the target
    if (hold && data->inertia_active) {
        inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_UNSPECIFIED);
    }

    // Aiming often pauses longer than the temp-layer timeout: keep the layer
    // (and the clicks on it) while held, and time out as usual after release
    // (unless a gesture key still holds it).
    if (data->temp_layer_enabled && data->temp_layer_layer_active &&
        !data->temp_layer_keep_active) {
        if (count > 0 || data->gesture_hold_count > 0) {
            k_work_cancel_delayable(&data->temp_layer_deactivation_work);
        } else {
            k_work_reschedule(&data->temp_layer_deactivation_work,
                              K_MSEC(data->temp_layer_deactivation_delay_ms));
        }
    }

    // What the filter still holds is released after the last key goes up
    if (count == 0 && busy) {
        k_work_schedule(&data->aim_work, K_MSEC(AIM_INTERVAL_MS));
    }
}

void zmk_input_processor_runtime_gesture_attach(const struct device *dev) {
    if (!dev) {
        return;
    }
    if (gesture_dev == NULL) {
        gesture_dev = dev;
    } else if (gesture_dev != dev) {
        LOG_WRN("Gestures already act on another processor, ignoring a second one");
    }
}

void zmk_input_processor_runtime_gesture(const struct device *dev, bool hold) {
    if (!dev) {
        return;
    }

    struct runtime_processor_data *data = dev->data;
    int64_t now = k_uptime_get();

    k_spinlock_key_t key = k_spin_lock(&data->gesture_lock);
    if (hold) {
        if (data->gesture_hold_count == 0) {
            rip_gesture_begin(&data->gesture);
            data->gesture_draining = false;
        }
        if (data->gesture_hold_count < UINT8_MAX) {
            data->gesture_hold_count++;
        }
    } else if (data->gesture_hold_count > 0) {
        data->gesture_hold_count--;
        if (data->gesture_hold_count == 0) {
            // The cursor stays put until the ball stops; whether the hold
            // used the ball decides the tap of its tap & hold key
            bool used = rip_gesture_end(&data->gesture);
            data->ball_used_at = used ? now : 0;
            data->ball_used_position = UINT32_MAX; // the behavior names its key next
            data->gesture_up_ms = (uint32_t)now;
            data->gesture_draining = true;
        }
    }
    uint8_t count = data->gesture_hold_count;
    k_spin_unlock(&data->gesture_lock, key);

    LOG_DBG("Gesture %s, held by %u", hold ? "pressed" : "released", count);

    if (hold && count == 1) {
        // The cursor stops where it is: what smoothing still holds back is
        // dropped and a glide ends
        key = k_spin_lock(&data->aim_lock);
        bool had_aim = aim_busy(data);
        data->aim_axis[0] = (struct rip_aim_axis){0};
        data->aim_axis[1] = (struct rip_aim_axis){0};
        k_spin_unlock(&data->aim_lock, key);
        if (had_aim) {
            k_work_cancel_delayable(&data->aim_work);
        }
        if (data->inertia_active) {
            inertia_stop(data, ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_UNSPECIFIED);
        }
    }

    // Like aim: the temp layer (where the key may be) stays while held and
    // times out as usual after the release
    if (data->temp_layer_enabled && data->temp_layer_layer_active &&
        !data->temp_layer_keep_active) {
        if (count > 0 || data->aim_hold_count > 0) {
            k_work_cancel_delayable(&data->temp_layer_deactivation_work);
        } else {
            k_work_reschedule(&data->temp_layer_deactivation_work,
                              K_MSEC(data->temp_layer_deactivation_delay_ms));
        }
    }
}

int zmk_input_processor_runtime_set_gesture(
    const struct device *dev, const uint32_t keycodes[ZMK_INPUT_PROCESSOR_GESTURE_DIRS],
    uint32_t distance, uint32_t still_ms, enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev || !keycodes) {
        return -EINVAL;
    }
    for (int i = 0; i < RIP_GESTURE_DIRS; i++) {
        if (!gesture_keycode_valid(keycodes[i])) {
            LOG_WRN("Gesture %d keycode 0x%08x is not a key", i, (unsigned int)keycodes[i]);
            return -EINVAL;
        }
    }

    struct runtime_processor_data *data = dev->data;
    uint16_t d = rip_gesture_clamp_distance(distance);
    uint16_t s = rip_gesture_clamp_still_ms(still_ms);

    k_spinlock_key_t key = k_spin_lock(&data->gesture_lock);
    memcpy(data->gesture_keycodes, keycodes, sizeof(data->gesture_keycodes));
    data->gesture_distance = d;
    data->gesture_still_ms = s;
    k_spin_unlock(&data->gesture_lock, key);
    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        memcpy(data->persistent_gesture_keycodes, keycodes,
               sizeof(data->persistent_gesture_keycodes));
        data->persistent_gesture_distance = d;
        data->persistent_gesture_still_ms = s;
    }

    LOG_INF("Set gestures to 0x%08x 0x%08x 0x%08x 0x%08x, %u counts, %u ms%s",
            (unsigned int)keycodes[0], (unsigned int)keycodes[1], (unsigned int)keycodes[2],
            (unsigned int)keycodes[3], (unsigned int)d, (unsigned int)s, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_gesture_test(const struct device *dev, bool enabled) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    k_spinlock_key_t key = k_spin_lock(&data->gesture_lock);
    data->gesture_test_until = enabled ? k_uptime_get() + GESTURE_TEST_MS : 0;
    k_spin_unlock(&data->gesture_lock, key);

    LOG_INF("Gesture test %s", enabled ? "on" : "off");
    return 0;
}

void zmk_input_processor_runtime_ball_hold_released(const struct device *dev, uint32_t position) {
    if (!dev) {
        return;
    }

    // The aim or gesture release just before recorded whether the ball was used
    struct runtime_processor_data *data = dev->data;
    data->ball_used_position = position;
}

bool zmk_input_processor_runtime_take_ball_used(const struct device *dev, uint32_t position) {
    if (!dev) {
        return false;
    }

    struct runtime_processor_data *data = dev->data;
    if (data->ball_used_at == 0 || data->ball_used_position != position) {
        return false;
    }
    int64_t at = data->ball_used_at;
    data->ball_used_at = 0;
    return k_uptime_get() - at <= BALL_USED_TAP_MS;
}

int zmk_input_processor_runtime_set_xy_to_scroll_enabled(
    const struct device *dev, bool enabled, enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->xy_to_scroll_enabled = enabled;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_xy_to_scroll_enabled = enabled;
    }

    LOG_INF("XY-to-scroll enabled: %d%s", enabled, write_mode_label(mode));

    return commit_write(dev, mode);
}

int zmk_input_processor_runtime_set_xy_swap_enabled(
    const struct device *dev, bool enabled, enum zmk_input_processor_runtime_write_mode mode) {
    if (!dev) {
        return -EINVAL;
    }

    struct runtime_processor_data *data = dev->data;
    data->xy_swap_enabled = enabled;

    if (mode != ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) {
        data->persistent_xy_swap_enabled = enabled;
    }

    LOG_INF("XY-swap enabled: %d%s", enabled, write_mode_label(mode));

    return commit_write(dev, mode);
}
