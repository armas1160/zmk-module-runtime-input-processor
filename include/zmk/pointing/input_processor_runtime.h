/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <stdbool.h>

/**
 * @brief Axis snap mode
 */
enum zmk_input_processor_axis_snap_mode {
    ZMK_INPUT_PROCESSOR_AXIS_SNAP_MODE_NONE = 0,
    ZMK_INPUT_PROCESSOR_AXIS_SNAP_MODE_X = 1,
    ZMK_INPUT_PROCESSOR_AXIS_SNAP_MODE_Y = 2,
};

/**
 * @brief Where a setter stores the new value
 *
 * Mirrors zmk-feature-custom-settings' write modes. PERSIST is value 0 so it is
 * the default for callers that do not care (and keeps the historical
 * always-persist behavior of the RPC).
 */
enum zmk_input_processor_runtime_write_mode {
    ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST = 0,   // Update baseline + save to flash
    ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY = 1,    // Update baseline in RAM only
    ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY = 2, // Apply to the live value only
};

/**
 * @brief Gesture directions, in the cursor frame; also the order of the
 * gesture keycodes
 */
enum zmk_input_processor_gesture_direction {
    ZMK_INPUT_PROCESSOR_GESTURE_LEFT = 0,
    ZMK_INPUT_PROCESSOR_GESTURE_RIGHT = 1,
    ZMK_INPUT_PROCESSOR_GESTURE_UP = 2,
    ZMK_INPUT_PROCESSOR_GESTURE_DOWN = 3,
};

#define ZMK_INPUT_PROCESSOR_GESTURE_DIRS 4

/**
 * @brief Runtime input processor configuration
 */
struct zmk_input_processor_runtime_config {
    uint32_t scale_multiplier;
    uint32_t scale_divisor;
    int32_t rotation_degrees;
    // Tilt compensation settings
    uint8_t tilt_degrees;      // 0..60, 0 = off; boost 1/cos(tilt) along the tilt axis
    int16_t tilt_axis_degrees; // (-90, 90], clockwise from up/down (90 = left/right)
    // Pointer acceleration: gain at low speed in percent (20..100, 100 = off)
    uint8_t accel_min_percent;
    // Aim mode while an aim key is held: speed in percent (10..100) and the
    // smoothing time constant (0..200 ms, 0 = no smoothing)
    uint8_t aim_scale_percent;
    uint8_t aim_smoothing_ms;
    // Normal smoothing while no aim key is held (0..200 ms, 0 = off)
    uint8_t smoothing_ms;
    // Temp-layer layer settings
    bool temp_layer_enabled;
    uint8_t temp_layer_layer;
    uint16_t temp_layer_activation_delay_ms;
    uint16_t temp_layer_deactivation_delay_ms;
    // Motion (|dx| + |dy| after scaling, 0..100) needed before the temp layer
    // activates; 0 = any motion
    uint8_t temp_layer_activation_distance;
    // Active layers bitmask (0 = all layers, otherwise each bit represents a layer)
    uint32_t active_layers;
    // Axis snap settings
    uint8_t axis_snap_mode;        // zmk_input_processor_axis_snap_mode
    uint16_t axis_snap_threshold;  // Threshold for unsnapping
    uint16_t axis_snap_timeout_ms; // Time window for checking threshold
    // Code mapping settings
    bool xy_to_scroll_enabled; // Map X/Y to horizontal/vertical scroll
    bool xy_swap_enabled;      // Swap X and Y axes
    // Axis reverse settings
    bool x_invert; // Whether to invert X axis
    bool y_invert; // Whether to invert Y axis
    // Inertia settings.
    bool inertia_enabled;
    uint16_t inertia_window_ms;
    uint16_t inertia_interval_ms;
    uint16_t inertia_threshold;
    uint8_t inertia_decay_percent;
    uint16_t inertia_normal_max_output;
    uint16_t inertia_fast_threshold;
    uint16_t inertia_fast_output_percent;
    bool inertia_notifications_enabled;
    bool inertia_active;
    // Gestures: what a flick fires while a gesture key is held (ZMK encoded
    // keycodes as &kp's parameter, 0 = nothing; left, right, up, down), the
    // sensor counts a flick covers and how long the ball must be still
    // before the next one (see zmk_input_processor_runtime_set_gesture)
    uint32_t gesture_keycodes[ZMK_INPUT_PROCESSOR_GESTURE_DIRS];
    uint16_t gesture_distance;
    uint16_t gesture_still_ms;
    // Whether gesture behaviors act on this processor (see gesture_attach)
    bool gesture_supported;
};

/**
 * @brief Set the scaling parameters for a runtime input processor
 *
 * @param dev Pointer to the device structure
 * @param multiplier Scaling multiplier (must be > 0)
 * @param divisor Scaling divisor (must be > 0)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_scaling(const struct device *dev, uint32_t multiplier,
                                            uint32_t divisor,
                                            enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set the rotation angle for a runtime input processor
 *
 * @param dev Pointer to the device structure
 * @param degrees Rotation angle in degrees
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_rotation(const struct device *dev, int32_t degrees,
                                             enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set the tilt compensation angle for a runtime input processor
 *
 * Motion along the tilt axis (see zmk_input_processor_runtime_set_tilt_axis)
 * is multiplied by 1 / cos(degrees), after rotation. 0 disables it.
 *
 * @param dev Pointer to the device structure
 * @param degrees Tilt in degrees; clamped to 0..60 (60 doubles the axis)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_tilt(const struct device *dev, uint32_t degrees,
                                         enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set the axis that tilt compensation boosts
 *
 * Like a clock hand, clockwise from the 12-6 o'clock (up/down) axis:
 * 0 = up/down, 30 = 1-7 o'clock, 90 = left/right, -30 = 11-5 o'clock.
 * The axis is in the frame after rotation and before axis inversion, XY swap
 * and XY-to-scroll mapping (the physical sensor frame); correct sensor
 * mounting in the sensor driver. On a diagonal axis, drivers that report only
 * the moving axis delay cross terms until the other axis reports.
 *
 * @param dev Pointer to the device structure
 * @param degrees Axis angle in degrees; normalized into (-90, 90] (180-degree periodic)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_tilt_axis(const struct device *dev, int32_t degrees,
                                              enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set the low-speed gain of pointer acceleration ("precision at low speed")
 *
 * Input at or below the devicetree accel-speed-low (counts per 100 ms) is
 * multiplied by percent / 100; at or above accel-speed-high it is unchanged,
 * with a smooth curve in between. Measured and applied after rotation, tilt,
 * inversion and axis snap, before scaling. 100 disables it.
 *
 * @param dev Pointer to the device structure
 * @param percent Gain at low speed in percent; clamped to 20..100 (100 = off)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_accel(const struct device *dev, uint32_t percent,
                                          enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set aim mode: the speed and smoothing used while an aim key is held
 *
 * While an aim behavior is held, cursor output is multiplied by
 * scale_percent / 100 and low-pass filtered with time constant smoothing_ms
 * (see zmk_input_processor_runtime_aim). The devicetree aim-scale-percent and
 * aim-smoothing-ms are the defaults and what reset restores.
 *
 * @param dev Pointer to the device structure
 * @param scale_percent Speed while aiming in percent; clamped to 10..100
 * @param smoothing_ms Smoothing time constant in ms; clamped to 0..200 (0 = no smoothing)
 * @param mode Where to store the values (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_aim(const struct device *dev, uint32_t scale_percent,
                                        uint32_t smoothing_ms,
                                        enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set normal smoothing: the aim filter at full speed while no aim key is held
 *
 * Cursor output is low-pass filtered with time constant ms, as aim mode does
 * but without slowing down; what the filter holds back is released once input
 * stops. While an aim key is held, the aim speed and smoothing apply instead,
 * and what is held back carries over between the two. 0 turns it off (output
 * exactly as without it). The devicetree smoothing-ms is the default.
 *
 * @param dev Pointer to the device structure
 * @param ms Smoothing time constant in ms; clamped to 0..200 (0 = off)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_smoothing(const struct device *dev, uint32_t ms,
                                              enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Reset processor to default values and save to persistent storage
 *
 * @param dev Pointer to the device structure
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_reset(const struct device *dev);

/**
 * @brief Persist every processor's current in-memory settings to flash
 *
 * Flushes values that were set with ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY.
 * Mirrors zmk-feature-custom-settings' save. No-op (returns 0) when built
 * without CONFIG_ZMK_CUSTOM_SETTINGS.
 *
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_save_all(void);

/**
 * @brief Drop every processor's unsaved changes and reload the saved values
 *
 * Reverts each processor to the values currently in flash, discarding any
 * memory-only changes; processors with nothing saved revert to devicetree
 * defaults. Mirrors zmk-feature-custom-settings' discard. No-op (returns 0)
 * when built without CONFIG_ZMK_CUSTOM_SETTINGS.
 *
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_discard_all(void);

/**
 * @brief Reset every processor to its devicetree defaults and persist them
 *
 * Mirrors zmk-feature-custom-settings' reset.
 *
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_reset_all(void);

/**
 * @brief Restore persistent values (used after temporary changes from behavior)
 *
 * @param dev Pointer to the device structure
 */
void zmk_input_processor_runtime_restore_persistent(const struct device *dev);

/**
 * @brief Get the current configuration of a runtime input processor
 *
 * @param dev Pointer to the device structure
 * @param name Pointer to store the processor name (can be NULL)
 * @param config Pointer to store the configuration (can be NULL)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_get_config(const struct device *dev, const char **name,
                                           struct zmk_input_processor_runtime_config *config);

/**
 * @brief Find a runtime input processor by name
 *
 * @param name Name of the processor to find
 * @return Pointer to the device structure, or NULL if not found
 */
const struct device *zmk_input_processor_runtime_find_by_name(const char *name);

/**
 * @brief Find a runtime input processor by ID
 *
 * @param id ID of the processor to find
 * @return Pointer to the device structure, or NULL if not found
 */
const struct device *zmk_input_processor_runtime_find_by_id(uint8_t id);

/**
 * @brief Get the ID of a runtime input processor
 *
 * @param dev Pointer to the device structure
 * @return ID of the processor, or -1 if not found
 */
int zmk_input_processor_runtime_get_id(const struct device *dev);

/**
 * @brief Iterate over all runtime input processors
 *
 * @param callback Callback function to call for each processor
 * @param user_data User data to pass to the callback
 * @return 0 on success, or the first non-zero value returned by callback
 */
int zmk_input_processor_runtime_foreach(int (*callback)(const struct device *dev, void *user_data),
                                        void *user_data);

/**
 * @brief Set temp-layer layer configuration
 *
 * @param dev Pointer to the device structure
 * @param enabled Whether temp-layer is enabled
 * @param layer Target layer ID for temp-layer
 * @param activation_delay_ms Delay before activating layer after input starts (ms)
 * @param deactivation_delay_ms Delay before deactivating layer after input stops (ms)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_temp_layer(const struct device *dev, bool enabled,
                                               uint8_t layer, uint32_t activation_delay_ms,
                                               uint32_t deactivation_delay_ms,
                                               enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set temp-layer enabled state
 *
 * @param dev Pointer to the device structure
 * @param enabled Whether temp-layer is enabled
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_temp_layer_enabled(
    const struct device *dev, bool enabled, enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set temp-layer target layer
 *
 * @param dev Pointer to the device structure
 * @param layer Target layer ID for temp-layer
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_temp_layer_layer(
    const struct device *dev, uint8_t layer, enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set temp-layer activation delay
 *
 * @param dev Pointer to the device structure
 * @param activation_delay_ms Delay before activating layer after input starts (ms)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_temp_layer_activation_delay(
    const struct device *dev, uint32_t activation_delay_ms,
    enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set temp-layer deactivation delay
 *
 * @param dev Pointer to the device structure
 * @param deactivation_delay_ms Delay before deactivating layer after input stops (ms)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_temp_layer_deactivation_delay(
    const struct device *dev, uint32_t deactivation_delay_ms,
    enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set temp-layer activation distance
 *
 * The layer activates only once the motion of one burst adds up to this many
 * output counts (|dx| + |dy| after scaling, about cursor pixels; a pause of
 * more than 150 ms, a key press or the layer switching starts the count over),
 * in addition to the activation delay. 0 activates on any motion, as before.
 * The devicetree temp-layer-activation-distance is the default.
 *
 * @param dev Pointer to the device structure
 * @param distance Output counts; clamped to 0..100 (0 = any motion)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_temp_layer_activation_distance(
    const struct device *dev, uint32_t distance, enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set active layers bitmask
 *
 * @param dev Pointer to the device structure
 * @param layers Bitmask of layers where processor should be active (0 = all layers)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_active_layers(const struct device *dev, uint32_t layers,
                                                  enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Notify temp-layer to keep the layer active (called by behavior)
 *
 * @param dev Pointer to the device structure
 * @param keep_active If true, prevent temp-layer from deactivating; if false, allow deactivation
 */
void zmk_input_processor_runtime_temp_layer_keep_active(const struct device *dev, bool keep_active);

/**
 * @brief Hold or release aim mode (called by the aim behavior)
 *
 * While held by at least one key, cursor output is slowed to the held key's
 * speed (zmk_input_processor_runtime_set_aim_key_scale) or else the aim
 * scale, and low-pass filtered with the aim smoothing (see
 * zmk_input_processor_runtime_set_aim), and the temp-layer layer stays
 * active. Each hold must be matched by one release.
 *
 * @param dev Pointer to the device structure
 * @param hold true on key press, false on key release
 */
void zmk_input_processor_runtime_aim(const struct device *dev, bool hold);

/**
 * @brief Set the speed of the held aim key (called by the aim behavior)
 *
 * While aiming, cursor output is slowed to this percent instead of the aim
 * scale (zmk_input_processor_runtime_set_aim). The aim behavior sets the
 * newest held key's speed before each hold and release. It only scales input
 * from then on, so what the filter holds back neither jumps nor is lost. Not
 * persisted.
 *
 * @param dev Pointer to the device structure
 * @param percent 0 = use the aim scale; anything else is clamped to 10..100
 */
void zmk_input_processor_runtime_set_aim_key_scale(const struct device *dev, uint32_t percent);

/**
 * @brief Let gesture behaviors act on this processor
 *
 * Called once at boot by a gesture behavior for the processor it names. Its
 * motion events then feed the gesture detector, and while gesture keys hold
 * the ball every other processor leaves the motion to it. Only one processor
 * can take gestures; the first wins.
 *
 * @param dev Pointer to the device structure
 */
void zmk_input_processor_runtime_gesture_attach(const struct device *dev);

/**
 * @brief Hold or release gesture mode (called by the gesture behavior)
 *
 * While held by at least one key, the processor's motion moves nothing and a
 * flick (the gesture distance within a short window, see set_gesture) taps
 * the keycode assigned to its direction once. Nothing more fires until the
 * ball stops (or turns back for the next flick); after the last release the
 * cursor stays put until the ball stops. The temp-layer layer stays active
 * while held. Each hold must be matched by one release.
 *
 * @param dev Pointer to the device structure
 * @param hold true on key press, false on key release
 */
void zmk_input_processor_runtime_gesture(const struct device *dev, bool hold);

/**
 * @brief Set what gestures fire and how far a flick must go
 *
 * @param dev Pointer to the device structure
 * @param keycodes ZMK encoded keycodes (as &kp's parameter, implicit modifiers
 *        allowed; 0 = nothing) for left, right, up and down. A keycode that is
 *        not a keyboard or consumer usage is rejected with -EINVAL
 * @param distance Sensor counts a flick covers within the window; clamped to
 *        40..2400
 * @param still_ms How long the ball must be still before the next flick;
 *        clamped to 50..1000
 * @param mode Where to store the values (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_gesture(
    const struct device *dev, const uint32_t keycodes[ZMK_INPUT_PROCESSOR_GESTURE_DIRS],
    uint32_t distance, uint32_t still_ms, enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Turn gesture test mode on or off (RAM only)
 *
 * In test mode a flick raises zmk_input_processor_gesture_fired (Studio shows
 * it) instead of tapping its keycode. It ends by itself 20 s after the last
 * request, so a client that goes away does not leave gestures off for long.
 *
 * @param dev Pointer to the device structure
 * @param enabled true to start (or extend) test mode, false to end it
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_gesture_test(const struct device *dev, bool enabled);

/**
 * @brief Name the key whose aim or gesture hold just ended
 *
 * Called by the aim and gesture behaviors right after releasing aim or the
 * gesture, with the key position, so the record of whether that hold used
 * the ball belongs to that key only.
 *
 * @param dev Pointer to the device structure
 * @param position Key position of the released hold
 */
void zmk_input_processor_runtime_ball_hold_released(const struct device *dev, uint32_t position);

/**
 * @brief Whether the aim or gesture hold of this key that just ended used the ball
 *
 * For the tap side of a tap & hold key whose hold is aim or a gesture: true
 * when the hold from the same key position ended a moment ago after moving
 * the ball (or firing a gesture), so the tap is not typed. Clears the record
 * when it matches.
 *
 * @param dev Pointer to the device structure
 * @param position Key position of the tap
 * @return true to drop the tap
 */
bool zmk_input_processor_runtime_take_ball_used(const struct device *dev, uint32_t position);

/**
 * @brief Set axis snap mode
 *
 * @param dev Pointer to the device structure
 * @param mode Snap mode (NONE, X, or Y)
 * @param write_mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_axis_snap_mode(
    const struct device *dev, uint8_t mode, enum zmk_input_processor_runtime_write_mode write_mode);

/**
 * @brief Set axis snap threshold
 *
 * @param dev Pointer to the device structure
 * @param threshold Threshold value for unsnapping
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_axis_snap_threshold(
    const struct device *dev, uint16_t threshold, enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set axis snap timeout
 *
 * @param dev Pointer to the device structure
 * @param timeout_ms Time window for checking threshold (ms)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_axis_snap_timeout(
    const struct device *dev, uint16_t timeout_ms,
    enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set all axis snap configuration
 *
 * @param dev Pointer to the device structure
 * @param mode Snap mode (NONE, X, or Y)
 * @param threshold Threshold value for unsnapping
 * @param timeout_ms Time window for checking threshold (ms)
 * @param write_mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_axis_snap(
    const struct device *dev, uint8_t mode, uint16_t threshold, uint16_t timeout_ms,
    enum zmk_input_processor_runtime_write_mode write_mode);

/**
 * @brief Set XY-to-scroll enabled state
 *
 * Maps X/Y input to horizontal/vertical scroll wheel events
 *
 * @param dev Pointer to the device structure
 * @param enabled Whether XY-to-scroll mapping is enabled
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_xy_to_scroll_enabled(
    const struct device *dev, bool enabled, enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set XY-swap enabled state
 *
 * Swaps X and Y axes
 *
 * @param dev Pointer to the device structure
 * @param enabled Whether XY-swap is enabled
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_xy_swap_enabled(
    const struct device *dev, bool enabled, enum zmk_input_processor_runtime_write_mode mode);

/**
 *
 * @param dev Pointer to the device structure
 * @param invert If true, invert X axis values (positive becomes negative and vice versa)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_x_invert(const struct device *dev, bool invert,
                                             enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set Y axis inversion
 *
 * @param dev Pointer to the device structure
 * @param invert If true, invert Y axis values (positive becomes negative and vice versa)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_y_invert(const struct device *dev, bool invert,
                                             enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set inertia measurement-window duration.
 *
 * @param dev Pointer to the device structure
 * @param window_ms Duration over which physical input is accumulated (must be > 0)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_inertia_window(
    const struct device *dev, uint16_t window_ms,
    enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set inertia output interval.
 *
 * @param dev Pointer to the device structure
 * @param interval_ms Cadence for inertia output events (must be > 0)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_inertia_interval(
    const struct device *dev, uint16_t interval_ms,
    enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set inertia trigger threshold.
 *
 * A same-direction cumulative input
 * amount reaching this threshold within the measurement window starts inertia.
 * Opposite-direction input totaling max(3, min(threshold / 2, 12))
 * within the window stops it.
 *
 * @param dev Pointer to the device structure
 * @param threshold Input amount threshold (must be greater than zero)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_inertia_threshold(
    const struct device *dev, uint16_t threshold,
    enum zmk_input_processor_runtime_write_mode mode);

/** Enable or disable inertia independently of its trigger threshold. */
int zmk_input_processor_runtime_set_inertia_enabled(
    const struct device *dev, bool enabled,
    enum zmk_input_processor_runtime_write_mode mode);

/** Set the second-stage post-scale input threshold (zero disables it). */
int zmk_input_processor_runtime_set_inertia_fast_threshold(
    const struct device *dev, uint16_t threshold,
    enum zmk_input_processor_runtime_write_mode mode);

/** Set the second-stage synthetic output percentage (100-1000). */
int zmk_input_processor_runtime_set_inertia_fast_output_percent(
    const struct device *dev, uint16_t percent,
    enum zmk_input_processor_runtime_write_mode mode);

/**
 * @brief Set inertia decay per output interval.
 *
 * Without new physical input, this percentage of the retained inertia
 * speed is removed after every output interval. Zero disables decay and 100
 * stops output after one further interval.
 *
 * @param dev Pointer to the device structure
 * @param decay_percent Amount to remove per output interval (0-100)
 * @param mode Where to store the value (persist to flash, memory-only, or temporary)
 * @return 0 on success, negative error code on failure
 */
int zmk_input_processor_runtime_set_inertia_decay(
    const struct device *dev, uint8_t decay_percent,
    enum zmk_input_processor_runtime_write_mode mode);

/** Set the normal-stage synthetic output limit (zero is unlimited). */
int zmk_input_processor_runtime_set_inertia_normal_max_output(
    const struct device *dev, uint16_t max_output,
    enum zmk_input_processor_runtime_write_mode mode);

/** Enable or disable inertia transition notifications in RAM only. */
int zmk_input_processor_runtime_set_inertia_notifications(const struct device *dev, bool enabled);

#if IS_ENABLED(CONFIG_ZMK_RUNTIME_INPUT_PROCESSOR_TEST)
/** @brief Test-only: apply a v1 blob from main. */
int zmk_input_processor_runtime_test_apply_legacy_v1(const struct device *dev);

/** @brief Test-only: apply a v2 blob without the normal-stage output limit. */
int zmk_input_processor_runtime_test_apply_legacy_v2(const struct device *dev);

/** @brief Test-only: apply a v3 blob (rotation 44) without the tilt fields. */
int zmk_input_processor_runtime_test_apply_legacy_v3(const struct device *dev);

/** @brief Test-only: apply a v4 blob (rotation 45) without the acceleration field. */
int zmk_input_processor_runtime_test_apply_legacy_v4(const struct device *dev);

/** @brief Test-only: apply a v5 blob (rotation 46) without the aim fields. */
int zmk_input_processor_runtime_test_apply_legacy_v5(const struct device *dev);

/** @brief Test-only: apply a v6 blob (rotation 47) without normal smoothing. */
int zmk_input_processor_runtime_test_apply_legacy_v6(const struct device *dev);

/** @brief Test-only: apply a v7 blob (rotation 48) without the activation distance. */
int zmk_input_processor_runtime_test_apply_legacy_v7(const struct device *dev);

/** @brief Test-only: verify timed report splitting and rolling expiration. */
int zmk_input_processor_runtime_test_inertia_sliding_window(void);
int zmk_input_processor_runtime_test_inertia_latest_bucket(void);

/**
 * @brief Test-only: force a processor's pending debounced settings save to
 * run now, instead of waiting out CONFIG_ZMK_SETTINGS_SAVE_DEBOUNCE.
 *
 * @param dev Pointer to the device structure
 */
void zmk_input_processor_runtime_test_flush_save(const struct device *dev);

/**
 * @brief Test-only: synchronously re-run the persisted-settings boot-apply
 * logic for every processor, bypassing the normal SYS_INIT delay.
 *
 * Lets a test simulate "reload after reboot" without waiting on real time:
 * write a persistent value, force zmk-feature-custom-settings to reload it
 * from its (fake, in test) settings backend via settings_load_subtree(), then
 * call this to verify the exact apply path the real boot work handler uses.
 */
void zmk_input_processor_runtime_test_apply_persisted_settings(void);

/**
 * @brief Test-only: finish one inertia interval without scheduling an
 * input event, returning the value that would be emitted.
 */
int zmk_input_processor_runtime_test_inertia_tick(const struct device *dev, int16_t *value);
int zmk_input_processor_runtime_test_inertia_tick_at(const struct device *dev, int64_t now,
                                                     int16_t *value);

/** @brief Test-only: report whether the processor is in inertia mode. */
bool zmk_input_processor_runtime_test_inertia_active(const struct device *dev);

/** @brief Test-only: return the final input-event code inertia will emit. */
int zmk_input_processor_runtime_test_inertia_output_code(const struct device *dev,
                                                             uint16_t *code);
#endif
