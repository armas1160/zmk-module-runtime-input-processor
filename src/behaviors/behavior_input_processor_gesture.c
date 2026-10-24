/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_input_processor_gesture

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/pointing/input_processor_runtime.h>
#include <zmk/behavior.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define GESTURE_KEYS_MAX 8

struct behavior_input_processor_gesture_config {
    const char *processor_name;
};

/* positions are the keys whose press reached the processor, oldest first. A
 * release only counts from one of them: a hold-tap with hold-while-undecided
 * that decides tap at once releases its hold binding without having pressed
 * it (see the aim behavior). */
struct behavior_input_processor_gesture_data {
    const struct device *processor;
    uint32_t position[GESTURE_KEYS_MAX];
    uint8_t count;
};

/* Looked up on first use rather than failing init: a split peripheral that
 * shares the devicetree has no processor and never runs the behavior. */
static const struct device *gesture_processor(const struct device *dev) {
    struct behavior_input_processor_gesture_data *data = dev->data;
    const struct behavior_input_processor_gesture_config *cfg = dev->config;

    if (!data->processor) {
        data->processor = zmk_input_processor_runtime_find_by_name(cfg->processor_name);
        if (!data->processor) {
            LOG_ERR("Input processor '%s' not found", cfg->processor_name);
        }
    }
    return data->processor;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_input_processor_gesture_data *data = dev->data;
    const struct device *processor = gesture_processor(dev);
    if (!processor) {
        return -ENODEV;
    }
    if (data->count >= GESTURE_KEYS_MAX) {
        // Not held, so its release is ignored too and nothing stays stuck
        LOG_WRN("Gesture held by more than %d keys, ignoring position %u", GESTURE_KEYS_MAX,
                event.position);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    data->position[data->count++] = event.position;
    zmk_input_processor_runtime_gesture(processor, true);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_input_processor_gesture_data *data = dev->data;

    // The newest press from this position
    int i = (int)data->count - 1;
    while (i >= 0 && data->position[i] != event.position) {
        i--;
    }
    if (i < 0) {
        LOG_DBG("Gesture release from position %u that did not press it, ignored",
                event.position);
        return ZMK_BEHAVIOR_OPAQUE;
    }
    for (; i + 1 < (int)data->count; i++) {
        data->position[i] = data->position[i + 1];
    }
    data->count--;

    // A recorded press found the processor, so the lookup is cached
    zmk_input_processor_runtime_gesture(gesture_processor(dev), false);
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_input_processor_gesture_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define GESTURE_INST(n)                                                                            \
    static struct behavior_input_processor_gesture_data behavior_input_processor_gesture_data_##n; \
    static const struct behavior_input_processor_gesture_config                                    \
        behavior_input_processor_gesture_config_##n = {                                            \
            .processor_name = DT_INST_PROP(n, processor_name),                                     \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &behavior_input_processor_gesture_data_##n,             \
                            &behavior_input_processor_gesture_config_##n, POST_KERNEL,             \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_input_processor_gesture_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GESTURE_INST)

/* Once every device is up, tell the named processor that gestures act on it:
 * it then knows whether the ball is still before the first press, and other
 * processors leave its motion alone while a gesture key holds the ball. On a
 * split peripheral there is no processor and nothing happens. */
#define GESTURE_ATTACH(n)                                                                          \
    zmk_input_processor_runtime_gesture_attach(                                                    \
        zmk_input_processor_runtime_find_by_name(DT_INST_PROP(n, processor_name)));

static int behavior_input_processor_gesture_attach(void) {
    DT_INST_FOREACH_STATUS_OKAY(GESTURE_ATTACH)
    return 0;
}

SYS_INIT(behavior_input_processor_gesture_attach, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
