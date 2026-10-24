/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_input_processor_tap_key

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/pointing/input_processor_runtime.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/*
 * The tap side of a tap & hold key whose hold is aim or a gesture: a key
 * press like &kp, except right after the hold used the ball. Pressed and let
 * go before the hold-tap's tapping term, the key would otherwise type its
 * letter after a quick aim or flick; the hold-tap releases the hold binding
 * and presses this one in one go, so the processor still remembers.
 */

#define TAP_KEY_DROPPED_MAX 8

struct behavior_input_processor_tap_key_config {
    const char *processor_name;
};

/* Positions whose press was dropped, so their release is dropped too. */
struct behavior_input_processor_tap_key_data {
    const struct device *processor;
    uint32_t dropped[TAP_KEY_DROPPED_MAX];
    uint8_t dropped_count;
};

static const struct device *tap_key_processor(const struct device *dev) {
    struct behavior_input_processor_tap_key_data *data = dev->data;
    const struct behavior_input_processor_tap_key_config *cfg = dev->config;

    if (!data->processor) {
        data->processor = zmk_input_processor_runtime_find_by_name(cfg->processor_name);
    }
    return data->processor;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_input_processor_tap_key_data *data = dev->data;
    const struct device *processor = tap_key_processor(dev);

    if (processor && zmk_input_processor_runtime_take_ball_used(processor) &&
        data->dropped_count < TAP_KEY_DROPPED_MAX) {
        LOG_DBG("Tap of position %u dropped: its hold used the ball", event.position);
        data->dropped[data->dropped_count++] = event.position;
        return ZMK_BEHAVIOR_OPAQUE;
    }
    return raise_zmk_keycode_state_changed_from_encoded(binding->param1, true, event.timestamp);
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_input_processor_tap_key_data *data = dev->data;

    for (int i = 0; i < data->dropped_count; i++) {
        if (data->dropped[i] == event.position) {
            data->dropped[i] = data->dropped[--data->dropped_count];
            return ZMK_BEHAVIOR_OPAQUE;
        }
    }
    return raise_zmk_keycode_state_changed_from_encoded(binding->param1, false, event.timestamp);
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
/* The same as &kp's, so a hold-tap using it offers a key as its tap. */
static const struct behavior_parameter_value_metadata param_values[] = {
    {
        .display_name = "Key",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_HID_USAGE,
    },
};

static const struct behavior_parameter_metadata_set param_metadata_set[] = {{
    .param1_values = param_values,
    .param1_values_len = ARRAY_SIZE(param_values),
}};

static const struct behavior_parameter_metadata metadata = {
    .sets_len = ARRAY_SIZE(param_metadata_set),
    .sets = param_metadata_set,
};
#endif

static const struct behavior_driver_api behavior_input_processor_tap_key_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define TAP_KEY_INST(n)                                                                            \
    static struct behavior_input_processor_tap_key_data behavior_input_processor_tap_key_data_##n; \
    static const struct behavior_input_processor_tap_key_config                                    \
        behavior_input_processor_tap_key_config_##n = {                                            \
            .processor_name = DT_INST_PROP(n, processor_name),                                     \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &behavior_input_processor_tap_key_data_##n,             \
                            &behavior_input_processor_tap_key_config_##n, POST_KERNEL,             \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_input_processor_tap_key_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TAP_KEY_INST)
