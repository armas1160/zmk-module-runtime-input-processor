/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_input_processor_aim

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/pointing/input_processor_runtime.h>
#include <zmk/behavior.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_input_processor_aim_config {
    const char *processor_name;
};

struct behavior_input_processor_aim_data {
    const struct device *processor;
};

/* Looked up on first use rather than failing init: a split peripheral that
 * shares the devicetree has no processor and never runs the behavior. */
static const struct device *aim_processor(const struct device *dev) {
    struct behavior_input_processor_aim_data *data = dev->data;
    const struct behavior_input_processor_aim_config *cfg = dev->config;

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
    const struct device *processor = aim_processor(dev);
    if (!processor) {
        return -ENODEV;
    }

    zmk_input_processor_runtime_aim(processor, true);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct device *processor = aim_processor(dev);
    if (!processor) {
        return -ENODEV;
    }

    zmk_input_processor_runtime_aim(processor, false);
    return ZMK_BEHAVIOR_OPAQUE;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
static const struct behavior_parameter_metadata metadata = {
    .sets_len = 0,
    .sets = NULL,
};
#endif

static const struct behavior_driver_api behavior_input_processor_aim_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .parameter_metadata = &metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define AIM_INST(n)                                                                                \
    static struct behavior_input_processor_aim_data behavior_input_processor_aim_data_##n;         \
    static const struct behavior_input_processor_aim_config                                        \
        behavior_input_processor_aim_config_##n = {                                                \
            .processor_name = DT_INST_PROP(n, processor_name),                                     \
    };                                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &behavior_input_processor_aim_data_##n,                 \
                            &behavior_input_processor_aim_config_##n, POST_KERNEL,                 \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_input_processor_aim_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AIM_INST)
