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

/* Most key positions that can hold one aim behavior at once. */
#define AIM_MAX_HELD 8

/* held_positions are the positions whose press reached the processor. A
 * release only counts from one of them: a hold-tap with hold-while-undecided
 * that decides tap at once (quick tap, require-prior-idle) releases its hold
 * binding without having pressed it, which would otherwise end another
 * key's hold. */
struct behavior_input_processor_aim_data {
    const struct device *processor;
    uint32_t held_positions[AIM_MAX_HELD];
    uint8_t held_count;
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
    struct behavior_input_processor_aim_data *data = dev->data;
    const struct device *processor = aim_processor(dev);
    if (!processor) {
        return -ENODEV;
    }
    if (data->held_count >= AIM_MAX_HELD) {
        // Not held, so its release is ignored too and nothing stays stuck
        LOG_WRN("Aim held by more than %d keys, ignoring position %u", AIM_MAX_HELD,
                event.position);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    data->held_positions[data->held_count++] = event.position;
    zmk_input_processor_runtime_aim(processor, true);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_input_processor_aim_data *data = dev->data;

    uint8_t i = 0;
    while (i < data->held_count && data->held_positions[i] != event.position) {
        i++;
    }
    if (i == data->held_count) {
        LOG_DBG("Aim release from position %u that did not press it, ignored", event.position);
        return ZMK_BEHAVIOR_OPAQUE;
    }
    data->held_positions[i] = data->held_positions[--data->held_count];

    // A recorded press found the processor, so the lookup is cached
    zmk_input_processor_runtime_aim(aim_processor(dev), false);
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
