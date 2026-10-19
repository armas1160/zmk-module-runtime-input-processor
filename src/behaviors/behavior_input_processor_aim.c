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

#include "../pointing/rip_aim_math.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_input_processor_aim_config {
    const char *processor_name;
};

/* keys are the positions whose press reached the processor, oldest first,
 * each with its speed parameter (0 = the processor's aim speed); the newest
 * decides the speed (see rip_aim_math.h). A release only counts from one of
 * them: a hold-tap with hold-while-undecided that decides tap at once (quick
 * tap, require-prior-idle) releases its hold binding without having pressed
 * it, which would otherwise end another key's hold. */
struct behavior_input_processor_aim_data {
    const struct device *processor;
    struct rip_aim_keys keys;
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
    if (!rip_aim_keys_press(&data->keys, event.position, binding->param1)) {
        // Not held, so its release is ignored too and nothing stays stuck
        LOG_WRN("Aim held by more than %d keys, ignoring position %u", RIP_AIM_KEYS_MAX,
                event.position);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    // The newest key's speed first, so its first event is already at that speed
    zmk_input_processor_runtime_set_aim_key_scale(processor, rip_aim_keys_percent(&data->keys));
    zmk_input_processor_runtime_aim(processor, true);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    struct behavior_input_processor_aim_data *data = dev->data;

    if (!rip_aim_keys_release(&data->keys, event.position)) {
        LOG_DBG("Aim release from position %u that did not press it, ignored", event.position);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    // A recorded press found the processor, so the lookup is cached. Back to
    // the newest key still held (or the processor's speed), then release
    const struct device *processor = aim_processor(dev);
    zmk_input_processor_runtime_set_aim_key_scale(processor, rip_aim_keys_percent(&data->keys));
    zmk_input_processor_runtime_aim(processor, false);
    return ZMK_BEHAVIOR_OPAQUE;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
/* One set, so a hold-tap with this as its hold binding offers the same param1
 * values (it takes the hold behavior's first set). */
static const struct behavior_parameter_value_metadata speed_values[] = {
    {
        .display_name = "Default",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_VALUE,
        .value = 0,
    },
    {
        .display_name = "Speed %",
        .type = BEHAVIOR_PARAMETER_VALUE_TYPE_RANGE,
        .range = {.min = RIP_AIM_MIN_PERCENT, .max = RIP_AIM_MAX_PERCENT},
    },
};

static const struct behavior_parameter_metadata_set metadata_sets[] = {{
    .param1_values = speed_values,
    .param1_values_len = ARRAY_SIZE(speed_values),
}};

static const struct behavior_parameter_metadata metadata = {
    .sets_len = ARRAY_SIZE(metadata_sets),
    .sets = metadata_sets,
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
