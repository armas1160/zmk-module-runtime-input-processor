/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

/**
 * @file rip_settings_test.c
 *
 * Exercises the zmk-feature-custom-settings-backed persistence added for the
 * runtime input processor (see docs/design/custom-settings-storage.md): a
 * Set*-equivalent write through the module's public firmware API must
 * survive a settings backend reload, not just an in-RAM cache.
 *
 * This runs from a SYS_INIT hook at APPLICATION level, i.e. before ZMK
 * main() calls settings_subsys_init()/settings_load() - so it registers its
 * own minimal fake in-RAM struct settings_store first (mirroring
 * zmk-feature-custom-settings' own src/test/custom_settings_test.c
 * test_settings_backend_init/test_settings_save/test_settings_load), then:
 *
 *  1. Writes a new persistent scaling value through the ordinary firmware
 *     API (what the Studio RPC handlers call) and force-flushes the
 *     debounced save.
 *  2. Clobbers the *device's own* runtime state (a temporary, non-persistent
 *     set to different values) so step 4 cannot pass by accident just
 *     because the device's live struct still happened to hold the right
 *     value.
 *  3. Forces zmk-feature-custom-settings to reload every entry from the fake
 *     settings backend (settings_load_subtree) - exactly what ZMK main()'s
 *     real settings_load() does at boot.
 *  4. Re-runs this module's boot-apply logic and confirms the reloaded value
 *     made it back into the processor's persistent/current fields.
 */

#include <errno.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#include <cormoran/zmk/custom_settings.h>
#include <drivers/input_processor.h>
#include <zmk/keymap.h>
#include <zmk/events/input_processor_inertia_state_changed.h>
#include <zmk/pointing/input_processor_runtime.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* processor-label of tests/studio/native_sim.keymap's runtime_input_processor. */
#define TEST_PROCESSOR_NAME "default"

static enum zmk_input_processor_inertia_stop_reason last_inertia_stop_reason;
static bool last_inertia_fast_input;
static uint32_t inertia_fast_input_transitions;

static int test_inertia_state_listener(const zmk_event_t *eh) {
    const struct zmk_input_processor_inertia_state_changed *ev =
        as_zmk_input_processor_inertia_state_changed(eh);
    if (ev) {
        if (ev->state == ZMK_INPUT_PROCESSOR_INERTIA_FAST_INPUT_STARTED ||
            ev->state == ZMK_INPUT_PROCESSOR_INERTIA_FAST_INPUT_STOPPED) {
            last_inertia_fast_input =
                ev->state == ZMK_INPUT_PROCESSOR_INERTIA_FAST_INPUT_STARTED;
            inertia_fast_input_transitions++;
        } else if (ev->state == ZMK_INPUT_PROCESSOR_INERTIA_STOPPED) {
            last_inertia_stop_reason = ev->stop_reason;
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(test_inertia_state, test_inertia_state_listener);
ZMK_SUBSCRIPTION(test_inertia_state, zmk_input_processor_inertia_state_changed);

/* --- Minimal fake in-RAM settings backend -------------------------------
 * ZMK main() normally calls settings_subsys_init() + settings_load() after
 * every SYS_INIT level has run, so this SYS_INIT-driven test cannot rely on
 * a real (flash-backed) settings store existing yet. Register a fake one
 * ourselves instead, exactly like zmk-feature-custom-settings' own
 * src/test/custom_settings_test.c does. */
#define TEST_SETTINGS_STORAGE_CAPACITY 4

struct test_settings_record {
    bool present;
    char name[SETTINGS_MAX_NAME_LEN];
    uint8_t data[CONFIG_ZMK_CUSTOM_SETTINGS_VALUE_MAX_SIZE];
    size_t len;
};

static struct test_settings_record test_settings_storage[TEST_SETTINGS_STORAGE_CAPACITY];

static struct test_settings_record *test_settings_find_record(const char *name) {
    for (size_t i = 0; i < ARRAY_SIZE(test_settings_storage); i++) {
        if (test_settings_storage[i].present &&
            strncmp(test_settings_storage[i].name, name, sizeof(test_settings_storage[i].name)) ==
                0) {
            return &test_settings_storage[i];
        }
    }
    return NULL;
}

static ssize_t test_settings_read_cb(void *cb_arg, void *data, size_t len) {
    const struct test_settings_record *record = cb_arg;
    size_t read_len = MIN(record->len, len);
    memcpy(data, record->data, read_len);
    return read_len;
}

static int test_settings_load(struct settings_store *cs, const struct settings_load_arg *arg) {
    ARG_UNUSED(cs);

    int first_error = 0;
    for (size_t i = 0; i < ARRAY_SIZE(test_settings_storage); i++) {
        struct test_settings_record *record = &test_settings_storage[i];
        if (!record->present) {
            continue;
        }
        int ret = settings_call_set_handler(record->name, record->len, test_settings_read_cb,
                                            record, arg);
        if (ret < 0 && first_error == 0) {
            first_error = ret;
        }
    }
    return first_error;
}

static int test_settings_save(struct settings_store *cs, const char *name, const char *value,
                              size_t val_len) {
    ARG_UNUSED(cs);

    struct test_settings_record *record = test_settings_find_record(name);
    if (value == NULL) {
        if (record) {
            record->present = false;
        }
        return 0;
    }

    if (val_len > sizeof(record->data)) {
        return -EMSGSIZE;
    }
    if (strlen(name) >= SETTINGS_MAX_NAME_LEN) {
        return -ENAMETOOLONG;
    }

    if (!record) {
        for (size_t i = 0; i < ARRAY_SIZE(test_settings_storage); i++) {
            if (!test_settings_storage[i].present) {
                record = &test_settings_storage[i];
                break;
            }
        }
    }
    if (!record) {
        return -ENOMEM;
    }

    record->present = true;
    strcpy(record->name, name);
    memcpy(record->data, value, val_len);
    record->len = val_len;
    return 0;
}

static const struct settings_store_itf test_settings_itf = {
    .csi_load = test_settings_load,
    .csi_save = test_settings_save,
};

static struct settings_store test_settings_store = {.cs_itf = &test_settings_itf};

static int test_settings_backend_init(void) {
    int ret = settings_subsys_init();
    if (ret < 0) {
        return ret;
    }
    settings_src_register(&test_settings_store);
    settings_dst_register(&test_settings_store);
    return 0;
}

/* --- The actual persistence round-trip test ----------------------------- */

static int test_scaling_persists_across_reload(void) {
    const struct device *dev = zmk_input_processor_runtime_find_by_name(TEST_PROCESSOR_NAME);
    if (!dev) {
        LOG_ERR("Test processor '%s' not found", TEST_PROCESSOR_NAME);
        return -ENODEV;
    }

    struct zmk_input_processor_runtime_config before;
    int ret = zmk_input_processor_runtime_get_config(dev, NULL, &before);
    if (ret < 0) {
        return ret;
    }

    // Pick values guaranteed to differ from the devicetree default.
    uint32_t new_multiplier = before.scale_multiplier + 5;
    uint32_t new_divisor = before.scale_divisor + 3;

    ret = zmk_input_processor_runtime_set_scaling(dev, new_multiplier, new_divisor,
                                                  ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        LOG_ERR("set_scaling failed: %d", ret);
        return ret;
    }

    // 1. Force the debounced save to run (and actually persist) now.
    zmk_input_processor_runtime_test_flush_save(dev);

    // 2. Clobber the *device's own* persistent_* fields back to devicetree
    //    defaults (zmk_input_processor_runtime_reset also reschedules a
    //    save of the defaults, but that debounced work is never flushed
    //    before this test's assertions run below, so it cannot race with
    //    step 3/4). This proves step 4 cannot pass by accident just because
    //    the device's struct simply never changed.
    zmk_input_processor_runtime_reset(dev);

    // 3. Force a reload from the settings backend, exactly like ZMK main()'s
    //    real settings_load() at boot.
    ret = settings_load_subtree("custom_settings");
    if (ret < 0) {
        LOG_ERR("settings_load_subtree failed: %d", ret);
        return ret;
    }

    // 4. Re-run the boot-apply logic and verify the persisted value won.
    zmk_input_processor_runtime_test_apply_persisted_settings();

    struct zmk_input_processor_runtime_config after;
    ret = zmk_input_processor_runtime_get_config(dev, NULL, &after);
    if (ret < 0) {
        return ret;
    }

    if (after.scale_multiplier != new_multiplier || after.scale_divisor != new_divisor) {
        LOG_ERR("Persisted scaling not restored: got %u/%u expected %u/%u", after.scale_multiplier,
                after.scale_divisor, new_multiplier, new_divisor);
        return -EINVAL;
    }

    LOG_INF("PASS: rip_settings_persist_reload scale=%u/%u", after.scale_multiplier,
            after.scale_divisor);
    return 0;
}

/* Exercise the processor through its public driver API so inertia
 * entry and reverse-direction exit are tested at the runtime boundary. */
static int process_test_relative_x_with_code(const struct device *dev, int32_t input,
                                             int32_t *output, uint16_t *code) {
    struct input_event event = {
        .type = INPUT_EV_REL,
        .code = INPUT_REL_X,
        .value = input,
    };

    int ret = zmk_input_processor_handle_event(dev, &event, 0, 0, NULL);
    if (ret != ZMK_INPUT_PROC_CONTINUE) {
        LOG_ERR("Input processor returned %d", ret);
        return -EINVAL;
    }

    *output = event.value;
    if (code) {
        *code = event.code;
    }
    return 0;
}

static int process_test_relative_x(const struct device *dev, int32_t input, int32_t *output) {
    return process_test_relative_x_with_code(dev, input, output, NULL);
}

static int process_test_relative_x_with_remainder(const struct device *dev, int16_t input,
                                                  int16_t *remainder, int32_t *output) {
    struct input_event event = {.type = INPUT_EV_REL, .code = INPUT_REL_X, .value = input};
    struct zmk_input_processor_state state = {.remainder = remainder};
    int ret = zmk_input_processor_handle_event(dev, &event, 0, 0, &state);
    *output = event.value;
    return ret == ZMK_INPUT_PROC_CONTINUE ? 0 : -EINVAL;
}

/* Advance just the measurement time past all reports; do not run asynchronous
 * work or sleep when checking the retained tail and its fractional decay. */
static int inertia_tail_tick(const struct device *dev, int16_t *value) {
    return zmk_input_processor_runtime_test_inertia_tick_at(dev, k_uptime_get() + 1000, value);
}

static int test_inertia(void) {
    int ret = zmk_input_processor_runtime_test_inertia_latest_bucket();
    if (ret < 0) {
        LOG_ERR("Inertia latest-bucket compensation failed");
        return ret;
    }
    LOG_INF("PASS: rip_inertia_latest_bucket_compensation");
    ret = zmk_input_processor_runtime_test_inertia_sliding_window();
    if (ret < 0) {
        LOG_ERR("Inertia sliding-window split/expiry check failed");
        return ret;
    }
    const struct device *dev = zmk_input_processor_runtime_find_by_name(TEST_PROCESSOR_NAME);
    if (!dev) {
        LOG_ERR("Test processor '%s' not found", TEST_PROCESSOR_NAME);
        return -ENODEV;
    }
    ret = zmk_input_processor_runtime_set_inertia_notifications(dev, true);
    if (ret < 0) {
        return ret;
    }

    /* Keep preceding persistence tests from affecting these output checks. */
    ret = zmk_input_processor_runtime_set_rotation(
        dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_scaling(dev, 1, 1,
                                                  ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_xy_to_scroll_enabled(
        dev, true, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_window(
        dev, 100, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_interval(
        dev, 20, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_decay(
        dev, 50, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    if (zmk_input_processor_runtime_set_inertia_decay(
            dev, 101, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) != -EINVAL) {
        LOG_ERR("Inertia decay accepted an out-of-range percentage");
        return -EINVAL;
    }

    /* Disabling inertia keeps the threshold available for later use. */
    ret = zmk_input_processor_runtime_set_inertia_enabled(
        dev, false, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    int32_t output;
    uint16_t output_code;
    if (ret < 0 ||
        (ret = process_test_relative_x_with_code(dev, 30, &output, &output_code)) < 0) {
        return ret;
    }
    if (output_code != INPUT_REL_HWHEEL || zmk_input_processor_runtime_test_inertia_active(dev)) {
        return -EINVAL;
    }

    ret = zmk_input_processor_runtime_set_inertia_enabled(
        dev, true, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0 || zmk_input_processor_runtime_set_inertia_threshold(
            dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) != -EINVAL) {
        return ret < 0 ? ret : -EINVAL;
    }
    ret = zmk_input_processor_runtime_set_inertia_threshold(
        dev, 10, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }

    /* The triggering input seeds the speed but must not add synthetic output
     * while the latest physical bucket already exceeds that speed. */
    if ((ret = process_test_relative_x(dev, 6, &output)) < 0 ||
        (ret = process_test_relative_x(dev, 5, &output)) < 0) {
        return ret;
    }
    if (!zmk_input_processor_runtime_test_inertia_active(dev)) {
        return -EINVAL;
    }
    ret = zmk_input_processor_runtime_test_inertia_output_code(dev, &output_code);
    if (ret < 0 || output_code != INPUT_REL_HWHEEL) {
        LOG_ERR("Automatic scroll output code: got %u expected %u", output_code,
                INPUT_REL_HWHEEL);
        return ret < 0 ? ret : -EINVAL;
    }
    int16_t inertia_output;
    if ((ret = zmk_input_processor_runtime_test_inertia_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 0) {
        LOG_ERR("Inertia added output on entry: %d", inertia_output);
        return ret < 0 ? ret : -EINVAL;
    }
    /* In inertia mode, post-entry input is accumulated in a 100 ms window
     * while output is emitted every 20 ms. Fractional output is carried. */
    if ((ret = process_test_relative_x(dev, 17, &output)) < 0) {
        return ret;
    }
    ret = inertia_tail_tick(dev, &inertia_output);
    if (ret < 0 || inertia_output != 5) {
        LOG_ERR("Inertia first output: got %d expected 5", inertia_output);
        return ret < 0 ? ret : -EINVAL;
    }
    /* The same measurement window now totals 34, increasing the speed. */
    if ((ret = process_test_relative_x(dev, 6, &output)) < 0 ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 7) {
        LOG_ERR("Inertia max output: got %d expected 7", inertia_output);
        return ret < 0 ? ret : -EINVAL;
    }

    /* With no new input, the retained speed decays across output intervals. */
    if ((ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 7) {
        LOG_ERR("Inertia initial decay output: got %d expected 7", inertia_output);
        return ret < 0 ? ret : -EINVAL;
    }
    if ((ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 3) {
        LOG_ERR("Inertia decayed output: got %d expected 3", inertia_output);
        return ret < 0 ? ret : -EINVAL;
    }

    /* An isolated reverse count is sensor jitter; deliberate reversal exits. */
    if ((ret = process_test_relative_x(dev, -1, &output)) < 0) {
        return ret;
    }
    if (!zmk_input_processor_runtime_test_inertia_active(dev) ||
        (ret = process_test_relative_x(dev, -4, &output)) < 0) {
        return ret < 0 ? ret : -EINVAL;
    }
    if (zmk_input_processor_runtime_test_inertia_active(dev)) {
        return -EINVAL;
    }
    if (last_inertia_stop_reason != ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_REVERSE_INPUT) {
        LOG_ERR("Inertia reverse stop reason missing");
        return -EINVAL;
    }

    /* A small decay rate follows a fractional exponential curve. */
    ret = zmk_input_processor_runtime_set_inertia_window(
        dev, 20, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_interval(
        dev, 20, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_decay(
        dev, 5, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0 || (ret = process_test_relative_x(dev, 6, &output)) < 0 ||
        (ret = process_test_relative_x(dev, 5, &output)) < 0 ||
        (ret = process_test_relative_x(dev, 50, &output)) < 0) {
        return ret;
    }
    if ((ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 61 ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 61 ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 57 ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 56) {
        LOG_ERR("Inertia fractional decay: got %d expected 56", inertia_output);
        return ret < 0 ? ret : -EINVAL;
    }

    /* Zero decay keeps output alive. Carry fractional counts so a slow
     * setting still produces movement across several output intervals. */
    ret = zmk_input_processor_runtime_set_inertia_window(
        dev, 100, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_interval(
        dev, 10, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_decay(
        dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_threshold(
        dev, 3, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0 || (ret = process_test_relative_x(dev, 3, &output)) < 0) {
        return ret;
    }
    int32_t total = 0;
    for (int i = 0; i < 20; i++) {
        ret = inertia_tail_tick(dev, &inertia_output);
        if (ret < 0) {
            return ret;
        }
        total += inertia_output;
    }
    if (total != 6 || !zmk_input_processor_runtime_test_inertia_active(dev)) {
        LOG_ERR("Zero-decay low-speed output: total=%d active=%d expected 6/1", total,
                zmk_input_processor_runtime_test_inertia_active(dev));
        return -EINVAL;
    }

    /* At low speed, decay must multiply the fractional retained speed each
     * interval. Integer-count decay emits on tick 7 instead of tick 8. */
    ret = zmk_input_processor_runtime_set_inertia_decay(
        dev, 5, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0 || (ret = process_test_relative_x(dev, 3, &output)) < 0) {
        return ret;
    }
    static const int16_t exponential_tail[] = {0, 0, 0, 1, 0, 0, 0, 1};
    for (size_t i = 0; i < ARRAY_SIZE(exponential_tail); i++) {
        ret = inertia_tail_tick(dev, &inertia_output);
        if (ret < 0 || inertia_output != exponential_tail[i]) {
            LOG_ERR("Exponential tail tick %u: got %d expected %d", (unsigned int)i,
                    inertia_output, exponential_tail[i]);
            return ret < 0 ? ret : -EINVAL;
        }
    }

    ret = zmk_input_processor_runtime_set_inertia_decay(
        dev, 100, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0 || (ret = process_test_relative_x(dev, 3, &output)) < 0 ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        !zmk_input_processor_runtime_test_inertia_active(dev) ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        zmk_input_processor_runtime_test_inertia_active(dev)) {
        LOG_ERR("100%% decay did not stop after one further interval");
        return ret < 0 ? ret : -EINVAL;
    }
    if (last_inertia_stop_reason != ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTLED) {
        LOG_ERR("Inertia settled stop reason missing");
        return -EINVAL;
    }

    /* A sub-count tail must end as soon as its entire future contribution,
     * including the carried fraction, cannot make another scroll count. */
    ret = zmk_input_processor_runtime_set_inertia_window(
        dev, 200, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0 || (ret = zmk_input_processor_runtime_set_inertia_interval(
                          dev, 20, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_decay(
             dev, 50, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_threshold(
             dev, 1, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_relative_x(dev, 1, &output)) < 0 ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 0 || !zmk_input_processor_runtime_test_inertia_active(dev) ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 0 || zmk_input_processor_runtime_test_inertia_active(dev) ||
        last_inertia_stop_reason != ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTLED) {
        LOG_ERR("Sub-count inertia tail remained active");
        return ret < 0 ? ret : -EINVAL;
    }

    /* Entry uses actual scaled reports; the synthetic tail uses raw speed
     * and carries the 1/60 scale remainder across output intervals. */
    ret = zmk_input_processor_runtime_set_scaling(
        dev, 1, 60, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0 || (ret = zmk_input_processor_runtime_set_inertia_decay(
                          dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_threshold(
             dev, 2, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0) {
        return ret;
    }
    int16_t physical_remainder = 0;
    if ((ret = process_test_relative_x_with_remainder(dev, 60, &physical_remainder, &output)) < 0 ||
        output != 1 || zmk_input_processor_runtime_test_inertia_active(dev) ||
        (ret = process_test_relative_x_with_remainder(dev, 59, &physical_remainder, &output)) < 0 ||
        output != 0 || zmk_input_processor_runtime_test_inertia_active(dev) ||
        (ret = process_test_relative_x_with_remainder(dev, 1, &physical_remainder, &output)) < 0 ||
        output != 1 || !zmk_input_processor_runtime_test_inertia_active(dev)) {
        LOG_ERR("Scaled inertia threshold did not follow physical scroll output");
        return ret < 0 ? ret : -EINVAL;
    }
    total = 0;
    for (int i = 0; i < 10; i++) {
        ret = inertia_tail_tick(dev, &inertia_output);
        if (ret < 0) {
            return ret;
        }
        total += inertia_output;
    }
    if (total != 2 || inertia_output != 1 ||
        !zmk_input_processor_runtime_test_inertia_active(dev)) {
        LOG_ERR("Raw inertia output lost its 1/60 scaling: total=%d last=%d", total,
                inertia_output);
        return -EINVAL;
    }
    /* The next raw report scales to zero, but must still raise the speed
     * used by the active inertia tail. */
    if ((ret = process_test_relative_x_with_remainder(dev, 59, &physical_remainder, &output)) < 0 ||
        output != 0) {
        return ret < 0 ? ret : -EINVAL;
    }
    total = 0;
    for (int i = 0; i < 20; i++) {
        ret = inertia_tail_tick(dev, &inertia_output);
        if (ret < 0) {
            return ret;
        }
        total += inertia_output;
    }
    if (total != 5) {
        LOG_ERR("Zero-scale physical input was lost to inertia: total=%d", total);
        return -EINVAL;
    }
    ret = zmk_input_processor_runtime_set_scaling(
        dev, 1, 1, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0 || zmk_input_processor_runtime_test_inertia_active(dev) ||
        last_inertia_stop_reason != ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_SETTINGS_CHANGED) {
        return ret < 0 ? ret : -EINVAL;
    }

    /* Turning off the only targeted layer must stop its active inertia before
     * another synthetic interval can emit scroll. */
    ret = zmk_input_processor_runtime_set_inertia_decay(
        dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0 || (ret = zmk_keymap_layer_activate(1, false)) < 0 ||
        (ret = zmk_input_processor_runtime_set_active_layers(
             dev, BIT(1), ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_relative_x(dev, 3, &output)) < 0 ||
        !zmk_input_processor_runtime_test_inertia_active(dev)) {
        LOG_ERR("Failed to start inertia on targeted layer");
        return ret < 0 ? ret : -EINVAL;
    }
    ret = zmk_keymap_layer_deactivate(1, false);
    if (ret < 0 || zmk_input_processor_runtime_test_inertia_active(dev) ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 0 || zmk_input_processor_runtime_test_inertia_active(dev)) {
        LOG_ERR("Inertia continued after targeted layer deactivation");
        return ret < 0 ? ret : -EINVAL;
    }
    if (last_inertia_stop_reason != ZMK_INPUT_PROCESSOR_INERTIA_STOP_REASON_LAYER_INACTIVE) {
        LOG_ERR("Inertia layer stop reason missing");
        return -EINVAL;
    }
    if ((ret = process_test_relative_x(dev, 10, &output)) < 0 ||
        zmk_input_processor_runtime_test_inertia_active(dev)) {
        LOG_ERR("Inertia restarted while targeted layer was inactive");
        return ret < 0 ? ret : -EINVAL;
    }

    /* Fast output starts only after the second scaled input threshold. */
    uint32_t fast_transitions_before = inertia_fast_input_transitions;
    if ((ret = zmk_input_processor_runtime_set_active_layers(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_window(
             dev, 100, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_interval(
             dev, 20, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_threshold(
             dev, 10, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_fast_threshold(
             dev, 20, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_fast_output_percent(
             dev, 200, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_normal_max_output(
             dev, 1, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        zmk_input_processor_runtime_set_inertia_normal_max_output(
            dev, 32768, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) != -EINVAL ||
        zmk_input_processor_runtime_set_inertia_fast_output_percent(
            dev, 99, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY) != -EINVAL ||
        (ret = process_test_relative_x(dev, 11, &output)) < 0 ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 1 || last_inertia_fast_input ||
        inertia_fast_input_transitions != fast_transitions_before + 1 ||
        (ret = process_test_relative_x(dev, 10, &output)) < 0 ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 8 || !last_inertia_fast_input ||
        inertia_fast_input_transitions != fast_transitions_before + 2) {
        LOG_ERR("Fast inertia stage failed: output=%d", inertia_output);
        return ret < 0 ? ret : -EINVAL;
    }
    ret = zmk_input_processor_runtime_set_inertia_normal_max_output(
        dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    if ((ret = zmk_input_processor_runtime_set_scaling(
             dev, 2, 1, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_fast_threshold(
             dev, 30, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_relative_x(dev, 6, &output)) < 0 ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 2 ||
        (ret = process_test_relative_x(dev, 9, &output)) < 0 ||
        (ret = inertia_tail_tick(dev, &inertia_output)) < 0 ||
        inertia_output != 12) {
        LOG_ERR("Scaled fast threshold failed: output=%d", inertia_output);
        return ret < 0 ? ret : -EINVAL;
    }

    /* The first fast notification also covers a start that already exceeds the fast threshold. */
    ret = zmk_input_processor_runtime_set_inertia_fast_threshold(
        dev, 10, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY);
    if (ret < 0) {
        return ret;
    }
    fast_transitions_before = inertia_fast_input_transitions;
    if ((ret = process_test_relative_x(dev, 11, &output)) < 0 ||
        !last_inertia_fast_input ||
        inertia_fast_input_transitions != fast_transitions_before + 1) {
        LOG_ERR("Fast input notification missing at inertia start");
        return ret < 0 ? ret : -EINVAL;
    }

    zmk_input_processor_runtime_restore_persistent(dev);
    zmk_input_processor_runtime_set_inertia_notifications(dev, false);
    LOG_INF("PASS: rip_inertia_decay_reverse_scroll");
    return 0;
}

/* Exercises the write modes and the save/discard-all operations:
 *  - a WRITE_MODE_MEMORY write updates the baseline in RAM but must NOT reach
 *    flash, so discard_all (reload from flash) reverts it;
 *  - a subsequent save_all flushes the in-RAM baseline to flash so it then
 *    survives a settings reload.
 * Uses rotation (independent of the scaling the first test leaves persisted).
 */
static int test_write_modes_and_save_discard(void) {
    const struct device *dev = zmk_input_processor_runtime_find_by_name(TEST_PROCESSOR_NAME);
    if (!dev) {
        LOG_ERR("Test processor '%s' not found", TEST_PROCESSOR_NAME);
        return -ENODEV;
    }

    // Establish a known persisted baseline (persist + flush to the backend).
    const int32_t persisted_rot = 11;
    int ret = zmk_input_processor_runtime_set_rotation(
        dev, persisted_rot, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_flush_save(dev);

    // MEMORY write: updates the baseline in RAM only (visible via get_config)
    // but must not touch flash.
    const int32_t memory_rot = persisted_rot + 7;
    ret = zmk_input_processor_runtime_set_rotation(dev, memory_rot,
                                                   ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
    if (ret < 0) {
        return ret;
    }

    struct zmk_input_processor_runtime_config cfg;
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.rotation_degrees != memory_rot) {
        LOG_ERR("Memory write not staged: got %d expected %d", cfg.rotation_degrees, memory_rot);
        return -EINVAL;
    }

    // discard_all must drop the unsaved memory change and revert to flash.
    ret = zmk_input_processor_runtime_discard_all();
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.rotation_degrees != persisted_rot) {
        LOG_ERR("Discard did not revert memory write: got %d expected %d", cfg.rotation_degrees,
                persisted_rot);
        return -EINVAL;
    }

    // A memory write followed by save_all must survive a settings reload.
    const int32_t saved_rot = persisted_rot + 3;
    ret = zmk_input_processor_runtime_set_rotation(dev, saved_rot,
                                                   ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_save_all();
    if (ret < 0) {
        return ret;
    }

    // Clobber the live struct, reload from the backend, re-apply.
    zmk_input_processor_runtime_reset(dev);
    ret = settings_load_subtree("custom_settings");
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_apply_persisted_settings();

    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.rotation_degrees != saved_rot) {
        LOG_ERR("save_all did not persist memory value: got %d expected %d", cfg.rotation_degrees,
                saved_rot);
        return -EINVAL;
    }

    // The tuned inertia defaults must survive unrelated writes.
    if (cfg.inertia_window_ms != 200 || cfg.inertia_interval_ms != 20 ||
        cfg.inertia_threshold != 12 || cfg.inertia_decay_percent != 8 ||
        cfg.inertia_normal_max_output != 0 ||
        !cfg.inertia_enabled || cfg.inertia_fast_threshold != 0 ||
        cfg.inertia_fast_output_percent != 200 ||
        cfg.inertia_notifications_enabled) {
        LOG_ERR("Unexpected inertia defaults: window=%u interval=%u threshold=%u decay=%u",
                cfg.inertia_window_ms, cfg.inertia_interval_ms, cfg.inertia_threshold,
                cfg.inertia_decay_percent);
        return -EINVAL;
    }
    ret = zmk_input_processor_runtime_set_inertia_window(
        dev, 250, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_interval(
        dev, 25, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_threshold(
        dev, 23, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_decay(
        dev, 7, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_normal_max_output(
        dev, 9, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        return ret;
    }
    if ((ret = zmk_input_processor_runtime_set_inertia_fast_threshold(
             dev, 48, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_fast_output_percent(
             dev, 250, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST)) < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_flush_save(dev);

    ret = zmk_input_processor_runtime_set_inertia_window(
        dev, 50, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_interval(
        dev, 5, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_threshold(
        dev, 7, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_decay(
        dev, 1, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_normal_max_output(
        dev, 3, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
    if (ret < 0) {
        return ret;
    }
    if ((ret = zmk_input_processor_runtime_set_inertia_fast_threshold(
             dev, 60, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_fast_output_percent(
             dev, 300, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY)) < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_set_inertia_notifications(dev, true);
    if (ret < 0) {
        return ret;
    }
    ret = zmk_input_processor_runtime_discard_all();
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.inertia_window_ms != 250 || cfg.inertia_interval_ms != 25 ||
        cfg.inertia_threshold != 23 || cfg.inertia_decay_percent != 7 ||
        cfg.inertia_normal_max_output != 9 ||
        cfg.inertia_fast_threshold != 48 || cfg.inertia_fast_output_percent != 250 ||
        !cfg.inertia_notifications_enabled) {
        LOG_ERR("Inertia discard did not restore flash value: window=%u interval=%u "
                "threshold=%u decay=%u",
                cfg.inertia_window_ms, cfg.inertia_interval_ms, cfg.inertia_threshold,
                cfg.inertia_decay_percent);
        return -EINVAL;
    }

    ret = zmk_input_processor_runtime_test_apply_legacy_v1(dev);
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.rotation_degrees != 42 || cfg.inertia_window_ms != 250 ||
        cfg.inertia_interval_ms != 25 || cfg.inertia_threshold != 23 ||
        cfg.inertia_decay_percent != 7 || cfg.inertia_normal_max_output != 9 ||
        cfg.inertia_fast_threshold != 48 ||
        cfg.inertia_fast_output_percent != 250) {
        LOG_ERR("Legacy v1 settings were not migrated");
        return -EINVAL;
    }
    ret = zmk_input_processor_runtime_test_apply_legacy_v2(dev);
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.rotation_degrees != 43 || cfg.inertia_normal_max_output != 9) {
        LOG_ERR("Legacy v2 settings were not migrated");
        return -EINVAL;
    }
    ret = zmk_input_processor_runtime_set_inertia_enabled(
        dev, false, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_flush_save(dev);
    ret = zmk_input_processor_runtime_set_inertia_enabled(
        dev, true, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
    if (ret < 0 || (ret = zmk_input_processor_runtime_discard_all()) < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.inertia_enabled || cfg.inertia_threshold != 23) {
        LOG_ERR("Inertia enabled flag did not persist independently of threshold");
        return -EINVAL;
    }
    LOG_INF("PASS: rip_settings_write_modes rot=%d legacy_v1_v2=ok", cfg.rotation_degrees);
    return 0;
}

static int process_test_relative(const struct device *dev, uint16_t code, int32_t input,
                                 int32_t *output) {
    struct input_event event = {.type = INPUT_EV_REL, .code = code, .value = input};
    int ret = zmk_input_processor_handle_event(dev, &event, 0, 0, NULL);
    *output = event.value;
    return ret == ZMK_INPUT_PROC_CONTINUE ? 0 : -EINVAL;
}

/* Feed X then Y then X. Each event is transformed on its own axis at once;
 * cross terms are carried to the other axis' next event. */
static int process_test_xyx(const struct device *dev, int32_t x, int32_t y, int32_t out[3]) {
    int ret;
    if ((ret = process_test_relative(dev, INPUT_REL_X, x, &out[0])) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_Y, y, &out[1])) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_X, x, &out[2])) < 0) {
        return ret;
    }
    return 0;
}

/* Feed `count` reports of one axis only, as drivers that skip a zero axis do. */
static int process_test_single_axis_run(const struct device *dev, uint16_t code, int32_t value,
                                        int count, int32_t *total) {
    *total = 0;
    for (int i = 0; i < count; i++) {
        int32_t out;
        int ret = process_test_relative(dev, code, value, &out);
        if (ret < 0) {
            return ret;
        }
        *total += out;
    }
    return 0;
}

/* Tilt compensation: clamping/normalization, the transform applied to
 * events (including the remainder carry), v4 persistence and the v3 upgrade. */
static int test_tilt(void) {
    const struct device *dev = zmk_input_processor_runtime_find_by_name(TEST_PROCESSOR_NAME);
    if (!dev) {
        LOG_ERR("Test processor '%s' not found", TEST_PROCESSOR_NAME);
        return -ENODEV;
    }

    struct zmk_input_processor_runtime_config cfg;
    int ret;

    /* Out-of-range requests are clamped / normalized, not rejected. */
    static const struct {
        uint32_t in;
        uint8_t out;
    } tilt_cases[] = {{70, 60}, {45, 45}, {0, 0}};
    for (size_t i = 0; i < ARRAY_SIZE(tilt_cases); i++) {
        ret = zmk_input_processor_runtime_set_tilt(dev, tilt_cases[i].in,
                                                   ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
        if (ret < 0) {
            return ret;
        }
        zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
        if (cfg.tilt_degrees != tilt_cases[i].out) {
            LOG_ERR("Tilt %u: got %u expected %u", tilt_cases[i].in, cfg.tilt_degrees,
                    tilt_cases[i].out);
            return -EINVAL;
        }
    }
    static const struct {
        int32_t in;
        int16_t out;
    } axis_cases[] = {{-90, 90}, {120, -60}, {270, 90}, {-30, -30}};
    for (size_t i = 0; i < ARRAY_SIZE(axis_cases); i++) {
        ret = zmk_input_processor_runtime_set_tilt_axis(
            dev, axis_cases[i].in, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
        if (ret < 0) {
            return ret;
        }
        zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
        if (cfg.tilt_axis_degrees != axis_cases[i].out) {
            LOG_ERR("Tilt axis %d: got %d expected %d", axis_cases[i].in, cfg.tilt_axis_degrees,
                    axis_cases[i].out);
            return -EINVAL;
        }
    }

    /* Isolate the transform from rotation, scaling and inertia. */
    if ((ret = zmk_input_processor_runtime_set_rotation(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_scaling(
             dev, 1, 1, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_enabled(
             dev, false, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0) {
        return ret;
    }

    /* Tilt 0 is the identity: events pass through untouched. */
    int32_t out[3];
    if ((ret = zmk_input_processor_runtime_set_tilt(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_xyx(dev, 3, 5, out)) < 0) {
        return ret;
    }
    if (out[0] != 3 || out[1] != 5 || out[2] != 3) {
        LOG_ERR("Identity transform changed events: %d %d %d", out[0], out[1], out[2]);
        return -EINVAL;
    }

    /* 60 degrees along the 12-6 o'clock axis doubles Y only, immediately. */
    if ((ret = zmk_input_processor_runtime_set_tilt(
             dev, 60, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_tilt_axis(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_xyx(dev, 3, 5, out)) < 0) {
        return ret;
    }
    if (out[0] != 3 || out[1] != 10 || out[2] != 3) {
        LOG_ERR("Tilt 60 axis 0: got %d %d %d expected 3 10 3", out[0], out[1], out[2]);
        return -EINVAL;
    }

    /* Turning the axis to 3-9 o'clock doubles X instead. */
    if ((ret = zmk_input_processor_runtime_set_tilt_axis(
             dev, 90, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_xyx(dev, 3, 5, out)) < 0) {
        return ret;
    }
    if (out[0] != 6 || out[1] != 5 || out[2] != 6) {
        LOG_ERR("Tilt 60 axis 90: got %d %d %d expected 6 5 6", out[0], out[1], out[2]);
        return -EINVAL;
    }

    /* The 1.414 gain at 45 degrees must not be lost to integer truncation,
     * and single-axis reports must not be dropped. */
    if ((ret = zmk_input_processor_runtime_set_tilt(
             dev, 45, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_tilt_axis(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0) {
        return ret;
    }
    int32_t total_x = 0;
    int32_t total_y = 0;
    if ((ret = process_test_single_axis_run(dev, INPUT_REL_X, 3, 100, &total_x)) < 0 ||
        (ret = process_test_single_axis_run(dev, INPUT_REL_Y, 3, 100, &total_y)) < 0) {
        return ret;
    }
    if (total_x != 300 || total_y < 423 || total_y > 425) {
        LOG_ERR("Tilt 45 single-axis runs: got x=%d y=%d expected 300/424", total_x, total_y);
        return -EINVAL;
    }
    total_x = 0;
    total_y = 0;
    for (int i = 0; i < 1000; i++) {
        int32_t x;
        int32_t y;
        if ((ret = process_test_relative(dev, INPUT_REL_X, 0, &x)) < 0 ||
            (ret = process_test_relative(dev, INPUT_REL_Y, 1, &y)) < 0) {
            return ret;
        }
        total_x += x;
        total_y += y;
    }
    if (total_x != 0 || total_y < 1413 || total_y > 1415) {
        LOG_ERR("Tilt 45 carry: got x=%d y=%d expected 0/1414", total_x, total_y);
        return -EINVAL;
    }

    /* Rotation 90 (right to down): Y gets X's cross term in the same report,
     * X gets Y's cross term one report later. */
    if ((ret = zmk_input_processor_runtime_set_tilt(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_rotation(
             dev, 90, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0) {
        return ret;
    }
    total_x = 0;
    total_y = 0;
    for (int i = 0; i < 10; i++) {
        int32_t x;
        int32_t y;
        if ((ret = process_test_relative(dev, INPUT_REL_X, 5, &x)) < 0 ||
            (ret = process_test_relative(dev, INPUT_REL_Y, 3, &y)) < 0) {
            return ret;
        }
        total_x += x;
        total_y += y;
    }
    if (total_x != -27 || total_y != 50) {
        LOG_ERR("Rotation 90 reports: got x=%d y=%d expected -27/50", total_x, total_y);
        return -EINVAL;
    }
    zmk_input_processor_runtime_restore_persistent(dev);

    /* v4 persistence round trip. */
    if ((ret = zmk_input_processor_runtime_set_tilt(
             dev, 45, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST)) < 0 ||
        (ret = zmk_input_processor_runtime_set_tilt_axis(
             dev, -30, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST)) < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_flush_save(dev);
    zmk_input_processor_runtime_reset(dev);
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.tilt_degrees != 0 || cfg.tilt_axis_degrees != 0) {
        LOG_ERR("Reset did not restore tilt defaults: %u@%d", cfg.tilt_degrees,
                cfg.tilt_axis_degrees);
        return -EINVAL;
    }
    ret = settings_load_subtree("custom_settings");
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_apply_persisted_settings();
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.tilt_degrees != 45 || cfg.tilt_axis_degrees != -30) {
        LOG_ERR("Persisted tilt not restored: got %u@%d expected 45@-30", cfg.tilt_degrees,
                cfg.tilt_axis_degrees);
        return -EINVAL;
    }

    /* A memory-only change is dropped by discard. */
    if ((ret = zmk_input_processor_runtime_set_tilt(
             dev, 30, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY)) < 0 ||
        (ret = zmk_input_processor_runtime_discard_all()) < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.tilt_degrees != 45 || cfg.tilt_axis_degrees != -30) {
        LOG_ERR("Tilt discard did not restore flash value: got %u@%d", cfg.tilt_degrees,
                cfg.tilt_axis_degrees);
        return -EINVAL;
    }

    /* A v3 record (no tilt fields) upgrades with the devicetree defaults. */
    ret = zmk_input_processor_runtime_test_apply_legacy_v3(dev);
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.rotation_degrees != 44 || cfg.tilt_degrees != 0 || cfg.tilt_axis_degrees != 0) {
        LOG_ERR("Legacy v3 settings were not migrated: rot=%d tilt=%u@%d", cfg.rotation_degrees,
                cfg.tilt_degrees, cfg.tilt_axis_degrees);
        return -EINVAL;
    }

    LOG_INF("PASS: rip_tilt_compensation");
    return 0;
}

/* Pointer acceleration: clamping, slow input slowed with the remainder
 * carried, fast input unchanged, 100 as the identity, v5 persistence and the
 * v4 upgrade. Uses the devicetree speeds (20 / 150 counts per 100 ms) and the
 * 40 ms default window; no time passes during this SYS_INIT test, so every
 * event shares one timestamp and stays in the window. */
static int test_accel(void) {
    const struct device *dev = zmk_input_processor_runtime_find_by_name(TEST_PROCESSOR_NAME);
    if (!dev) {
        LOG_ERR("Test processor '%s' not found", TEST_PROCESSOR_NAME);
        return -ENODEV;
    }

    struct zmk_input_processor_runtime_config cfg;
    int ret;

    /* Out-of-range requests are clamped, not rejected. */
    static const struct {
        uint32_t in;
        uint8_t out;
    } accel_cases[] = {{10, 20}, {150, 100}, {55, 55}};
    for (size_t i = 0; i < ARRAY_SIZE(accel_cases); i++) {
        ret = zmk_input_processor_runtime_set_accel(dev, accel_cases[i].in,
                                                    ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
        if (ret < 0) {
            return ret;
        }
        zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
        if (cfg.accel_min_percent != accel_cases[i].out) {
            LOG_ERR("Accel %u: got %u expected %u", accel_cases[i].in, cfg.accel_min_percent,
                    accel_cases[i].out);
            return -EINVAL;
        }
    }

    /* Isolate acceleration from rotation, tilt, scaling and inertia. */
    if ((ret = zmk_input_processor_runtime_set_rotation(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_tilt(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_scaling(
             dev, 1, 1, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_enabled(
             dev, false, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0) {
        return ret;
    }

    /* 100 is off: events pass through untouched. */
    int32_t x;
    int32_t y;
    if ((ret = zmk_input_processor_runtime_set_accel(
             dev, 100, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_X, 3, &x)) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_Y, -5, &y)) < 0) {
        return ret;
    }
    if (x != 3 || y != -5) {
        LOG_ERR("Accel off changed events: %d %d expected 3 -5", x, y);
        return -EINVAL;
    }

    /* 50%: eight slow 1-count steps (at most 20 counts per 100 ms) give 4;
     * the carried remainder slows them down without losing any. Setting the
     * value again clears the speed history, so Y starts slow too. */
    int32_t total_x = 0;
    int32_t total_y = 0;
    if ((ret = zmk_input_processor_runtime_set_accel(
             dev, 50, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_single_axis_run(dev, INPUT_REL_X, 1, 8, &total_x)) < 0 ||
        (ret = zmk_input_processor_runtime_set_accel(
             dev, 50, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_single_axis_run(dev, INPUT_REL_Y, -1, 8, &total_y)) < 0) {
        return ret;
    }
    if (total_x != 4 || total_y != -4) {
        LOG_ERR("Accel 50%% slow steps: got x=%d y=%d expected 4/-4", total_x, total_y);
        return -EINVAL;
    }

    /* A fast report (100 counts in 40 ms, 250 per 100 ms) keeps gain 1. */
    if ((ret = zmk_input_processor_runtime_set_accel(
             dev, 50, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_X, 100, &x)) < 0) {
        return ret;
    }
    if (x != 100) {
        LOG_ERR("Accel fast report: got %d expected 100", x);
        return -EINVAL;
    }
    zmk_input_processor_runtime_restore_persistent(dev);

    /* v5 persistence round trip. */
    ret = zmk_input_processor_runtime_set_accel(dev, 60,
                                                ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_flush_save(dev);
    zmk_input_processor_runtime_reset(dev);
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.accel_min_percent != 100) {
        LOG_ERR("Reset did not restore the accel default: %u", cfg.accel_min_percent);
        return -EINVAL;
    }
    ret = settings_load_subtree("custom_settings");
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_apply_persisted_settings();
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.accel_min_percent != 60) {
        LOG_ERR("Persisted accel not restored: got %u expected 60", cfg.accel_min_percent);
        return -EINVAL;
    }

    /* A memory-only change is dropped by discard. */
    if ((ret = zmk_input_processor_runtime_set_accel(
             dev, 30, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY)) < 0 ||
        (ret = zmk_input_processor_runtime_discard_all()) < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.accel_min_percent != 60) {
        LOG_ERR("Accel discard did not restore flash value: got %u", cfg.accel_min_percent);
        return -EINVAL;
    }

    /* A v4 record (no acceleration field) upgrades with the devicetree default. */
    ret = zmk_input_processor_runtime_test_apply_legacy_v4(dev);
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.rotation_degrees != 45 || cfg.accel_min_percent != 100) {
        LOG_ERR("Legacy v4 settings were not migrated: rot=%d accel=%u", cfg.rotation_degrees,
                cfg.accel_min_percent);
        return -EINVAL;
    }

    LOG_INF("PASS: rip_pointer_accel");
    return 0;
}

static int check_aim_config(const struct device *dev, uint8_t scale, uint8_t smoothing,
                            const char *what) {
    struct zmk_input_processor_runtime_config cfg;
    int ret = zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (ret < 0) {
        return ret;
    }
    if (cfg.aim_scale_percent != scale || cfg.aim_smoothing_ms != smoothing) {
        LOG_ERR("%s: aim %u%%/%u ms expected %u%%/%u ms", what, cfg.aim_scale_percent,
                cfg.aim_smoothing_ms, scale, smoothing);
        return -EINVAL;
    }
    return 0;
}

/* Aim mode settings: clamping, smoothing 0 as no smoothing (only the
 * slowdown), write modes, v6 persistence and the v5 upgrade. test.dtsi sets
 * no aim properties, so the devicetree defaults are 40 % and 50 ms. No time
 * passes during this SYS_INIT test, so the filter releases nothing on its
 * own. */
static int test_aim(void) {
    const struct device *dev = zmk_input_processor_runtime_find_by_name(TEST_PROCESSOR_NAME);
    if (!dev) {
        LOG_ERR("Test processor '%s' not found", TEST_PROCESSOR_NAME);
        return -ENODEV;
    }

    struct zmk_input_processor_runtime_config cfg;
    int ret;

    if ((ret = check_aim_config(dev, 40, 50, "Aim default")) < 0) {
        return ret;
    }

    /* Out-of-range requests are clamped, not rejected. */
    static const struct {
        uint32_t scale_in;
        uint32_t smoothing_in;
        uint8_t scale_out;
        uint8_t smoothing_out;
    } aim_cases[] = {{5, 300, 10, 200}, {150, 0, 100, 0}, {55, 120, 55, 120}};
    for (size_t i = 0; i < ARRAY_SIZE(aim_cases); i++) {
        ret = zmk_input_processor_runtime_set_aim(dev, aim_cases[i].scale_in,
                                                  aim_cases[i].smoothing_in,
                                                  ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY);
        if (ret < 0) {
            return ret;
        }
        if ((ret = check_aim_config(dev, aim_cases[i].scale_out, aim_cases[i].smoothing_out,
                                    "Aim clamp")) < 0) {
            return ret;
        }
    }

    /* A temporary change leaves the saved value alone. */
    if ((ret = zmk_input_processor_runtime_set_aim(
             dev, 30, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = check_aim_config(dev, 55, 120, "Aim temporary")) < 0) {
        return ret;
    }

    /* Isolate aim from rotation, tilt, scaling, acceleration and inertia. */
    if ((ret = zmk_input_processor_runtime_set_rotation(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_tilt(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_scaling(
             dev, 1, 1, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_accel(
             dev, 100, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_enabled(
             dev, false, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0) {
        return ret;
    }

    /* 50 % with smoothing 0: every event comes out at once at half speed,
     * the sub-count rest carried (10, 3, 3 -> 5, 1, 2). */
    int32_t out[4];
    if ((ret = zmk_input_processor_runtime_set_aim(
             dev, 50, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0) {
        return ret;
    }
    zmk_input_processor_runtime_aim(dev, true);
    if ((ret = process_test_relative(dev, INPUT_REL_X, 10, &out[0])) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_X, 3, &out[1])) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_X, 3, &out[2])) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_Y, -4, &out[3])) < 0) {
        zmk_input_processor_runtime_aim(dev, false);
        return ret;
    }
    if (out[0] != 5 || out[1] != 1 || out[2] != 2 || out[3] != -2) {
        LOG_ERR("Aim 50%% unsmoothed: got %d %d %d %d expected 5 1 2 -2", out[0], out[1], out[2],
                out[3]);
        zmk_input_processor_runtime_aim(dev, false);
        return -EINVAL;
    }

    /* With smoothing, motion from rest is held back at first; turning the
     * smoothing off releases it with the next event. */
    if ((ret = zmk_input_processor_runtime_set_aim(
             dev, 50, 50, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_X, 10, &out[0])) < 0 ||
        (ret = zmk_input_processor_runtime_set_aim(
             dev, 50, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_X, 0, &out[1])) < 0) {
        zmk_input_processor_runtime_aim(dev, false);
        return ret;
    }
    zmk_input_processor_runtime_aim(dev, false);
    if (out[0] != 0 || out[1] != 5) {
        LOG_ERR("Aim smoothing switch: got %d then %d expected 0 then 5", out[0], out[1]);
        return -EINVAL;
    }
    zmk_input_processor_runtime_restore_persistent(dev);

    /* v6 persistence round trip; reset restores the devicetree defaults. */
    ret = zmk_input_processor_runtime_set_aim(dev, 70, 120,
                                              ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_flush_save(dev);
    zmk_input_processor_runtime_reset(dev);
    if ((ret = check_aim_config(dev, 40, 50, "Aim reset")) < 0) {
        return ret;
    }
    ret = settings_load_subtree("custom_settings");
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_apply_persisted_settings();
    if ((ret = check_aim_config(dev, 70, 120, "Aim persisted")) < 0) {
        return ret;
    }

    /* A memory-only change is dropped by discard. */
    if ((ret = zmk_input_processor_runtime_set_aim(
             dev, 20, 10, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY)) < 0 ||
        (ret = zmk_input_processor_runtime_discard_all()) < 0 ||
        (ret = check_aim_config(dev, 70, 120, "Aim discard")) < 0) {
        return ret;
    }

    /* A v5 record (no aim fields) keeps its own values and takes the
     * devicetree aim defaults. */
    if ((ret = zmk_input_processor_runtime_set_accel(
             dev, 70, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY)) < 0 ||
        (ret = zmk_input_processor_runtime_test_apply_legacy_v5(dev)) < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.rotation_degrees != 46 || cfg.accel_min_percent != 70 || cfg.aim_scale_percent != 40 ||
        cfg.aim_smoothing_ms != 50) {
        LOG_ERR("Legacy v5 settings were not migrated: rot=%d accel=%u aim=%u%%/%u ms",
                cfg.rotation_degrees, cfg.accel_min_percent, cfg.aim_scale_percent,
                cfg.aim_smoothing_ms);
        return -EINVAL;
    }

    LOG_INF("PASS: rip_aim_settings");
    return 0;
}

static int check_smoothing_config(const struct device *dev, uint8_t smoothing, const char *what) {
    struct zmk_input_processor_runtime_config cfg;
    int ret = zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (ret < 0) {
        return ret;
    }
    if (cfg.smoothing_ms != smoothing) {
        LOG_ERR("%s: smoothing %u ms expected %u ms", what, cfg.smoothing_ms, smoothing);
        return -EINVAL;
    }
    return 0;
}

/* Normal smoothing: clamping, 0 as passthrough, what is held back carried
 * into aim mode, write modes, v7 persistence and the v6 upgrade. test.dtsi
 * sets no smoothing-ms, so the devicetree default is 0. No time passes during
 * this SYS_INIT test, so the filter releases nothing on its own. */
static int test_smoothing(void) {
    const struct device *dev = zmk_input_processor_runtime_find_by_name(TEST_PROCESSOR_NAME);
    if (!dev) {
        LOG_ERR("Test processor '%s' not found", TEST_PROCESSOR_NAME);
        return -ENODEV;
    }

    struct zmk_input_processor_runtime_config cfg;
    int ret;

    if ((ret = check_smoothing_config(dev, 0, "Smoothing default")) < 0) {
        return ret;
    }

    /* Out-of-range requests are clamped, not rejected. */
    static const struct {
        uint32_t in;
        uint8_t out;
    } smoothing_cases[] = {{300, 200}, {0, 0}, {80, 80}};
    for (size_t i = 0; i < ARRAY_SIZE(smoothing_cases); i++) {
        if ((ret = zmk_input_processor_runtime_set_smoothing(
                 dev, smoothing_cases[i].in, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY)) < 0 ||
            (ret = check_smoothing_config(dev, smoothing_cases[i].out, "Smoothing clamp")) < 0) {
            return ret;
        }
    }

    /* A temporary change leaves the saved value alone. */
    if ((ret = zmk_input_processor_runtime_set_smoothing(
             dev, 10, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = check_smoothing_config(dev, 80, "Smoothing temporary")) < 0) {
        return ret;
    }

    /* Isolate the filter from rotation, tilt, scaling, acceleration and inertia. */
    if ((ret = zmk_input_processor_runtime_set_rotation(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_tilt(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_scaling(
             dev, 1, 1, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_accel(
             dev, 100, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_enabled(
             dev, false, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0) {
        return ret;
    }

    /* 0 passes events untouched. */
    int32_t out[3];
    if ((ret = zmk_input_processor_runtime_set_smoothing(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_X, 7, &out[0])) < 0) {
        return ret;
    }
    if (out[0] != 7) {
        LOG_ERR("Smoothing 0 changed an event: 7 -> %d", out[0]);
        return -EINVAL;
    }

    /* With smoothing, motion from rest is held back at full speed; pressing
     * aim (here unsmoothed at 100 %) lets it out with the next event, once. */
    if ((ret = zmk_input_processor_runtime_set_smoothing(
             dev, 50, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_aim(
             dev, 100, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_X, 10, &out[0])) < 0) {
        return ret;
    }
    zmk_input_processor_runtime_aim(dev, true);
    if ((ret = process_test_relative(dev, INPUT_REL_X, 0, &out[1])) < 0 ||
        (ret = process_test_relative(dev, INPUT_REL_X, 0, &out[2])) < 0) {
        zmk_input_processor_runtime_aim(dev, false);
        return ret;
    }
    zmk_input_processor_runtime_aim(dev, false);
    if (out[0] != 0 || out[1] != 10 || out[2] != 0) {
        LOG_ERR("Smoothing into aim: got %d, %d, %d expected 0, 10, 0", out[0], out[1], out[2]);
        return -EINVAL;
    }
    zmk_input_processor_runtime_restore_persistent(dev);

    /* v7 persistence round trip; reset restores the devicetree default. */
    ret = zmk_input_processor_runtime_set_smoothing(dev, 120,
                                                    ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_PERSIST);
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_flush_save(dev);
    zmk_input_processor_runtime_reset(dev);
    if ((ret = check_smoothing_config(dev, 0, "Smoothing reset")) < 0) {
        return ret;
    }
    ret = settings_load_subtree("custom_settings");
    if (ret < 0) {
        return ret;
    }
    zmk_input_processor_runtime_test_apply_persisted_settings();
    if ((ret = check_smoothing_config(dev, 120, "Smoothing persisted")) < 0) {
        return ret;
    }

    /* A memory-only change is dropped by discard. */
    if ((ret = zmk_input_processor_runtime_set_smoothing(
             dev, 30, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY)) < 0 ||
        (ret = zmk_input_processor_runtime_discard_all()) < 0 ||
        (ret = check_smoothing_config(dev, 120, "Smoothing discard")) < 0) {
        return ret;
    }

    /* A v6 record (no normal smoothing) keeps its own values and takes the
     * devicetree default. */
    if ((ret = zmk_input_processor_runtime_set_aim(
             dev, 55, 70, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_MEMORY)) < 0 ||
        (ret = zmk_input_processor_runtime_test_apply_legacy_v6(dev)) < 0) {
        return ret;
    }
    zmk_input_processor_runtime_get_config(dev, NULL, &cfg);
    if (cfg.rotation_degrees != 47 || cfg.aim_scale_percent != 55 || cfg.aim_smoothing_ms != 70 ||
        cfg.smoothing_ms != 0) {
        LOG_ERR("Legacy v6 settings were not migrated: rot=%d aim=%u%%/%u ms smoothing=%u ms",
                cfg.rotation_degrees, cfg.aim_scale_percent, cfg.aim_smoothing_ms,
                cfg.smoothing_ms);
        return -EINVAL;
    }

    LOG_INF("PASS: rip_normal_smoothing");
    return 0;
}

/* Per-key aim speed: 0 uses the aim scale, others are clamped to 10..100,
 * and a change while aiming only scales input from then on. */
static int test_aim_key_scale(void) {
    const struct device *dev = zmk_input_processor_runtime_find_by_name(TEST_PROCESSOR_NAME);
    if (!dev) {
        LOG_ERR("Test processor '%s' not found", TEST_PROCESSOR_NAME);
        return -ENODEV;
    }

    int ret;
    /* Isolate the filter; aim at 50 % without smoothing, normal smoothing off. */
    if ((ret = zmk_input_processor_runtime_set_rotation(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_tilt(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_scaling(
             dev, 1, 1, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_accel(
             dev, 100, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_inertia_enabled(
             dev, false, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_smoothing(
             dev, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0 ||
        (ret = zmk_input_processor_runtime_set_aim(
             dev, 50, 0, ZMK_INPUT_PROCESSOR_RUNTIME_WRITE_MODE_TEMPORARY)) < 0) {
        return ret;
    }

    static const struct {
        uint32_t key;
        int32_t out;
    } cases[] = {{0, 5}, {20, 2}, {5, 1}, {250, 10}, {0, 5}};
    zmk_input_processor_runtime_aim(dev, true);
    for (size_t i = 0; i < ARRAY_SIZE(cases); i++) {
        int32_t out;
        zmk_input_processor_runtime_set_aim_key_scale(dev, cases[i].key);
        if ((ret = process_test_relative(dev, INPUT_REL_X, 10, &out)) < 0) {
            break;
        }
        if (out != cases[i].out) {
            LOG_ERR("Aim key speed %u: 10 -> %d expected %d", cases[i].key, out, cases[i].out);
            ret = -EINVAL;
            break;
        }
    }
    zmk_input_processor_runtime_set_aim_key_scale(dev, 0);
    zmk_input_processor_runtime_aim(dev, false);
    zmk_input_processor_runtime_restore_persistent(dev);
    if (ret < 0) {
        return ret;
    }

    LOG_INF("PASS: rip_aim_key_scale");
    return 0;
}

static int rip_settings_test_init(void) {
    int ret = test_settings_backend_init();
    if (ret < 0) {
        LOG_ERR("FAIL: rip_settings_persist_reload backend_init ret=%d", ret);
        return 0; // Never fail boot because of a test failure.
    }

    ret = test_scaling_persists_across_reload();
    if (ret < 0) {
        LOG_ERR("FAIL: rip_settings_persist_reload ret=%d", ret);
    }

    ret = test_inertia();
    if (ret < 0) {
        LOG_ERR("FAIL: rip_inertia_decay_reverse_scroll ret=%d", ret);
    }

    ret = test_write_modes_and_save_discard();
    if (ret < 0) {
        LOG_ERR("FAIL: rip_settings_write_modes ret=%d", ret);
    }

    ret = test_tilt();
    if (ret < 0) {
        LOG_ERR("FAIL: rip_tilt_compensation ret=%d", ret);
    }

    ret = test_accel();
    if (ret < 0) {
        LOG_ERR("FAIL: rip_pointer_accel ret=%d", ret);
    }

    ret = test_aim();
    if (ret < 0) {
        LOG_ERR("FAIL: rip_aim_settings ret=%d", ret);
    }

    ret = test_smoothing();
    if (ret < 0) {
        LOG_ERR("FAIL: rip_normal_smoothing ret=%d", ret);
    }

    ret = test_aim_key_scale();
    if (ret < 0) {
        LOG_ERR("FAIL: rip_aim_key_scale ret=%d", ret);
    }
    return 0;
}
SYS_INIT(rip_settings_test_init, APPLICATION, 99);
