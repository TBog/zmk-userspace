/*
 * Copyright (c) 2024 TBog.rocks
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT tbog_lock_indicator

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

#include <zmk/event_manager.h>
#include <zmk/events/hid_indicators_changed.h>
#include <zmk/hid.h>
#include <zmk/hid_indicators.h>

//#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct lock_indicator_config {
    const struct gpio_dt_spec led_gpio;
    uint8_t indicator_mask;
};

#define LI_STRUCT(inst)                                                                          \
    static struct lock_indicator_config lock_indicator_config_##inst = {                         \
        .led_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, gpios, {0}),                                  \
        .indicator_mask = DT_INST_PROP_OR(inst, indicator_mask, HID_KBD_LED_CAPS_LOCK),          \
    };

DT_INST_FOREACH_CHILD(0, LI_STRUCT)

#define LOCK_INDICATOR_DATA_REF_AND_COMMA(n) \
    &lock_indicator_config_##n,

static struct lock_indicator_config *lock_indicator_instances[] = {
    DT_INST_FOREACH_CHILD(0, LOCK_INDICATOR_DATA_REF_AND_COMMA)
};

static zmk_hid_indicators_t zmk_event_indicator_previous = {0};

#define LOCK_INDICATOR_INSTANCE_COUNT ARRAY_SIZE(lock_indicator_instances)

static int lock_indicator_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    const zmk_hid_indicators_t indicators = ev->indicators;
    const size_t num_indicators = LOCK_INDICATOR_INSTANCE_COUNT;

    LOG_INF("lock_indicator_listener indicators=%#04X count=%d", indicators, num_indicators);
    // Iterate over all instances
    for (size_t i = 0; i < num_indicators; i+=1) {
        const struct lock_indicator_config *data = lock_indicator_instances[i];
        const bool old_led_state = (zmk_event_indicator_previous & data->indicator_mask) != 0;
        const bool new_led_state = (indicators & data->indicator_mask) != 0;
        if (old_led_state != new_led_state) {
            LOG_INF("lock-indicator #%d set led (pin %d) state=%s", i, data->led_gpio.pin, new_led_state ? "ON" : "OFF");
            gpio_pin_set_dt(&data->led_gpio, new_led_state);
        } else {
            LOG_INF("lock-indicator #%d not changed", i);
        }
    }

    zmk_event_indicator_previous = indicators;

    return ZMK_EV_EVENT_BUBBLE;
}

static int sys_lock_indicator_init() {
    // Iterate over all instances
    for (size_t i = 0; i < LOCK_INDICATOR_INSTANCE_COUNT; i+=1) {
        const struct lock_indicator_config *data = lock_indicator_instances[i];
    
        if (!device_is_ready(data->led_gpio.port)) {
            LOG_WRN("Lock Indicator GPIO device not ready\n");
            continue;//return -ENODEV;
        }
    
        int ret = gpio_pin_configure_dt(&data->led_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure Lock Indicator GPIO: %d\n", ret);
            continue;//return ret;
        }
        gpio_pin_set_dt(&data->led_gpio, 0);
    }
    return 0;
}

ZMK_LISTENER(lock_indicator, lock_indicator_listener);
ZMK_SUBSCRIPTION(lock_indicator, zmk_hid_indicators_changed);
SYS_INIT(sys_lock_indicator_init, POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY);

//#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)