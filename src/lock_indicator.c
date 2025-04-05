/*
 * Copyright (c) 2024 TBog.rocks
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT tbog_lock_indicator

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
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
    uint8_t status;
};

#define LI_STRUCT(n)                                                                          \
    static struct lock_indicator_config lock_indicator_config_##n = {                         \
        .led_gpio = GPIO_DT_SPEC_INST_GET(n, gpios),                                          \
        .indicator_mask = DT_INST_PROP_OR(n, indicator_mask, HID_KBD_LED_CAPS_LOCK),          \
        .status = 0,                                                                          \
    };

DT_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, LI_STRUCT)

#define LOCK_INDICATOR_DATA_REF_AND_COMMA(n) \
    &lock_indicator_config_##n,

static struct lock_indicator_config *lock_indicator_instance[] = {
    DT_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, LOCK_INDICATOR_DATA_REF_AND_COMMA)
};

static zmk_hid_indicators_t zmk_event_indicator_previous = {0};

#define LOCK_INDICATOR_INSTANCE_COUNT ARRAY_SIZE(lock_indicator_instance)

static int lock_indicator_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);
    const zmk_hid_indicators_t indicators = ev->indicators;
    const size_t num_indicators = LOCK_INDICATOR_INSTANCE_COUNT;

    LOG_INF("lock_indicator_listener indicators=%#04X count=%d", indicators, num_indicators);
    // Iterate over all instances
    for (size_t i = 0; i < num_indicators; i+=1) {
        const struct lock_indicator_config *data = lock_indicator_instance[i];
        if (data->status != -1) {
            LOG_WRN("Lock Indicator #%d status = %d", i, data->status);
            continue;
        }
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
    size_t count = 0;
    const size_t num_indicators = LOCK_INDICATOR_INSTANCE_COUNT;
    // Iterate over all instances
    for (size_t i = 0; i < num_indicators; i+=1) {
        const struct lock_indicator_config *data = lock_indicator_instance[i];
    
        if (!gpio_is_ready_dt(&data->led_gpio)) {
            if (!data->led_gpio.port) {
                LOG_WRN("Lock Indicator #%d GPIO not set correctly", i);
                continue;
            }

            const struct device *gpio_dev = device_get_binding(data->led_gpio.port->name);
            if (!gpio_dev || !device_is_ready(gpio_dev)) {
                LOG_WRN("Lock Indicator #%d GPIO device '%s' not ready", i, data->led_gpio.port->name);
                continue;
            }

            LOG_WRN("Lock Indicator #%d GPIO pin %d not ready", i, data->led_gpio.pin);
            //continue;//return -ENODEV;
        }
    
        int ret = gpio_pin_configure_dt(&data->led_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret < 0) {
            LOG_WRN("Failed to configure Lock Indicator #%d GPIO: %d", i, ret);
            continue;//return ret;
        }
        // ensure LED is off
        gpio_pin_set_dt(&data->led_gpio, 0);
        LOG_DBG("Lock Indicator #%d initialized and turned off", i);
        data->status = -1;
        count += 1;
    }
    LOG_INF("lock-indicator initialized %d/%d", count, num_indicators);
    return 0;
}

ZMK_LISTENER(lock_indicator, lock_indicator_listener);
ZMK_SUBSCRIPTION(lock_indicator, zmk_hid_indicators_changed);
SYS_INIT(sys_lock_indicator_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

//#endif // DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)