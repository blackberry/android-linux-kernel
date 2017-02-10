/*
 * LED Core
 *
 * Copyright 2005 Openedhand Ltd.
 * Copyright (C) 2016 BlackBerry Limited
 *
 * Author: Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#ifndef __LEDS_H_INCLUDED
#define __LEDS_H_INCLUDED

#include <linux/device.h>
#include <linux/rwsem.h>
#include <linux/leds.h>

static inline void __led_set_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
#ifdef CONFIG_BBRY
	if (!led_cdev->manual_override_en) {
		int limit = (led_cdev->brightness_limit * led_cdev->max_brightness + 127) / 255;
		if (value > limit) {
			pr_info("clamping %s to brightness_limit %d\n", led_cdev->name, limit);
			value = limit;
		}
	} else
#endif
	if (value > led_cdev->max_brightness)
		value = led_cdev->max_brightness;

#ifdef CONFIG_BBRY
	if ((led_cdev->flags & LED_LOG_FIRST_BRIGHTNESS) &&
		led_cdev->brightness == 0 && value > 0)
		pr_info("%s 1st brightness %d\n",  led_cdev->name, value);
#endif
	led_cdev->brightness = value;
	if (!(led_cdev->flags & LED_SUSPENDED))
		led_cdev->brightness_set(led_cdev, value);
}

static inline int led_get_brightness(struct led_classdev *led_cdev)
{
	return led_cdev->brightness;
}

void led_stop_software_blink(struct led_classdev *led_cdev);

extern struct rw_semaphore leds_list_lock;
extern struct list_head leds_list;

#ifdef CONFIG_LEDS_TRIGGERS
void led_trigger_set_default(struct led_classdev *led_cdev);
void led_trigger_set(struct led_classdev *led_cdev,
			struct led_trigger *trigger);
void led_trigger_remove(struct led_classdev *led_cdev);

static inline void *led_get_trigger_data(struct led_classdev *led_cdev)
{
	return led_cdev->trigger_data;
}

#else
#define led_trigger_set_default(x) do {} while (0)
#define led_trigger_set(x, y) do {} while (0)
#define led_trigger_remove(x) do {} while (0)
#define led_get_trigger_data(x) (NULL)
#endif

ssize_t led_trigger_store(struct device *dev, struct device_attribute *attr,
			const char *buf, size_t count);
ssize_t led_trigger_show(struct device *dev, struct device_attribute *attr,
			char *buf);

#endif	/* __LEDS_H_INCLUDED */
