/*
 * Driver for hall sensors GPIO pins to generate holster/keypad slider event
 *
 * Copyright 2015 BlackBerry Limited
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/device.h>

enum hall_state_e {
	STATE_UNKNOWN = -1,
	STATE_SLIDER_CLOSED,
	STATE_SLIDER_TRANSITION,
	STATE_SLIDER_OPEN,
	STATE_SMART_FLIP_CLOSED,
	STATE_HOLSTER_CLOSED,
	NUM_OF_STATES
};

enum slider_state_e {
	SLIDER_UNKNOWN = -1,
	SLIDER_CLOSED,
	SLIDER_OPEN,
	SLIDER_TRANSITION
};


enum holster_enum_e {
	HOLSTER_UNKNOWN = -1,
	HOLSTER_OPEN,
	HOLSTER_CLOSED
};

enum flip_enum_e {
	FLIP_UNKNOWN = -1,
	FLIP_OPEN,
	FLIP_CLOSED
};

/*
    Truth Table:
                   open    mid    close
    Open(Full KB)   0       1       1
    Mid(ASDF Row)   1       1       1
    Mid(ZXCV Row)   1       0       1
    Mid(Space Row)  1       1       1
    Closed(Vir. KB) 1       1       0
    Closed(Flip)    1       0       0
    Closed(Holster) 0       1       0
    Invalid         0       0       0
    Invlaid         0       0       1
*/

static unsigned int state_counter[NUM_OF_STATES];

#define SLIDER_OPEN_EVENT(open, mid, closed)		(open == 0 && closed == 1)
#define SLIDER_TRANSITION_EVENT(open, mid, closed)	(open == 1 && closed == 1)
#define SLIDER_CLOSE_EVENT(open, mid, closed) 		(open == 1 && closed == 0)
#define SMART_FLIP_CLOSE_EVENT(open, mid, closed)	(open == 1 && mid == 0 && closed == 0)
#define SMART_FLIP_OPEN_EVENT(open, mid, closed)	SLIDER_CLOSE_EVENT(open, mid, closed)
#define HOLSTER_CLOSE_EVENT(open, mid, closed)		(open == 0 && closed == 0)
#define HOLSTER_OPEN_EVENT(open, mid, closed)		SLIDER_CLOSE_EVENT(open, mid, closed)
#define STATE_CHANGE(state)				do { \
								if(ddata->hall_state == state) break; \
								ddata->hall_state = state; \
								state_counter[state]++; \
								pr_info("hall_state_counter: closed composite %u transition %u open %u flip_closed %u holster %u\n", \
									state_counter[STATE_SLIDER_CLOSED], state_counter[STATE_SLIDER_TRANSITION], \
									state_counter[STATE_SLIDER_OPEN], state_counter[STATE_SMART_FLIP_CLOSED], \
									state_counter[STATE_HOLSTER_CLOSED]); \
							} while(0)

struct hall_button {
	/* Configuration parameters */
	unsigned int code;	/* input event code (KEY_*, SW_*) */
	int gpio;		/* -1 if this key does not support gpio */
	int active_low;
	const char *desc;
	unsigned int type;	/* input event type (EV_KEY, EV_SW, EV_ABS) */
	int wakeup;		/* configure the button as a wake-up source */
	int debounce_interval;	/* debounce ticks interval in msecs */
	bool can_disable;
	int value;		/* axis value for EV_ABS */
	unsigned int irq;	/* Irq number in case of interrupt keys */
};

struct hall_platform_data {
	struct hall_button *buttons;
	int nbuttons;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	const char *name;		/* input device name */
	unsigned int sf_debounce;	/* Smart Flip debounce in msecs */
};

struct gpio_button_data {
	const struct hall_button *button;
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	unsigned int timer_debounce;	/* in msecs */
	unsigned int irq;
};

struct hall_drvdata {
	const struct hall_platform_data *pdata;
	struct input_dev *input;
	int open_gpio;
	int closed_gpio;
	int mid_gpio;
	enum slider_state_e slider_state;
	enum holster_enum_e holster_state;
	enum flip_enum_e flip_state;
	enum hall_state_e hall_state;
	struct delayed_work sf_work;	/* Smart Flip work func */
	struct gpio_button_data data[0];
};

static void hall_set_cur_state(struct input_dev *input, int code, int input_value)
{
	input_event(input, EV_SW, code, input_value);
	input_sync(input);
}

static void hall_gpio_report_event(struct hall_drvdata *ddata)
{
	struct input_dev *input = ddata->input;
	const struct hall_platform_data *pdata = ddata->pdata;
	int closed_state;
	int open_state;
	int mid_state;

	closed_state = (gpio_get_value_cansleep(ddata->closed_gpio) ? 1 : 0);
	open_state = (gpio_get_value_cansleep(ddata->open_gpio) ? 1 : 0);
	mid_state = (gpio_get_value_cansleep(ddata->mid_gpio) ? 1 : 0);

	switch(ddata->hall_state)
	{
		case STATE_SLIDER_OPEN:
			if(SLIDER_TRANSITION_EVENT(open_state, mid_state, closed_state)) {
				/* The slider is open and we got a slider transition event */
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 1);
				STATE_CHANGE(STATE_SLIDER_TRANSITION);
				if(pdata->sf_debounce > 0)
					schedule_delayed_work(&ddata->sf_work, msecs_to_jiffies(pdata->sf_debounce));
			} else if(SLIDER_CLOSE_EVENT(open_state, mid_state, closed_state)) {
				/* The slider is open and we got a slider close event */
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 1);
				hall_set_cur_state(input, SW_KEYPAD_SLIDE, SLIDER_CLOSED);
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 0);
				STATE_CHANGE(STATE_SLIDER_CLOSED);
			}
			break;
		case STATE_SLIDER_TRANSITION:
			if(SLIDER_OPEN_EVENT(open_state, mid_state, closed_state)) {
				/* The slider is in transition and we got a slider open event */
				hall_set_cur_state(input, SW_KEYPAD_SLIDE, SLIDER_OPEN);
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 0);
				STATE_CHANGE(STATE_SLIDER_OPEN);
			} else if(SMART_FLIP_CLOSE_EVENT(open_state, mid_state, closed_state)) {
				/* The slider is in transition and we got a smart flip event */
				/* This occurs in the following scenario: */
				/*  1. Flip case was open. Slider was open. (011) */
				/*  2. Flip case was closed. (001) */
				/*  3. Slider began transitioning to closed. (101) */
				/*  4. Slider finished closing (100) */
				/* The end state is that both the slider and flip case are closed. */
				hall_set_cur_state(input, SW_KEYPAD_SLIDE, SLIDER_CLOSED);
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 0);
				hall_set_cur_state(input, SW_SMART_FLIP, FLIP_CLOSED);
				STATE_CHANGE(STATE_SMART_FLIP_CLOSED);
				ddata->flip_state = FLIP_CLOSED;
			} else if(SLIDER_CLOSE_EVENT(open_state, mid_state, closed_state)) {
				/* The slider is in transition and we got a slider close event */
				hall_set_cur_state(input, SW_KEYPAD_SLIDE, SLIDER_CLOSED);
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 0);
				STATE_CHANGE(STATE_SLIDER_CLOSED);
			}
			break;
		case STATE_SLIDER_CLOSED:
			if(SLIDER_OPEN_EVENT(open_state, mid_state, closed_state)) {
				/* The slider is closed and we got a slider open event */
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 1);
				hall_set_cur_state(input, SW_KEYPAD_SLIDE, SLIDER_OPEN);
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 0);
				STATE_CHANGE(STATE_SLIDER_OPEN);
			} else if(SLIDER_TRANSITION_EVENT(open_state, mid_state, closed_state)) {
				/* The slider is closed and we got a slider transition event */
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 1);
				STATE_CHANGE(STATE_SLIDER_TRANSITION);
				if(pdata->sf_debounce > 0)
					schedule_delayed_work(&ddata->sf_work, msecs_to_jiffies(pdata->sf_debounce));
			} else if(HOLSTER_CLOSE_EVENT(open_state, mid_state, closed_state) && ddata->holster_state == HOLSTER_OPEN) {
				/* The slider is closed and we got a holster close event */
				if(ddata->flip_state == FLIP_CLOSED) {
					hall_set_cur_state(input, SW_SMART_FLIP, FLIP_OPEN);
					ddata->flip_state = FLIP_OPEN;
					msleep(pdata->sf_debounce);
				}
				hall_set_cur_state(input, SW_LID, HOLSTER_CLOSED);
				STATE_CHANGE(STATE_HOLSTER_CLOSED);
				ddata->holster_state = HOLSTER_CLOSED;
			} else if(SMART_FLIP_CLOSE_EVENT(open_state, mid_state, closed_state)) {
				/* The slider is closed and we got a smart flip event */
				/* We can delay handling this event since it could be */
				/* a transition event or a holster */
				if(pdata->sf_debounce > 0)
					schedule_delayed_work(&ddata->sf_work, msecs_to_jiffies(pdata->sf_debounce));
				else {
					hall_set_cur_state(input, SW_SMART_FLIP, FLIP_CLOSED);
					ddata->flip_state = FLIP_CLOSED;
				}
				STATE_CHANGE(STATE_SMART_FLIP_CLOSED);
			}
			break;
		case STATE_HOLSTER_CLOSED:
			if(HOLSTER_OPEN_EVENT(open_state, mid_state, closed_state) && ddata->holster_state == HOLSTER_CLOSED) {
				/* The holster is closed and we received a holster open event */
				if(ddata->flip_state == FLIP_CLOSED) {
					hall_set_cur_state(input, SW_SMART_FLIP, FLIP_OPEN);
					ddata->flip_state = FLIP_OPEN;
					msleep(pdata->sf_debounce);
				}
				hall_set_cur_state(input, SW_LID, HOLSTER_OPEN);
				STATE_CHANGE(STATE_SLIDER_CLOSED);
				ddata->holster_state = HOLSTER_OPEN;
			}
			break;
		case STATE_SMART_FLIP_CLOSED:
			if(SLIDER_TRANSITION_EVENT(open_state, mid_state, closed_state)) {
				/* The smart flip is closed and we received a keypad transition event */
				/* This could be because we sent a flip closed event erraneously before */
				/* and got into this state. So we send a flip open event first */
				if(ddata->flip_state == FLIP_CLOSED) {
					hall_set_cur_state(input, SW_SMART_FLIP, FLIP_OPEN);
					ddata->flip_state = FLIP_OPEN;
					msleep(pdata->sf_debounce);
				}
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 1);
				STATE_CHANGE(STATE_SLIDER_TRANSITION);
			} else if(HOLSTER_CLOSE_EVENT(open_state, mid_state, closed_state) && ddata->holster_state == HOLSTER_OPEN) {
				/* The smart flip is closed and we received a holster event */
				/* This could be because we sent a flip closed event erraneously before */
				/* and got into this state. So we send a flip open event first */
				if(ddata->flip_state == FLIP_CLOSED) {
					hall_set_cur_state(input, SW_SMART_FLIP, FLIP_OPEN);
					ddata->flip_state = FLIP_OPEN;
					msleep(pdata->sf_debounce);
				}
				hall_set_cur_state(input, SW_LID, HOLSTER_CLOSED);
				STATE_CHANGE(STATE_HOLSTER_CLOSED);
				ddata->holster_state = HOLSTER_CLOSED;
			} else if(SMART_FLIP_OPEN_EVENT(open_state, mid_state, closed_state)) {
				/* The smart flip is closed and we got a smart flip event */
				/* We can delay handling this event since it could be */
				/* a transition event or a holster */
				if(pdata->sf_debounce > 0)
					schedule_delayed_work(&ddata->sf_work, msecs_to_jiffies(pdata->sf_debounce));
				else {
					hall_set_cur_state(input, SW_SMART_FLIP, FLIP_OPEN);
					ddata->flip_state = FLIP_OPEN;
				}
				STATE_CHANGE(STATE_SLIDER_CLOSED);
			}
			break;
		default:
			pr_err("Invalid State\n");
			break;
	}
	pr_info("hall_gpio_report_event: open %d mid %d close %d state %d\n", open_state, mid_state, closed_state, ddata->hall_state);
}

static void hall_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
	    container_of(work, struct gpio_button_data, work);
	struct hall_drvdata *ddata = input_get_drvdata(bdata->input);

	hall_gpio_report_event(ddata);

	if (bdata->button->wakeup)
		pm_relax(bdata->input->dev.parent);
}

static void hall_sf_work_func(struct work_struct *work)
{
	struct hall_drvdata *ddata = container_of(work, struct hall_drvdata, sf_work.work);
	struct input_dev *input = ddata->input;
	int closed_state;
	int open_state;
	int mid_state;

	closed_state = (gpio_get_value_cansleep(ddata->closed_gpio) ? 1 : 0);
	open_state = (gpio_get_value_cansleep(ddata->open_gpio) ? 1 : 0);
	mid_state = (gpio_get_value_cansleep(ddata->mid_gpio) ? 1 : 0);

	/* Here we handle the delayed flip events */
	switch(ddata->hall_state)
	{
		case STATE_SMART_FLIP_CLOSED:
			if(SMART_FLIP_CLOSE_EVENT(open_state, mid_state, closed_state) && ddata->flip_state == FLIP_OPEN) {
				/* The smart flip is still in closed state and the flip event is still valid */
				/* We send the smart flip close event and move to smart flip closed state */
				hall_set_cur_state(input, SW_SMART_FLIP, FLIP_CLOSED);
				ddata->flip_state = FLIP_CLOSED;
			}
			break;
		case STATE_SLIDER_CLOSED:
			if(SMART_FLIP_OPEN_EVENT(open_state, mid_state, closed_state) && ddata->flip_state == FLIP_CLOSED) {
				/* The smart flip is still in closed state and the flip event is still valid. */
				/* We send the smart flip open event and move to slider closed state */
				hall_set_cur_state(input, SW_SMART_FLIP, FLIP_OPEN);
				STATE_CHANGE(STATE_SLIDER_CLOSED);
				ddata->flip_state = FLIP_OPEN;
			}
			break;
		case STATE_SLIDER_TRANSITION:
			if(SLIDER_OPEN_EVENT(open_state, mid_state, closed_state)) {
				/* The slider is in transition and we got a slider open event */
				hall_set_cur_state(input, SW_KEYPAD_SLIDE, SLIDER_OPEN);
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 0);
				STATE_CHANGE(STATE_SLIDER_OPEN);
			} else if(SLIDER_CLOSE_EVENT(open_state, mid_state, closed_state)) {
				/* The slider is in transition and we got a slider close event */
				hall_set_cur_state(input, SW_KEYPAD_SLIDE, SLIDER_CLOSED);
				hall_set_cur_state(input, SW_KEYPAD_TRANSITION, 0);
				STATE_CHANGE(STATE_SLIDER_CLOSED);
			}
			break;
		default:
			break;
	}
	pr_info("hall_sf_work_func: open %d mid %d close %d state %d\n", open_state, mid_state, closed_state, ddata->hall_state);
}

static void hall_gpio_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;

	schedule_work(&bdata->work);
}

static irqreturn_t hall_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;

	if (bdata->button->wakeup)
		pm_stay_awake(bdata->input->dev.parent);
	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
		jiffies + msecs_to_jiffies(bdata->timer_debounce));
	else
		schedule_work(&bdata->work);

	return IRQ_HANDLED;
}

static int hall_setup_key(struct platform_device *pdev,
	struct input_dev *input,
	struct gpio_button_data *bdata,
	const struct hall_button *button)
{
	const char *desc = button->desc ? button->desc : "hall";
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error;

	bdata->input = input;
	bdata->button = button;

	if (!gpio_is_valid(button->gpio))
		return -EINVAL;

	error = gpio_request_one(button->gpio, GPIOF_IN, desc);
	if (error < 0) {
		dev_err(dev, "Failed to request GPIO %d, error %d\n",
		button->gpio, error);
		return error;
	}

	if (button->debounce_interval) {
		error = gpio_set_debounce(button->gpio,
		button->debounce_interval * 1000);
		/* use timer if gpiolib doesn't provide debounce */
		if (error < 0)
			bdata->timer_debounce = button->debounce_interval;
	}

	irq = gpio_to_irq(button->gpio);
	if (irq < 0) {
		error = irq;
		dev_err(dev, "Unable to get irq for GPIO %d, error %d\n",
		button->gpio, error);
		goto fail;
	}
	bdata->irq = irq;

	INIT_WORK(&bdata->work, hall_gpio_work_func);
	setup_timer(&bdata->timer, hall_gpio_timer, (unsigned long)bdata);

	isr = hall_gpio_isr;
	irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	input_set_capability(input, button->type ? : EV_KEY, button->code);

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(bdata->irq, isr, irqflags,
		desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
		bdata->irq, error);
		goto fail;
	}

	return 0;

fail:
	if (gpio_is_valid(button->gpio))
		gpio_free(button->gpio);

	return error;
}

static int hall_open(struct input_dev *input)
{
	struct hall_drvdata *ddata = input_get_drvdata(input);
	const struct hall_platform_data *pdata = ddata->pdata;
	int error;

	if (pdata->enable) {
		error = pdata->enable(input->dev.parent);
		if (error)
			return error;
	}

	/* Report current state of buttons that are connected to GPIOs */
	hall_gpio_report_event(ddata);

	return 0;
}

static void hall_close(struct input_dev *input)
{
	struct hall_drvdata *ddata = input_get_drvdata(input);
	const struct hall_platform_data *pdata = ddata->pdata;

	if (pdata->disable)
		pdata->disable(input->dev.parent);
}

/*
 * Handlers for alternative sources of platform_data
 */

#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static struct hall_platform_data *
hall_get_devtree_pdata(struct device *dev)
{
	struct device_node *node, *pp;
	struct hall_platform_data *pdata;
	struct hall_button *button;
	int error;
	int nbuttons;
	int i;

	node = dev->of_node;
	if (!node) {
		error = -ENODEV;
		goto err_out;
	}

	nbuttons = of_get_child_count(node);
	if (nbuttons == 0) {
		error = -ENODEV;
		goto err_out;
	}

	pdata = kzalloc(sizeof(*pdata) + nbuttons * sizeof(*button),
	GFP_KERNEL);
	if (!pdata) {
		error = -ENOMEM;
		goto err_out;
	}

	pdata->buttons = (struct hall_button *)(pdata + 1);
	pdata->nbuttons = nbuttons;
	pdata->sf_debounce = 0;

	pdata->name = of_get_property(node, "input-name", NULL);
	of_property_read_u32(node, "smart-flip-debounce", &pdata->sf_debounce);

	i = 0;
	for_each_child_of_node(node, pp) {
		int gpio;
		enum of_gpio_flags flags;

		if (!of_find_property(pp, "gpios", NULL)) {
			pdata->nbuttons--;
			dev_warn(dev, "Found button without gpios\n");
			continue;
		}

		gpio = of_get_gpio_flags(pp, 0, &flags);
		if (gpio < 0) {
			error = gpio;
			if (error != -EPROBE_DEFER)
				dev_err(dev,
				"Failed to get gpio flags error: %d\n", error);
			goto err_free_pdata;
		}

		button = &pdata->buttons[i++];

		button->gpio = gpio;
		button->active_low = flags & OF_GPIO_ACTIVE_LOW;

		if (of_property_read_u32(pp, "linux,code", &button->code)) {
			dev_err(dev, "Button without keycode: 0x%x\n",
			button->gpio);
			error = -EINVAL;
			goto err_free_pdata;
		}

		button->desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type",
		&button->type))
			button->type = EV_KEY;

		button->wakeup = !!of_get_property(pp, "gpio-key,wakeup",
		NULL);

		if (of_property_read_u32(pp, "debounce-interval",
		&button->debounce_interval))
			button->debounce_interval = 5;
	}

	if (pdata->nbuttons == 0) {
		error = -EINVAL;
		goto err_free_pdata;
	}

	return pdata;

err_free_pdata:
	kfree(pdata);
err_out:
	return ERR_PTR(error);
}

static struct of_device_id hall_of_match[] = {
	{ .compatible = "hall", },
	{ },
};
MODULE_DEVICE_TABLE(of, hall_of_match);

#else

static inline struct hall_platform_data *
hall_get_devtree_pdata(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}

#endif

static void gpio_remove_key(struct gpio_button_data *bdata)
{
	free_irq(bdata->irq, bdata);
	if (bdata->timer_debounce)
		del_timer_sync(&bdata->timer);
	cancel_work_sync(&bdata->work);
	if (gpio_is_valid(bdata->button->gpio))
		gpio_free(bdata->button->gpio);
}

static int hall_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct hall_platform_data *pdata = dev_get_platdata(dev);
	struct hall_drvdata *ddata;
	struct input_dev *input;
	int i = 0, error;
	int wakeup = 0;

	if (!pdata) {
		pdata = hall_get_devtree_pdata(dev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);
	}

	ddata = kzalloc(sizeof(struct hall_drvdata) +
	pdata->nbuttons * sizeof(struct gpio_button_data), GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->pdata = pdata;
	ddata->input = input;

	/* Initializing gpios to -1 */
	ddata->open_gpio = -1;
	ddata->closed_gpio = -1;
	ddata->mid_gpio = -1;

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	if(pdata->sf_debounce > 0)
		INIT_DELAYED_WORK(&ddata->sf_work, hall_sf_work_func);

	input->name = pdata->name ? : pdev->name;
	input->phys = "hall/input0";
	input->dev.parent = &pdev->dev;
	input->open = hall_open;
	input->close = hall_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;

	set_bit(SW_LID, input->swbit);
	set_bit(SW_SMART_FLIP, input->swbit);
	set_bit(SW_KEYPAD_SLIDE, input->swbit);
	set_bit(SW_KEYPAD_TRANSITION, input->swbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		const struct hall_button *button = &pdata->buttons[i];
		struct gpio_button_data *bdata = &ddata->data[i];

		if (0 == strncmp(button->desc, "open_sensor",
		strlen("open_sensor")))
			ddata->open_gpio = button->gpio;

		if (0 == strncmp(button->desc, "closed_sensor",
		strlen("closed_sensor")))
			ddata->closed_gpio = button->gpio;

		if (0 == strncmp(button->desc, "mid_sensor",
		strlen("mid_sensor")))
			ddata->mid_gpio = button->gpio;

		error = hall_setup_key(pdev, input, bdata, button);
		if (error)
			goto fail1;

		if (button->wakeup)
			wakeup = 1;
	}

	if (ddata->open_gpio == -1 || ddata->mid_gpio == -1 || ddata->closed_gpio == -1) {
		dev_err(dev,
		"hall gpio pin is invalid: closed_gpio %d mid_gpio %d open_gpio %d\n",
		ddata->closed_gpio, ddata->mid_gpio, ddata->open_gpio);
		goto fail2;
	}


	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
		error);
		goto fail2;
	}

	device_init_wakeup(&pdev->dev, wakeup);

	return 0;

fail2:
	while (--i >= 0)
		gpio_remove_key(&ddata->data[i]);

	platform_set_drvdata(pdev, NULL);
fail1:
	input_free_device(input);
	kfree(ddata);
	/* If we have no platform data, we allocated pdata dynamically. */
	if (!dev_get_platdata(&pdev->dev))
		kfree(pdata);

	return error;
}

static int hall_remove(struct platform_device *pdev)
{
	struct hall_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < ddata->pdata->nbuttons; i++)
		gpio_remove_key(&ddata->data[i]);

	input_unregister_device(input);

	/* If we have no platform data, we allocated pdata dynamically. */
	if (!dev_get_platdata(&pdev->dev))
		kfree(ddata->pdata);

	kfree(ddata);

	return 0;
}

static int hall_suspend(struct device *dev)
{
	struct hall_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		enable_irq_wake(bdata->irq);
	}
	return 0;
}

static int hall_resume(struct device *dev)
{
	struct hall_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ddata->pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		disable_irq_wake(bdata->irq);
	}
	return 0;
}

static SIMPLE_DEV_PM_OPS(hall_pm_ops, hall_suspend, hall_resume);

static struct platform_driver hall_device_driver = {
	.probe		= hall_probe,
	.remove		= hall_remove,
	.driver		= {
		.name	= "hall",
		.owner	= THIS_MODULE,
		.pm	= &hall_pm_ops,
		.of_match_table = of_match_ptr(hall_of_match),
	}
};

static int __init hall_init(void)
{
	return platform_driver_register(&hall_device_driver);
}

static void __exit hall_exit(void)
{
	platform_driver_unregister(&hall_device_driver);
}

late_initcall(hall_init);
module_exit(hall_exit);

MODULE_DESCRIPTION("HALL Sensor Driver");
MODULE_AUTHOR("BlackBerry Limited");
MODULE_LICENSE("GPL v2");
