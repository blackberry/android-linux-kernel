#ifndef __HALL_H
#define __HALL_H



#define HALL_VDD_MAX_UV 1950000
#define HALL_VDD_MIN_UV 1750000


struct hall_data {
	int irq_gpio_cover;
	int irq_gpio_vr;
	int tp_is_suspend;
	unsigned int hall_cover_state;
	unsigned int hall_vr_state;
	struct pinctrl		*pinctrl;
	struct pinctrl_state	*pin_default;
	struct workqueue_struct *hall_wq;
	struct work_struct hall_cover_work;
	struct work_struct hall_vr_work;
	
	void (*tp_set_sensitivity)(int);
	struct regulator	*vdd;
	struct platform_device	*pdev;
	int power_enabled;
};

#endif


