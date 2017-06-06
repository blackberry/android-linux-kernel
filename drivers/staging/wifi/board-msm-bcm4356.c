/*
 * Copyright (C) 2015 BlackBerry Limited
 * Copyright (C) 2014 Motorola Mobility LLC
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
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/skbuff.h>
#include <linux/wlan_plat.h>
#include <linux/mmc/host.h>
#include <linux/if.h>

#ifndef CONFIG_BBRY
#include <mach/gpio.h>
#include <mach/board.h>
#include <mach/msm_pcie.h>
#else
#include <linux/gpio.h>
#include <linux/msm_pcie.h>
#endif /* CONFIG_BBRY */
#include <linux/of_gpio.h>

#include <linux/fcntl.h>
#include <linux/fs.h>
#ifndef CONFIG_BBRY
#include "pcie.h"
#endif /* CONFIG_BBRY */

#define BCM_DBG pr_debug

#ifndef CONFIG_BBRY
static int gpio_wl_reg_on = 82;
#else
static int gpio_wl_reg_on = 89;
#endif

#ifdef CONFIG_DHD_USE_STATIC_BUF

#define WLAN_STATIC_SCAN_BUF0		5
#define WLAN_STATIC_SCAN_BUF1		6
#define WLAN_STATIC_DHD_INFO_BUF	7
#define WLAN_STATIC_DHD_IF_FLOW_LKUP    9
#define WLAN_SCAN_BUF_SIZE		(64 * 1024)
#define WLAN_DHD_INFO_BUF_SIZE		(24 * 1024)

#ifndef CONFIG_BBRY
#define WLAN_DHD_IF_FLOW_LKUP_SIZE      (20 * 1024)
#else
#define WLAN_DHD_IF_FLOW_LKUP_SIZE      (48 * 1024)
#endif /* CONFIG_BBRY */

#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#ifdef CONFIG_BCMDHD_PCIE
#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	0
#define WLAN_SECTION_SIZE_2	0
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)
#else
#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)
#endif /* CONFIG_BCMDHD_PCIE */

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define DHD_SKB_1PAGE_BUF_NUM	8
#define DHD_SKB_2PAGE_BUF_NUM	8
#define DHD_SKB_4PAGE_BUF_NUM	1

#define WLAN_SKB_1_2PAGE_BUF_NUM	((DHD_SKB_1PAGE_BUF_NUM) + \
		(DHD_SKB_2PAGE_BUF_NUM))
#define WLAN_SKB_BUF_NUM	((WLAN_SKB_1_2PAGE_BUF_NUM) + \
		(DHD_SKB_4PAGE_BUF_NUM))


static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wlan_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wlan_mem_prealloc wlan_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

void *wlan_static_scan_buf0 = NULL;
void *wlan_static_scan_buf1 = NULL;
void *wlan_static_dhd_info_buf = NULL;
void *wlan_static_if_flow_lkup = NULL;

static void *brcm_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;

	if (section == WLAN_STATIC_SCAN_BUF0)
		return wlan_static_scan_buf0;

	if (section == WLAN_STATIC_SCAN_BUF1)
		return wlan_static_scan_buf1;

	if (section == WLAN_STATIC_DHD_INFO_BUF) {
		if (size > WLAN_DHD_INFO_BUF_SIZE) {
			pr_err("request DHD_INFO size(%lu) is bigger than static size(%d).\n",
				size, WLAN_DHD_INFO_BUF_SIZE);
			return NULL;
		}
		return wlan_static_dhd_info_buf;
	}
	if (section == WLAN_STATIC_DHD_IF_FLOW_LKUP)  {
		if (size > WLAN_DHD_IF_FLOW_LKUP_SIZE) {
			pr_err("request DHD_IF_FLOW_LKUP size(%lu) is bigger than static size(%d).\n",
				size, WLAN_DHD_IF_FLOW_LKUP_SIZE);
			return NULL;
		}

		return wlan_static_if_flow_lkup;
	}
	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wlan_mem_array[section].size < size)
		return NULL;

	return wlan_mem_array[section].mem_ptr;
}

static int brcm_init_wlan_mem(void)
{
	int i, j;

	for (i = 0; i < DHD_SKB_1PAGE_BUF_NUM; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (i = DHD_SKB_1PAGE_BUF_NUM; i < WLAN_SKB_1_2PAGE_BUF_NUM; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0; i < PREALLOC_WLAN_SEC_NUM; i++) {
		if (wlan_mem_array[i].size > 0) {
			wlan_mem_array[i].mem_ptr =
				kmalloc(wlan_mem_array[i].size, GFP_KERNEL);
			if (!wlan_mem_array[i].mem_ptr)
				goto err_mem_alloc;
		}
	}

	wlan_static_scan_buf0 = kmalloc(WLAN_SCAN_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_scan_buf0) {
		pr_err("Failed to alloc wlan_static_scan_buf0\n");
		goto err_mem_alloc;
	}

	wlan_static_scan_buf1 = kmalloc(WLAN_SCAN_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_scan_buf1) {
		pr_err("Failed to alloc wlan_static_scan_buf1\n");
		goto err_mem_alloc;
	}

	wlan_static_dhd_info_buf = kmalloc(WLAN_DHD_INFO_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_info_buf) {
		pr_err("Failed to alloc wlan_static_dhd_info_buf\n");
		goto err_mem_alloc;
	}
#ifdef CONFIG_BCMDHD_PCIE
	wlan_static_if_flow_lkup = kmalloc(WLAN_DHD_IF_FLOW_LKUP_SIZE, GFP_KERNEL);
	if (!wlan_static_if_flow_lkup) {
		pr_err("Failed to alloc wlan_static_if_flow_lkup\n");
		goto err_mem_alloc;
	}
#endif /* CONFIG_BCMDHD_PCIE */

	return 0;

err_mem_alloc:
	if (wlan_static_dhd_info_buf)
		kfree(wlan_static_dhd_info_buf);

	if (wlan_static_scan_buf1)
		kfree(wlan_static_scan_buf1);

	if (wlan_static_scan_buf0)
		kfree(wlan_static_scan_buf0);

	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wlan_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}
#endif /* CONFIG_DHD_USE_STATIC_BUF */

#if 0 /* Remove bootstrapping to prevent stack dump at driver bind */
/***
 * To config the wifi GPIO_4 as pull-down from msm side
 * It's only being configured here once when board boot up
 * It's Only applied to pre V8 HW */
void __init brcm_wifi_init_gpio_coex(struct device *dev, struct device_node *np)
{
	struct pinctrl *pinctrl = NULL;
	struct pinctrl_state *pins_default = NULL;
	int gp_134;

	if (!dev) {
		pr_err("%s: Err: brcm_device_wlan.dev not exist yet\n", __func__);
		goto coex_err;
	}
	if (!np) {
		pr_err("%s: Err: wlan device node null pointer\n", __func__);
		goto coex_err;
	}

	/* pin configuration */
	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL (pinctrl)) {
		pr_err("%s: pinctrl is %p. This is expected for device newer than V8\n",
				__func__, pinctrl);
		goto coex_err;
	}
	pins_default = pinctrl_lookup_state(pinctrl, "default");
	if (IS_ERR_OR_NULL (pins_default)) {
		pr_err("%s: Err: pins_default is %p\n",
				__func__, pins_default);
		goto coex_err;
	}
	if (pinctrl_select_state(pinctrl, pins_default)) {
		pr_err("%s: Err: pinctrl select state failed\n", __func__);
		goto coex_err;
	}

	/* gpio operation */
	gp_134 = of_get_named_gpio(np, "wl_lte_coex_tx", 0);
	if (gp_134 < 0) {
		pr_err("%s: failed to get wl_lte_coex_tx GPIO %d\n", __func__, gp_134);
		goto coex_err;
	} else
		pr_info("%s: wl_lte_coex_tx GPIO %d\n", __func__, gp_134);

	if (gpio_request(gp_134, "WL_LTE_COEX_TX")) {
		pr_err("%s: Failed to request gpio %d for WL_LTE_COEX_TX\n",
			__func__, gp_134);
		goto coex_err;
	} else
		pr_info("%s: gpio_request gp_134 done\n", __func__);

	if (gpio_direction_output(gp_134, 0))
		pr_err("%s: wl_lte_coex_tx failed to output 0\n", __func__);
	else
		pr_info("%s: wl_lte_coex_tx output 0\n", __func__);

	if (gpio_get_value(gp_134))
		pr_info("%s: Initial gp_134: [%d]\n",
			__func__, gpio_get_value(gp_134));

	pr_err("sleep 200ms for wlan bootstraping");
	msleep(200);

coex_err:
	return;
}
#endif /* #if 0 */

int __init brcm_wifi_init_gpio(struct device_node *np)
{
	if (np) {
		int wl_reg_on;
		wl_reg_on = of_get_named_gpio(np, "wl_reg_on", 0);
		if (wl_reg_on >= 0) {
			gpio_wl_reg_on = wl_reg_on;
			BCM_DBG("%s: wl_reg_on GPIO %d\n", __func__, wl_reg_on);
		}
	}

	if (gpio_request(gpio_wl_reg_on, "WL_REG_ON"))
		pr_err("%s: Faiiled to request gpio %d for WL_REG_ON\n",
			__func__, gpio_wl_reg_on);
	else
		pr_err("%s: gpio_request WL_REG_ON done\n", __func__);

	if (gpio_direction_output(gpio_wl_reg_on, 1))
		pr_err("%s: WL_REG_ON failed to pull up\n", __func__);
	else
		BCM_DBG("%s: WL_REG_ON is pulled up\n", __func__);

	if (gpio_get_value(gpio_wl_reg_on))
		BCM_DBG("%s: Initial WL_REG_ON: [%d]\n",
			__func__, gpio_get_value(gpio_wl_reg_on));

	return 0;
}

int brcm_wlan_power(int on)
{
	BCM_DBG("------------------------------------------------");
	BCM_DBG("------------------------------------------------\n");
	pr_info("%s Enter: power %s\n", __func__, on ? "on" : "off");

	if (on) {
		if (gpio_direction_output(gpio_wl_reg_on, 1)) {
			pr_err("%s: WL_REG_ON didn't output high\n", __func__);
			return -EIO;
		}
		if (!gpio_get_value(gpio_wl_reg_on))
			pr_err("[%s] gpio didn't set high.\n", __func__);
	} else {
		if (gpio_direction_output(gpio_wl_reg_on, 0)) {
			pr_err("%s: WL_REG_ON didn't output low\n", __func__);
			return -EIO;
		}
	}
	return 0;
}
EXPORT_SYMBOL(brcm_wlan_power);

static int brcm_wlan_reset(int onoff)
{
	return 0;
}

static int brcm_wlan_set_carddetect(int val)
{
	return 0;
}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ        4
struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
};

#ifdef CONFIG_BBRY
/* NOTE - all ETSI countries are mapped to GB for simplicity */
static struct cntry_locales_custom bbry_wlan_translate_custom_table[] = {
	{ "",   "XT", -1 },  /* Universal if Country code is unknown or empty */
	{ "AF", "GB", -1 },
	{ "AL", "GB", -1 },
	{ "DZ", "DZ", -1 },
	{ "AS", "AS", -1 },
	{ "AD", "AD", -1 },
	{ "AO", "AO", -1 },
	{ "AI", "AI", -1 },
	{ "AG", "AG", -1 },
	{ "AR", "AR", -1 },
	{ "AM", "AM", -1 },
	{ "AW", "AW", -1 },
	{ "AU", "AU", -1 },
	{ "AT", "GB", -1 },
	{ "AZ", "AZ", -1 },
	{ "BS", "BS", -1 },
	{ "BH", "BH", -1 },
	{ "BD", "BD", -1 },
	{ "BB", "BB", -1 },
	{ "BY", "BY", -1 },
	{ "BE", "GB", -1 },
	{ "BZ", "BZ", -1 },
	{ "BJ", "BJ", -1 },
	{ "BM", "BM", -1 },
	{ "BT", "GB", -1 },
	{ "BO", "BO", -1 },
	{ "BA", "GB", -1 },
	{ "BW", "BW", -1 },
	{ "BR", "BR", -1 },
	{ "IO", "IO", -1 },
	{ "VG", "VG", -1 },
	{ "BN", "BN", -1 },
	{ "BG", "GB", -1 },
	{ "BF", "BF", -1 },
	{ "BI", "BI", -1 },
	{ "KH", "KH", -1 },
	{ "CM", "CM", -1 },
	{ "CA", "US", -1 },
	{ "CV", "CV", -1 },
	{ "KY", "KY", -1 },
	{ "CF", "CF", -1 },
	{ "TD", "TD", -1 },
	{ "CL", "CL", -1 },
	{ "CN", "CN", -1 },
	{ "CX", "CX", -1 },
	{ "CO", "CO", -1 },
	{ "KM", "KM", -1 },
	{ "CD", "CD", -1 },
	{ "CG", "CG", -1 },
	{ "CK", "CK", -1 },
	{ "CR", "CR", -1 },
	{ "CI", "CI", -1 },
	{ "CY", "GB", -1 },
	{ "CZ", "GB", -1 },
	{ "DK", "GB", -1 },
	{ "DJ", "DJ", -1 },
	{ "DM", "DM", -1 },
	{ "DO", "DO", -1 },
	{ "EC", "EC", -1 },
	{ "EG", "EG", -1 },
	{ "SV", "SV", -1 },
	{ "GQ", "GQ", -1 },
	{ "ER", "ER", -1 },
	{ "EE", "GB", -1 },
	{ "ET", "GB", -1 },
	{ "FK", "FK", -1 },
	{ "FO", "FO", -1 },
	{ "FJ", "FJ", -1 },
	{ "FI", "GB", -1 },
	{ "FR", "GB", -1 },
	{ "GF", "GF", -1 },
	{ "PF", "PF", -1 },
	{ "TF", "TF", -1 },
	{ "GA", "GA", -1 },
	{ "GM", "GM", -1 },
	{ "GE", "GE", -1 },
	{ "DE", "GB", -1 },
	{ "GH", "GH", -1 },
	{ "GI", "GI", -1 },
	{ "GR", "GB", -1 },
	{ "GD", "GD", -1 },
	{ "GP", "GP", -1 },
	{ "GU", "GU", -1 },
	{ "GT", "GT", -1 },
	{ "GN", "GN", -1 },
	{ "GW", "GW", -1 },
	{ "GY", "GY", -1 },
	{ "HT", "HT", -1 },
	{ "VA", "VA", -1 },
	{ "HN", "HN", -1 },
	{ "HK", "HK", -1 },
	{ "HR", "GB", -1 },
	{ "HU", "GB", -1 },
	{ "IS", "GB", -1 },
	{ "IN", "IN", -1 },
	{ "ID", "ID", -1 },
	{ "IQ", "IQ", -1 },
	{ "IE", "GB", -1 },
	{ "IL", "IL", -1 },
	{ "IT", "GB", -1 },
	{ "JM", "JM", -1 },
	{ "JP", "JP", -1 },
	{ "JO", "JO", -1 },
	{ "KZ", "KZ", -1 },
	{ "KE", "KE", -1 },
	{ "KI", "KI", -1 },
	{ "KR", "KR", -1 },
	{ "KW", "KW", -1 },
	{ "KG", "KG", -1 },
	{ "LA", "LA", -1 },
	{ "LV", "GB", -1 },
	{ "LB", "LB", -1 },
	{ "LS", "GB", -1 },
	{ "LR", "LR", -1 },
	{ "LY", "LY", -1 },
	{ "LI", "GB", -1 },
	{ "LT", "GB", -1 },
	{ "LU", "GB", -1 },
	{ "MO", "MO", -1 },
	{ "MK", "GB", -1 },
	{ "MG", "MG", -1 },
	{ "MW", "MW", -1 },
	{ "MY", "MY", -1 },
	{ "MV", "MV", -1 },
	{ "ML", "ML", -1 },
	{ "MT", "GB", -1 },
	{ "MQ", "GB", -1 },
	{ "MR", "GB", -1 },
	{ "MU", "MU", -1 },
	{ "YT", "YT", -1 },
	{ "MX", "MX", -1 },
	{ "FM", "FM", -1 },
	{ "MD", "GB", -1 },
	{ "MC", "GB", -1 },
	{ "MN", "MN", -1 },
	{ "ME", "GB", -1 },
	{ "MS", "MS", -1 },
	{ "MA", "MA", -1 },
	{ "MZ", "MZ", -1 },
	{ "MM", "MM", -1 },
	{ "NA", "NA", -1 },
	{ "NR", "NR", -1 },
	{ "NP", "NP", -1 },
	{ "AN", "AN", -1 },
	{ "NL", "GB", -1 },
	{ "NC", "NC", -1 },
	{ "NZ", "NZ", -1 },
	{ "NI", "NI", -1 },
	{ "NE", "NE", -1 },
	{ "NG", "NG", -1 },
	{ "NU", "NU", -1 },
	{ "NF", "NF", -1 },
	{ "MP", "MP", -1 },
	{ "NO", "GB", -1 },
	{ "OM", "OM", -1 },
	{ "PK", "PK", -1 },
	{ "PW", "PW", -1 },
	{ "PA", "PA", -1 },
	{ "PG", "PG", -1 },
	{ "PY", "PY", -1 },
	{ "PE", "PE", -1 },
	{ "PH", "PH", -1 },
	{ "PL", "GB", -1 },
	{ "PT", "GB", -1 },
	{ "PR", "PR", -1 },
	{ "QA", "QA", -1 },
	{ "RE", "RE", -1 },
	{ "RO", "GB", -1 },
	{ "RU", "RU", -1 },
	{ "RW", "RW", -1 },
	{ "KN", "KN", -1 },
	{ "LC", "LC", -1 },
	{ "PM", "PM", -1 },
	{ "VC", "GB", -1 },
	{ "WS", "GB", -1 },
	{ "SM", "SM", -1 },
	{ "ST", "ST", -1 },
	{ "SA", "SA", -1 },
	{ "SN", "SN", -1 },
	{ "RS", "RS", -1 },
	{ "SC", "SC", -1 },
	{ "SL", "SL", -1 },
	{ "SG", "SG", -1 },
	{ "SK", "GB", -1 },
	{ "SI", "GB", -1 },
	{ "SB", "SB", -1 },
	{ "SO", "SO", -1 },
	{ "ZA", "ZA", -1 },
	{ "SS", "SS", -1 },
	{ "ES", "GB", -1 },
	{ "LK", "LK", -1 },
	{ "SR", "SR", -1 },
	{ "SZ", "SZ", -1 },
	{ "SE", "GB", -1 },
	{ "CH", "GB", -1 },
	{ "TW", "TW", -1 },
	{ "TJ", "TJ", -1 },
	{ "TZ", "TZ", -1 },
	{ "TH", "TH", -1 },
	{ "TG", "GB", -1 },
	{ "TO", "TO", -1 },
	{ "TN", "TN", -1 },
	{ "TR", "GB", -1 },
	{ "TM", "TM", -1 },
	{ "TC", "TC", -1 },
	{ "TV", "TV", -1 },
	{ "VI", "VI", -1 },
	{ "UG", "UG", -1 },
	{ "UA", "UA", -1 },
	{ "AE", "AE", -1 },
	{ "GB", "GB", -1 },
	{ "UM", "UM", -1 },
	{ "US", "US", -1 },
	{ "UY", "UY", -1 },
	{ "UZ", "UZ", -1 },
	{ "VU", "VU", -1 },
	{ "VE", "VE", -1 },
	{ "VN", "VN", -1 },
	{ "WF", "GB", -1 },
	{ "EH", "EH", -1 },
	{ "YE", "YE", -1 },
	{ "ZM", "ZM", -1 },
	{ "ZW", "ZW", -1 },
	{ "ALL", "ALL", -1 }
};
#else
static struct cntry_locales_custom brcm_wlan_translate_custom_table[] = {
	/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XT", 49},  /* Universal if Country code is unknown or empty */
	{"US", "US", 176},
	{"AE", "AE", 1},
	{"AR", "AR", 21},
	{"AT", "AT", 4},
	{"AU", "AU", 40},
	{"BE", "BE", 4},
	{"BG", "BG", 4},
	{"BN", "BN", 4},
	{"BR", "BR", 4},
	{"CA", "US", 176},   /* Previousely was CA/31 */
	{"CH", "CH", 4},
	{"CY", "CY", 4},
	{"CZ", "CZ", 4},
	{"DE", "DE", 7},
	{"DK", "DK", 4},
	{"EE", "EE", 4},
	{"ES", "ES", 4},
	{"FI", "FI", 4},
	{"FR", "FR", 5},
	{"GB", "GB", 6},
	{"GR", "GR", 4},
	{"HK", "HK", 2},
	{"HR", "HR", 4},
	{"HU", "HU", 4},
	{"IE", "IE", 5},
	{"IN", "IN", 28},
	{"IS", "IS", 4},
	{"IT", "IT", 4},
	{"ID", "ID", 13},
	{"JP", "JP", 86},
	{"KR", "KR", 57},
	{"KW", "KW", 5},
	{"LI", "LI", 4},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"LV", "LV", 4},
	{"MA", "MA", 2},
	{"MT", "MT", 4},
	{"MX", "MX", 20},
	{"MY", "MY", 16},
	{"NL", "NL", 4},
	{"NO", "NO", 4},
	{"NZ", "NZ", 4},
	{"PL", "PL", 4},
	{"PT", "PT", 4},
	{"PY", "PY", 2},
	{"RO", "RO", 4},
	{"RU", "RU", 13},
	{"SE", "SE", 4},
	{"SG", "SG", 19},
	{"SI", "SI", 4},
	{"SK", "SK", 4},
	{"TH", "TH", 5},
	{"TR", "TR", 7},
	{"TW", "TW", 1},
	{"VN", "VN", 4},
};

struct cntry_locales_custom brcm_wlan_translate_nodfs_table[] = {
	{"",   "XT", 50},  /* Universal if Country code is unknown or empty */
	{"US", "US", 177},
	{"AU", "AU", 41},
	{"BR", "BR", 18},
	{"CA", "US", 177},
	{"CH", "E0", 33},
	{"CY", "E0", 33},
	{"CZ", "E0", 33},
	{"DE", "E0", 33},
	{"DK", "E0", 33},
	{"EE", "E0", 33},
	{"ES", "E0", 33},
	{"EU", "E0", 33},
	{"FI", "E0", 33},
	{"FR", "E0", 33},
	{"GB", "E0", 33},
	{"GR", "E0", 33},
	{"HK", "SG", 20},
	{"HR", "E0", 33},
	{"HU", "E0", 33},
	{"ID", "ID", 5},
	{"IE", "E0", 33},
	{"IN", "IN", 29},
	{"IS", "E0", 33},
	{"IT", "E0", 33},
	{"JP", "JP", 87},
	{"KR", "KR", 79},
	{"KW", "KW", 5},
	{"LI", "E0", 33},
	{"LT", "E0", 33},
	{"LU", "E0", 33},
	{"LV", "LV", 4},
	{"MA", "MA", 2},
	{"MT", "E0", 33},
	{"MY", "MY", 17},
	{"MX", "US", 177},
	{"NL", "E0", 33},
	{"NO", "E0", 33},
	{"PL", "E0", 33},
	{"PT", "E0", 33},
	{"RO", "E0", 33},
	{"SE", "E0", 33},
	{"SG", "SG", 20},
	{"SI", "E0", 33},
	{"SK", "E0", 33},
	{"SZ", "E0", 33},
	{"TH", "TH", 9},
	{"TW", "TW", 60},
};
#endif /* CONFIG_BBRY */

static void *brcm_wlan_get_country_code(char *ccode, u32 flags)
{
	struct cntry_locales_custom *locales;
	int size;
	int i;

	if (!ccode)
		return NULL;

#ifdef CONFIG_BBRY
	locales = bbry_wlan_translate_custom_table;
	size = ARRAY_SIZE(bbry_wlan_translate_custom_table);
#else
	if (flags & WLAN_PLAT_NODFS_FLAG) {
		locales = brcm_wlan_translate_nodfs_table;
		size = ARRAY_SIZE(brcm_wlan_translate_nodfs_table);
	} else {
		locales = brcm_wlan_translate_custom_table;
		size = ARRAY_SIZE(brcm_wlan_translate_custom_table);
	}
#endif /* CONFIG_BBRY */

	for (i = 0; i < size; i++)
		if (strcmp(ccode, locales[i].iso_abbrev) == 0)
			return &locales[i];
	return &locales[0];
}

static unsigned char brcm_mac_addr[IFHWADDRLEN] = { 0, 0x90, 0x4c, 0, 0, 0 };

#ifndef CONFIG_BBRY
static int __init brcm_mac_addr_setup(char *str)
{
	char macstr[IFHWADDRLEN*3];
	char *macptr = macstr;
	char *token;
	int i = 0;

	if (!str)
		return 0;
	BCM_DBG("wlan MAC = %s\n", str);
	if (strlen(str) >= sizeof(macstr))
		return 0;
	strlcpy(macstr, str, sizeof(macstr));

	while ((token = strsep(&macptr, ":")) != NULL) {
		unsigned long val;
		int res;

		if (i >= IFHWADDRLEN)
			break;
		res = kstrtoul(token, 0x10, &val);
		if (res < 0)
			break;
		brcm_mac_addr[i++] = (u8)val;
	}

	if (i < IFHWADDRLEN && strlen(macstr)==IFHWADDRLEN*2) {
		/* try again with wrong format (sans colons) */
		u64 mac;
		if (kstrtoull(macstr, 0x10, &mac) < 0)
			return 0;
		for (i=0; i<IFHWADDRLEN; i++)
			brcm_mac_addr[IFHWADDRLEN-1-i] = (u8)((0xFF)&(mac>>(i*8)));
	}

	return i==IFHWADDRLEN ? 1:0;
}

__setup("androidboot.wifimacaddr=", brcm_mac_addr_setup);

static int brcm_wifi_get_mac_addr(unsigned char *buf)
{
	uint rand_mac;

	if (!buf)
		return -EFAULT;

	if ((brcm_mac_addr[4] == 0) && (brcm_mac_addr[5] == 0)) {
		prandom_seed((uint)jiffies);
		rand_mac = prandom_u32();
		brcm_mac_addr[3] = (unsigned char)rand_mac;
		brcm_mac_addr[4] = (unsigned char)(rand_mac >> 8);
		brcm_mac_addr[5] = (unsigned char)(rand_mac >> 16);
	}
	memcpy(buf, brcm_mac_addr, IFHWADDRLEN);

	return 0;
}
#else
static int read_mac_addr (void) {
    int rc = 0;
	int mac_len = 0;
	struct device_node *node = NULL;
	const void *macp = NULL;

	node = of_find_compatible_node(NULL, NULL, "oem,board_mac_addr");
	if (!node) {
		pr_err("%s: Error finding compatible node oem,board_mac_addr\n", __func__);
		rc = -1;
	} else if(of_find_compatible_node(node, NULL, "oem,board_mac_addr")) {
		pr_err("%s: Found 2 compatible node oem,board_mac_addr\n", __func__);
		rc = -1;
	} else {
		macp = of_get_property(node, "macaddresswlan", &mac_len);
		if (!macp) {
			pr_err("%s: Did not find macaddresswlan props\n", __func__);
			rc = -1;
		} else if (mac_len != IFHWADDRLEN) {
			pr_err("%s: Invaild size, got %d, expected %d\n",
					__func__,
					mac_len,
					IFHWADDRLEN);
			rc = -1;
		} else {
			memcpy(brcm_mac_addr, macp, IFHWADDRLEN);
			rc = 0;
		}
	}
    return rc;
}

static int brcm_wifi_get_mac_addr(unsigned char *buf)
{
	static bool first_time = true;

	if (!buf)
		return -EFAULT;

	if (first_time) {
		int rc = 0;
		uint rand_mac = 0;

		first_time = false;
		rc = read_mac_addr();
		if (rc != 0) {
			/* couldn't get the real mac then generate a fake one */
			prandom_seed((uint)jiffies);
			rand_mac = prandom_u32();
			brcm_mac_addr[3] = (unsigned char)rand_mac;
			brcm_mac_addr[4] = (unsigned char)(rand_mac >> 8);
			brcm_mac_addr[5] = (unsigned char)(rand_mac >> 16);
		}
#if 0 /* removing mac address print due to PII */
		pr_err("%s: %s %02x:%02x:%02x:%02x:%02x:%02x\n",
			__func__,
			(rc == 0)? "READ MAC ADDR from DT" : "RANDOM MAC ADDR",
			brcm_mac_addr[0], brcm_mac_addr[1], brcm_mac_addr[2],
			brcm_mac_addr[3], brcm_mac_addr[4], brcm_mac_addr[5]);
#endif
	}

	memcpy(buf, brcm_mac_addr, IFHWADDRLEN);

#if 0 /* unmask OUI since we are getting close to launch */
	/* Mask OUI with Broadcom's for now FIXME when we go public */
	buf[0] = 0x00;
	buf[1] = 0x90;
	buf[2] = 0x4c;
	pr_err("%s: WLAN MAC OUI is masked\n", __func__);
#endif

	return 0;
}

#endif /* CONFIG_BBRY */

static struct resource brcm_wlan_resources[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		.start	= 0, /* Dummy */
		.end	= 0, /* Dummy */
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE
			| IORESOURCE_IRQ_HIGHLEVEL, /* Dummy */
	},
};

static struct wifi_platform_data brcm_wlan_control = {
	.set_power	= brcm_wlan_power,
	.set_reset	= brcm_wlan_reset,
	.set_carddetect	= brcm_wlan_set_carddetect,
	.get_mac_addr = brcm_wifi_get_mac_addr,
#ifdef CONFIG_DHD_USE_STATIC_BUF
	.mem_prealloc	= brcm_wlan_mem_prealloc,
#endif
	.get_country_code = brcm_wlan_get_country_code,
};

static struct platform_device brcm_device_wlan = {
	.name		= "bcmdhd_wlan",
	.id		= 1,
	.num_resources	= ARRAY_SIZE(brcm_wlan_resources),
	.resource	= brcm_wlan_resources,
	.dev		= {
		.platform_data = &brcm_wlan_control,
	},
};

int __init brcm_wlan_init(void)
{
	int rc;
	struct device_node *np;

	BCM_DBG("%s: START\n", __func__);

#ifdef CONFIG_DHD_USE_STATIC_BUF
	brcm_init_wlan_mem();
#endif
	rc = platform_device_register(&brcm_device_wlan);

	np = of_find_compatible_node(NULL, NULL, "android,bcmdhd_wlan");
	brcm_device_wlan.dev.of_node = np;

#if 0 /* Remove bootstrapping to prevent stack dump at driver bind */
	brcm_wifi_init_gpio_coex( &(brcm_device_wlan.dev), np);
#endif
	brcm_wifi_init_gpio(np);
	return rc;
}
subsys_initcall(brcm_wlan_init);
