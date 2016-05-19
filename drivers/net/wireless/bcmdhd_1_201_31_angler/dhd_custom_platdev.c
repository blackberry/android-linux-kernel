/*
 * Custom file for dealing platform specific resouce
 *
 * Copyright (C) 2016 BlackBerry Limited
 * Copyright (C) 1999-2015, Broadcom Corporation
 *
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 *
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 *
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: dhd_custom_msm.c 520105 2014-12-10 07:22:01Z $
 *
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

#if defined(CONFIG_ARCH_MSM)
#if defined(CONFIG_64BIT)
#include <linux/msm_pcie.h>
#else
#include <mach/msm_pcie.h>
#endif
#endif

#include <linux/of_gpio.h>

#include <linux/fcntl.h>
#include <linux/fs.h>

#if defined(CONFIG_BCMDHD_PCIE) && defined(CONFIG_ARCH_MSM) && defined(CONFIG_64BIT)
#include <linux/msm_pcie.h>
#endif

#define DHD_DT_COMPAT_ENTRY	"android,bcmdhd_wlan"

#define BCM_DBG pr_debug

static int gpio_wl_reg_on = -1;
static int brcm_wake_irq = -1;

#if !defined(CONFIG_WIFI_CONTROL_FUNC)
#define WLAN_PLAT_NODFS_FLAG	0x01
#define WLAN_PLAT_AP_FLAG	0x02
#endif

#ifdef CONFIG_DHD_USE_STATIC_BUF

enum dhd_prealloc_index {
	DHD_PREALLOC_PROT = 0,
	DHD_PREALLOC_RXBUF,
	DHD_PREALLOC_DATABUF,
	DHD_PREALLOC_OSL_BUF,
	DHD_PREALLOC_SKB_BUF,
	DHD_PREALLOC_WIPHY_ESCAN0 = 5,
	DHD_PREALLOC_WIPHY_ESCAN1 = 6,
	DHD_PREALLOC_DHD_INFO = 7,
	DHD_PREALLOC_DHD_WLFC_INFO = 8,
	DHD_PREALLOC_IF_FLOW_LKUP = 9,
	DHD_PREALLOC_FLOWRING = 10,
	DHD_PREALLOC_MAX
};

#define STATIC_BUF_MAX_NUM	20
#define STATIC_BUF_SIZE	(PAGE_SIZE*2)

#define DHD_PREALLOC_PROT_SIZE   	(512)
#define DHD_PREALLOC_WIPHY_ESCAN0_SIZE	(64 * 1024)
#define DHD_PREALLOC_DHD_INFO_SIZE		(24 * 1024)
#ifdef CONFIG_64BIT
#define DHD_PREALLOC_IF_FLOW_LKUP_SIZE	(20 * 1024 * 2)
#else
#define DHD_PREALLOC_IF_FLOW_LKUP_SIZE	(20 * 1024)
#endif
#define DHD_PREALLOC_OSL_BUF_SIZE      (STATIC_BUF_MAX_NUM * STATIC_BUF_SIZE)

#define WLAN_SCAN_BUF_SIZE		(64 * 1024)

#if defined(CONFIG_64BIT)
#define WLAN_DHD_INFO_BUF_SIZE		(24 * 1024)
#define WLAN_DHD_WLFC_BUF_SIZE		(64 * 1024)
#define WLAN_DHD_IF_FLOW_LKUP_SIZE	(64 * 1024)
#else
#define WLAN_DHD_INFO_BUF_SIZE		(16 * 1024)
#define WLAN_DHD_WLFC_BUF_SIZE		(16 * 1024)
#define WLAN_DHD_IF_FLOW_LKUP_SIZE	(20 * 1024)
#endif /* CONFIG_64BIT */
#define WLAN_DHD_MEMDUMP_SIZE		(800 * 1024)

#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24

#ifdef CONFIG_BCMDHD_PCIE
#define DHD_SKB_1PAGE_BUFSIZE	(PAGE_SIZE*1)
#define DHD_SKB_2PAGE_BUFSIZE	(PAGE_SIZE*2)
#define DHD_SKB_4PAGE_BUFSIZE	(PAGE_SIZE*4)

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	0
#define WLAN_SECTION_SIZE_2	0
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_1PAGE_BUF_NUM	0
#define DHD_SKB_2PAGE_BUF_NUM	64
#define DHD_SKB_4PAGE_BUF_NUM	0

#else
#define DHD_SKB_HDRSIZE		336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define DHD_SKB_1PAGE_BUF_NUM	8
#define DHD_SKB_2PAGE_BUF_NUM	8
#define DHD_SKB_4PAGE_BUF_NUM	1
#endif /* CONFIG_BCMDHD_PCIE */

#define WLAN_SKB_1_2PAGE_BUF_NUM	((DHD_SKB_1PAGE_BUF_NUM) + \
		(DHD_SKB_2PAGE_BUF_NUM))
#define WLAN_SKB_BUF_NUM	((WLAN_SKB_1_2PAGE_BUF_NUM) + \
		(DHD_SKB_4PAGE_BUF_NUM))


void *wlan_static_prot = NULL;
void *wlan_static_scan_buf0 = NULL;
void *wlan_static_scan_buf1 = NULL;
void *wlan_static_dhd_info_buf = NULL;
void *wlan_static_if_flow_lkup = NULL;
void *wlan_static_osl_buf = NULL;

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];


static void *dhd_wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == DHD_PREALLOC_PROT)
		return wlan_static_prot;

	if (section == DHD_PREALLOC_SKB_BUF)
		return wlan_static_skb;

	if (section == DHD_PREALLOC_WIPHY_ESCAN0)
		return wlan_static_scan_buf0;

	if (section == DHD_PREALLOC_WIPHY_ESCAN1)
		return wlan_static_scan_buf1;

	if (section == DHD_PREALLOC_OSL_BUF) {
		if (size > DHD_PREALLOC_OSL_BUF_SIZE) {
			pr_err("request OSL_BUF(%lu) is bigger than static size(%ld).\n",
				size, DHD_PREALLOC_OSL_BUF_SIZE);
			return NULL;
		}
		return wlan_static_osl_buf;
	}

	if (section == DHD_PREALLOC_DHD_INFO) {
		if (size > DHD_PREALLOC_DHD_INFO_SIZE) {
			pr_err("request DHD_INFO size(%lu) is bigger than static size(%d).\n",
				size, DHD_PREALLOC_DHD_INFO_SIZE);
			return NULL;
		}
		return wlan_static_dhd_info_buf;
	}
	if (section == DHD_PREALLOC_IF_FLOW_LKUP)  {
		if (size > DHD_PREALLOC_IF_FLOW_LKUP_SIZE) {
			pr_err("request DHD_IF_FLOW_LKUP size(%lu) is bigger than static size(%d).\n",
				size, DHD_PREALLOC_IF_FLOW_LKUP_SIZE);
			return NULL;
		}

		return wlan_static_if_flow_lkup;
	}
	if ((section < 0) || (section > DHD_PREALLOC_MAX))
		pr_err("request section id(%d) is out of max index %d\n",
				section, DHD_PREALLOC_MAX);

	return NULL;
}

static int dhd_init_wlan_mem(void)
{

	int i;
	int j;

	for (i = 0; i < DHD_SKB_1PAGE_BUF_NUM; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i]) {
			goto err_skb_alloc;
		}
	}

	for (i = DHD_SKB_1PAGE_BUF_NUM; i < WLAN_SKB_1_2PAGE_BUF_NUM; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i]) {
			goto err_skb_alloc;
		}
	}

#if !defined(CONFIG_BCMDHD_PCIE)
	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i]) {
		goto err_skb_alloc;
	}
#endif /* !CONFIG_BCMDHD_PCIE */

	wlan_static_prot = kmalloc(DHD_PREALLOC_PROT_SIZE, GFP_KERNEL);
	if (!wlan_static_prot) {
		pr_err("Failed to alloc wlan_static_prot\n");
		goto err_mem_alloc;
	}

	wlan_static_osl_buf = kmalloc(DHD_PREALLOC_OSL_BUF_SIZE, GFP_KERNEL);
	if (!wlan_static_osl_buf) {
		pr_err("Failed to alloc wlan_static_osl_buf\n");
		goto err_mem_alloc;
	}

	wlan_static_scan_buf0 = kmalloc(DHD_PREALLOC_WIPHY_ESCAN0_SIZE, GFP_KERNEL);
	if (!wlan_static_scan_buf0) {
		pr_err("Failed to alloc wlan_static_scan_buf0\n");
		goto err_mem_alloc;
	}


	wlan_static_dhd_info_buf = kmalloc(DHD_PREALLOC_DHD_INFO_SIZE, GFP_KERNEL);
	if (!wlan_static_dhd_info_buf) {
		pr_err("Failed to alloc wlan_static_dhd_info_buf\n");
		goto err_mem_alloc;
	}
#ifdef CONFIG_BCMDHD_PCIE
	wlan_static_if_flow_lkup = kmalloc(DHD_PREALLOC_IF_FLOW_LKUP_SIZE, GFP_KERNEL);
	if (!wlan_static_if_flow_lkup) {
		pr_err("Failed to alloc wlan_static_if_flow_lkup\n");
		goto err_mem_alloc;
	}
#endif /* CONFIG_BCMDHD_PCIE */

	return 0;

err_mem_alloc:

	if (wlan_static_prot)
		kfree(wlan_static_prot);

	if (wlan_static_dhd_info_buf)
		kfree(wlan_static_dhd_info_buf);

	if (wlan_static_scan_buf1)
		kfree(wlan_static_scan_buf1);

	if (wlan_static_scan_buf0)
		kfree(wlan_static_scan_buf0);

	if (wlan_static_osl_buf)
		kfree(wlan_static_osl_buf);

#ifdef CONFIG_BCMDHD_PCIE
	if (wlan_static_if_flow_lkup)
		kfree(wlan_static_if_flow_lkup);
#endif
	pr_err("Failed to mem_alloc for WLAN\n");

	i = WLAN_SKB_BUF_NUM;

err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0; j < i; j++) {
		dev_kfree_skb(wlan_static_skb[j]);
	}

	return -ENOMEM;
}
#endif /* CONFIG_DHD_USE_STATIC_BUF */

int dhd_wifi_init_gpio(void)
{
	int wl_reg_on;
	int wl_host_wake;
	struct device_node *np;

	np = of_find_compatible_node(NULL, NULL, DHD_DT_COMPAT_ENTRY);
	if (!np) {
		pr_warn("%s: BRCM WLAN mode not found\n", __func__);
		return -ENODEV;
	}

	/* get wlan_reg_on */
	wl_reg_on = of_get_named_gpio(np, "wl_reg_on", 0);
	if (wl_reg_on >= 0) {
		gpio_wl_reg_on = wl_reg_on;
		BCM_DBG("%s: gpio_wl_reg_on:%d.\n", __FUNCTION__, gpio_wl_reg_on);
	}

	/* get host_wake irq */
	wl_host_wake = of_get_named_gpio(np, "wl_host_wake", 0);
	if (wl_host_wake >= 0) {
#if !defined(CONFIG_BBRY)
		BCM_DBG("%s: wl_host_wake:%d.\n", __FUNCTION__, wl_host_wake);
#endif /* CONFIG_BBRY */
		brcm_wake_irq = gpio_to_irq(wl_host_wake);
#ifdef CONFIG_BBRY
		pr_info("%s: wl_host_wake %d, brcm_wake_irq %d\n",
			__FUNCTION__, wl_host_wake, brcm_wake_irq);
#endif /* CONFIG_BBRY */
	}

	if (gpio_request(gpio_wl_reg_on, "WL_REG_ON"))
		pr_err("%s: Faiiled to request gpio %d for WL_REG_ON\n",
			__func__, gpio_wl_reg_on);
	else
		pr_err("%s: gpio_request WL_REG_ON done\n", __func__);

	return 0;
}

int dhd_wlan_power(int on)
{
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
EXPORT_SYMBOL(dhd_wlan_power);

static int dhd_wlan_reset(int onoff)
{
	return 0;
}

static int dhd_wlan_set_carddetect(int val)
{
#if defined(CONFIG_BCMDHD_PCIE) && defined(CONFIG_ARCH_MSM) && defined(CONFIG_64BIT)
#if defined(CONFIG_BBRY) && defined(CONFIG_ARCH_MSM8992)
	pr_err("%s: enumerate RC0, %d\n", __func__, val);
	return msm_pcie_enumerate(0);
#else
	return msm_pcie_enumerate(1);
#endif /* CONFIG_BBRY && CONFIG_ARCH_MSM8992 */
#else
	return 0;
#endif /* CONFIG_BCMDHD_PCIE && defined(CONFIG_ARCH_MSM) && defined(CONFIG_64BIT) */

}

/* Customized Locale table : OPTIONAL feature */
#define WLC_CNTRY_BUF_SZ        4
struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int  custom_locale_rev;
};

struct country_tables {
	struct cntry_locales_custom *sta_dfs_table;
	size_t sta_dfs_size;
	struct cntry_locales_custom *sta_nodfs_table;
	size_t sta_nodfs_size;
	struct cntry_locales_custom *ap_table;
	size_t ap_size;
};

static struct cntry_locales_custom bcm4354_sdio_translate_sta_dfs_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XZ", 11},  /* Universal if Country code is unknown or empty */
	{"US", "US", 0},
	{"AE", "AE", 1},
	{"AR", "AR", 21},
	{"AT", "AT", 4},
	{"AU", "AU", 6},
	{"BE", "BE", 4},
	{"BG", "BG", 4},
	{"BN", "BN", 4},
	{"BR", "BR", 4},
	{"CA", "US", 0},   /* Previously was CA/31 */
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
	{"IN", "IN", 3},
	{"IS", "IS", 4},
	{"IT", "IT", 4},
	{"ID", "ID", 13},
	{"JP", "JP", 58},
	{"KR", "KR", 57},
	{"KW", "KW", 5},
	{"LI", "LI", 4},
	{"LT", "LT", 4},
	{"LU", "LU", 3},
	{"LV", "LV", 4},
	{"MA", "MA", 2},
	{"MT", "MT", 4},
	{"MX", "MX", 20},
	{"MY", "MY", 3},
	{"NL", "NL", 4},
	{"NO", "NO", 4},
	{"NZ", "NZ", 4},
	{"PL", "PL", 4},
	{"PT", "PT", 4},
	{"PY", "PY", 2},
	{"QA", "QA", 0},
	{"RO", "RO", 4},
	{"RU", "RU", 13},
	{"SA", "SA", 26},
	{"SE", "SE", 4},
	{"SG", "SG", 4},
	{"SI", "SI", 4},
	{"SK", "SK", 4},
	{"TH", "TH", 5},
	{"TR", "TR", 7},
	{"TW", "TW", 1},
	{"VN", "VN", 4},
	{"IR", "XZ", 11},
	{"SD", "XZ", 11},
	{"SY", "XZ", 11},
	{"GL", "XZ", 11},
	{"PS", "XZ", 11},
	{"TL", "XZ", 11},
	{"MH", "XZ", 11},
};

static struct cntry_locales_custom bcm4354_sdio_translate_sta_nodfs_table[] = {
	{"",   "XZ", 40},  /* Universal if Country code is unknown or empty */
	{"US", "US", 172},
	{"AM", "E0", 26},
	{"AU", "AU", 37},
	{"BG", "E0", 26},
	{"BR", "BR", 18},
	{"CA", "US", 172},
	{"CH", "E0", 26},
	{"CY", "E0", 26},
	{"CZ", "E0", 26},
	{"DE", "E0", 26},
	{"DK", "E0", 26},
	{"DZ", "E0", 26},
	{"EE", "E0", 26},
	{"ES", "E0", 26},
	{"EU", "E0", 26},
	{"FI", "E0", 26},
	{"FR", "E0", 26},
	{"GB", "E0", 26},
	{"GR", "E0", 26},
	{"HK", "SG", 17},
	{"HR", "E0", 26},
	{"HU", "E0", 26},
	{"ID", "ID", 1},
	{"IE", "E0", 26},
	{"IL", "E0", 26},
	{"IN", "IN", 27},
	{"IQ", "E0", 26},
	{"IS", "E0", 26},
	{"IT", "E0", 26},
	{"JP", "JP", 83},
	{"KR", "KR", 79},
	{"KW", "E0", 26},
	{"KZ", "E0", 26},
	{"LI", "E0", 26},
	{"LT", "E0", 26},
	{"LU", "E0", 26},
	{"LV", "LV", 4},
	{"LY", "E0", 26},
	{"MA", "E0", 26},
	{"MT", "E0", 26},
	{"MY", "MY", 15},
	{"MX", "US", 172},
	{"NL", "E0", 26},
	{"NO", "E0", 26},
	{"OM", "E0", 26},
	{"PL", "E0", 26},
	{"PT", "E0", 26},
	{"QA", "QA", 0},
	{"RO", "E0", 26},
	{"RS", "E0", 26},
	{"SA", "SA", 26},
	{"SE", "E0", 26},
	{"SG", "SG", 17},
	{"SI", "E0", 26},
	{"SK", "E0", 26},
	{"SZ", "E0", 26},
	{"TH", "TH", 9},
	{"TN", "E0", 26},
	{"TR", "E0", 26},
	{"TW", "TW", 60},
	{"ZA", "E0", 26},
};

#if defined(CONFIG_BBRY) && defined(CONFIG_BCM4356)
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
static struct cntry_locales_custom bcm4356_pcie_translate_sta_dfs_table[] = {
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
	{"CN", "CN", 24},
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
	{"ID", "ID", 5},
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

static struct cntry_locales_custom bcm4356_pcie_translate_sta_nodfs_table[] = {
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
	{"IE", "E0", 33},
	{"IN", "IN", 29},
	{"ID", "ID", 5},
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
#endif /* CONFIG_BBRY && CONFIG_BCM4356 */

static struct cntry_locales_custom bcm4356_pcie_translate_ap_table[] = {
	{"JP", "JP", 991},
};

static struct cntry_locales_custom bcm4358_pcie_translate_sta_dfs_table[] = {
	/* Table should be filled out based on custom platform regulatory requirement */
	{"",   "XT", 994},  /* Universal if Country code is unknown or empty */
	{"US", "US", 975},
	{"AE", "AE", 1},
	{"AR", "AR", 21},
	{"AT", "AT", 4},
	{"AU", "AU", 989},
	{"BE", "BE", 4},
	{"BG", "BG", 4},
	{"BN", "BN", 4},
	{"BR", "BR", 4},
	{"CA", "US", 975},
	{"CH", "E0", 961},
	{"CN", "CN", 24},
	{"CO", "CO", 17},
	{"CY", "E0", 961},
	{"CZ", "E0", 961},
	{"DE", "E0", 961},
	{"DK", "E0", 961},
	{"EE", "E0", 961},
	{"ES", "E0", 961},
	{"EU", "E0", 961},
	{"FI", "E0", 961},
	{"FR", "E0", 961},
	{"GB", "E0", 961},
	{"GR", "E0", 961},
	{"HK", "HK", 2},
	{"HR", "E0", 961},
	{"HU", "E0", 961},
	{"ID", "ID", 5},
	{"IE", "E0", 961},
	{"IN", "IN", 998},
	{"IS", "E0", 961},
	{"IT", "E0", 961},
	{"ID", "ID", 5},
	{"JP", "JP", 989},
	{"KR", "KR", 984},
	{"KW", "KW", 5},
	{"LI", "E0", 961},
	{"LT", "E0", 961},
	{"LU", "E0", 961},
	{"LV", "LV", 4},
	{"MA", "MA", 2},
	{"MT", "E0", 961},
	{"MX", "US", 975},
	{"MY", "MY", 19},
	{"NL", "E0", 961},
	{"NO", "E0", 961},
	{"NZ", "NZ", 4},
	{"PL", "E0", 961},
	{"PR", "PR", 20},
	{"PT", "E0", 961},
	{"PY", "PY", 2},
	{"RO", "E0", 961},
	{"RU", "RU", 62},
	{"SA", "SA", 5},
	{"SE", "E0", 961},
	{"SG", "SG", 996},
	{"SI", "E0", 961},
	{"SK", "E0", 961},
	{"SZ", "E0", 961},
	{"TH", "TH", 5},
	{"TR", "TR", 7},
	{"TW", "TW", 992},
	{"VN", "VN", 4},
	{"ZA", "ZA", 6},
};

static struct cntry_locales_custom bcm4358_pcie_translate_sta_nodfs_table[] = {
	{"",   "XT", 995},  /* Universal if Country code is unknown or empty */
	{"US", "US", 976},
	{"AT", "AT", 16},
	{"AU", "AU", 988},
	{"BE", "BE", 15},
	{"BR", "BR", 18},
	{"CA", "CA", 78},
	{"CH", "E0", 960},
	{"CO", "CO", 40},
	{"CY", "E0", 960},
	{"CZ", "E0", 960},
	{"DE", "E0", 960},
	{"DK", "E0", 960},
	{"EE", "E0", 960},
	{"ES", "E0", 960},
	{"EU", "E0", 960},
	{"FI", "E0", 960},
	{"FR", "E0", 960},
	{"GB", "E0", 960},
	{"GR", "E0", 960},
	{"HK", "HK", 7},
	{"HR", "E0", 960},
	{"HU", "E0", 960},
	{"ID", "ID", 28},
	{"IE", "E0", 960},
	{"IN", "IN", 997},
	{"IS", "E0", 960},
	{"IT", "E0", 960},
	{"JP", "JP", 987},
	{"KR", "KR", 983},
	{"KW", "KW", 6},
	{"LI", "E0", 960},
	{"LT", "E0", 960},
	{"LU", "E0", 960},
	{"LV", "LV", 4},
	{"MA", "MA", 2},
	{"MT", "E0", 960},
	{"MY", "MY", 20},
	{"MX", "US", 976},
	{"NL", "E0", 960},
	{"NO", "E0", 960},
	{"NZ", "NZ", 10},
	{"PL", "E0", 960},
	{"PR", "PR", 39},
	{"PT", "E0", 960},
	{"RO", "E0", 960},
	{"RU", "RU", 63},
	{"SA", "SA", 27},
	{"SE", "E0", 960},
	{"SG", "SG", 995},
	{"SI", "E0", 960},
	{"SK", "E0", 960},
	{"SZ", "E0", 960},
	{"TH", "TH", 9},
	{"TR", "TR", 16},
	{"TW", "TW", 991},
	{"ZA", "ZA", 15},
};

static struct cntry_locales_custom bcm4358_pcie_translate_ap_table[] = {
	{"", "XT", 993},
	{"JP", "JP", 988},
};

static struct country_tables dhd_country_tables = {
	.sta_dfs_table = bcm4358_pcie_translate_sta_dfs_table,
	.sta_dfs_size = ARRAY_SIZE(bcm4358_pcie_translate_sta_dfs_table),
	.sta_nodfs_table = bcm4358_pcie_translate_sta_nodfs_table,
	.sta_nodfs_size = ARRAY_SIZE(bcm4358_pcie_translate_sta_nodfs_table),
	.ap_table = bcm4358_pcie_translate_ap_table,
	.ap_size = ARRAY_SIZE(bcm4358_pcie_translate_ap_table),
};

static void *dhd_wlan_get_country_code(char *ccode, u32 flags)
{
	struct cntry_locales_custom *locales;
	int size;
	int i;

	if (!ccode)
		return NULL;

	if (flags & WLAN_PLAT_AP_FLAG) {
		locales = dhd_country_tables.ap_table;
		size = dhd_country_tables.ap_size;
#if defined(CONFIG_BBRY) && defined(CONFIG_BCM4356)
		/* instrument code for _AP_FLAG case */
		for (i = 0; i < size; i++) {
			if (strcmp(ccode, locales[i].iso_abbrev) == 0) {
				pr_info("%s: _AP_FLAG with ccode %s, locale matched\n", __func__, ccode);
				return &locales[i];
			}
		}
		pr_info("%s: _AP_FLAG with ccode %s, no locale matched\n", __func__, ccode);
#else
		for (i = 0; i < size; i++)
			if (strcmp(ccode, locales[i].iso_abbrev) == 0)
				return &locales[i];
#endif /* CONFIG_BBRY && CONFIG_BCM4356 */
	}

	if (flags & WLAN_PLAT_NODFS_FLAG) {
		locales = dhd_country_tables.sta_nodfs_table;
		size = dhd_country_tables.sta_nodfs_size;
	} else {
		locales = dhd_country_tables.sta_dfs_table;
		size = dhd_country_tables.sta_dfs_size;
	}

	for (i = 0; i < size; i++)
		if (strcmp(ccode, locales[i].iso_abbrev) == 0)
			return &locales[i];
	return &locales[0];
}

static int dhd_wifi_init_country(void)
{
	struct device_node *np;
	const char *chip_name;

	np = of_find_compatible_node(NULL, NULL, DHD_DT_COMPAT_ENTRY);
	if (!np) {
		pr_warn("%s: BRCM WLAN node not found\n", __func__);
		return -ENODEV;
	}
	if (of_property_read_string(np, "wl_chip", &chip_name)) {
		pr_warn("%s: BRCM WLAN chip name not found\n", __func__);
		return -ENODEV;
	}
	if (strcmp(chip_name, "bcm4354_sdio") == 0) {
		dhd_country_tables.sta_dfs_table =
			bcm4354_sdio_translate_sta_dfs_table;
		dhd_country_tables.sta_dfs_size =
			ARRAY_SIZE(bcm4354_sdio_translate_sta_dfs_table);
		dhd_country_tables.sta_nodfs_table =
			bcm4354_sdio_translate_sta_nodfs_table;
		dhd_country_tables.sta_nodfs_size =
			ARRAY_SIZE(bcm4354_sdio_translate_sta_nodfs_table);
		dhd_country_tables.ap_table = NULL;
		dhd_country_tables.ap_size = 0;
	} else if (strcmp(chip_name, "bcm4356_pcie") == 0) {
#if defined(CONFIG_BBRY) && defined(CONFIG_BCM4356)
		dhd_country_tables.sta_dfs_table =
			bbry_wlan_translate_custom_table;
		dhd_country_tables.sta_dfs_size =
			ARRAY_SIZE(bbry_wlan_translate_custom_table);
		dhd_country_tables.sta_nodfs_table =
			bbry_wlan_translate_custom_table;
		dhd_country_tables.sta_nodfs_size =
			ARRAY_SIZE(bbry_wlan_translate_custom_table);
#else
		dhd_country_tables.sta_dfs_table =
			bcm4356_pcie_translate_sta_dfs_table;
		dhd_country_tables.sta_dfs_size =
			ARRAY_SIZE(bcm4356_pcie_translate_sta_dfs_table);
		dhd_country_tables.sta_nodfs_table =
			bcm4356_pcie_translate_sta_nodfs_table;
		dhd_country_tables.sta_nodfs_size =
			ARRAY_SIZE(bcm4356_pcie_translate_sta_nodfs_table);
#endif /* CONFIG_BBRY && CONFIG_BCM4356 */
		dhd_country_tables.ap_table =
			bcm4356_pcie_translate_ap_table;
		dhd_country_tables.ap_size =
			ARRAY_SIZE(bcm4356_pcie_translate_ap_table);
	}
	return 0;
}

static unsigned char brcm_mac_addr[IFHWADDRLEN] = { 0, 0x90, 0x4c, 0, 0, 0 };

#ifndef CONFIG_BBRY
static int __init dhd_mac_addr_setup(char *str)
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

	while (((token = strsep(&macptr, ":")) != NULL) && (i < IFHWADDRLEN)) {
		unsigned long val;
		int res;

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

__setup("androidboot.wifimacaddr=", dhd_mac_addr_setup);

static int dhd_wifi_get_mac_addr(unsigned char *buf)
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

static int dhd_wifi_get_mac_addr(unsigned char *buf)
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

static int dhd_wlan_get_wake_irq(void)
{
	return brcm_wake_irq;
}

struct resource dhd_wlan_resources[] = {
	[0] = {
		.name	= "bcmdhd_wlan_irq",
		.start	= 0, /* Dummy */
		.end	= 0, /* Dummy */
		.flags	= IORESOURCE_IRQ | IORESOURCE_IRQ_SHAREABLE
#ifdef CONFIG_BBRY
			| IORESOURCE_IRQ_HIGHLEVEL, /* Dummy */
#else
			| IORESOURCE_IRQ_HIGHEDGE, /* IRQF_TRIGGER_RISING */
#endif /* CONFIG_BBRY */
	},
};
EXPORT_SYMBOL(dhd_wlan_resources);


struct wifi_platform_data dhd_wlan_control = {
	.set_power	= dhd_wlan_power,
	.set_reset	= dhd_wlan_reset,
	.set_carddetect	= dhd_wlan_set_carddetect,
	.get_mac_addr	= dhd_wifi_get_mac_addr,
#ifdef CONFIG_DHD_USE_STATIC_BUF
	.mem_prealloc	= dhd_wlan_mem_prealloc,
#endif
	.get_wake_irq	= dhd_wlan_get_wake_irq,
	.get_country_code = dhd_wlan_get_country_code,
};
EXPORT_SYMBOL(dhd_wlan_control);

int __init dhd_wlan_init(void)
{
	int ret;

	printk(KERN_INFO "%s: START\n", __FUNCTION__);

#ifdef CONFIG_DHD_USE_STATIC_BUF
	ret = dhd_init_wlan_mem();
#endif

	dhd_wifi_init_country();
	ret = dhd_wifi_init_gpio();
	dhd_wlan_resources[0].start = dhd_wlan_resources[0].end =
		brcm_wake_irq;

	return ret;
}

void __exit dhd_wlan_exit(void)
{
#ifdef CONFIG_DHD_USE_STATIC_BUF
	int i;

	for (i = 0; i < DHD_SKB_1PAGE_BUF_NUM; i++) {
		if (wlan_static_skb[i])
			dev_kfree_skb(wlan_static_skb[i]);
	}

	for (i = DHD_SKB_1PAGE_BUF_NUM; i < WLAN_SKB_1_2PAGE_BUF_NUM; i++) {
		if (wlan_static_skb[i])
			dev_kfree_skb(wlan_static_skb[i]);
	}

#if !defined(CONFIG_BCMDHD_PCIE)
	if (wlan_static_skb[i])
		dev_kfree_skb(wlan_static_skb[i]);
#endif /* !CONFIG_BCMDHD_PCIE */

	if (wlan_static_prot)
		kfree(wlan_static_prot);

	if (wlan_static_osl_buf)
		kfree(wlan_static_osl_buf);

	if (wlan_static_scan_buf0)
		kfree(wlan_static_scan_buf0);

	if (wlan_static_dhd_info_buf)
		kfree(wlan_static_dhd_info_buf);

	if (wlan_static_scan_buf1)
		kfree(wlan_static_scan_buf1);

#ifdef CONFIG_BCMDHD_PCIE
	if (wlan_static_if_flow_lkup)
		kfree(wlan_static_if_flow_lkup);
#endif
#endif /* CONFIG_BCMDHD_USE_STATIC_BUF */
	return;
}
