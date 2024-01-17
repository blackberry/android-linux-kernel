/*
 * Copyright (c) 2014-2016, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/pinctrl.h>

#include "pinctrl-msm.h"

#define FUNCTION(fname)					\
	[msm_mux_##fname] = {		                \
		.name = #fname,				\
		.groups = fname##_groups,               \
		.ngroups = ARRAY_SIZE(fname##_groups),	\
	}

#define REG_BASE 0x0
#define REG_SIZE 0x1000
#define PINGROUP(id, f1, f2, f3, f4, f5, f6, f7, f8, f9)	\
	{					        \
		.name = "gpio" #id,			\
		.pins = gpio##id##_pins,		\
		.npins = (unsigned)ARRAY_SIZE(gpio##id##_pins),	\
		.funcs = (int[]){			\
			msm_mux_NA, /* gpio mode */	\
			msm_mux_##f1,			\
			msm_mux_##f2,			\
			msm_mux_##f3,			\
			msm_mux_##f4,			\
			msm_mux_##f5,			\
			msm_mux_##f6,			\
			msm_mux_##f7,			\
			msm_mux_##f8,			\
			msm_mux_##f9			\
		},				        \
		.nfuncs = 10,				\
		.ctl_reg = REG_BASE + REG_SIZE*id,			\
		.io_reg = REG_BASE + 0x4 + REG_SIZE * id,		\
		.intr_cfg_reg = REG_BASE + 0x8 + REG_SIZE * id,		\
		.intr_status_reg = REG_BASE + 0xc + REG_SIZE * id,	\
		.intr_target_reg = REG_BASE + 0x8 + REG_SIZE * id,	\
		.mux_bit = 2,			\
		.pull_bit = 0,			\
		.drv_bit = 6,			\
		.oe_bit = 9,			\
		.in_bit = 0,			\
		.out_bit = 1,			\
		.intr_enable_bit = 0,		\
		.intr_status_bit = 0,		\
		.intr_target_bit = 5,		\
		.intr_target_kpss_val = 4,	\
		.intr_raw_status_bit = 4,	\
		.intr_polarity_bit = 1,		\
		.intr_detection_bit = 2,	\
		.intr_detection_width = 2,	\
	}

#define SDC_QDSD_PINGROUP(pg_name, ctl, pull, drv)	\
	{					        \
		.name = #pg_name,			\
		.pins = pg_name##_pins,			\
		.npins = (unsigned)ARRAY_SIZE(pg_name##_pins),	\
		.ctl_reg = ctl,				\
		.io_reg = 0,				\
		.intr_cfg_reg = 0,			\
		.intr_status_reg = 0,			\
		.intr_target_reg = 0,			\
		.mux_bit = -1,				\
		.pull_bit = pull,			\
		.drv_bit = drv,				\
		.oe_bit = -1,				\
		.in_bit = -1,				\
		.out_bit = -1,				\
		.intr_enable_bit = -1,			\
		.intr_status_bit = -1,			\
		.intr_target_bit = -1,			\
		.intr_raw_status_bit = -1,		\
		.intr_polarity_bit = -1,		\
		.intr_detection_bit = -1,		\
		.intr_detection_width = -1,		\
	}
static const struct pinctrl_pin_desc mdm9640_pins[] = {
	PINCTRL_PIN(0, "GPIO_0"),
	PINCTRL_PIN(1, "GPIO_1"),
	PINCTRL_PIN(2, "GPIO_2"),
	PINCTRL_PIN(3, "GPIO_3"),
	PINCTRL_PIN(4, "GPIO_4"),
	PINCTRL_PIN(5, "GPIO_5"),
	PINCTRL_PIN(6, "GPIO_6"),
	PINCTRL_PIN(7, "GPIO_7"),
	PINCTRL_PIN(8, "GPIO_8"),
	PINCTRL_PIN(9, "GPIO_9"),
	PINCTRL_PIN(10, "GPIO_10"),
	PINCTRL_PIN(11, "GPIO_11"),
	PINCTRL_PIN(12, "GPIO_12"),
	PINCTRL_PIN(13, "GPIO_13"),
	PINCTRL_PIN(14, "GPIO_14"),
	PINCTRL_PIN(15, "GPIO_15"),
	PINCTRL_PIN(16, "GPIO_16"),
	PINCTRL_PIN(17, "GPIO_17"),
	PINCTRL_PIN(18, "GPIO_18"),
	PINCTRL_PIN(19, "GPIO_19"),
	PINCTRL_PIN(20, "GPIO_20"),
	PINCTRL_PIN(21, "GPIO_21"),
	PINCTRL_PIN(22, "GPIO_22"),
	PINCTRL_PIN(23, "GPIO_23"),
	PINCTRL_PIN(24, "GPIO_24"),
	PINCTRL_PIN(25, "GPIO_25"),
	PINCTRL_PIN(26, "GPIO_26"),
	PINCTRL_PIN(27, "GPIO_27"),
	PINCTRL_PIN(28, "GPIO_28"),
	PINCTRL_PIN(29, "GPIO_29"),
	PINCTRL_PIN(30, "GPIO_30"),
	PINCTRL_PIN(31, "GPIO_31"),
	PINCTRL_PIN(32, "GPIO_32"),
	PINCTRL_PIN(33, "GPIO_33"),
	PINCTRL_PIN(34, "GPIO_34"),
	PINCTRL_PIN(35, "GPIO_35"),
	PINCTRL_PIN(36, "GPIO_36"),
	PINCTRL_PIN(37, "GPIO_37"),
	PINCTRL_PIN(38, "GPIO_38"),
	PINCTRL_PIN(39, "GPIO_39"),
	PINCTRL_PIN(40, "GPIO_40"),
	PINCTRL_PIN(41, "GPIO_41"),
	PINCTRL_PIN(42, "GPIO_42"),
	PINCTRL_PIN(43, "GPIO_43"),
	PINCTRL_PIN(44, "GPIO_44"),
	PINCTRL_PIN(45, "GPIO_45"),
	PINCTRL_PIN(46, "GPIO_46"),
	PINCTRL_PIN(47, "GPIO_47"),
	PINCTRL_PIN(48, "GPIO_48"),
	PINCTRL_PIN(49, "GPIO_49"),
	PINCTRL_PIN(50, "GPIO_50"),
	PINCTRL_PIN(51, "GPIO_51"),
	PINCTRL_PIN(52, "GPIO_52"),
	PINCTRL_PIN(53, "GPIO_53"),
	PINCTRL_PIN(54, "GPIO_54"),
	PINCTRL_PIN(55, "GPIO_55"),
	PINCTRL_PIN(56, "GPIO_56"),
	PINCTRL_PIN(57, "GPIO_57"),
	PINCTRL_PIN(58, "GPIO_58"),
	PINCTRL_PIN(59, "GPIO_59"),
	PINCTRL_PIN(60, "GPIO_60"),
	PINCTRL_PIN(61, "GPIO_61"),
	PINCTRL_PIN(62, "GPIO_62"),
	PINCTRL_PIN(63, "GPIO_63"),
	PINCTRL_PIN(64, "GPIO_64"),
	PINCTRL_PIN(65, "GPIO_65"),
	PINCTRL_PIN(66, "GPIO_66"),
	PINCTRL_PIN(67, "GPIO_67"),
	PINCTRL_PIN(68, "GPIO_68"),
	PINCTRL_PIN(69, "GPIO_69"),
	PINCTRL_PIN(70, "GPIO_70"),
	PINCTRL_PIN(71, "GPIO_71"),
	PINCTRL_PIN(72, "GPIO_72"),
	PINCTRL_PIN(73, "GPIO_73"),
	PINCTRL_PIN(74, "GPIO_74"),
	PINCTRL_PIN(75, "GPIO_75"),
	PINCTRL_PIN(76, "GPIO_76"),
	PINCTRL_PIN(77, "GPIO_77"),
	PINCTRL_PIN(78, "GPIO_78"),
	PINCTRL_PIN(79, "GPIO_79"),
	PINCTRL_PIN(80, "GPIO_80"),
	PINCTRL_PIN(81, "GPIO_81"),
	PINCTRL_PIN(82, "GPIO_82"),
	PINCTRL_PIN(83, "GPIO_83"),
	PINCTRL_PIN(84, "GPIO_84"),
	PINCTRL_PIN(85, "GPIO_85"),
	PINCTRL_PIN(86, "GPIO_86"),
	PINCTRL_PIN(87, "GPIO_87"),
	PINCTRL_PIN(88, "GPIO_88"),
	PINCTRL_PIN(89, "GPIO_89"),
	PINCTRL_PIN(90, "GPIO_90"),
	PINCTRL_PIN(91, "GPIO_91"),
	PINCTRL_PIN(92, "GPIO_92"),
	PINCTRL_PIN(93, "GPIO_93"),
	PINCTRL_PIN(94, "GPIO_94"),
	PINCTRL_PIN(95, "GPIO_95"),
	PINCTRL_PIN(96, "GPIO_96"),
	PINCTRL_PIN(97, "GPIO_97"),
	PINCTRL_PIN(98, "GPIO_98"),
	PINCTRL_PIN(99, "GPIO_99"),
	PINCTRL_PIN(100, "SDC1_CLK"),
	PINCTRL_PIN(101, "SDC1_CMD"),
	PINCTRL_PIN(102, "SDC1_DATA"),
	PINCTRL_PIN(103, "SDC2_CLK"),
	PINCTRL_PIN(104, "SDC2_CMD"),
	PINCTRL_PIN(105, "SDC2_DATA"),
	PINCTRL_PIN(106, "QDSD_CLK"),
	PINCTRL_PIN(107, "QDSD_CMD"),
	PINCTRL_PIN(108, "QDSD_DATA0"),
	PINCTRL_PIN(109, "QDSD_DATA1"),
	PINCTRL_PIN(110, "QDSD_DATA2"),
	PINCTRL_PIN(111, "QDSD_DATA3"),
};

#define DECLARE_MSM_GPIO_PINS(pin) \
	static const unsigned int gpio##pin##_pins[] = { pin }
DECLARE_MSM_GPIO_PINS(0);
DECLARE_MSM_GPIO_PINS(1);
DECLARE_MSM_GPIO_PINS(2);
DECLARE_MSM_GPIO_PINS(3);
DECLARE_MSM_GPIO_PINS(4);
DECLARE_MSM_GPIO_PINS(5);
DECLARE_MSM_GPIO_PINS(6);
DECLARE_MSM_GPIO_PINS(7);
DECLARE_MSM_GPIO_PINS(8);
DECLARE_MSM_GPIO_PINS(9);
DECLARE_MSM_GPIO_PINS(10);
DECLARE_MSM_GPIO_PINS(11);
DECLARE_MSM_GPIO_PINS(12);
DECLARE_MSM_GPIO_PINS(13);
DECLARE_MSM_GPIO_PINS(14);
DECLARE_MSM_GPIO_PINS(15);
DECLARE_MSM_GPIO_PINS(16);
DECLARE_MSM_GPIO_PINS(17);
DECLARE_MSM_GPIO_PINS(18);
DECLARE_MSM_GPIO_PINS(19);
DECLARE_MSM_GPIO_PINS(20);
DECLARE_MSM_GPIO_PINS(21);
DECLARE_MSM_GPIO_PINS(22);
DECLARE_MSM_GPIO_PINS(23);
DECLARE_MSM_GPIO_PINS(24);
DECLARE_MSM_GPIO_PINS(25);
DECLARE_MSM_GPIO_PINS(26);
DECLARE_MSM_GPIO_PINS(27);
DECLARE_MSM_GPIO_PINS(28);
DECLARE_MSM_GPIO_PINS(29);
DECLARE_MSM_GPIO_PINS(30);
DECLARE_MSM_GPIO_PINS(31);
DECLARE_MSM_GPIO_PINS(32);
DECLARE_MSM_GPIO_PINS(33);
DECLARE_MSM_GPIO_PINS(34);
DECLARE_MSM_GPIO_PINS(35);
DECLARE_MSM_GPIO_PINS(36);
DECLARE_MSM_GPIO_PINS(37);
DECLARE_MSM_GPIO_PINS(38);
DECLARE_MSM_GPIO_PINS(39);
DECLARE_MSM_GPIO_PINS(40);
DECLARE_MSM_GPIO_PINS(41);
DECLARE_MSM_GPIO_PINS(42);
DECLARE_MSM_GPIO_PINS(43);
DECLARE_MSM_GPIO_PINS(44);
DECLARE_MSM_GPIO_PINS(45);
DECLARE_MSM_GPIO_PINS(46);
DECLARE_MSM_GPIO_PINS(47);
DECLARE_MSM_GPIO_PINS(48);
DECLARE_MSM_GPIO_PINS(49);
DECLARE_MSM_GPIO_PINS(50);
DECLARE_MSM_GPIO_PINS(51);
DECLARE_MSM_GPIO_PINS(52);
DECLARE_MSM_GPIO_PINS(53);
DECLARE_MSM_GPIO_PINS(54);
DECLARE_MSM_GPIO_PINS(55);
DECLARE_MSM_GPIO_PINS(56);
DECLARE_MSM_GPIO_PINS(57);
DECLARE_MSM_GPIO_PINS(58);
DECLARE_MSM_GPIO_PINS(59);
DECLARE_MSM_GPIO_PINS(60);
DECLARE_MSM_GPIO_PINS(61);
DECLARE_MSM_GPIO_PINS(62);
DECLARE_MSM_GPIO_PINS(63);
DECLARE_MSM_GPIO_PINS(64);
DECLARE_MSM_GPIO_PINS(65);
DECLARE_MSM_GPIO_PINS(66);
DECLARE_MSM_GPIO_PINS(67);
DECLARE_MSM_GPIO_PINS(68);
DECLARE_MSM_GPIO_PINS(69);
DECLARE_MSM_GPIO_PINS(70);
DECLARE_MSM_GPIO_PINS(71);
DECLARE_MSM_GPIO_PINS(72);
DECLARE_MSM_GPIO_PINS(73);
DECLARE_MSM_GPIO_PINS(74);
DECLARE_MSM_GPIO_PINS(75);
DECLARE_MSM_GPIO_PINS(76);
DECLARE_MSM_GPIO_PINS(77);
DECLARE_MSM_GPIO_PINS(78);
DECLARE_MSM_GPIO_PINS(79);
DECLARE_MSM_GPIO_PINS(80);
DECLARE_MSM_GPIO_PINS(81);
DECLARE_MSM_GPIO_PINS(82);
DECLARE_MSM_GPIO_PINS(83);
DECLARE_MSM_GPIO_PINS(84);
DECLARE_MSM_GPIO_PINS(85);
DECLARE_MSM_GPIO_PINS(86);
DECLARE_MSM_GPIO_PINS(87);
DECLARE_MSM_GPIO_PINS(88);
DECLARE_MSM_GPIO_PINS(89);
DECLARE_MSM_GPIO_PINS(90);
DECLARE_MSM_GPIO_PINS(91);
DECLARE_MSM_GPIO_PINS(92);
DECLARE_MSM_GPIO_PINS(93);
DECLARE_MSM_GPIO_PINS(94);
DECLARE_MSM_GPIO_PINS(95);
DECLARE_MSM_GPIO_PINS(96);
DECLARE_MSM_GPIO_PINS(97);
DECLARE_MSM_GPIO_PINS(98);
DECLARE_MSM_GPIO_PINS(99);

static const unsigned int sdc1_clk_pins[] = { 100 };
static const unsigned int sdc1_cmd_pins[] = { 101 };
static const unsigned int sdc1_data_pins[] = { 102 };
static const unsigned int sdc2_clk_pins[] = { 103 };
static const unsigned int sdc2_cmd_pins[] = { 104 };
static const unsigned int sdc2_data_pins[] = { 105 };
static const unsigned int qdsd_clk_pins[] = { 106 };
static const unsigned int qdsd_cmd_pins[] = { 107 };
static const unsigned int qdsd_data0_pins[] = { 108 };
static const unsigned int qdsd_data1_pins[] = { 109 };
static const unsigned int qdsd_data2_pins[] = { 110 };
static const unsigned int qdsd_data3_pins[] = { 111 };

enum mdm9640_functions {
	msm_mux_uim2_data,
	msm_mux_,
	msm_mux_blsp_uart_tx_a1,
	msm_mux_qdss_stm31,
	msm_mux_ebi0_wrcdc,
	msm_mux_native_etdac1,
	msm_mux_uim2_present,
	msm_mux_blsp_uart_rx_a1,
	msm_mux_blsp_spi1,
	msm_mux_qdss_stm30,
	msm_mux_native_etdac0,
	msm_mux_uim2_reset,
	msm_mux_blsp_uart_cts_n_a1,
	msm_mux_blsp_i2c_sda_a1,
	msm_mux_qdss_stm29,
	msm_mux_uim2_clk,
	msm_mux_blsp_uart_rfr_n_a1,
	msm_mux_blsp_i2c_scl_a1,
	msm_mux_qdss_stm28,
	msm_mux_blsp_spi2,
	msm_mux_blsp_uart2,
	msm_mux_qdss_stm23,
	msm_mux_qdss_tracedata_a,
	msm_mux_native_dac4,
	msm_mux_qdss_stm22,
	msm_mux_native_dac5,
	msm_mux_blsp_i2c_sda_a2,
	msm_mux_ldo_en,
	msm_mux_qdss_stm21,
	msm_mux_native_dac6,
	msm_mux_blsp_i2c_scl_a2,
	msm_mux_ldo_update,
	msm_mux_qdss_stm20,
	msm_mux_native_dac7,
	msm_mux_pri_mi2s_ws_b,
	msm_mux_blsp_spi3,
	msm_mux_blsp_uart3,
	msm_mux_qdss_cti_trig1_out_b,
	msm_mux_pwr_modem,
	msm_mux_pri_mi2s_data0_b,
	msm_mux_qdss_cti_trig1_in_b,
	msm_mux_pwr_nav,
	msm_mux_pri_mi2s_data1_b,
	msm_mux_blsp_i2c3,
	msm_mux_pwr_crypto,
	msm_mux_pri_mi2s_sck_b,
	msm_mux_pri_mi2s_ws_a,
	msm_mux_blsp_uart_tx_a4,
	msm_mux_qdss_stm19,
	msm_mux_pri_mi2s_data0_a,
	msm_mux_blsp_uart_rx_a4,
	msm_mux_qdss_stm18,
	msm_mux_pri_mi2s_data1_a,
	msm_mux_blsp_uart_cts_n_a4,
	msm_mux_blsp_i2c_sda_a4,
	msm_mux_slimbus_data,
	msm_mux_qdss_stm17,
	msm_mux_bimc_dte0,
	msm_mux_native_tsens,
	msm_mux_pri_mi2s_sck_a,
	msm_mux_blsp_uart_rfr_n_a4,
	msm_mux_blsp_i2c_scl_a4,
	msm_mux_slimbus_clk,
	msm_mux_qdss_stm16,
	msm_mux_bimc_dte1,
	msm_mux_sec_mi2s_ws_a,
	msm_mux_blsp_spi4,
	msm_mux_blsp_uart_tx_b4,
	msm_mux_uim3_data,
	msm_mux_qdss_stm27,
	msm_mux_qdss_tracedata_b,
	msm_mux_native_txdac1,
	msm_mux_sec_mi2s_data0_a,
	msm_mux_blsp_uart_rx_b4,
	msm_mux_uim3_present,
	msm_mux_qdss_stm26,
	msm_mux_sec_mi2s_data1_a,
	msm_mux_blsp_uart_cts_n_b4,
	msm_mux_blsp_i2c_sda_b4,
	msm_mux_uim3_reset,
	msm_mux_qdss_stm25,
	msm_mux_sec_mi2s_sck_a,
	msm_mux_blsp_uart_rfr_n_b4,
	msm_mux_blsp_i2c_scl_b4,
	msm_mux_uim3_clk,
	msm_mux_qdss_stm24,
	msm_mux_sec_mi2s_ws_b,
	msm_mux_ebi2_a,
	msm_mux_blsp_uart_tx_b1,
	msm_mux_uim4_data,
	msm_mux_sec_mi2s_data0_b,
	msm_mux_ebi2_lcd,
	msm_mux_blsp_uart_rx_b1,
	msm_mux_uim4_present,
	msm_mux_native_txdac0,
	msm_mux_sec_mi2s_data1_b,
	msm_mux_blsp_uart_cts_n_b1,
	msm_mux_uim4_reset,
	msm_mux_sec_mi2s_sck_b,
	msm_mux_blsp_uart_rfr_n_b1,
	msm_mux_uim4_clk,
	msm_mux_m_voc_ext_vfr_ref_irq_a,
	msm_mux_adsp_ext_vfr_irq_a,
	msm_mux_qdss_stm11,
	msm_mux_epm1,
	msm_mux_m_voc_ext_vfr_ref_irq_2_a,
	msm_mux_native_char0,
	msm_mux_pa_indicator,
	msm_mux_qdss_traceclk_a,
	msm_mux_modem_tsync,
	msm_mux_nav_dr,
	msm_mux_adsp_ext_vfr_irq_b,
	msm_mux_qdss_stm10,
	msm_mux_native_char,
	msm_mux_antpair_wlan,
	msm_mux_native_char3,
	msm_mux_native_char2,
	msm_mux_native_char1,
	msm_mux_nav_pps_in_a,
	msm_mux_native_dac3,
	msm_mux_gsm_tx1,
	msm_mux_cri_trng1,
	msm_mux_gsm_tx2,
	msm_mux_native_dac,
	msm_mux_cri_trng0,
	msm_mux_cri_trng,
	msm_mux_ssbi2,
	msm_mux_ssbi1,
	msm_mux_prng_rosc,
	msm_mux_native_bbrx1,
	msm_mux_blsp_i2c_sda_b2,
	msm_mux_native_bbrx0,
	msm_mux_blsp_i2c_scl_b2,
	msm_mux_native_gpsadc,
	msm_mux_qdss_tracectl_a,
	msm_mux_native_dac0,
	msm_mux_epm2,
	msm_mux_nav_pps_in_b,
	msm_mux_native_dac1,
	msm_mux_coex_uart,
	msm_mux_qdss_cti,
	msm_mux_qdss_traceclk_b,
	msm_mux_native_dac2,
	msm_mux_qdss_tracectl_b,
	msm_mux_ap2mdm_status,
	msm_mux_mdm2ap_status,
	msm_mux_ap2mdm_err,
	msm_mux_mdm2ap_err,
	msm_mux_qdss_stm15,
	msm_mux_ap2mdm_vdd,
	msm_mux_qdss_stm14,
	msm_mux_native_dac8,
	msm_mux_mdm2ap_vdd,
	msm_mux_qdss_stm13,
	msm_mux_native_dac9,
	msm_mux_ap2mdm_wake,
	msm_mux_m_voc_ext_vfr_ref_irq_2_b,
	msm_mux_qdss_stm12,
	msm_mux_native_dac10,
	msm_mux_pciehost_rst,
	msm_mux_pci_e,
	msm_mux_qdss_stm9,
	msm_mux_native_dac11,
	msm_mux_qdss_cti_trig1_out_a,
	msm_mux_qdss_cti_trig2_in_b,
	msm_mux_qdss_stm8,
	msm_mux_native_dac12,
	msm_mux_qdss_cti_trig1_in_a,
	msm_mux_qdss_cti_trig2_out_b,
	msm_mux_qdss_stm7,
	msm_mux_native_dac13,
	msm_mux_pcie_clkreq,
	msm_mux_qdss_stm6,
	msm_mux_qdss_stm5,
	msm_mux_pcie_host,
	msm_mux_qdss_stm4,
	msm_mux_qdss_cti_trig2_out_a,
	msm_mux_qdss_stm3,
	msm_mux_pcie_device,
	msm_mux_m_voc_ext_vfr_ref_irq_b,
	msm_mux_qdss_stm2,
	msm_mux_uim_batt,
	msm_mux_qdss_stm1,
	msm_mux_qdss_cti_trig2_in_a,
	msm_mux_qdss_stm0,
	msm_mux_i2s_mclk,
	msm_mux_audio_ref,
	msm_mux_dbg_out,
	msm_mux_gcc_plltest,
	msm_mux_uim1_data,
	msm_mux_uim1_present,
	msm_mux_uim1_reset,
	msm_mux_uim1_clk,
	msm_mux_blsp_i2c_sda_b1,
	msm_mux_blsp_i2c_scl_b1,
	msm_mux_NA,
};

static const char * const uim2_data_groups[] = {
	"gpio0",
};
static const char * const _groups[] = {
	"gpio0", "gpio1", "gpio2", "gpio3", "gpio4", "gpio5", "gpio6", "gpio7",
	"gpio8", "gpio9", "gpio10", "gpio11", "gpio12", "gpio13", "gpio14",
	"gpio15", "gpio16", "gpio17", "gpio18", "gpio19", "gpio20", "gpio21",
	"gpio22", "gpio23", "gpio24", "gpio26", "gpio27", "gpio29", "gpio30",
	"gpio31", "gpio32", "gpio33", "gpio34", "gpio35", "gpio36", "gpio38",
	"gpio39", "gpio40", "gpio41", "gpio42", "gpio43", "gpio44", "gpio45",
	"gpio46", "gpio47", "gpio48", "gpio49", "gpio50", "gpio52", "gpio53",
	"gpio62", "gpio63", "gpio64", "gpio67", "gpio69", "gpio70", "gpio71",
	"gpio72", "gpio73", "gpio74", "gpio75", "gpio76", "gpio77", "gpio78",
	"gpio79", "gpio80", "gpio81", "gpio82", "gpio83", "gpio84", "gpio85",
	"gpio86", "gpio87", "gpio88", "gpio89", "gpio90", "gpio91", "gpio92",
	"gpio93", "gpio94", "gpio95", "gpio96", "gpio97", "gpio98", "gpio99",
	"gpio99",
};
static const char * const blsp_uart_tx_a1_groups[] = {
	"gpio0",
};
static const char * const qdss_stm31_groups[] = {
	"gpio0",
};
static const char * const ebi0_wrcdc_groups[] = {
	"gpio0", "gpio2",
};
static const char * const native_etdac1_groups[] = {
	"gpio0", "gpio8", "gpio9", "gpio10", "gpio15",
};
static const char * const uim2_present_groups[] = {
	"gpio1",
};
static const char * const blsp_uart_rx_a1_groups[] = {
	"gpio1",
};
static const char * const blsp_spi1_groups[] = {
	"gpio0", "gpio1", "gpio2", "gpio3", "gpio68", "gpio69", "gpio71",
};
static const char * const qdss_stm30_groups[] = {
	"gpio1",
};
static const char * const native_etdac0_groups[] = {
	"gpio1", "gpio2", "gpio3", "gpio37", "gpio39",
};
static const char * const uim2_reset_groups[] = {
	"gpio2",
};
static const char * const blsp_uart_cts_n_a1_groups[] = {
	"gpio2",
};
static const char * const blsp_i2c_sda_a1_groups[] = {
	"gpio2",
};
static const char * const qdss_stm29_groups[] = {
	"gpio2",
};
static const char * const uim2_clk_groups[] = {
	"gpio3",
};
static const char * const blsp_uart_rfr_n_a1_groups[] = {
	"gpio3",
};
static const char * const blsp_i2c_scl_a1_groups[] = {
	"gpio3",
};
static const char * const qdss_stm28_groups[] = {
	"gpio3",
};
static const char * const blsp_spi2_groups[] = {
	"gpio4", "gpio5", "gpio6", "gpio7", "gpio60", "gpio68", "gpio71",
};
static const char * const blsp_uart2_groups[] = {
	"gpio4", "gpio5", "gpio6", "gpio7",
};
static const char * const qdss_stm23_groups[] = {
	"gpio4",
};
static const char * const qdss_tracedata_a_groups[] = {
	"gpio4", "gpio5", "gpio6", "gpio7", "gpio8", "gpio9", "gpio12",
	"gpio13", "gpio14", "gpio15", "gpio16", "gpio17", "gpio18", "gpio19",
	"gpio69", "gpio71",
};
static const char * const native_dac4_groups[] = {
	"gpio4", "gpio54",
};
static const char * const qdss_stm22_groups[] = {
	"gpio5",
};
static const char * const native_dac5_groups[] = {
	"gpio5", "gpio55",
};
static const char * const blsp_i2c_sda_a2_groups[] = {
	"gpio6",
};
static const char * const ldo_en_groups[] = {
	"gpio6",
};
static const char * const qdss_stm21_groups[] = {
	"gpio6",
};
static const char * const native_dac6_groups[] = {
	"gpio6", "gpio56",
};
static const char * const blsp_i2c_scl_a2_groups[] = {
	"gpio7",
};
static const char * const ldo_update_groups[] = {
	"gpio7",
};
static const char * const qdss_stm20_groups[] = {
	"gpio7",
};
static const char * const native_dac7_groups[] = {
	"gpio7", "gpio57",
};
static const char * const pri_mi2s_ws_b_groups[] = {
	"gpio8",
};
static const char * const blsp_spi3_groups[] = {
	"gpio8", "gpio9", "gpio10", "gpio11", "gpio68", "gpio69", "gpio71",
};
static const char * const blsp_uart3_groups[] = {
	"gpio8", "gpio9", "gpio10", "gpio11",
};
static const char * const qdss_cti_trig1_out_b_groups[] = {
	"gpio8",
};
static const char * const pwr_modem_groups[] = {
	"gpio8",
};
static const char * const pri_mi2s_data0_b_groups[] = {
	"gpio9",
};
static const char * const qdss_cti_trig1_in_b_groups[] = {
	"gpio9",
};
static const char * const pwr_nav_groups[] = {
	"gpio9",
};
static const char * const pri_mi2s_data1_b_groups[] = {
	"gpio10",
};
static const char * const blsp_i2c3_groups[] = {
	"gpio10", "gpio11",
};
static const char * const pwr_crypto_groups[] = {
	"gpio10",
};
static const char * const pri_mi2s_sck_b_groups[] = {
	"gpio11",
};
static const char * const pri_mi2s_ws_a_groups[] = {
	"gpio12",
};
static const char * const blsp_uart_tx_a4_groups[] = {
	"gpio12",
};
static const char * const qdss_stm19_groups[] = {
	"gpio12",
};
static const char * const pri_mi2s_data0_a_groups[] = {
	"gpio13",
};
static const char * const blsp_uart_rx_a4_groups[] = {
	"gpio13",
};
static const char * const qdss_stm18_groups[] = {
	"gpio13",
};
static const char * const pri_mi2s_data1_a_groups[] = {
	"gpio14",
};
static const char * const blsp_uart_cts_n_a4_groups[] = {
	"gpio14",
};
static const char * const blsp_i2c_sda_a4_groups[] = {
	"gpio14",
};
static const char * const slimbus_data_groups[] = {
	"gpio14",
};
static const char * const qdss_stm17_groups[] = {
	"gpio14",
};
static const char * const bimc_dte0_groups[] = {
	"gpio14", "gpio59",
};
static const char * const native_tsens_groups[] = {
	"gpio14",
};
static const char * const pri_mi2s_sck_a_groups[] = {
	"gpio15",
};
static const char * const blsp_uart_rfr_n_a4_groups[] = {
	"gpio15",
};
static const char * const blsp_i2c_scl_a4_groups[] = {
	"gpio15",
};
static const char * const slimbus_clk_groups[] = {
	"gpio15",
};
static const char * const qdss_stm16_groups[] = {
	"gpio15",
};
static const char * const bimc_dte1_groups[] = {
	"gpio15", "gpio60",
};
static const char * const sec_mi2s_ws_a_groups[] = {
	"gpio16",
};
static const char * const blsp_spi4_groups[] = {
	"gpio16", "gpio17", "gpio18", "gpio19", "gpio68", "gpio69", "gpio71",
};
static const char * const blsp_uart_tx_b4_groups[] = {
	"gpio16",
};
static const char * const uim3_data_groups[] = {
	"gpio16",
};
static const char * const qdss_stm27_groups[] = {
	"gpio16",
};
static const char * const qdss_tracedata_b_groups[] = {
	"gpio16", "gpio17", "gpio18", "gpio19", "gpio30", "gpio31", "gpio32",
	"gpio33", "gpio34", "gpio35", "gpio36", "gpio39", "gpio40", "gpio43",
	"gpio50", "gpio69",
};
static const char * const native_txdac1_groups[] = {
	"gpio16", "gpio17", "gpio18", "gpio19", "gpio20",
};
static const char * const sec_mi2s_data0_a_groups[] = {
	"gpio17",
};
static const char * const blsp_uart_rx_b4_groups[] = {
	"gpio17",
};
static const char * const uim3_present_groups[] = {
	"gpio17",
};
static const char * const qdss_stm26_groups[] = {
	"gpio17",
};
static const char * const sec_mi2s_data1_a_groups[] = {
	"gpio18",
};
static const char * const blsp_uart_cts_n_b4_groups[] = {
	"gpio18",
};
static const char * const blsp_i2c_sda_b4_groups[] = {
	"gpio18",
};
static const char * const uim3_reset_groups[] = {
	"gpio18",
};
static const char * const qdss_stm25_groups[] = {
	"gpio18",
};
static const char * const sec_mi2s_sck_a_groups[] = {
	"gpio19",
};
static const char * const blsp_uart_rfr_n_b4_groups[] = {
	"gpio19",
};
static const char * const blsp_i2c_scl_b4_groups[] = {
	"gpio19",
};
static const char * const uim3_clk_groups[] = {
	"gpio19",
};
static const char * const qdss_stm24_groups[] = {
	"gpio19",
};
static const char * const sec_mi2s_ws_b_groups[] = {
	"gpio20",
};
static const char * const ebi2_a_groups[] = {
	"gpio20",
};
static const char * const blsp_uart_tx_b1_groups[] = {
	"gpio20",
};
static const char * const uim4_data_groups[] = {
	"gpio20",
};
static const char * const sec_mi2s_data0_b_groups[] = {
	"gpio21",
};
static const char * const ebi2_lcd_groups[] = {
	"gpio21", "gpio22", "gpio23",
};
static const char * const blsp_uart_rx_b1_groups[] = {
	"gpio21",
};
static const char * const uim4_present_groups[] = {
	"gpio21",
};
static const char * const native_txdac0_groups[] = {
	"gpio21", "gpio22", "gpio23", "gpio41", "gpio68",
};
static const char * const sec_mi2s_data1_b_groups[] = {
	"gpio22",
};
static const char * const blsp_uart_cts_n_b1_groups[] = {
	"gpio22",
};
static const char * const uim4_reset_groups[] = {
	"gpio22",
};
static const char * const sec_mi2s_sck_b_groups[] = {
	"gpio23",
};
static const char * const blsp_uart_rfr_n_b1_groups[] = {
	"gpio23",
};
static const char * const uim4_clk_groups[] = {
	"gpio23",
};
static const char * const m_voc_ext_vfr_ref_irq_a_groups[] = {
	"gpio24",
};
static const char * const adsp_ext_vfr_irq_a_groups[] = {
	"gpio24",
};
static const char * const qdss_stm11_groups[] = {
	"gpio24",
};
static const char * const epm1_groups[] = {
	"gpio25",
};
static const char * const m_voc_ext_vfr_ref_irq_2_a_groups[] = {
	"gpio25",
};
static const char * const native_char0_groups[] = {
	"gpio31",
};
static const char * const pa_indicator_groups[] = {
	"gpio36",
};
static const char * const qdss_traceclk_a_groups[] = {
	"gpio36",
};
static const char * const modem_tsync_groups[] = {
	"gpio38",
};
static const char * const nav_dr_groups[] = {
	"gpio38",
};
static const char * const adsp_ext_vfr_irq_b_groups[] = {
	"gpio25",
};
static const char * const qdss_stm10_groups[] = {
	"gpio25",
};
static const char * const native_char_groups[] = {
	"gpio27",
};
static const char * const antpair_wlan_groups[] = {
	"gpio28", "gpio37",
};
static const char * const native_char3_groups[] = {
	"gpio28",
};
static const char * const native_char2_groups[] = {
	"gpio29",
};
static const char * const native_char1_groups[] = {
	"gpio30",
};
static const char * const nav_pps_in_a_groups[] = {
	"gpio38",
};
static const char * const native_dac3_groups[] = {
	"gpio40", "gpio53", "gpio79",
};
static const char * const gsm_tx1_groups[] = {
	"gpio41",
};
static const char * const cri_trng1_groups[] = {
	"gpio41",
};
static const char * const gsm_tx2_groups[] = {
	"gpio42",
};
static const char * const native_dac_groups[] = {
	"gpio42", "gpio43", "gpio44", "gpio45", "gpio46", "gpio71",
};
static const char * const cri_trng0_groups[] = {
	"gpio42",
};
static const char * const cri_trng_groups[] = {
	"gpio43",
};
static const char * const ssbi2_groups[] = {
	"gpio46",
};
static const char * const ssbi1_groups[] = {
	"gpio47",
};
static const char * const prng_rosc_groups[] = {
	"gpio47",
};
static const char * const native_bbrx1_groups[] = {
	"gpio47",
};
static const char * const blsp_i2c_sda_b2_groups[] = {
	"gpio48",
};
static const char * const native_bbrx0_groups[] = {
	"gpio48",
};
static const char * const blsp_i2c_scl_b2_groups[] = {
	"gpio49",
};
static const char * const native_gpsadc_groups[] = {
	"gpio49", "gpio64",
};
static const char * const qdss_tracectl_a_groups[] = {
	"gpio50",
};
static const char * const native_dac0_groups[] = {
	"gpio50", "gpio72", "gpio75", "gpio76",
};
static const char * const epm2_groups[] = {
	"gpio51",
};
static const char * const nav_pps_in_b_groups[] = {
	"gpio51",
};
static const char * const native_dac1_groups[] = {
	"gpio51", "gpio69", "gpio74", "gpio77",
};
static const char * const coex_uart_groups[] = {
	"gpio52", "gpio53",
};
static const char * const qdss_cti_groups[] = {
	"gpio52", "gpio53", "gpio92", "gpio93",
};
static const char * const qdss_traceclk_b_groups[] = {
	"gpio52",
};
static const char * const native_dac2_groups[] = {
	"gpio52", "gpio73", "gpio78",
};
static const char * const qdss_tracectl_b_groups[] = {
	"gpio53",
};
static const char * const ap2mdm_status_groups[] = {
	"gpio54",
};
static const char * const mdm2ap_status_groups[] = {
	"gpio55",
};
static const char * const ap2mdm_err_groups[] = {
	"gpio56",
};
static const char * const mdm2ap_err_groups[] = {
	"gpio57",
};
static const char * const qdss_stm15_groups[] = {
	"gpio57",
};
static const char * const ap2mdm_vdd_groups[] = {
	"gpio58",
};
static const char * const qdss_stm14_groups[] = {
	"gpio58",
};
static const char * const native_dac8_groups[] = {
	"gpio58",
};
static const char * const mdm2ap_vdd_groups[] = {
	"gpio59",
};
static const char * const qdss_stm13_groups[] = {
	"gpio59",
};
static const char * const native_dac9_groups[] = {
	"gpio59",
};
static const char * const ap2mdm_wake_groups[] = {
	"gpio60",
};
static const char * const m_voc_ext_vfr_ref_irq_2_b_groups[] = {
	"gpio60",
};
static const char * const qdss_stm12_groups[] = {
	"gpio60",
};
static const char * const native_dac10_groups[] = {
	"gpio60",
};
static const char * const pciehost_rst_groups[] = {
	"gpio61",
};
static const char * const pci_e_groups[] = {
	"gpio61", "gpio61", "gpio65",
};
static const char * const qdss_stm9_groups[] = {
	"gpio61",
};
static const char * const native_dac11_groups[] = {
	"gpio61",
};
static const char * const qdss_cti_trig1_out_a_groups[] = {
	"gpio62",
};
static const char * const qdss_cti_trig2_in_b_groups[] = {
	"gpio62",
};
static const char * const qdss_stm8_groups[] = {
	"gpio62",
};
static const char * const native_dac12_groups[] = {
	"gpio62",
};
static const char * const qdss_cti_trig1_in_a_groups[] = {
	"gpio63",
};
static const char * const qdss_cti_trig2_out_b_groups[] = {
	"gpio63",
};
static const char * const qdss_stm7_groups[] = {
	"gpio63",
};
static const char * const native_dac13_groups[] = {
	"gpio63",
};
static const char * const pcie_clkreq_groups[] = {
	"gpio64",
};
static const char * const qdss_stm6_groups[] = {
	"gpio64",
};
static const char * const qdss_stm5_groups[] = {
	"gpio65",
};
static const char * const pcie_host_groups[] = {
	"gpio66",
};
static const char * const qdss_stm4_groups[] = {
	"gpio66",
};
static const char * const qdss_cti_trig2_out_a_groups[] = {
	"gpio67",
};
static const char * const qdss_stm3_groups[] = {
	"gpio67",
};
static const char * const pcie_device_groups[] = {
	"gpio68",
};
static const char * const m_voc_ext_vfr_ref_irq_b_groups[] = {
	"gpio68",
};
static const char * const qdss_stm2_groups[] = {
	"gpio68",
};
static const char * const uim_batt_groups[] = {
	"gpio69",
};
static const char * const qdss_stm1_groups[] = {
	"gpio69",
};
static const char * const qdss_cti_trig2_in_a_groups[] = {
	"gpio70",
};
static const char * const qdss_stm0_groups[] = {
	"gpio70",
};
static const char * const i2s_mclk_groups[] = {
	"gpio71",
};
static const char * const audio_ref_groups[] = {
	"gpio71",
};
static const char * const dbg_out_groups[] = {
	"gpio71",
};
static const char * const gcc_plltest_groups[] = {
	"gpio73", "gpio74",
};
static const char * const uim1_data_groups[] = {
	"gpio76",
};
static const char * const uim1_present_groups[] = {
	"gpio77",
};
static const char * const uim1_reset_groups[] = {
	"gpio78",
};
static const char * const uim1_clk_groups[] = {
	"gpio79",
};
static const char * const blsp_i2c_sda_b1_groups[] = {
	"gpio84",
};
static const char * const blsp_i2c_scl_b1_groups[] = {
	"gpio85",
};

static const struct msm_function mdm9640_functions[] = {
	FUNCTION(uim2_data),
	FUNCTION(),
	FUNCTION(blsp_uart_tx_a1),
	FUNCTION(qdss_stm31),
	FUNCTION(ebi0_wrcdc),
	FUNCTION(native_etdac1),
	FUNCTION(uim2_present),
	FUNCTION(blsp_uart_rx_a1),
	FUNCTION(blsp_spi1),
	FUNCTION(qdss_stm30),
	FUNCTION(native_etdac0),
	FUNCTION(uim2_reset),
	FUNCTION(blsp_uart_cts_n_a1),
	FUNCTION(blsp_i2c_sda_a1),
	FUNCTION(qdss_stm29),
	FUNCTION(uim2_clk),
	FUNCTION(blsp_uart_rfr_n_a1),
	FUNCTION(blsp_i2c_scl_a1),
	FUNCTION(qdss_stm28),
	FUNCTION(blsp_spi2),
	FUNCTION(blsp_uart2),
	FUNCTION(qdss_stm23),
	FUNCTION(qdss_tracedata_a),
	FUNCTION(native_dac4),
	FUNCTION(qdss_stm22),
	FUNCTION(native_dac5),
	FUNCTION(blsp_i2c_sda_a2),
	FUNCTION(ldo_en),
	FUNCTION(qdss_stm21),
	FUNCTION(native_dac6),
	FUNCTION(blsp_i2c_scl_a2),
	FUNCTION(ldo_update),
	FUNCTION(qdss_stm20),
	FUNCTION(native_dac7),
	FUNCTION(pri_mi2s_ws_b),
	FUNCTION(blsp_spi3),
	FUNCTION(blsp_uart3),
	FUNCTION(qdss_cti_trig1_out_b),
	FUNCTION(pwr_modem),
	FUNCTION(pri_mi2s_data0_b),
	FUNCTION(qdss_cti_trig1_in_b),
	FUNCTION(pwr_nav),
	FUNCTION(pri_mi2s_data1_b),
	FUNCTION(blsp_i2c3),
	FUNCTION(pwr_crypto),
	FUNCTION(pri_mi2s_sck_b),
	FUNCTION(pri_mi2s_ws_a),
	FUNCTION(blsp_uart_tx_a4),
	FUNCTION(qdss_stm19),
	FUNCTION(pri_mi2s_data0_a),
	FUNCTION(blsp_uart_rx_a4),
	FUNCTION(qdss_stm18),
	FUNCTION(pri_mi2s_data1_a),
	FUNCTION(blsp_uart_cts_n_a4),
	FUNCTION(blsp_i2c_sda_a4),
	FUNCTION(slimbus_data),
	FUNCTION(qdss_stm17),
	FUNCTION(bimc_dte0),
	FUNCTION(native_tsens),
	FUNCTION(pri_mi2s_sck_a),
	FUNCTION(blsp_uart_rfr_n_a4),
	FUNCTION(blsp_i2c_scl_a4),
	FUNCTION(slimbus_clk),
	FUNCTION(qdss_stm16),
	FUNCTION(bimc_dte1),
	FUNCTION(sec_mi2s_ws_a),
	FUNCTION(blsp_spi4),
	FUNCTION(blsp_uart_tx_b4),
	FUNCTION(uim3_data),
	FUNCTION(qdss_stm27),
	FUNCTION(qdss_tracedata_b),
	FUNCTION(native_txdac1),
	FUNCTION(sec_mi2s_data0_a),
	FUNCTION(blsp_uart_rx_b4),
	FUNCTION(uim3_present),
	FUNCTION(qdss_stm26),
	FUNCTION(sec_mi2s_data1_a),
	FUNCTION(blsp_uart_cts_n_b4),
	FUNCTION(blsp_i2c_sda_b4),
	FUNCTION(uim3_reset),
	FUNCTION(qdss_stm25),
	FUNCTION(sec_mi2s_sck_a),
	FUNCTION(blsp_uart_rfr_n_b4),
	FUNCTION(blsp_i2c_scl_b4),
	FUNCTION(uim3_clk),
	FUNCTION(qdss_stm24),
	FUNCTION(sec_mi2s_ws_b),
	FUNCTION(ebi2_a),
	FUNCTION(blsp_uart_tx_b1),
	FUNCTION(uim4_data),
	FUNCTION(sec_mi2s_data0_b),
	FUNCTION(ebi2_lcd),
	FUNCTION(blsp_uart_rx_b1),
	FUNCTION(uim4_present),
	FUNCTION(native_txdac0),
	FUNCTION(sec_mi2s_data1_b),
	FUNCTION(blsp_uart_cts_n_b1),
	FUNCTION(uim4_reset),
	FUNCTION(sec_mi2s_sck_b),
	FUNCTION(blsp_uart_rfr_n_b1),
	FUNCTION(uim4_clk),
	FUNCTION(m_voc_ext_vfr_ref_irq_a),
	FUNCTION(adsp_ext_vfr_irq_a),
	FUNCTION(qdss_stm11),
	FUNCTION(epm1),
	FUNCTION(m_voc_ext_vfr_ref_irq_2_a),
	FUNCTION(native_char0),
	FUNCTION(pa_indicator),
	FUNCTION(qdss_traceclk_a),
	FUNCTION(modem_tsync),
	FUNCTION(nav_dr),
	FUNCTION(adsp_ext_vfr_irq_b),
	FUNCTION(qdss_stm10),
	FUNCTION(native_char),
	FUNCTION(antpair_wlan),
	FUNCTION(native_char3),
	FUNCTION(native_char2),
	FUNCTION(native_char1),
	FUNCTION(nav_pps_in_a),
	FUNCTION(native_dac3),
	FUNCTION(gsm_tx1),
	FUNCTION(cri_trng1),
	FUNCTION(gsm_tx2),
	FUNCTION(native_dac),
	FUNCTION(cri_trng0),
	FUNCTION(cri_trng),
	FUNCTION(ssbi2),
	FUNCTION(ssbi1),
	FUNCTION(prng_rosc),
	FUNCTION(native_bbrx1),
	FUNCTION(blsp_i2c_sda_b2),
	FUNCTION(native_bbrx0),
	FUNCTION(blsp_i2c_scl_b2),
	FUNCTION(native_gpsadc),
	FUNCTION(qdss_tracectl_a),
	FUNCTION(native_dac0),
	FUNCTION(epm2),
	FUNCTION(nav_pps_in_b),
	FUNCTION(native_dac1),
	FUNCTION(coex_uart),
	FUNCTION(qdss_cti),
	FUNCTION(qdss_traceclk_b),
	FUNCTION(native_dac2),
	FUNCTION(qdss_tracectl_b),
	FUNCTION(ap2mdm_status),
	FUNCTION(mdm2ap_status),
	FUNCTION(ap2mdm_err),
	FUNCTION(mdm2ap_err),
	FUNCTION(qdss_stm15),
	FUNCTION(ap2mdm_vdd),
	FUNCTION(qdss_stm14),
	FUNCTION(native_dac8),
	FUNCTION(mdm2ap_vdd),
	FUNCTION(qdss_stm13),
	FUNCTION(native_dac9),
	FUNCTION(ap2mdm_wake),
	FUNCTION(m_voc_ext_vfr_ref_irq_2_b),
	FUNCTION(qdss_stm12),
	FUNCTION(native_dac10),
	FUNCTION(pciehost_rst),
	FUNCTION(pci_e),
	FUNCTION(qdss_stm9),
	FUNCTION(native_dac11),
	FUNCTION(qdss_cti_trig1_out_a),
	FUNCTION(qdss_cti_trig2_in_b),
	FUNCTION(qdss_stm8),
	FUNCTION(native_dac12),
	FUNCTION(qdss_cti_trig1_in_a),
	FUNCTION(qdss_cti_trig2_out_b),
	FUNCTION(qdss_stm7),
	FUNCTION(native_dac13),
	FUNCTION(pcie_clkreq),
	FUNCTION(qdss_stm6),
	FUNCTION(qdss_stm5),
	FUNCTION(pcie_host),
	FUNCTION(qdss_stm4),
	FUNCTION(qdss_cti_trig2_out_a),
	FUNCTION(qdss_stm3),
	FUNCTION(pcie_device),
	FUNCTION(m_voc_ext_vfr_ref_irq_b),
	FUNCTION(qdss_stm2),
	FUNCTION(uim_batt),
	FUNCTION(qdss_stm1),
	FUNCTION(qdss_cti_trig2_in_a),
	FUNCTION(qdss_stm0),
	FUNCTION(i2s_mclk),
	FUNCTION(audio_ref),
	FUNCTION(dbg_out),
	FUNCTION(gcc_plltest),
	FUNCTION(uim1_data),
	FUNCTION(uim1_present),
	FUNCTION(uim1_reset),
	FUNCTION(uim1_clk),
	FUNCTION(blsp_i2c_sda_b1),
	FUNCTION(blsp_i2c_scl_b1),
};

static const struct msm_pingroup mdm9640_groups[] = {
	PINGROUP(0, uim2_data, blsp_spi1, blsp_uart_tx_a1, qdss_stm31,
		 ebi0_wrcdc, native_etdac1, NA, NA, NA),
	PINGROUP(1, uim2_present, blsp_spi1, blsp_uart_rx_a1, qdss_stm30,
		 native_etdac0, NA, NA, NA, NA),
	PINGROUP(2, uim2_reset, blsp_spi1, blsp_uart_cts_n_a1, blsp_i2c_sda_a1,
		 qdss_stm29, ebi0_wrcdc, native_etdac0, NA, NA),
	PINGROUP(3, uim2_clk, blsp_spi1, blsp_uart_rfr_n_a1, blsp_i2c_scl_a1,
		 qdss_stm28, NA, native_etdac0, NA, NA),
	PINGROUP(4, blsp_spi2, blsp_uart2, NA, qdss_stm23, qdss_tracedata_a,
		 native_dac4, NA, NA, NA),
	PINGROUP(5, blsp_spi2, blsp_uart2, NA, qdss_stm22, qdss_tracedata_a,
		 native_dac5, NA, NA, NA),
	PINGROUP(6, blsp_spi2, blsp_uart2, blsp_i2c_sda_a2, ldo_en, NA,
		 qdss_stm21, qdss_tracedata_a, native_dac6, NA),
	PINGROUP(7, blsp_spi2, blsp_uart2, blsp_i2c_scl_a2, ldo_update, NA,
		 qdss_stm20, qdss_tracedata_a, native_dac7, NA),
	PINGROUP(8, pri_mi2s_ws_b, blsp_spi3, blsp_uart3, NA, qdss_tracedata_a,
		 qdss_cti_trig1_out_b, pwr_modem, native_etdac1, NA),
	PINGROUP(9, pri_mi2s_data0_b, blsp_spi3, blsp_uart3, NA,
		 qdss_tracedata_a, qdss_cti_trig1_in_b, pwr_nav, NA,
		 native_etdac1),
	PINGROUP(10, pri_mi2s_data1_b, blsp_spi3, blsp_uart3, blsp_i2c3, NA,
		 pwr_crypto, NA, native_etdac1, NA),
	PINGROUP(11, pri_mi2s_sck_b, blsp_spi3, blsp_uart3, blsp_i2c3, NA, NA,
		 NA, NA, NA),
	PINGROUP(12, pri_mi2s_ws_a, blsp_uart_tx_a4, NA, qdss_stm19,
		 qdss_tracedata_a, NA, NA, NA, NA),
	PINGROUP(13, pri_mi2s_data0_a, blsp_uart_rx_a4, NA, qdss_stm18,
		 qdss_tracedata_a, NA, NA, NA, NA),
	PINGROUP(14, pri_mi2s_data1_a, blsp_uart_cts_n_a4, blsp_i2c_sda_a4,
		 slimbus_data, NA, NA, qdss_stm17, qdss_tracedata_a, bimc_dte0),
	PINGROUP(15, pri_mi2s_sck_a, blsp_uart_rfr_n_a4, blsp_i2c_scl_a4,
		 slimbus_clk, NA, NA, qdss_stm16, qdss_tracedata_a, bimc_dte1),
	PINGROUP(16, sec_mi2s_ws_a, blsp_spi4, blsp_uart_tx_b4, uim3_data, NA,
		 qdss_stm27, qdss_tracedata_a, qdss_tracedata_b, native_txdac1),
	PINGROUP(17, sec_mi2s_data0_a, blsp_spi4, blsp_uart_rx_b4,
		 uim3_present, qdss_stm26, qdss_tracedata_a, qdss_tracedata_b,
		 native_txdac1, NA),
	PINGROUP(18, sec_mi2s_data1_a, blsp_spi4, blsp_uart_cts_n_b4,
		 blsp_i2c_sda_b4, uim3_reset, NA, qdss_stm25, qdss_tracedata_a,
		 qdss_tracedata_b),
	PINGROUP(19, sec_mi2s_sck_a, blsp_spi4, blsp_uart_rfr_n_b4,
		 blsp_i2c_scl_b4, uim3_clk, NA, qdss_stm24, qdss_tracedata_a,
		 qdss_tracedata_b),
	PINGROUP(20, sec_mi2s_ws_b, ebi2_a, blsp_uart_tx_b1, uim4_data,
		 native_txdac1, NA, NA, NA, NA),
	PINGROUP(21, sec_mi2s_data0_b, ebi2_lcd, blsp_uart_rx_b1, uim4_present,
		 NA, native_txdac0, NA, NA, NA),
	PINGROUP(22, sec_mi2s_data1_b, ebi2_lcd, blsp_uart_cts_n_b1,
		 uim4_reset, native_txdac0, NA, NA, NA, NA),
	PINGROUP(23, sec_mi2s_sck_b, ebi2_lcd, blsp_uart_rfr_n_b1, uim4_clk,
		 native_txdac0, NA, NA, NA, NA),
	PINGROUP(24, m_voc_ext_vfr_ref_irq_a, adsp_ext_vfr_irq_a, NA,
		 qdss_stm11, NA, NA, NA, NA, NA),
	PINGROUP(25, m_voc_ext_vfr_ref_irq_2_a, adsp_ext_vfr_irq_b, NA,
		 qdss_stm10, NA, NA, NA, NA, NA),
	PINGROUP(26, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(27, NA, NA, NA, native_char, NA, NA, NA, NA, NA),
	PINGROUP(28, NA, NA, NA, native_char3, NA, NA, NA, NA, NA),
	PINGROUP(29, NA, NA, NA, native_char2, NA, NA, NA, NA, NA),
	PINGROUP(30, NA, qdss_tracedata_b, native_char1, NA, NA, NA, NA, NA,
		 NA),
	PINGROUP(31, NA, qdss_tracedata_b, native_char0, NA, NA, NA, NA, NA,
		 NA),
	PINGROUP(32, NA, qdss_tracedata_b, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(33, NA, qdss_tracedata_b, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(34, NA, qdss_tracedata_b, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(35, NA, qdss_tracedata_b, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(36, NA, pa_indicator, qdss_traceclk_a, qdss_tracedata_b, NA,
		 NA, NA, NA, NA),
	PINGROUP(37, NA, native_etdac0, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(38, NA, modem_tsync, nav_dr, nav_pps_in_a, NA, NA, NA, NA, NA),
	PINGROUP(39, NA, qdss_tracedata_b, native_etdac0, NA, NA, NA, NA, NA,
		 NA),
	PINGROUP(40, NA, qdss_tracedata_b, native_dac3, NA, NA, NA, NA, NA, NA),
	PINGROUP(41, gsm_tx1, native_txdac0, NA, cri_trng1, NA, NA, NA, NA, NA),
	PINGROUP(42, gsm_tx2, native_dac, NA, cri_trng0, NA, NA, NA, NA, NA),
	PINGROUP(43, NA, qdss_tracedata_b, native_dac, NA, cri_trng, NA, NA,
		 NA, NA),
	PINGROUP(44, NA, native_dac, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(45, NA, native_dac, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(46, NA, ssbi2, native_dac, NA, NA, NA, NA, NA, NA),
	PINGROUP(47, NA, ssbi1, prng_rosc, native_bbrx1, NA, NA, NA, NA, NA),
	PINGROUP(48, NA, blsp_i2c_sda_b2, native_bbrx0, NA, NA, NA, NA, NA, NA),
	PINGROUP(49, NA, blsp_i2c_scl_b2, native_gpsadc, NA, NA, NA, NA, NA,
		 NA),
	PINGROUP(50, NA, qdss_tracectl_a, qdss_tracedata_b, native_dac0, NA,
		 NA, NA, NA, NA),
	PINGROUP(51, nav_pps_in_b, native_dac1, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(52, coex_uart, qdss_cti, qdss_traceclk_b, native_dac2, NA, NA,
		 NA, NA, NA),
	PINGROUP(53, coex_uart, qdss_cti, qdss_tracectl_b, native_dac3, NA, NA,
		 NA, NA, NA),
	PINGROUP(54, NA, native_dac4, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(55, NA, native_dac5, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(56, NA, native_dac6, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(57, NA, qdss_stm15, native_dac7, NA, NA, NA, NA, NA, NA),
	PINGROUP(58, NA, qdss_stm14, native_dac8, NA, NA, NA, NA, NA, NA),
	PINGROUP(59, NA, qdss_stm13, bimc_dte0, native_dac9, NA, NA, NA, NA,
		 NA),
	PINGROUP(60, m_voc_ext_vfr_ref_irq_2_b, blsp_spi2, NA, NA, qdss_stm12,
		 bimc_dte1, native_dac10, NA, NA),
	PINGROUP(61, pci_e, pci_e, NA, NA, qdss_stm9, native_dac11, NA, NA, NA),
	PINGROUP(62, qdss_cti_trig1_out_a, qdss_cti_trig2_in_b, NA, qdss_stm8,
		 native_dac12, NA, NA, NA, NA),
	PINGROUP(63, qdss_cti_trig1_in_a, qdss_cti_trig2_out_b, NA, qdss_stm7,
		 native_dac13, NA, NA, NA, NA),
	PINGROUP(64, pcie_clkreq, qdss_stm6, native_gpsadc, NA, NA, NA, NA, NA,
		 NA),
	PINGROUP(65, NA, qdss_stm5, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(66, NA, qdss_stm4, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(67, qdss_cti_trig2_out_a, NA, qdss_stm3, NA, NA, NA, NA, NA,
		 NA),
	PINGROUP(68, m_voc_ext_vfr_ref_irq_b, blsp_spi1, blsp_spi2, blsp_spi3,
		 blsp_spi4, NA, qdss_stm2, native_txdac0, NA),
	PINGROUP(69, uim_batt, blsp_spi1, blsp_spi3, blsp_spi4, qdss_stm1,
		 qdss_tracedata_a, qdss_tracedata_b, native_dac1, NA),
	PINGROUP(70, qdss_cti_trig2_in_a, NA, qdss_stm0, NA, NA, NA, NA, NA,
		 NA),
	PINGROUP(71, i2s_mclk, audio_ref, blsp_spi1, blsp_spi2, blsp_spi3,
		 blsp_spi4, dbg_out, qdss_tracedata_a, native_dac),
	PINGROUP(72, NA, native_dac0, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(73, NA, gcc_plltest, native_dac2, NA, NA, NA, NA, NA, NA),
	PINGROUP(74, NA, gcc_plltest, native_dac1, NA, NA, NA, NA, NA, NA),
	PINGROUP(75, NA, native_dac0, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(76, uim1_data, native_dac0, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(77, uim1_present, native_dac1, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(78, uim1_reset, native_dac2, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(79, uim1_clk, native_dac3, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(80, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(81, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(82, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(83, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(84, NA, NA, blsp_i2c_sda_b1, NA, NA, NA, NA, NA, NA),
	PINGROUP(85, NA, NA, blsp_i2c_scl_b1, NA, NA, NA, NA, NA, NA),
	PINGROUP(86, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(87, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(88, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(89, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(90, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(91, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(92, NA, NA, qdss_cti, NA, NA, NA, NA, NA, NA),
	PINGROUP(93, NA, NA, qdss_cti, NA, NA, NA, NA, NA, NA),
	PINGROUP(94, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(95, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(96, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(97, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(98, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	PINGROUP(99, NA, NA, NA, NA, NA, NA, NA, NA, NA),
	SDC_QDSD_PINGROUP(sdc1_clk, 0x10a000, 13, 6),
	SDC_QDSD_PINGROUP(sdc1_cmd, 0x10a000, 11, 3),
	SDC_QDSD_PINGROUP(sdc1_data, 0x10a000, 9, 0),
	SDC_QDSD_PINGROUP(sdc2_clk, 0x109000, 14, 6),
	SDC_QDSD_PINGROUP(sdc2_cmd, 0x109000, 11, 3),
	SDC_QDSD_PINGROUP(sdc2_data, 0x109000, 9, 0),
	SDC_QDSD_PINGROUP(qdsd_clk, 0x19c000, 3, 0),
	SDC_QDSD_PINGROUP(qdsd_cmd, 0x19c000, 8, 5),
	SDC_QDSD_PINGROUP(qdsd_data0, 0x19c000, 13, 10),
	SDC_QDSD_PINGROUP(qdsd_data1, 0x19c000, 18, 15),
	SDC_QDSD_PINGROUP(qdsd_data2, 0x19c000, 23, 20),
	SDC_QDSD_PINGROUP(qdsd_data3, 0x19c000, 28, 25),
};

static const struct msm_pinctrl_soc_data mdm9640_pinctrl = {
	.pins = mdm9640_pins,
	.npins = ARRAY_SIZE(mdm9640_pins),
	.functions = mdm9640_functions,
	.nfunctions = ARRAY_SIZE(mdm9640_functions),
	.groups = mdm9640_groups,
	.ngroups = ARRAY_SIZE(mdm9640_groups),
	.ngpios = 100,
};

static int mdm9640_pinctrl_probe(struct platform_device *pdev)
{
	return msm_pinctrl_probe(pdev, &mdm9640_pinctrl);
}

static const struct of_device_id mdm9640_pinctrl_of_match[] = {
	{ .compatible = "qcom,mdm9640-pinctrl", },
	{ },
};

static struct platform_driver mdm9640_pinctrl_driver = {
	.driver = {
		.name = "mdm9640-pinctrl",
		.owner = THIS_MODULE,
		.of_match_table = mdm9640_pinctrl_of_match,
	},
	.probe = mdm9640_pinctrl_probe,
	.remove = msm_pinctrl_remove,
};

static int __init mdm9640_pinctrl_init(void)
{
	return platform_driver_register(&mdm9640_pinctrl_driver);
}
arch_initcall(mdm9640_pinctrl_init);

static void __exit mdm9640_pinctrl_exit(void)
{
	platform_driver_unregister(&mdm9640_pinctrl_driver);
}
module_exit(mdm9640_pinctrl_exit);

MODULE_DESCRIPTION("Qualcomm mdm9640 pinctrl driver");
MODULE_LICENSE("GPL v2");
MODULE_DEVICE_TABLE(of, mdm9640_pinctrl_of_match);
