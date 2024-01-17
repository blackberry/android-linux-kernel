/*
 * Copyright (C) 2010 Trusted Logic S.A.
 * modifications copyright (C) 2015 NXP B.V.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define PN5XX_MAGIC    0xE9

/*
 * PN5XX power control via ioctl
 * PN5XX_SET_PWR(0): power off
 * PN5XX_SET_PWR(1): power on
 * PN5XX_SET_PWR(2): reset and power on with firmware download enabled
 */

#define PWR_OFF 0
#define PWR_ON  1
#define PWR_FW  2

#define CLK_OFF 0
#define CLK_ON  1

#define PN5XX_SET_PWR    _IOW(PN5XX_MAGIC, 0x01, unsigned int)

/*
 * SPI Request NFCC to enable p61 power, only in param
 * Only for SPI
 * level 1 = Enable power
 * level 0 = Disable power
 */
#define P61_SET_SPI_PWR    _IOW(PN5XX_MAGIC, 0x02, unsigned int)

/* SPI or DWP can call this ioctl to get the current
 * power state of P61
 *
*/
#define P61_GET_PWR_STATUS    _IOR(PN5XX_MAGIC, 0x03, unsigned int)

/* DWP side this ioctl will be called
 * level 1 = Wired access is enabled/ongoing
 * level 0 = Wired access is disalbed/stopped
*/
#define P61_SET_WIRED_ACCESS _IOW(PN5XX_MAGIC, 0x04, unsigned int)

/*
  NFC Init will call the ioctl to register the PID with the i2c driver
*/
#define PN5XX_SET_NFC_SERVICE_PID _IOW(PN5XX_MAGIC, 0x05, long)
/*
  NFC and SPI will call the ioctl to get the i2c/spi bus access
*/
#define PN5XX_GET_ESE_ACCESS _IOW(PN5XX_MAGIC, 0x06, long)

/*
  NFC and SPI will call the ioctl to update the power scheme
*/
#define PN5XX_SET_POWER_SCHEME _IOW(PN5XX_MAGIC, 0x07, long)
#define PN5XX_CLK_REQ   _IOW(PN5XX_MAGIC, 0x09, unsigned int)

typedef enum p61_access_state{
    P61_STATE_INVALID = 0x0000,
    P61_STATE_IDLE = 0x0100, /* p61 is free to use */
    P61_STATE_WIRED = 0x0200,  /* p61 is being accessed by DWP (NFCC)*/
    P61_STATE_SPI = 0x0400, /* P61 is being accessed by SPI */
    P61_STATE_DWNLD = 0x0800, /* NFCC fw download is in progress */
    P61_STATE_SPI_PRIO = 0x1000, /*Start of p61 access by SPI on priority*/
    P61_STATE_SPI_PRIO_END = 0x2000, /*End of p61 access by SPI on priority*/
    P61_STATE_SPI_END = 0x4000,
}p61_access_state_t;

typedef enum chip_type_pwr_scheme{
    PN67T_PWR_SCHEME = 0x01,
    PN80T_LEGACY_PWR_SCHEME,
    PN80T_EXT_PMU_SCHEME,
}chip_pwr_scheme_t;

struct pn5xx_i2c_platform_data {
    unsigned int irq_gpio;
    unsigned int ven_gpio;
    unsigned int firm_gpio;
    unsigned int ese_pwr_gpio; /* gpio to give power to p61, only TEE should use this */
    unsigned int clkreq_gpio;
    struct regulator *pvdd_reg;
    struct regulator *vbat_reg;
    struct regulator *pmuvcc_reg;
    struct regulator *sevdd_reg;
    unsigned int iso_rst_gpio; /* gpio used for ISO hard reset P73*/
};
