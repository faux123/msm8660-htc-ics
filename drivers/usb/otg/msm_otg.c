/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>

#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/gadget.h>
#include <linux/usb/hcd.h>
#include <linux/usb/msm_hsusb.h>
#include <linux/usb/msm_hsusb_hw.h>
#include <linux/usb/htc_info.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/pm8xxx/pm8921-charger.h>
#include <linux/pm_qos_params.h>

#include <mach/clk.h>
#include <mach/cable_detect.h>

#ifdef CONFIG_FORCE_FAST_CHARGE
#include <linux/fastchg.h>
#endif

#define MSM_USB_BASE	(motg->regs)
#define DRIVER_NAME	"msm_otg"

static int htc_otg_vbus;
static int htc_otg_id = 1;
static struct msm_otg *the_msm_otg;

static DEFINE_MUTEX(notify_sem);
static void send_usb_connect_notify(struct work_struct *w)
{
	static struct t_usb_status_notifier *notifier;
	struct msm_otg *motg = container_of(w, struct msm_otg,	notifier_work);
	if (!motg)
		return;

	motg->connect_type_ready = 1;
	USBH_INFO("send connect type %d\n", motg->connect_type);
#ifdef CONFIG_FORCE_FAST_CHARGE
	if (motg->connect_type == CONNECT_TYPE_USB) {
		USB_peripheral_detected = USB_ACC_DETECTED; /* Inform forced fast charge that a USB accessory has been attached */
		USBH_INFO("USB forced fast charge : USB device currently attached");
	} else {
		USB_peripheral_detected = USB_ACC_NOT_DETECTED; /* Inform forced fast charge that a USB accessory has not been attached */
		USBH_INFO("USB forced fast charge : No USB device currently attached");
	}
#endif
	mutex_lock(&notify_sem);
	list_for_each_entry(notifier, &g_lh_usb_notifier_list, notifier_link) {
		if (notifier->func != NULL) {
			/* Notify other drivers about connect type. */
			/* use slow charging for unknown type*/
			if (motg->connect_type == CONNECT_TYPE_UNKNOWN)
				notifier->func(CONNECT_TYPE_USB);
			else
				notifier->func(motg->connect_type);
		}
	}
	mutex_unlock(&notify_sem);
}

int usb_register_notifier(struct t_usb_status_notifier *notifier)
{
	if (!notifier || !notifier->name || !notifier->func)
		return -EINVAL;

	mutex_lock(&notify_sem);
	list_add(&notifier->notifier_link,
		&g_lh_usb_notifier_list);
	mutex_unlock(&notify_sem);
	return 0;
}

int usb_is_connect_type_ready(void)
{
	if (!the_msm_otg)
		return 0;
	return the_msm_otg->connect_type_ready;
}
EXPORT_SYMBOL(usb_is_connect_type_ready);

int usb_get_connect_type(void)
{
	if (!the_msm_otg)
		return 0;
#ifdef CONFIG_MACH_VERDI_LTE
	if (the_msm_otg->connect_type == CONNECT_TYPE_USB_9V_AC)
		return CONNECT_TYPE_9V_AC;
#endif
	return the_msm_otg->connect_type;
}
EXPORT_SYMBOL(usb_get_connect_type);

#define ID_TIMER_FREQ		(jiffies + msecs_to_jiffies(2000))
#define ULPI_IO_TIMEOUT_USEC	(10 * 1000)

#ifdef CONFIG_ARCH_MSM8X60
#define USB_PHY_3P3_VOL_MIN	3450000 /* uV */
#define USB_PHY_3P3_VOL_MAX	3450000 /* uV */
#else
#define USB_PHY_3P3_VOL_MIN	3050000 /* uV */
#define USB_PHY_3P3_VOL_MAX	3300000 /* uV */
#endif
#define USB_PHY_3P3_HPM_LOAD	50000	/* uA */
#define USB_PHY_3P3_LPM_LOAD	4000	/* uA */

#define USB_PHY_1P8_VOL_MIN	1800000 /* uV */
#define USB_PHY_1P8_VOL_MAX	1800000 /* uV */
#define USB_PHY_1P8_HPM_LOAD	50000	/* uA */
#define USB_PHY_1P8_LPM_LOAD	4000	/* uA */

#define USB_PHY_VDD_DIG_VOL_MIN	1045000 /* uV */
#define USB_PHY_VDD_DIG_VOL_MAX	1320000 /* uV */

static bool debug_aca_enabled;

/* Prevent idle power collapse(pc) while operating in peripheral mode */
static void otg_pm_qos_update_latency(struct msm_otg *dev, int vote)
{
	struct msm_otg_platform_data *pdata = dev->pdata;
	u32 swfi_latency = 0;

	if (!pdata)
		return;

	swfi_latency = pdata->swfi_latency + 1;

	if (vote)
		pm_qos_update_request(&dev->pm_qos_req_dma,
				swfi_latency);
	else
		pm_qos_update_request(&dev->pm_qos_req_dma,
				PM_QOS_DEFAULT_VALUE);
}

static struct regulator *hsusb_3p3;
static struct regulator *hsusb_1p8;
static struct regulator *hsusb_vddcx;

static inline bool aca_enabled(void)
{
#ifdef CONFIG_USB_MSM_ACA
	return true;
#else
	return debug_aca_enabled;
#endif
}

static int msm_hsusb_init_vddcx(struct msm_otg *motg, int init)
{
	int ret = 0;

	if (init) {
		hsusb_vddcx = regulator_get(motg->otg.dev, motg->pdata->vddcx_name);
		if (IS_ERR(hsusb_vddcx)) {
			USBH_ERR("unable to get hsusb vddcx\n");
			return PTR_ERR(hsusb_vddcx);
		}

		ret = regulator_set_voltage(hsusb_vddcx,
				USB_PHY_VDD_DIG_VOL_MIN,
				USB_PHY_VDD_DIG_VOL_MAX);
		if (ret) {
			USBH_ERR("unable to set the voltage "
					"for hsusb vddcx\n");
			regulator_put(hsusb_vddcx);
			return ret;
		}

		ret = regulator_enable(hsusb_vddcx);
		if (ret) {
			regulator_set_voltage(hsusb_vddcx, 0,
			USB_PHY_VDD_DIG_VOL_MIN);
			regulator_put(hsusb_vddcx);
			USBH_ERR("unable to enable the hsusb vddcx\n");
			return ret;
		}

	} else {

		ret = regulator_disable(hsusb_vddcx);
		if (ret) {
			USBH_ERR("unable to disable hsusb vddcx\n");
			return ret;
		}

		ret = regulator_set_voltage(hsusb_vddcx, 0,
			USB_PHY_VDD_DIG_VOL_MIN);
		if (ret) {
			USBH_ERR("unable to set the voltage"
					"for hsusb vddcx\n");
			return ret;
		}

		regulator_put(hsusb_vddcx);
	}

	return ret;
}

static int msm_hsusb_ldo_init(struct msm_otg *motg, int init)
{
	int rc = 0;

	if (init) {
		if (motg->pdata->ldo_3v3_name)
			hsusb_3p3 = regulator_get(motg->otg.dev, motg->pdata->ldo_3v3_name);
		else
			hsusb_3p3 = regulator_get(motg->otg.dev, "HSUSB_3p3");
		if (IS_ERR(hsusb_3p3)) {
			USBH_ERR("unable to get hsusb 3p3\n");
			return PTR_ERR(hsusb_3p3);
		}

		rc = regulator_set_voltage(hsusb_3p3, USB_PHY_3P3_VOL_MIN,
				USB_PHY_3P3_VOL_MAX);
		if (rc) {
			USBH_ERR("unable to set voltage level for"
					"hsusb 3p3\n");
			goto put_3p3;
		}

		USBH_INFO("%s: ldo3v3, min volt: %d, max volt: %d\n", __func__,
				USB_PHY_3P3_VOL_MIN, USB_PHY_3P3_VOL_MAX);

		if (motg->pdata->ldo_1v8_name)
			hsusb_1p8 = regulator_get(motg->otg.dev, motg->pdata->ldo_1v8_name);
		else
			hsusb_1p8 = regulator_get(motg->otg.dev, "HSUSB_1p8");
		if (IS_ERR(hsusb_1p8)) {
			USBH_ERR("unable to get hsusb 1p8\n");
			rc = PTR_ERR(hsusb_1p8);
			goto put_3p3_lpm;
		}
		rc = regulator_set_voltage(hsusb_1p8, USB_PHY_1P8_VOL_MIN,
				USB_PHY_1P8_VOL_MAX);
		if (rc) {
			dev_err(motg->otg.dev, "unable to set voltage level for"
					"hsusb 1p8\n");
			goto put_1p8;
		}

		return 0;
	}

put_1p8:
	regulator_set_voltage(hsusb_1p8, 0, USB_PHY_1P8_VOL_MAX);
	regulator_put(hsusb_1p8);
put_3p3_lpm:
	regulator_set_voltage(hsusb_3p3, 0, USB_PHY_3P3_VOL_MAX);
put_3p3:
	regulator_put(hsusb_3p3);
	return rc;
}
#ifdef CONFIG_PM_SLEEP
#define USB_PHY_SUSP_DIG_VOL  500000
static int msm_hsusb_config_vddcx(int high)
{
	int max_vol = USB_PHY_VDD_DIG_VOL_MAX;
	int min_vol;
	int ret;

	if (high)
		min_vol = USB_PHY_VDD_DIG_VOL_MIN;
	else
		min_vol = USB_PHY_SUSP_DIG_VOL;

	ret = regulator_set_voltage(hsusb_vddcx, min_vol, max_vol);
	if (ret) {
		USBH_ERR("%s: unable to set the voltage for regulator "
			"HSUSB_VDDCX\n", __func__);
		return ret;
	}

	USBH_DEBUG("%s: min_vol:%d max_vol:%d\n", __func__, min_vol, max_vol);

	return ret;
}
#else
static int msm_hsusb_config_vddcx(int high)
{
	return 0;
}
#endif

static int msm_hsusb_ldo_enable(struct msm_otg *motg, int on)
{
	int ret = 0;

	if (IS_ERR(hsusb_1p8)) {
		USBH_ERR("%s: HSUSB_1p8 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (IS_ERR(hsusb_3p3)) {
		USBH_ERR("%s: HSUSB_3p3 is not initialized\n", __func__);
		return -ENODEV;
	}

	if (on) {
		ret = regulator_set_optimum_mode(hsusb_1p8,
				USB_PHY_1P8_HPM_LOAD);
		if (ret < 0) {
			USBH_ERR("%s: Unable to set HPM of the regulator:"
				"HSUSB_1p8\n", __func__);
			return ret;
		}

		ret = regulator_enable(hsusb_1p8);
		if (ret) {
			USBH_ERR("%s: unable to enable the hsusb 1p8\n",
				__func__);
			regulator_set_optimum_mode(hsusb_1p8, 0);
			return ret;
		}

		ret = regulator_set_optimum_mode(hsusb_3p3,
				USB_PHY_3P3_HPM_LOAD);
		if (ret < 0) {
			USBH_ERR("%s: Unable to set HPM of the regulator:"
				"HSUSB_3p3\n", __func__);
			regulator_set_optimum_mode(hsusb_1p8, 0);
			regulator_disable(hsusb_1p8);
			return ret;
		}

		ret = regulator_enable(hsusb_3p3);
		if (ret) {
			USBH_ERR("%s: unable to enable the hsusb 3p3\n",
				__func__);
			regulator_set_optimum_mode(hsusb_3p3, 0);
			regulator_set_optimum_mode(hsusb_1p8, 0);
			regulator_disable(hsusb_1p8);
			return ret;
		}

	} else {
		ret = regulator_disable(hsusb_1p8);
		if (ret) {
			USBH_ERR("%s: unable to disable the hsusb 1p8\n",
				__func__);
			return ret;
		}

		ret = regulator_set_optimum_mode(hsusb_1p8, 0);
		if (ret < 0)
			USBH_ERR("%s: Unable to set LPM of the regulator:"
				"HSUSB_1p8\n", __func__);

		ret = regulator_disable(hsusb_3p3);
		if (ret) {
			USBH_ERR("%s: unable to disable the hsusb 3p3\n",
				 __func__);
			return ret;
		}
		ret = regulator_set_optimum_mode(hsusb_3p3, 0);
		if (ret < 0)
			USBH_ERR("%s: Unable to set LPM of the regulator:"
				"HSUSB_3p3\n", __func__);
	}

	USBH_DEBUG("reg (%s)\n", on ? "HPM" : "LPM");
	return ret < 0 ? ret : 0;
}

static const char *state_string(enum usb_otg_state state)
{
	switch (state) {
	case OTG_STATE_A_IDLE:		return "a_idle";
	case OTG_STATE_A_WAIT_VRISE:	return "a_wait_vrise";
	case OTG_STATE_A_WAIT_BCON:	return "a_wait_bcon";
	case OTG_STATE_A_HOST:		return "a_host";
	case OTG_STATE_A_SUSPEND:	return "a_suspend";
	case OTG_STATE_A_PERIPHERAL:	return "a_peripheral";
	case OTG_STATE_A_WAIT_VFALL:	return "a_wait_vfall";
	case OTG_STATE_A_VBUS_ERR:	return "a_vbus_err";
	case OTG_STATE_B_IDLE:		return "b_idle";
	case OTG_STATE_B_SRP_INIT:	return "b_srp_init";
	case OTG_STATE_B_PERIPHERAL:	return "b_peripheral";
	case OTG_STATE_B_WAIT_ACON:	return "b_wait_acon";
	case OTG_STATE_B_HOST:		return "b_host";
	default:			return "UNDEFINED";
	}
}

static const char *chg_state_string(enum usb_chg_state state)
{
	switch (state) {
	case USB_CHG_STATE_UNDEFINED:		return "CHG_STATE_UNDEFINED";
	case USB_CHG_STATE_WAIT_FOR_DCD:	return "CHG_WAIT_FOR_DCD";
	case USB_CHG_STATE_DCD_DONE:	return "CHG_DCD_DONE";
	case USB_CHG_STATE_PRIMARY_DONE:		return "CHG_PRIMARY_DONE";
	case USB_CHG_STATE_SECONDARY_DONE:	return "CHG_SECONDARY_DONE";
	case USB_CHG_STATE_DETECTED:	return "CHG_DETECTED";
	default:			return "UNDEFINED";
	}
}

static void msm_hsusb_mhl_switch_enable(struct msm_otg *motg, bool on)
{
	static struct regulator *mhl_analog_switch;
	struct msm_otg_platform_data *pdata = motg->pdata;

	if (!pdata->mhl_enable)
		return;

	if (on) {
		mhl_analog_switch = regulator_get(motg->otg.dev,
					       "mhl_ext_3p3v");
		if (IS_ERR(mhl_analog_switch)) {
			pr_err("Unable to get mhl_analog_switch\n");
			return;
		}

		if (regulator_enable(mhl_analog_switch)) {
			pr_err("unable to enable mhl_analog_switch\n");
			goto put_analog_switch;
		}
		return;
	}

	regulator_disable(mhl_analog_switch);
put_analog_switch:
	regulator_put(mhl_analog_switch);
}

static int ulpi_read(struct otg_transceiver *otg, u32 reg)
{
	struct msm_otg *motg = container_of(otg, struct msm_otg, otg);
	int cnt = 0;

	if (motg->pdata->phy_type == CI_45NM_INTEGRATED_PHY)
		udelay(200);

	/* initiate read operation */
	writel(ULPI_RUN | ULPI_READ | ULPI_ADDR(reg),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while (cnt < ULPI_IO_TIMEOUT_USEC) {
		if (!(readl(USB_ULPI_VIEWPORT) & ULPI_RUN))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= ULPI_IO_TIMEOUT_USEC) {
		USBH_ERR("ulpi_read: timeout %08x reg: 0x%x\n",
			readl(USB_ULPI_VIEWPORT), reg);
		return -ETIMEDOUT;
	}
	return ULPI_DATA_READ(readl(USB_ULPI_VIEWPORT));
}

static int ulpi_write(struct otg_transceiver *otg, u32 val, u32 reg)
{
	struct msm_otg *motg = container_of(otg, struct msm_otg, otg);
	int cnt = 0;

	if (motg->pdata->phy_type == CI_45NM_INTEGRATED_PHY)
		udelay(200);

	/* initiate write operation */
	writel(ULPI_RUN | ULPI_WRITE |
	       ULPI_ADDR(reg) | ULPI_DATA(val),
	       USB_ULPI_VIEWPORT);

	/* wait for completion */
	while (cnt < ULPI_IO_TIMEOUT_USEC) {
		if (!(readl(USB_ULPI_VIEWPORT) & ULPI_RUN))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= ULPI_IO_TIMEOUT_USEC) {
		USBH_ERR("ulpi_write: timeout reg: 0x%x ,val: 0x%x\n", reg, val);
		return -ETIMEDOUT;
	}
	return 0;
}

ssize_t otg_show_usb_phy_setting(char *buf)
{
	struct msm_otg *motg = the_msm_otg;
	unsigned length = 0;
	int i;

	for (i = 0; i <= 0x14; i++)
		length += sprintf(buf + length, "0x%x = 0x%x\n",
					i, ulpi_read(&motg->otg, i));

	for (i = 0x30; i <= 0x37; i++)
		length += sprintf(buf + length, "0x%x = 0x%x\n",
					i, ulpi_read(&motg->otg, i));

	if (motg->pdata->phy_type == SNPS_28NM_INTEGRATED_PHY) {
	for (i = 0x80; i <= 0x83; i++)
		length += sprintf(buf + length, "0x%x = 0x%x\n",
					i, ulpi_read(&motg->otg, i));
	}

	return length;
}

ssize_t otg_store_usb_phy_setting(const char *buf, size_t count)
{
	struct msm_otg *motg = the_msm_otg;
	char *token[10];
	unsigned long reg;
	unsigned long value;
	int i;

	USBH_INFO("%s\n", buf);
	for (i = 0; i < 2; i++)
		token[i] = strsep((char **)&buf, " ");

	i = strict_strtoul(token[0], 16, (unsigned long *)&reg);
	if (i < 0) {
		USBH_ERR("%s: reg %d\n", __func__, i);
		return 0;
	}
	i = strict_strtoul(token[1], 16, (unsigned long *)&value);
	if (i < 0) {
		USBH_ERR("%s: value %d\n", __func__, i);
		return 0;
	}
	USBH_INFO("Set 0x%02lx = 0x%02lx\n", reg, value);

	ulpi_write(&motg->otg, value, reg);

	return count;
}

static struct otg_io_access_ops msm_otg_io_ops = {
	.read = ulpi_read,
	.write = ulpi_write,
};

static void ulpi_init(struct msm_otg *motg)
{
	struct msm_otg_platform_data *pdata = motg->pdata;
	int *seq = pdata->phy_init_seq;

	if (!seq)
		return;

	while (seq[0] >= 0) {
		USBH_INFO("ulpi: write 0x%02x to 0x%02x\n",
				seq[0], seq[1]);
		ulpi_write(&motg->otg, seq[0], seq[1]);
		seq += 2;
	}
}

static int msm_otg_link_clk_reset(struct msm_otg *motg, bool assert)
{
	int ret;

	if (assert) {
		ret = clk_reset(motg->clk, CLK_RESET_ASSERT);
		if (ret)
			USBH_ERR("usb hs_clk assert failed\n");
	} else {
		ret = clk_reset(motg->clk, CLK_RESET_DEASSERT);
		if (ret)
			USBH_ERR("usb hs_clk deassert failed\n");
	}
	return ret;
}

static int msm_otg_phy_clk_reset(struct msm_otg *motg)
{
	int ret;

	if (IS_ERR(motg->phy_reset_clk))
		return 0;

	ret = clk_reset(motg->phy_reset_clk, CLK_RESET_ASSERT);
	if (ret) {
		USBH_ERR("usb phy clk assert failed\n");
		return ret;
	}
	usleep_range(10000, 12000);
	ret = clk_reset(motg->phy_reset_clk, CLK_RESET_DEASSERT);
	if (ret)
		USBH_ERR("usb phy clk deassert failed\n");
	return ret;
}

static int msm_otg_phy_reset(struct msm_otg *motg)
{
	u32 val;
	int ret;
	int retries;

	ret = msm_otg_link_clk_reset(motg, 1);
	if (ret)
		return ret;
	ret = msm_otg_phy_clk_reset(motg);
	if (ret)
		return ret;
	ret = msm_otg_link_clk_reset(motg, 0);
	if (ret)
		return ret;

	val = readl(USB_PORTSC) & ~PORTSC_PTS_MASK;
	writel(val | PORTSC_PTS_ULPI, USB_PORTSC);

	for (retries = 3; retries > 0; retries--) {
		ret = ulpi_write(&motg->otg, ULPI_FUNC_CTRL_SUSPENDM,
				ULPI_CLR(ULPI_FUNC_CTRL));
		if (!ret)
			break;
		ret = msm_otg_phy_clk_reset(motg);
		if (ret)
			return ret;
	}
	if (!retries)
		return -ETIMEDOUT;

	/* This reset calibrates the phy, if the above write succeeded */
	ret = msm_otg_phy_clk_reset(motg);
	if (ret)
		return ret;

	for (retries = 3; retries > 0; retries--) {
		ret = ulpi_read(&motg->otg, ULPI_DEBUG);
		if (ret != -ETIMEDOUT)
			break;
		ret = msm_otg_phy_clk_reset(motg);
		if (ret)
			return ret;
	}
	if (!retries)
		return -ETIMEDOUT;

	USBH_INFO("phy_reset: success\n");
	return 0;
}

#define LINK_RESET_TIMEOUT_USEC		(250 * 1000)
static int msm_otg_link_reset(struct msm_otg *motg)
{
	int cnt = 0;

	writel_relaxed(USBCMD_RESET, USB_USBCMD);
	while (cnt < LINK_RESET_TIMEOUT_USEC) {
		if (!(readl_relaxed(USB_USBCMD) & USBCMD_RESET))
			break;
		udelay(1);
		cnt++;
	}
	if (cnt >= LINK_RESET_TIMEOUT_USEC)
		return -ETIMEDOUT;

	/* select ULPI phy */
	writel_relaxed(0x80000000, USB_PORTSC);
	writel_relaxed(0x0, USB_AHBBURST);
	writel_relaxed(0x00, USB_AHBMODE);

	return 0;
}

static int msm_otg_reset(struct otg_transceiver *otg)
{
	struct msm_otg *motg = container_of(otg, struct msm_otg, otg);
	struct msm_otg_platform_data *pdata = motg->pdata;
	int ret;
	u32 val = 0;
	u32 ulpi_val = 0;
	USBH_INFO("%s\n", __func__);

#ifdef CONFIG_FORCE_FAST_CHARGE
	USB_porttype_detected = NO_USB_DETECTED; /* No USB plugged, clear fast charge detected port value */
#endif

	clk_enable(motg->clk);
	if (motg->pdata->phy_reset)
		ret = motg->pdata->phy_reset();
	else
		ret = msm_otg_phy_reset(motg);
	if (ret) {
		USBH_ERR("phy_reset failed\n");
		return ret;
	}

	/* suppress id signal from phy */
	if (readl(USB_OTGSC) & OTGSC_IDPU)
		writel(readl(USB_OTGSC) & ~OTGSC_IDPU, USB_OTGSC);

	ret = msm_otg_link_reset(motg);
	if (ret) {
		dev_err(otg->dev, "link reset failed\n");
		return ret;
	}
	msleep(100);

	ulpi_init(motg);

	/* Ensure that RESET operation is completed before turning off clock */
	mb();

	clk_disable(motg->clk);

	if ((pdata->otg_control == OTG_PHY_CONTROL) || motg->pdata->phy_notify_enabled) {
		val = readl_relaxed(USB_OTGSC);
		if (pdata->mode == USB_OTG) {
			ulpi_val = ULPI_INT_IDGRD | ULPI_INT_SESS_VALID;
			val |= OTGSC_IDIE | OTGSC_BSVIE;
		} else if (pdata->mode == USB_PERIPHERAL) {
			ulpi_val = ULPI_INT_SESS_VALID;
			val |= OTGSC_BSVIE;
		}
		writel_relaxed(val, USB_OTGSC);
		ulpi_write(otg, ulpi_val, ULPI_USB_INT_EN_RISE);
		ulpi_write(otg, ulpi_val, ULPI_USB_INT_EN_FALL);
	}

	return 0;
}

static int msm_otg_set_suspend(struct otg_transceiver *otg, int suspend)
{
	struct msm_otg *motg = container_of(otg, struct msm_otg, otg);

	/*
	 * Allow bus suspend only for host mode.  Device mode bus suspend
	 * is not implemented yet.
	 */
	if (!test_bit(ID, &motg->inputs) || test_bit(ID_A, &motg->inputs)) {
		/*
		 * ID_GND --> ID_A transition can not be detected in LPM.
		 * Disallow host bus suspend when ACA is enabled.
		 */
		if (suspend && !aca_enabled())
			pm_runtime_put(otg->dev);
		else
			pm_runtime_resume(otg->dev);
	}

	return 0;
}

#define PHY_SUSPEND_TIMEOUT_USEC	(500 * 1000)
#define PHY_RESUME_TIMEOUT_USEC	(100 * 1000)

#ifdef CONFIG_PM_SLEEP
static int msm_otg_suspend(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	struct usb_bus *bus = otg->host;
	struct msm_otg_platform_data *pdata = motg->pdata;
	int cnt = 0, ret = 0;
	bool session_active;
	u32 phy_ctrl_val = 0;

	if (atomic_read(&motg->in_lpm))
		return 0;

	USBH_INFO("%s\n", __func__);

	disable_irq(motg->irq);
	session_active = (otg->host && !test_bit(ID, &motg->inputs)) ||
				test_bit(B_SESS_VLD, &motg->inputs);
	/*
	 * Chipidea 45-nm PHY suspend sequence:
	 *
	 * Interrupt Latch Register auto-clear feature is not present
	 * in all PHY versions. Latch register is clear on read type.
	 * Clear latch register to avoid spurious wakeup from
	 * low power mode (LPM).
	 *
	 * PHY comparators are disabled when PHY enters into low power
	 * mode (LPM). Keep PHY comparators ON in LPM only when we expect
	 * VBUS/Id notifications from USB PHY. Otherwise turn off USB
	 * PHY comparators. This save significant amount of power.
	 *
	 * PLL is not turned off when PHY enters into low power mode (LPM).
	 * Disable PLL for maximum power savings.
	 */

	if (motg->pdata->phy_type == CI_45NM_INTEGRATED_PHY) {
		ulpi_read(otg, 0x14);
		if ((pdata->otg_control == OTG_PHY_CONTROL) || motg->pdata->phy_notify_enabled)
			ulpi_write(otg, 0x01, 0x30);
		ulpi_write(otg, 0x08, 0x09);
	}

	/*
	 * Turn off the OTG comparators, if depends on PMIC for
	 * VBUS and ID notifications.
	 */
	if ((motg->caps & ALLOW_PHY_COMP_DISABLE) && !session_active) {
		ulpi_write(otg, OTG_COMP_DISABLE,
			ULPI_SET(ULPI_PWR_CLK_MNG_REG));
		motg->lpm_flags |= PHY_OTG_COMP_DISABLED;
	}

	/*
	 * PHY may take some time or even fail to enter into low power
	 * mode (LPM). Hence poll for 500 msec and reset the PHY and link
	 * in failure case.
	 */
	writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC);
	while (cnt < PHY_SUSPEND_TIMEOUT_USEC) {
		if (readl(USB_PORTSC) & PORTSC_PHCD)
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= PHY_SUSPEND_TIMEOUT_USEC) {
		USBH_ERR("Unable to suspend PHY\n");
		msm_otg_reset(otg);
		enable_irq(motg->irq);
		return -ETIMEDOUT;
	}

	/*
	 * PHY has capability to generate interrupt asynchronously in low
	 * power mode (LPM). This interrupt is level triggered. So USB IRQ
	 * line must be disabled till async interrupt enable bit is cleared
	 * in USBCMD register. Assert STP (ULPI interface STOP signal) to
	 * block data communication from PHY.
	 */
	/* Remove ASYNC_INTR_CTRL to avoid random wakeup */
	writel(readl(USB_USBCMD) | ULPI_STP_CTRL, USB_USBCMD);

	if (motg->caps & ALLOW_PHY_RETENTION && !session_active) {
		phy_ctrl_val = readl_relaxed(USB_PHY_CTRL);
		if (motg->pdata->otg_control == OTG_PHY_CONTROL)
			/* Enable PHY HV interrupts to wake MPM/Link */
			phy_ctrl_val |=
				(PHY_IDHV_INTEN | PHY_OTGSESSVLDHV_INTEN);

		writel_relaxed(phy_ctrl_val & ~PHY_RETEN, USB_PHY_CTRL);
		motg->lpm_flags |= PHY_RETENTIONED;
	}

	/* Ensure that above operation is completed before turning off clocks */
	mb();
	clk_disable(motg->pclk);
	if (motg->core_clk)
		clk_disable(motg->core_clk);

	if (motg->caps & ALLOW_PHY_POWER_COLLAPSE && !session_active) {
		msm_hsusb_ldo_enable(motg, 0);
		motg->lpm_flags |= PHY_PWR_COLLAPSED;
	}

	if (motg->pdata->vddcx_name) {
		msm_hsusb_config_vddcx(0);
		msm_hsusb_mhl_switch_enable(motg, 0);
	}

	/* usb phy no more require PXO clock, hence vote for PXO disable*/
	ret = msm_xo_mode_vote(motg->xo_handle, MSM_XO_MODE_OFF);
	if (ret)
		USBH_ERR("%s failed to devote for"
			"PXO buffer%d\n", __func__, ret);


	if (device_may_wakeup(otg->dev)) {
		enable_irq_wake(motg->irq);
		if (motg->pdata->pmic_id_irq)
			enable_irq_wake(motg->pdata->pmic_id_irq);
	}
	if (bus)
		clear_bit(HCD_FLAG_HW_ACCESSIBLE, &(bus_to_hcd(bus))->flags);

	atomic_set(&motg->in_lpm, 1);
	enable_irq(motg->irq);

	USBH_INFO("USB in low power mode\n");

	return 0;
}

static int msm_otg_resume(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	struct usb_bus *bus = otg->host;
	int cnt = 0, ret = 0;
	unsigned temp;
	u32 phy_ctrl_val = 0;

	if (!atomic_read(&motg->in_lpm))
		return 0;

	USBH_INFO("%s\n", __func__);

	clk_enable(motg->core_clk);
	clk_enable(motg->pclk);

	if (motg->lpm_flags & PHY_PWR_COLLAPSED) {
		msm_hsusb_ldo_enable(motg, 1);
		motg->lpm_flags &= ~PHY_PWR_COLLAPSED;
	}

	msm_hsusb_mhl_switch_enable(motg, 1);
	if (motg->pdata->vddcx_name)
		msm_hsusb_config_vddcx(1);
	if (motg->lpm_flags & PHY_RETENTIONED) {
		phy_ctrl_val = readl_relaxed(USB_PHY_CTRL);
		phy_ctrl_val |= PHY_RETEN;
		if ((motg->pdata->otg_control == OTG_PHY_CONTROL) || motg->pdata->phy_notify_enabled)
			/* Disable PHY HV interrupts */
			phy_ctrl_val &=
				~(PHY_IDHV_INTEN | PHY_OTGSESSVLDHV_INTEN);
		writel_relaxed(phy_ctrl_val, USB_PHY_CTRL);
		motg->lpm_flags &= ~PHY_RETENTIONED;
	}

	/* Vote for PXO when waking up the phy */
	ret = msm_xo_mode_vote(motg->xo_handle, MSM_XO_MODE_ON);
	if (ret)
		USBH_ERR("%s failed to vote for"
			"PXO buffer%d\n", __func__, ret);


	temp = readl(USB_USBCMD);
	temp &= ~ULPI_STP_CTRL;
	writel(temp, USB_USBCMD);

	/*
	 * PHY comes out of low power mode (LPM) in case of wakeup
	 * from asynchronous interrupt.
	 */
	if (!(readl(USB_PORTSC) & PORTSC_PHCD))
		goto skip_phy_resume;

	writel(readl(USB_PORTSC) & ~PORTSC_PHCD, USB_PORTSC);
	while (cnt < PHY_RESUME_TIMEOUT_USEC) {
		if (!(readl(USB_PORTSC) & PORTSC_PHCD))
			break;
		udelay(1);
		cnt++;
	}

	if (cnt >= PHY_RESUME_TIMEOUT_USEC) {
		/*
		 * This is a fatal error. Reset the link and
		 * PHY. USB state can not be restored. Re-insertion
		 * of USB cable is the only way to get USB working.
		 */
		USBH_ERR("Unable to resume USB."
				"Re-plugin the cable\n");
		msm_otg_reset(otg);
	}

skip_phy_resume:
	/* Turn on the OTG comparators on resume */
	if (motg->lpm_flags & PHY_OTG_COMP_DISABLED) {
		ulpi_write(otg, OTG_COMP_DISABLE,
			ULPI_CLR(ULPI_PWR_CLK_MNG_REG));
		motg->lpm_flags &= ~PHY_OTG_COMP_DISABLED;
	}
	if (device_may_wakeup(otg->dev)) {
		disable_irq_wake(motg->irq);
		if (motg->pdata->pmic_id_irq)
			disable_irq_wake(motg->pdata->pmic_id_irq);
	}
	if (bus)
		set_bit(HCD_FLAG_HW_ACCESSIBLE, &(bus_to_hcd(bus))->flags);

	atomic_set(&motg->in_lpm, 0);

	if (aca_enabled() && !irq_read_line(motg->pdata->pmic_id_irq)) {
		clear_bit(ID, &motg->inputs);
		schedule_work(&motg->sm_work);
	}

	if (motg->async_int) {
		motg->async_int = 0;
		enable_irq(motg->irq);
	}

	USBH_INFO("USB exited from low power mode\n");

	return 0;
}
#endif

static void msm_otg_notify_charger(struct msm_otg *motg, unsigned mA)
{
	if ((motg->chg_type == USB_ACA_DOCK_CHARGER ||
		motg->chg_type == USB_ACA_A_CHARGER ||
		motg->chg_type == USB_ACA_B_CHARGER ||
		motg->chg_type == USB_ACA_C_CHARGER) &&
			mA > IDEV_ACA_CHG_LIMIT)
		mA = IDEV_ACA_CHG_LIMIT;

	if (motg->cur_power == mA)
		return;

	USBH_INFO("Avail curr from USB = %u\n", mA);
	pm8921_charger_vbus_draw(mA);
	motg->cur_power = mA;
}

static void msm_otg_notify_usb_attached(void)
{
	struct msm_otg *motg = the_msm_otg;

	if (motg->connect_type != CONNECT_TYPE_USB) {
		motg->connect_type = CONNECT_TYPE_USB;
		queue_work(motg->usb_wq, &motg->notifier_work);
	}
	motg->ac_detect_count = 0;
	del_timer(&motg->ac_detect_timer);
}

static int msm_otg_set_power(struct otg_transceiver *otg, unsigned mA)
{
	struct msm_otg *motg = container_of(otg, struct msm_otg, otg);

	/*
	 * Gadget driver uses set_power method to notify about the
	 * available current based on suspend/configured states.
	 *
	 * IDEV_CHG can be drawn irrespective of suspend/un-configured
	 * states when CDP/ACA is connected.
	 */
	if (motg->chg_type == USB_SDP_CHARGER)
		msm_otg_notify_charger(motg, mA);

	return 0;
}

static void msm_otg_start_host(struct otg_transceiver *otg, int on)
{
	struct msm_otg *motg = container_of(otg, struct msm_otg, otg);
	struct msm_otg_platform_data *pdata = motg->pdata;
	struct usb_hcd *hcd;

	if (!otg->host)
		return;

	hcd = bus_to_hcd(otg->host);

	if (on) {
		USBH_DEBUG("host on\n");

		/*
		 * Some boards have a switch cotrolled by gpio
		 * to enable/disable internal HUB. Enable internal
		 * HUB before kicking the host.
		 */
		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_A_HOST);
		usb_add_hcd(hcd, hcd->irq, IRQF_SHARED);
	} else {
		USBH_DEBUG("host off\n");

		usb_remove_hcd(hcd);
		/* HCD core reset all bits of PORTSC. select ULPI phy */
		writel_relaxed(0x80000000, USB_PORTSC);

		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_UNDEFINED);
	}
}

static int msm_otg_usbdev_notify(struct notifier_block *self,
			unsigned long action, void *priv)
{
	struct msm_otg *motg = container_of(self, struct msm_otg, usbdev_nb);
	struct usb_device *udev = priv;

	if (!aca_enabled())
		goto out;

	if (action == USB_BUS_ADD || action == USB_BUS_REMOVE)
		goto out;

	if (udev->bus != motg->otg.host)
		goto out;
	/*
	 * Interested in devices connected directly to the root hub.
	 * ACA dock can supply IDEV_CHG irrespective devices connected
	 * on the accessory port.
	 */
	if (!udev->parent || udev->parent->parent ||
			motg->chg_type == USB_ACA_DOCK_CHARGER)
		goto out;

	switch (action) {
	case USB_DEVICE_ADD:
		usb_disable_autosuspend(udev);
		/* fall through */
	case USB_DEVICE_CONFIG:
		if (udev->actconfig)
			motg->mA_port = udev->actconfig->desc.bMaxPower * 2;
		else
			motg->mA_port = IUNIT;
		break;
	case USB_DEVICE_REMOVE:
		motg->mA_port = IUNIT;
		break;
	default:
		break;
	}
	if (test_bit(ID_A, &motg->inputs))
		msm_otg_notify_charger(motg, IDEV_ACA_CHG_MAX -
				motg->mA_port);
out:
	return NOTIFY_OK;
}

static int msm_otg_set_host(struct otg_transceiver *otg, struct usb_bus *host)
{
	struct msm_otg *motg = container_of(otg, struct msm_otg, otg);
	struct usb_hcd *hcd;

	/*
	 * Fail host registration if this board can support
	 * only peripheral configuration.
	 */
	if (motg->pdata->mode == USB_PERIPHERAL) {
		USBH_ERR("Host mode is not supported\n");
		return -ENODEV;
	}

	if (!host) {
		if (otg->state == OTG_STATE_A_HOST) {
			pm_runtime_get_sync(otg->dev);
			usb_unregister_notify(&motg->usbdev_nb);
			msm_otg_start_host(otg, 0);
			if (motg->pdata->vbus_power)
				motg->pdata->vbus_power(0);
			otg->host = NULL;
			otg->state = OTG_STATE_UNDEFINED;
			schedule_work(&motg->sm_work);
		} else {
			otg->host = NULL;
		}

		return 0;
	}

	hcd = bus_to_hcd(host);
	hcd->power_budget = motg->pdata->power_budget;

	motg->usbdev_nb.notifier_call = msm_otg_usbdev_notify;
	usb_register_notify(&motg->usbdev_nb);
	otg->host = host;
	USBH_DEBUG("host driver registered w/ tranceiver\n");

	/*
	 * Kick the state machine work, if peripheral is not supported
	 * or peripheral is already registered with us.
	 */
	if (motg->pdata->mode == USB_HOST || otg->gadget) {
		pm_runtime_get_sync(otg->dev);
		schedule_work(&motg->sm_work);
	}

	return 0;
}

static void msm_otg_start_peripheral(struct otg_transceiver *otg, int on)
{
	struct msm_otg *motg = container_of(otg, struct msm_otg, otg);
	struct msm_otg_platform_data *pdata = motg->pdata;

	if (!otg->gadget)
		return;

	if (on) {
		USBH_DEBUG("gadget on\n");
		/* FIXME: hold a wake_lock here... */
		wake_lock(&motg->usb_otg_wlock);
		/*
		 * Some boards have a switch cotrolled by gpio
		 * to enable/disable internal HUB. Disable internal
		 * HUB before kicking the gadget.
		 */
		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_B_PERIPHERAL);
		/*
		 * vote for minimum dma_latency to prevent idle
		 * power collapse(pc) while running in peripheral mode.
		 */
		otg_pm_qos_update_latency(motg, 1);
		usb_gadget_vbus_connect(otg->gadget);
	} else {
		USBH_DEBUG("gadget off\n");
		usb_gadget_vbus_disconnect(otg->gadget);
		otg_pm_qos_update_latency(motg, 0);
		if (pdata->setup_gpio)
			pdata->setup_gpio(OTG_STATE_UNDEFINED);
		/* FIXME: release a wake lock here... */
		wake_unlock(&motg->usb_otg_wlock);
	}

}

static int msm_otg_set_peripheral(struct otg_transceiver *otg,
			struct usb_gadget *gadget)
{
	struct msm_otg *motg = container_of(otg, struct msm_otg, otg);

	/*
	 * Fail peripheral registration if this board can support
	 * only host configuration.
	 */
	if (motg->pdata->mode == USB_HOST) {
		USBH_ERR("Peripheral mode is not supported\n");
		return -ENODEV;
	}

	if (!gadget) {
		if (otg->state == OTG_STATE_B_PERIPHERAL) {
			pm_runtime_get_sync(otg->dev);
			msm_otg_start_peripheral(otg, 0);
			otg->gadget = NULL;
			otg->state = OTG_STATE_UNDEFINED;
			schedule_work(&motg->sm_work);
		} else {
			otg->gadget = NULL;
		}

		return 0;
	}
	otg->gadget = gadget;
	USBH_DEBUG("peripheral driver registered w/ tranceiver\n");

	/*
	 * Kick the state machine work, if host is not supported
	 * or host is already registered with us.
	 */
	if (motg->pdata->mode == USB_PERIPHERAL || otg->host) {
		pm_runtime_get_sync(otg->dev);
		schedule_work(&motg->sm_work);
	}

	return 0;
}

static bool msm_chg_aca_detect(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	u32 int_sts;
	bool ret = false;

	if (!aca_enabled())
		goto out;

	if (motg->pdata->phy_type == CI_45NM_INTEGRATED_PHY)
		goto out;

	int_sts = ulpi_read(otg, ULPI_CHRG_DET_OUTPUT);
	switch (int_sts & 0x1C) {
	case 0x08:
		if (!test_and_set_bit(ID_A, &motg->inputs)) {
			USBH_DEBUG("ID_A\n");
			motg->chg_type = USB_ACA_A_CHARGER;
			motg->chg_state = USB_CHG_STATE_DETECTED;
			clear_bit(ID_B, &motg->inputs);
			clear_bit(ID_C, &motg->inputs);
			set_bit(ID, &motg->inputs);
			ret = true;
		}
		break;
	case 0x0C:
		if (!test_and_set_bit(ID_B, &motg->inputs)) {
			USBH_DEBUG("ID_B\n");
			motg->chg_type = USB_ACA_B_CHARGER;
			motg->chg_state = USB_CHG_STATE_DETECTED;
			clear_bit(ID_A, &motg->inputs);
			clear_bit(ID_C, &motg->inputs);
			set_bit(ID, &motg->inputs);
			ret = true;
		}
		break;
	case 0x10:
		if (!test_and_set_bit(ID_C, &motg->inputs)) {
			USBH_DEBUG("ID_C\n");
			motg->chg_type = USB_ACA_C_CHARGER;
			motg->chg_state = USB_CHG_STATE_DETECTED;
			clear_bit(ID_A, &motg->inputs);
			clear_bit(ID_B, &motg->inputs);
			set_bit(ID, &motg->inputs);
			ret = true;
		}
		break;
	case 0x04:
		if (test_and_clear_bit(ID, &motg->inputs)) {
			dev_dbg(otg->dev, "ID_GND\n");
			motg->chg_type = USB_INVALID_CHARGER;
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			clear_bit(ID_A, &motg->inputs);
			clear_bit(ID_B, &motg->inputs);
			clear_bit(ID_C, &motg->inputs);
			ret = true;
		}
		break;
	default:
		ret = test_and_clear_bit(ID_A, &motg->inputs) |
			test_and_clear_bit(ID_B, &motg->inputs) |
			test_and_clear_bit(ID_C, &motg->inputs) |
			!test_and_set_bit(ID, &motg->inputs);
		if (ret) {
			USBH_DEBUG("ID A/B/C/GND is no more\n");
			motg->chg_type = USB_INVALID_CHARGER;
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
		}
	}
out:
	return ret;
}

static void msm_chg_enable_aca_det(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;

	if (!aca_enabled())
		return;

	switch (motg->pdata->phy_type) {
	case SNPS_28NM_INTEGRATED_PHY:
		/* Disable ID_GND in link and PHY */
		writel_relaxed(readl_relaxed(USB_OTGSC) & ~(OTGSC_IDPU |
				OTGSC_IDIE), USB_OTGSC);
		ulpi_write(otg, ULPI_OTG_CTRL_ID_PULLUP, ULPI_OTG_CTRL_C);
		ulpi_write(otg, ULPI_INT_IDGRD, ULPI_USB_INT_EN_RISE_C);
		ulpi_write(otg, ULPI_INT_IDGRD, ULPI_USB_INT_EN_FALL_C);
		/* Enable ACA ID detection */
		ulpi_write(otg, ULPI_ACAENB, ULPI_CHRG_DET_CTRL_S);
		break;
	default:
		break;
	}
}

static void msm_chg_enable_aca_intr(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;

	if (!aca_enabled())
		return;

	switch (motg->pdata->phy_type) {
	case SNPS_28NM_INTEGRATED_PHY:
		/* Enable ACA Detection interrupt (on any RID change) */
		ulpi_write(otg, ULPI_ACADETINTENB, ULPI_ALT_INT_EN_S);
		break;
	default:
		break;
	}
}

static void msm_chg_disable_aca_intr(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;

	if (!aca_enabled())
		return;

	switch (motg->pdata->phy_type) {
	case SNPS_28NM_INTEGRATED_PHY:
		ulpi_write(otg, ULPI_ACADETINTENB, ULPI_ALT_INT_EN_C);
		break;
	default:
		break;
	}
}

static bool msm_chg_check_aca_intr(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	bool ret = false;

	if (!aca_enabled())
		return ret;

	switch (motg->pdata->phy_type) {
	case SNPS_28NM_INTEGRATED_PHY:
		if (ulpi_read(otg, ULPI_ALT_INT_LATCH) & ULPI_ACADETINTLCH) {
			USBH_DEBUG("RID change\n");
			ulpi_write(otg, ULPI_ACADETINTLCH, ULPI_ALT_INT_LATCH_S);
			ret = msm_chg_aca_detect(motg);
		}
	default:
		break;
	}
	return ret;
}

static void msm_otg_id_timer_func(unsigned long data)
{
	struct msm_otg *motg = (struct msm_otg *) data;

	if (!aca_enabled())
		return;

	if (atomic_read(&motg->in_lpm)) {
		dev_dbg(motg->otg.dev, "timer: in lpm\n");
		return;
	}

	if (msm_chg_check_aca_intr(motg)) {
		dev_dbg(motg->otg.dev, "timer: aca work\n");
		schedule_work(&motg->sm_work);
	}

	if (!test_bit(ID, &motg->inputs) || test_bit(ID_A, &motg->inputs))
		mod_timer(&motg->id_timer, ID_TIMER_FREQ);
}

static bool msm_chg_check_secondary_det(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	u32 chg_det;
	bool ret = false;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(otg, 0x34);
		ret = chg_det & (1 << 4);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		chg_det = ulpi_read(otg, ULPI_CHRG_DET_OUTPUT);
		ret = chg_det & ULPI_CHGDET;
		break;
	default:
		break;
	}
	return ret;
}

static void msm_chg_enable_secondary_det(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	u32 chg_det;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(otg, 0x34);
		/* Turn off charger block */
		chg_det |= ~(1 << 1);
		ulpi_write(otg, chg_det, 0x34);
		udelay(20);
		/* control chg block via ULPI */
		chg_det &= ~(1 << 3);
		ulpi_write(otg, chg_det, 0x34);
		/* put it in host mode for enabling D- source */
		chg_det &= ~(1 << 2);
		ulpi_write(otg, chg_det, 0x34);
		/* Turn on chg detect block */
		chg_det &= ~(1 << 1);
		ulpi_write(otg, chg_det, 0x34);
		udelay(20);
		/* enable chg detection */
		chg_det &= ~(1 << 0);
		ulpi_write(otg, chg_det, 0x34);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		/*
		 * Configure DM as current source, DP as current sink
		 * and enable battery charging comparators.
		 */
		ulpi_write(otg, ULPI_CHRGSEL, ULPI_CHRG_DET_CTRL_S);
		ulpi_write(otg, ULPI_VDATSRCENB, ULPI_CHRG_DET_CTRL_S);
		ulpi_write(otg, ULPI_VDATDETENB, ULPI_CHRG_DET_CTRL_S);
		break;
	default:
		break;
	}
}

static bool msm_chg_check_primary_det(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	u32 chg_det;
	bool ret = false;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(otg, 0x34);
		ret = chg_det & (1 << 4);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		chg_det = ulpi_read(otg, ULPI_CHRG_DET_OUTPUT);
		ret = chg_det & ULPI_CHGDET;
		break;
	default:
		break;
	}
	return ret;
}

static void msm_chg_enable_primary_det(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	u32 chg_det;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(otg, 0x34);
		/* enable chg detection */
		chg_det &= ~(1 << 0);
		ulpi_write(otg, chg_det, 0x34);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		/*
		 * Configure DP as current source, DM as current sink
		 * and enable battery charging comparators.
		 */
		ulpi_write(otg, ULPI_VDATSRCENB, ULPI_CHRG_DET_CTRL_S);
		ulpi_write(otg, ULPI_VDATDETENB, ULPI_CHRG_DET_CTRL_S);
		break;
	default:
		break;
	}
}

static bool msm_chg_check_dcd(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	u32 line_state;
	bool ret = false;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		line_state = ulpi_read(otg, 0x15);
		ret = !(line_state & 1);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		line_state = ulpi_read(otg, ULPI_CHRG_DET_OUTPUT);
		ret = line_state & ULPI_DCDOUT;
		break;
	default:
		break;
	}
	return ret;
}

static void msm_chg_disable_dcd(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	u32 chg_det;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(otg, 0x34);
		chg_det &= ~(1 << 5);
		ulpi_write(otg, chg_det, 0x34);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		ulpi_write(otg, ULPI_DCDENB, ULPI_CHRG_DET_CTRL_C);
		break;
	default:
		break;
	}
}

static void msm_chg_enable_dcd(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	u32 chg_det;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(otg, 0x34);
		/* Turn on D+ current source */
		chg_det |= (1 << 5);
		ulpi_write(otg, chg_det, 0x34);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		/* Data contact detection enable */
		ulpi_write(otg, ULPI_DCDENB, ULPI_CHRG_DET_CTRL_S);
		break;
	default:
		break;
	}
}

static void msm_chg_block_on(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	u32 func_ctrl, chg_det;

	/* put the controller in non-driving mode */
	func_ctrl = ulpi_read(otg, ULPI_FUNC_CTRL);
	func_ctrl &= ~ULPI_FUNC_CTRL_OPMODE_MASK;
	func_ctrl |= ULPI_FUNC_CTRL_OPMODE_NONDRIVING;
	ulpi_write(otg, func_ctrl, ULPI_FUNC_CTRL);

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(otg, 0x34);
		/* control chg block via ULPI */
		chg_det &= ~(1 << 3);
		ulpi_write(otg, chg_det, 0x34);
		/* Turn on chg detect block */
		chg_det &= ~(1 << 1);
		ulpi_write(otg, chg_det, 0x34);
		udelay(20);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		/* Clear charger detecting control bits */
		ulpi_write(otg, 0x1F, ULPI_CHRG_DET_CTRL_C);
		/* Clear alt interrupt latch and enable bits */
		ulpi_write(otg, 0x1F, ULPI_ALT_INT_LATCH_S);
		ulpi_write(otg, 0x1F, ULPI_ALT_INT_EN_C);
		udelay(100);
		break;
	default:
		break;
	}
}

static void msm_chg_block_off(struct msm_otg *motg)
{
	struct otg_transceiver *otg = &motg->otg;
	u32 func_ctrl, chg_det;

	switch (motg->pdata->phy_type) {
	case CI_45NM_INTEGRATED_PHY:
		chg_det = ulpi_read(otg, 0x34);
		/* Turn off charger block */
		chg_det |= ~(1 << 1);
		ulpi_write(otg, chg_det, 0x34);
		break;
	case SNPS_28NM_INTEGRATED_PHY:
		/* Clear charger detecting control bits */
		ulpi_write(otg, 0x3F, ULPI_CHRG_DET_CTRL_C);
		/* Clear alt interrupt latch and enable bits */
		ulpi_write(otg, 0x1F, ULPI_ALT_INT_LATCH_S);
		ulpi_write(otg, 0x1F, ULPI_ALT_INT_EN_C);
		break;
	default:
		break;
	}

	/* put the controller in normal mode */
	func_ctrl = ulpi_read(otg, ULPI_FUNC_CTRL);
	func_ctrl &= ~ULPI_FUNC_CTRL_OPMODE_MASK;
	func_ctrl |= ULPI_FUNC_CTRL_OPMODE_NORMAL;
	ulpi_write(otg, func_ctrl, ULPI_FUNC_CTRL);
}

static const char *chg_to_string(enum usb_chg_type chg_type)
{
	switch (chg_type) {
	case USB_SDP_CHARGER:		return "USB_SDP_CHARGER";
	case USB_DCP_CHARGER:		return "USB_DCP_CHARGER";
	case USB_CDP_CHARGER:		return "USB_CDP_CHARGER";
	case USB_ACA_A_CHARGER:		return "USB_ACA_A_CHARGER";
	case USB_ACA_B_CHARGER:		return "USB_ACA_B_CHARGER";
	case USB_ACA_C_CHARGER:		return "USB_ACA_C_CHARGER";
	case USB_ACA_DOCK_CHARGER:	return "USB_ACA_DOCK_CHARGER";
	default:			return "INVALID_CHARGER";
	}
}

#define MSM_CHG_DCD_POLL_TIME		(100 * HZ/1000) /* 100 msec */
#define MSM_CHG_DCD_MAX_RETRIES		6 /* Tdcd_tmout = 6 * 100 msec */
#define MSM_CHG_PRIMARY_DET_TIME	(40 * HZ/1000) /* TVDPSRC_ON */
#define MSM_CHG_SECONDARY_DET_TIME	(40 * HZ/1000) /* TVDMSRC_ON */
static void msm_chg_detect_work(struct work_struct *w)
{
	struct msm_otg *motg = container_of(w, struct msm_otg, chg_work.work);
	bool is_dcd, tmout, vout, is_aca;
	unsigned long delay;

	USBH_INFO("%s: state:%s\n", __func__,
		chg_state_string(motg->chg_state));

	switch (motg->chg_state) {
	case USB_CHG_STATE_UNDEFINED:
		msm_chg_block_on(motg);
		msm_chg_enable_dcd(motg);
		msm_chg_enable_aca_det(motg);
		motg->chg_state = USB_CHG_STATE_WAIT_FOR_DCD;
		motg->dcd_retries = 0;
		delay = MSM_CHG_DCD_POLL_TIME;
		break;
	case USB_CHG_STATE_WAIT_FOR_DCD:
		is_aca = msm_chg_aca_detect(motg);
		if (is_aca) {
			/*
			 * ID_A can be ACA dock too. continue
			 * primary detection after DCD.
			 */
			if (test_bit(ID_A, &motg->inputs)) {
				motg->chg_state = USB_CHG_STATE_WAIT_FOR_DCD;
			} else {
				delay = 0;
				break;
			}
		}
		is_dcd = msm_chg_check_dcd(motg);
		tmout = ++motg->dcd_retries == MSM_CHG_DCD_MAX_RETRIES;
		if (is_dcd || tmout) {
			msm_chg_disable_dcd(motg);
			msm_chg_enable_primary_det(motg);
			delay = MSM_CHG_PRIMARY_DET_TIME;
			motg->chg_state = USB_CHG_STATE_DCD_DONE;
		} else {
			delay = MSM_CHG_DCD_POLL_TIME;
		}
		break;
	case USB_CHG_STATE_DCD_DONE:
		vout = msm_chg_check_primary_det(motg);
		if (vout) {
			if (test_bit(ID_A, &motg->inputs)) {
				motg->chg_type = USB_ACA_DOCK_CHARGER;
				motg->chg_state = USB_CHG_STATE_DETECTED;
				delay = 0;
				break;
			}
			msm_chg_enable_secondary_det(motg);
			delay = MSM_CHG_SECONDARY_DET_TIME;
			motg->chg_state = USB_CHG_STATE_PRIMARY_DONE;
		} else {
			if (test_bit(ID_A, &motg->inputs)) {
				motg->chg_type = USB_ACA_A_CHARGER;
				motg->chg_state = USB_CHG_STATE_DETECTED;
				delay = 0;
				break;
			}
			motg->chg_type = USB_SDP_CHARGER;
			motg->chg_state = USB_CHG_STATE_DETECTED;
			motg->connect_type = CONNECT_TYPE_UNKNOWN;
			delay = 0;
		}
		break;
	case USB_CHG_STATE_PRIMARY_DONE:
		vout = msm_chg_check_secondary_det(motg);
		if (vout)
			motg->chg_type = USB_DCP_CHARGER;
		else
			motg->chg_type = USB_CDP_CHARGER;
		motg->connect_type = CONNECT_TYPE_AC;
		motg->chg_state = USB_CHG_STATE_SECONDARY_DONE;
		/* fall through */
	case USB_CHG_STATE_SECONDARY_DONE:
		motg->chg_state = USB_CHG_STATE_DETECTED;
	case USB_CHG_STATE_DETECTED:
		msm_chg_block_off(motg);
		msm_chg_enable_aca_det(motg);
		msm_chg_enable_aca_intr(motg);
		USBH_INFO("chg_type = %s\n",
			chg_to_string(motg->chg_type));
#ifdef CONFIG_FORCE_FAST_CHARGE
		switch (motg->chg_type) {
		case USB_SDP_CHARGER:		USB_porttype_detected = USB_SDP_DETECTED;
						break;
		case USB_DCP_CHARGER:		USB_porttype_detected = USB_DCP_DETECTED;
						break;
		case USB_CDP_CHARGER:		USB_porttype_detected = USB_CDP_DETECTED;
						break;
		case USB_ACA_A_CHARGER:		USB_porttype_detected = USB_ACA_A_DETECTED;
						break;
		case USB_ACA_B_CHARGER:		USB_porttype_detected = USB_ACA_B_DETECTED;
						break;
		case USB_ACA_C_CHARGER:		USB_porttype_detected = USB_ACA_C_DETECTED;
						break;
		case USB_ACA_DOCK_CHARGER:	USB_porttype_detected = USB_ACA_DOCK_DETECTED;
						break;
		default:			USB_porttype_detected = USB_INVALID_DETECTED;
						break;
		}
#endif
		schedule_work(&motg->sm_work);

		queue_work(motg->usb_wq, &motg->notifier_work);
		return;
	default:
		return;
	}

	schedule_delayed_work(&motg->chg_work, delay);
}

/*
 * We support OTG, Peripheral only and Host only configurations. In case
 * of OTG, mode switch (host-->peripheral/peripheral-->host) can happen
 * via Id pin status or user request (debugfs). Id/BSV interrupts are not
 * enabled when switch is controlled by user and default mode is supplied
 * by board file, which can be changed by userspace later.
 */
static void msm_otg_init_sm(struct msm_otg *motg)
{
	struct msm_otg_platform_data *pdata = motg->pdata;
	u32 otgsc = readl(USB_OTGSC);

	switch (pdata->mode) {
	case USB_OTG:
		if (pdata->otg_control == OTG_USER_CONTROL) {
			if (pdata->default_mode == USB_HOST) {
				clear_bit(ID, &motg->inputs);
			} else if (pdata->default_mode == USB_PERIPHERAL) {
				set_bit(ID, &motg->inputs);
				set_bit(B_SESS_VLD, &motg->inputs);
			} else {
				set_bit(ID, &motg->inputs);
				clear_bit(B_SESS_VLD, &motg->inputs);
			}
		} else if (pdata->otg_control == OTG_PHY_CONTROL) {
			if (otgsc & OTGSC_ID)
				set_bit(ID, &motg->inputs);
			else
				clear_bit(ID, &motg->inputs);
			if (otgsc & OTGSC_BSV)
				set_bit(B_SESS_VLD, &motg->inputs);
			else
				clear_bit(B_SESS_VLD, &motg->inputs);
		} else if (pdata->otg_control == OTG_PMIC_CONTROL) {
			if (htc_otg_id)
				set_bit(ID, &motg->inputs);
			else
				clear_bit(ID, &motg->inputs);

			if ((otgsc & OTGSC_BSV) || htc_otg_vbus) {
				set_bit(B_SESS_VLD, &motg->inputs);
			} else
				clear_bit(B_SESS_VLD, &motg->inputs);
		}
		break;
	case USB_HOST:
		clear_bit(ID, &motg->inputs);
		break;
	case USB_PERIPHERAL:
		set_bit(ID, &motg->inputs);
		if (pdata->otg_control == OTG_PHY_CONTROL) {
			if (otgsc & OTGSC_BSV)
				set_bit(B_SESS_VLD, &motg->inputs);
			else
				clear_bit(B_SESS_VLD, &motg->inputs);
		} else {
			if ((otgsc & OTGSC_BSV) || htc_otg_vbus)
				set_bit(B_SESS_VLD, &motg->inputs);
			else
				clear_bit(B_SESS_VLD, &motg->inputs);
		}
		break;
	default:
		break;
	}

	if (test_bit(B_SESS_VLD, &motg->inputs)) {
		if (motg->pdata->usb_uart_switch)
			motg->pdata->usb_uart_switch(0);
	}
}

static void msm_otg_sm_work(struct work_struct *w)
{
	struct msm_otg *motg = container_of(w, struct msm_otg, sm_work);
	struct otg_transceiver *otg = &motg->otg;

	USBH_INFO("%s: state:%s bit:0x%08x\n", __func__,
		state_string(otg->state), (unsigned) motg->inputs);

	pm_runtime_resume(otg->dev);
	switch (otg->state) {
	case OTG_STATE_UNDEFINED:
		dev_dbg(otg->dev, "OTG_STATE_UNDEFINED state\n");
		msm_otg_reset(otg);
		msm_otg_init_sm(motg);
		otg->state = OTG_STATE_B_IDLE;
		if (!test_bit(B_SESS_VLD, &motg->inputs) &&
				test_bit(ID, &motg->inputs)) {
			pm_runtime_put_noidle(otg->dev);
			pm_runtime_suspend(otg->dev);
			break;
		}
		/* FALL THROUGH */
	case OTG_STATE_B_IDLE:
		dev_dbg(otg->dev, "OTG_STATE_B_IDLE state\n");
		if ((!test_bit(ID, &motg->inputs) ||
				test_bit(ID_A, &motg->inputs)) && otg->host) {
			USBH_INFO("!id || id_a\n");
			if (motg->chg_type == USB_ACA_DOCK_CHARGER)
				msm_otg_notify_charger(motg,
						IDEV_ACA_CHG_MAX);
			else if (test_bit(ID_A, &motg->inputs))
				msm_otg_notify_charger(motg,
						IDEV_ACA_CHG_MAX - IUNIT);
			else if (motg->pdata->vbus_power)
				motg->pdata->vbus_power(1);
			msm_otg_start_host(otg, 1);
			/*
			 * Link can not generate PHY_ALT interrupt
			 * in host mode when no device is attached
			 * to the port.  It is also observed PHY_ALT
			 * interrupt missing upon Micro-A cable disconnect.
			 * Hence disable PHY_ALT interrupt and perform
			 * polling to detect RID change.
			 */
			msm_chg_enable_aca_det(motg);
			msm_chg_disable_aca_intr(motg);
			mod_timer(&motg->id_timer, ID_TIMER_FREQ);
			otg->state = OTG_STATE_A_HOST;
		} else if (test_bit(B_SESS_VLD, &motg->inputs)) {
			USBH_INFO("b_sess_vld\n");
			switch (motg->chg_state) {
			case USB_CHG_STATE_UNDEFINED:
				schedule_delayed_work(&motg->chg_work, HZ / 10);
				break;
			case USB_CHG_STATE_DETECTED:
				switch (motg->chg_type) {
				case USB_DCP_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_CHG_MAX);
					if (motg->reset_phy_before_lpm)
						msm_otg_reset(otg);
					pm_runtime_put_noidle(otg->dev);
					pm_runtime_suspend(otg->dev);
					break;
				case USB_ACA_B_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_ACA_CHG_MAX);
					/*
					 * (ID_B --> ID_C) PHY_ALT interrupt can
					 * not be detected in LPM.
					 */
					break;
				case USB_CDP_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_CHG_MAX);
					msm_otg_start_peripheral(otg, 1);
					otg->state = OTG_STATE_B_PERIPHERAL;
					break;
				case USB_ACA_C_CHARGER:
					msm_otg_notify_charger(motg,
							IDEV_ACA_CHG_MAX);
					msm_otg_start_peripheral(otg, 1);
					otg->state = OTG_STATE_B_PERIPHERAL;
					break;
				case USB_SDP_CHARGER:
					/* Change USB SDP charger current from 100mA to 500mA
					 * msm_otg_notify_charger(motg, IUNIT);
					 */
					msm_otg_notify_charger(motg, IDEV_CHG_MIN);
					msm_otg_start_peripheral(otg, 1);
					otg->state = OTG_STATE_B_PERIPHERAL;
					motg->ac_detect_count = 0;
					mod_timer(&motg->ac_detect_timer, jiffies + (3 * HZ));
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
		} else {
			USBH_INFO("!b_sess_vld && id\n");
			/*
			 * If charger detection work is pending, decrement
			 * the pm usage counter to balance with the one that
			 * is incremented in charger detection work.
			 */
			cancel_delayed_work_sync(&motg->chg_work);
			msm_otg_notify_charger(motg, 0);
			motg->chg_state = USB_CHG_STATE_UNDEFINED;
			motg->chg_type = USB_INVALID_CHARGER;
			msm_otg_reset(otg);

			if (motg->connect_type != CONNECT_TYPE_NONE) {
				motg->connect_type = CONNECT_TYPE_NONE;
				queue_work(motg->usb_wq, &motg->notifier_work);
			}

			pm_runtime_put_noidle(otg->dev);
			pm_runtime_suspend(otg->dev);
		}
		break;
	case OTG_STATE_B_PERIPHERAL:
		dev_dbg(otg->dev, "OTG_STATE_B_PERIPHERAL state\n");
		if (!test_bit(B_SESS_VLD, &motg->inputs) ||
				!test_bit(ID, &motg->inputs)) {
			msm_otg_start_peripheral(otg, 0);
			otg->state = OTG_STATE_B_IDLE;
			schedule_work(w);

			if (motg->connect_type != CONNECT_TYPE_NONE) {
				motg->connect_type = CONNECT_TYPE_NONE;
				queue_work(motg->usb_wq, &motg->notifier_work);
			}
			motg->ac_detect_count = 0;
			del_timer(&motg->ac_detect_timer);
		} else if (test_bit(ID_C, &motg->inputs)) {
			USBH_INFO("id_c\n");
			msm_otg_notify_charger(motg, IDEV_ACA_CHG_MAX);
		} else if (test_bit(B_SESS_VLD, &motg->inputs)) {
			/* redetect to China AC*/
			if (motg->chg_type == USB_DCP_CHARGER) {
				msm_otg_start_peripheral(otg, 0);
				otg->state = OTG_STATE_B_IDLE;
				schedule_work(w);

				motg->ac_detect_count = 0;
				del_timer(&motg->ac_detect_timer);
			} else
				USBH_DEBUG("do nothing !!!\n");

		} else
			USBH_DEBUG("do nothing !!\n");
		break;
	case OTG_STATE_A_HOST:
		dev_dbg(otg->dev, "OTG_STATE_A_HOST state\n");
		if (test_bit(ID, &motg->inputs) &&
				!test_bit(ID_A, &motg->inputs)) {
			USBH_INFO("id && !id_a\n");
			msm_otg_start_host(otg, 0);
			if (motg->pdata->vbus_power) {
				motg->pdata->vbus_power(0);
				msleep(100); /* TA_WAIT_VFALL */
			}
			/*
			 * Exit point of host mode.
			 *
			 * 1. Micro-A cable disconnect: Just schedule
			 * the work. PHY is reset in B_IDLE and LPM
			 * is allowed.
			 * 2. ID_GND --> ID_B: No need to reset the PHY.
			 * HCD core clears all PORTSC bits and initializes
			 * the controller to host mode in remove_hcd.
			 * Restore PORTSC transceiver select bits (ULPI)
			 * and reset the controller to change MODE bits.
			 * PHY_ALT interrupt can not occur in host mode.
			 */
			del_timer_sync(&motg->id_timer);
			if (motg->chg_state != USB_CHG_STATE_UNDEFINED) {
				msm_otg_link_reset(motg);
				msm_chg_enable_aca_intr(motg);
			}
			otg->state = OTG_STATE_B_IDLE;
			schedule_work(w);
		} else if (test_bit(ID_A, &motg->inputs)) {
			USBH_INFO("id_a\n");
			if (motg->pdata->vbus_power)
				motg->pdata->vbus_power(0);
			msm_otg_notify_charger(motg,
					IDEV_ACA_CHG_MAX - motg->mA_port);
		} else if (!test_bit(ID, &motg->inputs)) {
			USBH_INFO("!id\n");
			msm_otg_notify_charger(motg, 0);
			if (motg->pdata->vbus_power)
				motg->pdata->vbus_power(1);
		}
		break;
	default:
		break;
	}
}

void msm_hsusb_vbus_notif_register(void (*vbus_notif)(int))
{
	struct msm_otg *motg = the_msm_otg;
	if (motg) {
		motg->vbus_notification_cb = vbus_notif;
		USBH_INFO("%s: success\n", __func__);
	}
}

static irqreturn_t msm_otg_irq(int irq, void *data)
{
	struct msm_otg *motg = data;
	struct otg_transceiver *otg = &motg->otg;
	u32 otgsc = 0, usbsts;

	if (atomic_read(&motg->in_lpm)) {
		disable_irq_nosync(irq);
		motg->async_int = 1;
		pm_request_resume(otg->dev);
		return IRQ_HANDLED;
	}

	usbsts = readl(USB_USBSTS);
	if ((usbsts & PHY_ALT_INT)) {
		dev_dbg(otg->dev, "PHY_ALT interrupt\n");
		writel(PHY_ALT_INT, USB_USBSTS);
		if (msm_chg_check_aca_intr(motg)) {
			dev_dbg(otg->dev, "ACA work from IRQ\n");
			schedule_work(&motg->sm_work);
		}
		return IRQ_HANDLED;
	}

	otgsc = readl(USB_OTGSC);
	if (!(otgsc & (OTGSC_IDIS | OTGSC_BSVIS)))
		return IRQ_NONE;

	if ((otgsc & OTGSC_IDIS) && (otgsc & OTGSC_IDIE)) {
		if (otgsc & OTGSC_ID) {
			dev_dbg(otg->dev, "ID set\n");
			set_bit(ID, &motg->inputs);
		} else {
			dev_dbg(otg->dev, "ID clear\n");
			clear_bit(ID, &motg->inputs);
			msm_chg_enable_aca_det(motg);
		}
		schedule_work(&motg->sm_work);
	} else if ((otgsc & OTGSC_BSVIS) && (otgsc & OTGSC_BSVIE)) {
		if (motg->pdata->otg_control == OTG_PHY_CONTROL) {
			if (otgsc & OTGSC_BSV) {
				dev_dbg(otg->dev, "BSV set\n");
				set_bit(B_SESS_VLD, &motg->inputs);
			} else {
				dev_dbg(otg->dev, "BSV clear\n");
				clear_bit(B_SESS_VLD, &motg->inputs);
				msm_chg_check_aca_intr(motg);
			}
			schedule_work(&motg->sm_work);
		}
		if (motg->vbus_notification_cb) {
			if (otgsc & OTGSC_BSV) {
				USBH_DEBUG("BSV set\n");
				motg->vbus_notification_cb(1);
			} else {
				USBH_DEBUG("BSV clear\n");
				motg->vbus_notification_cb(0);
			}
		}
	}

	writel(otgsc, USB_OTGSC);
	return IRQ_HANDLED;
}

/* The dedicated 9V detection GPIO will be high if VBUS is in and over 6V.
 * Since D+/D- status is not involved, there is no timing issue between
 * D+/D- and VBUS. 9V AC should NOT be found here.
 */
static void ac_detect_expired(unsigned long _data)
{
	u32 delay = 0;
	struct msm_otg *motg = the_msm_otg;
	struct otg_transceiver *otg = &motg->otg;

	USBH_INFO("%s: count = %d, connect_type = %d\n", __func__,
			motg->ac_detect_count, motg->connect_type);

	if (motg->connect_type == CONNECT_TYPE_USB || motg->ac_detect_count >= 3)
		return;

	/* detect shorted D+/D-, indicating AC power */
	if ((readl(USB_PORTSC) & PORTSC_LS) != PORTSC_LS) {
		/* Some carkit can't be recognized as AC mode.
		 * Add SW solution here to notify battery driver should
		 * work as AC charger when car mode activated.
		 */
#ifdef CONFIG_CABLE_DETECT_ACCESSORY
		if (cable_get_accessory_type() == DOCK_STATE_CAR) {
				USBH_INFO("car mode charger\n");
				motg->chg_type = USB_DCP_CHARGER;
				motg->chg_state = USB_CHG_STATE_DETECTED;
				motg->connect_type = CONNECT_TYPE_AC;
				motg->ac_detect_count = 0;

				msm_otg_start_peripheral(otg, 0);
				otg->state = OTG_STATE_B_IDLE;
				schedule_work(&motg->sm_work);

				queue_work(motg->usb_wq, &motg->notifier_work);
				return;
		}
#endif
		motg->ac_detect_count++;
		if (motg->ac_detect_count == 1)
			delay = 5 * HZ;
		else if (motg->ac_detect_count == 2)
			delay = 10 * HZ;

		mod_timer(&motg->ac_detect_timer, jiffies + delay);
	} else {
		USBH_INFO("AC charger\n");
		motg->chg_type = USB_DCP_CHARGER;
		motg->chg_state = USB_CHG_STATE_DETECTED;
		motg->connect_type = CONNECT_TYPE_AC;
		motg->ac_detect_count = 0;

		msm_otg_start_peripheral(otg, 0);
		otg->state = OTG_STATE_B_IDLE;
		schedule_work(&motg->sm_work);

		queue_work(motg->usb_wq, &motg->notifier_work);
	}
}

#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
void msm_otg_set_id_state(int id)
{
	struct msm_otg *motg = the_msm_otg;
	struct otg_transceiver *otg;
	USBH_INFO("%s: %d\n", __func__, id);

	htc_otg_id = id;
	if (!motg) {
		USBH_INFO("OTG does not probe yet\n");
		return;
	}

	otg = &motg->otg;

	if (atomic_read(&motg->in_lpm)) {
		pm_request_resume(otg->dev);
	}

	if (id) {
		set_bit(ID, &motg->inputs);
		USBH_INFO("Set ID\n");
	} else {
		clear_bit(ID, &motg->inputs);
		USBH_INFO("Clear ID\n");
	}

	/* Hold a wake_lock so that it will not sleep in detection */
	wake_lock_timeout(&motg->cable_detect_wlock, 3 * HZ);
	schedule_work(&motg->sm_work);
}

static void usb_host_cable_detect(bool cable_in)
{
	if (cable_in)
		msm_otg_set_id_state(0);
	else
		msm_otg_set_id_state(1);
}
#endif

void msm_otg_set_vbus_state(int online)
{
	struct msm_otg *motg = the_msm_otg;
	struct otg_transceiver *otg;
	USBH_INFO("%s: %d\n", __func__, online);

	htc_otg_vbus = online;
	/* FIXME: vbus notification is earlier than otg probe */

	if (!motg) {
		USBH_INFO("OTG does not probe yet\n");
		return;
	}

	otg = &motg->otg;

	if (atomic_read(&motg->in_lpm)) {
		pm_request_resume(otg->dev);
	}
#if 0
	/* We depend on PMIC for only VBUS ON interrupt */
	if (!atomic_read(&motg->in_lpm) || !online)
		return;
#endif

	/* for non-cable_detect && software switch project */
	if (motg->pdata->usb_uart_switch)
		motg->pdata->usb_uart_switch(!online);

	if (online) {
		set_bit(B_SESS_VLD, &motg->inputs);
		/* VBUS interrupt will be triggered while HOST 5V power turn on */
		/* set_bit(ID, &motg->inputs); */
	} else
		clear_bit(B_SESS_VLD, &motg->inputs);

	/* Hold a wake_lock so that it will not sleep in detection */
	wake_lock_timeout(&motg->cable_detect_wlock, 3 * HZ);
	schedule_work(&motg->sm_work);
}

#if 0 /* HTC doesn't use this pin to trigger ID interrupt */
static irqreturn_t msm_pmic_id_irq(int irq, void *data)
{
	struct msm_otg *motg = data;

	if (atomic_read(&motg->in_lpm) && !motg->async_int)
		msm_otg_irq(motg->irq, motg);

	return IRQ_HANDLED;
}
#endif

static int msm_otg_mode_show(struct seq_file *s, void *unused)
{
	struct msm_otg *motg = s->private;
	struct otg_transceiver *otg = &motg->otg;

	switch (otg->state) {
	case OTG_STATE_A_HOST:
		seq_printf(s, "host\n");
		break;
	case OTG_STATE_B_PERIPHERAL:
		seq_printf(s, "peripheral\n");
		break;
	default:
		seq_printf(s, "none\n");
		break;
	}

	return 0;
}

static int msm_otg_mode_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_mode_show, inode->i_private);
}

static ssize_t msm_otg_mode_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	struct seq_file *s = file->private_data;
	struct msm_otg *motg = s->private;
	char buf[16];
	struct otg_transceiver *otg = &motg->otg;
	int status = count;
	enum usb_mode_type req_mode;

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count))) {
		status = -EFAULT;
		goto out;
	}

	if (!strncmp(buf, "host", 4)) {
		req_mode = USB_HOST;
	} else if (!strncmp(buf, "peripheral", 10)) {
		req_mode = USB_PERIPHERAL;
	} else if (!strncmp(buf, "none", 4)) {
		req_mode = USB_NONE;
	} else {
		status = -EINVAL;
		goto out;
	}

	switch (req_mode) {
	case USB_NONE:
		switch (otg->state) {
		case OTG_STATE_A_HOST:
		case OTG_STATE_B_PERIPHERAL:
			set_bit(ID, &motg->inputs);
			clear_bit(B_SESS_VLD, &motg->inputs);
			break;
		default:
			goto out;
		}
		break;
	case USB_PERIPHERAL:
		switch (otg->state) {
		case OTG_STATE_B_IDLE:
		case OTG_STATE_A_HOST:
			set_bit(ID, &motg->inputs);
			set_bit(B_SESS_VLD, &motg->inputs);
			break;
		default:
			goto out;
		}
		break;
	case USB_HOST:
		switch (otg->state) {
		case OTG_STATE_B_IDLE:
		case OTG_STATE_B_PERIPHERAL:
			clear_bit(ID, &motg->inputs);
			break;
		default:
			goto out;
		}
		break;
	default:
		goto out;
	}

	pm_runtime_resume(otg->dev);
	schedule_work(&motg->sm_work);
out:
	return status;
}

const struct file_operations msm_otg_mode_fops = {
	.open = msm_otg_mode_open,
	.read = seq_read,
	.write = msm_otg_mode_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_show_otg_state(struct seq_file *s, void *unused)
{
	struct msm_otg *motg = s->private;
	struct otg_transceiver *otg = &motg->otg;

	seq_printf(s, "%s\n", otg_state_string(otg->state));
	return 0;
}

static int msm_otg_otg_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_show_otg_state, inode->i_private);
}

const struct file_operations msm_otg_state_fops = {
	.open = msm_otg_otg_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_show_chg_type(struct seq_file *s, void *unused)
{
	struct msm_otg *motg = s->private;

	seq_printf(s, chg_to_string(motg->chg_type));
	return 0;
}

static int msm_otg_chg_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_show_chg_type, inode->i_private);
}

const struct file_operations msm_otg_chg_fops = {
	.open = msm_otg_chg_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int msm_otg_aca_show(struct seq_file *s, void *unused)
{
	if (debug_aca_enabled)
		seq_printf(s, "enabled\n");
	else
		seq_printf(s, "disabled\n");

	return 0;
}

static int msm_otg_aca_open(struct inode *inode, struct file *file)
{
	return single_open(file, msm_otg_aca_show, inode->i_private);
}

static ssize_t msm_otg_aca_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	char buf[8];

	memset(buf, 0x00, sizeof(buf));

	if (copy_from_user(&buf, ubuf, min_t(size_t, sizeof(buf) - 1, count)))
		return -EFAULT;

	if (!strncmp(buf, "enable", 6))
		debug_aca_enabled = true;
	else
		debug_aca_enabled = false;

	return count;
}

const struct file_operations msm_otg_aca_fops = {
	.open = msm_otg_aca_open,
	.read = seq_read,
	.write = msm_otg_aca_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static struct dentry *msm_otg_dbg_root;
static struct dentry *msm_otg_dbg_mode;
static struct dentry *msm_otg_chg_type;
static struct dentry *msm_otg_dbg_aca;
static struct dentry *msm_otg_dbg_state;

static int msm_otg_debugfs_init(struct msm_otg *motg)
{

	msm_otg_dbg_root = debugfs_create_dir("msm_otg", NULL);

	if (!msm_otg_dbg_root || IS_ERR(msm_otg_dbg_root))
		return -ENODEV;

	if (motg->pdata->mode == USB_OTG)
	{
		msm_otg_dbg_mode = debugfs_create_file("mode", S_IRUGO |
			S_IWUSR, msm_otg_dbg_root, motg,
			&msm_otg_mode_fops);

		if (!msm_otg_dbg_mode) {
			debugfs_remove(msm_otg_dbg_root);
			msm_otg_dbg_root = NULL;
			return -ENODEV;
		}
	}

	msm_otg_chg_type = debugfs_create_file("chg_type", S_IRUGO,
		msm_otg_dbg_root, motg,
		&msm_otg_chg_fops);

	if (!msm_otg_chg_type) {
		debugfs_remove_recursive(msm_otg_dbg_root);
		return -ENODEV;
	}

	msm_otg_dbg_aca = debugfs_create_file("aca", S_IRUGO | S_IWUSR,
		msm_otg_dbg_root, motg,
		&msm_otg_aca_fops);

	if (!msm_otg_dbg_aca) {
		debugfs_remove_recursive(msm_otg_dbg_root);
		return -ENODEV;
	}

	msm_otg_dbg_state = debugfs_create_file("otg_state", S_IRUGO,
				msm_otg_dbg_root, motg, &msm_otg_state_fops);

	if (!msm_otg_dbg_state) {
		debugfs_remove_recursive(msm_otg_dbg_root);
		return -ENODEV;
	}
	return 0;
}

static void msm_otg_debugfs_cleanup(void)
{
	debugfs_remove_recursive(msm_otg_dbg_root);
}

#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
static struct t_usb_host_status_notifier usb_host_status_notifier = {
	.name = "usb_host",
	.func = usb_host_cable_detect,
};
#endif

static u64 msm_otg_dma_mask = DMA_BIT_MASK(64);
static struct platform_device *msm_otg_add_pdev(
		struct platform_device *ofdev, const char *name)
{
	struct platform_device *pdev;
	const struct resource *res = ofdev->resource;
	unsigned int num = ofdev->num_resources;
	int retval;

	pdev = platform_device_alloc(name, -1);
	if (!pdev) {
		retval = -ENOMEM;
		goto error;
	}

	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	pdev->dev.dma_mask = &msm_otg_dma_mask;

	if (num) {
		retval = platform_device_add_resources(pdev, res, num);
		if (retval)
			goto error;
	}

	retval = platform_device_add(pdev);
	if (retval)
		goto error;

	return pdev;

error:
	platform_device_put(pdev);
	return ERR_PTR(retval);
}

static int msm_otg_setup_devices(struct platform_device *ofdev,
		enum usb_mode_type mode, bool init)
{
	const char *gadget_name = "msm_hsusb";
	const char *host_name = "msm_hsusb_host";
	static struct platform_device *gadget_pdev;
	static struct platform_device *host_pdev;
	int retval = 0;

	if (!init) {
		if (gadget_pdev)
			platform_device_unregister(gadget_pdev);
		if (host_pdev)
			platform_device_unregister(host_pdev);
		return 0;
	}

	switch (mode) {
	case USB_OTG:
		/* fall through */
	case USB_PERIPHERAL:
		gadget_pdev = msm_otg_add_pdev(ofdev, gadget_name);
		if (IS_ERR(gadget_pdev)) {
			retval = PTR_ERR(gadget_pdev);
			break;
		}
		if (mode == USB_PERIPHERAL)
			break;
		/* fall through */
	case USB_HOST:
		host_pdev = msm_otg_add_pdev(ofdev, host_name);
		if (IS_ERR(host_pdev)) {
			retval = PTR_ERR(host_pdev);
			if (mode == USB_OTG)
				platform_device_unregister(gadget_pdev);
		}
		break;
	default:
		break;
	}

	return retval;
}

struct msm_otg_platform_data *msm_otg_dt_to_pdata(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct msm_otg_platform_data *pdata;
	int len = 0;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("unable to allocate platform data\n");
		return NULL;
	}
	of_get_property(node, "qcom,hsusb-otg-phy-init-seq", &len);
	if (len) {
		pdata->phy_init_seq = devm_kzalloc(&pdev->dev, len, GFP_KERNEL);
		if (!pdata->phy_init_seq)
			return NULL;
		of_property_read_u32_array(node, "qcom,hsusb-otg-phy-init-seq",
				pdata->phy_init_seq,
				len/sizeof(*pdata->phy_init_seq));
	}
	of_property_read_u32(node, "qcom,hsusb-otg-power-budget",
				&pdata->power_budget);
	of_property_read_u32(node, "qcom,hsusb-otg-mode",
				&pdata->mode);
	of_property_read_u32(node, "qcom,hsusb-otg-otg-control",
				&pdata->otg_control);
	of_property_read_u32(node, "qcom,hsusb-otg-default-mode",
				&pdata->default_mode);
	of_property_read_u32(node, "qcom,hsusb-otg-phy-type",
				&pdata->phy_type);
	of_property_read_u32(node, "qcom,hsusb-otg-pmic-id-irq",
				&pdata->pmic_id_irq);
	return pdata;
}

static int __init msm_otg_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct resource *res;
	struct msm_otg *motg;
	struct otg_transceiver *otg;
	struct msm_otg_platform_data *pdata;

	dev_info(&pdev->dev, "msm_otg probe\n");

	if (pdev->dev.of_node) {
		dev_dbg(&pdev->dev, "device tree enabled\n");
		pdata = msm_otg_dt_to_pdata(pdev);
		if (!pdata)
			return -ENOMEM;
		ret = msm_otg_setup_devices(pdev, pdata->mode, true);
		if (ret) {
			dev_err(&pdev->dev, "devices setup failed\n");
			return ret;
		}
	} else if (!pdev->dev.platform_data) {
		dev_err(&pdev->dev, "No platform data given. Bailing out\n");
		return -ENODEV;
	} else {
		pdata = pdev->dev.platform_data;
	}

	motg = kzalloc(sizeof(struct msm_otg), GFP_KERNEL);
	if (!motg) {
		dev_err(&pdev->dev, "unable to allocate msm_otg\n");
		return -ENOMEM;
	}

	the_msm_otg = motg;
	motg->pdata = pdata;
	motg->connect_type_ready = 0;
	otg = &motg->otg;
	otg->dev = &pdev->dev;
	motg->reset_phy_before_lpm = pdata->reset_phy_before_lpm;
	/*
	 * ACA ID_GND threshold range is overlapped with OTG ID_FLOAT.  Hence
	 * PHY treat ACA ID_GND as float and no interrupt is generated.  But
	 * PMIC can detect ACA ID_GND and generate an interrupt.
	 */
	if (aca_enabled() && !(motg->pdata->otg_control == OTG_PMIC_CONTROL)) {
		dev_err(&pdev->dev, "ACA can not be enabled without PMIC\n");
		ret = -EINVAL;
		goto free_motg;
	}

	if (motg->pdata->rpc_connect) {
		ret = motg->pdata->rpc_connect(1);
		if (ret) {
			dev_err(&pdev->dev, "rpc connect failed\n");
			goto free_motg;
		}
	}

	/* Some targets don't support PHY clock. */
	motg->phy_reset_clk = clk_get(&pdev->dev, "phy_clk");
	if (IS_ERR(motg->phy_reset_clk))
		dev_err(&pdev->dev, "failed to get phy_clk\n");

	motg->clk = clk_get(&pdev->dev, "alt_core_clk");
	if (IS_ERR(motg->clk)) {
		dev_err(&pdev->dev, "failed to get alt_core_clk\n");
		ret = PTR_ERR(motg->clk);
		goto put_phy_reset_clk;
	}
	clk_set_rate(motg->clk, 60000000);

	/* pm qos request to prevent apps idle power collapse */
	pm_qos_add_request(&motg->pm_qos_req_dma,
		PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	/*
	 * USB Core is running its protocol engine based on CORE CLK,
	 * CORE CLK  must be running at >55Mhz for correct HSUSB
	 * operation and USB core cannot tolerate frequency changes on
	 * CORE CLK. For such USB cores, vote for maximum clk frequency
	 * on pclk source
	 */
	motg->core_clk = clk_get(&pdev->dev, "core_clk");
	if (IS_ERR(motg->core_clk)) {
		motg->core_clk = NULL;
		dev_err(&pdev->dev, "failed to get core_clk\n");
		ret = PTR_ERR(motg->clk);
		goto put_clk;
	}
	clk_set_rate(motg->core_clk, INT_MAX);

	motg->pclk = clk_get(&pdev->dev, "iface_clk");
	if (IS_ERR(motg->pclk)) {
		dev_err(&pdev->dev, "failed to get iface_clk\n");
		ret = PTR_ERR(motg->pclk);
		goto put_core_clk;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform resource mem\n");
		ret = -ENODEV;
		goto put_pclk;
	}

	motg->regs = ioremap(res->start, resource_size(res));
	if (!motg->regs) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = -ENOMEM;
		goto put_pclk;
	}
	dev_info(&pdev->dev, "OTG regs = %p\n", motg->regs);

	motg->irq = platform_get_irq(pdev, 0);
	if (!motg->irq) {
		dev_err(&pdev->dev, "platform_get_irq failed\n");
		ret = -ENODEV;
		goto free_regs;
	}

	clk_enable(motg->pclk);

	if (motg->pdata->vddcx_name) {
		ret = msm_hsusb_init_vddcx(motg, 1);
		if (ret) {
			dev_err(&pdev->dev, "hsusb vddcx init failed\n");
			goto free_regs;
		}

		ret = msm_hsusb_config_vddcx(1);
		if (ret) {
			dev_err(&pdev->dev, "hsusb vddcx configuration failed\n");
			goto free_init_vddcx;
		}
	}
#if defined(CONFIG_ARCH_MSM8X60) || defined(CONFIG_ARCH_MSM8960)
	ret = msm_hsusb_ldo_init(motg, 1);
	if (ret) {
		dev_err(&pdev->dev, "hsusb vreg configuration failed\n");
		goto free_regs;
	}

	ret = msm_hsusb_ldo_enable(motg, 1);
	if (ret) {
		dev_err(&pdev->dev, "hsusb vreg enable failed\n");
		goto free_ldo_init;
	}
#endif
	clk_enable(motg->core_clk);

	writel(0, USB_USBINTR);
	writel(0, USB_OTGSC);
	/* Ensure that above STOREs are completed before enabling interrupts */
	mb();

	motg->usb_wq = create_singlethread_workqueue("msm_hsusb");
	if (motg->usb_wq == 0) {
		USB_ERR("fail to create workqueue\n");
		goto free_ldo_init;
	}

	wake_lock_init(&motg->usb_otg_wlock, WAKE_LOCK_SUSPEND, "msm_otg");
	wake_lock_init(&motg->cable_detect_wlock, WAKE_LOCK_SUSPEND, "msm_usb_cable");

	INIT_WORK(&motg->sm_work, msm_otg_sm_work);
	INIT_WORK(&motg->notifier_work, send_usb_connect_notify);
	INIT_DELAYED_WORK(&motg->chg_work, msm_chg_detect_work);
	setup_timer(&motg->id_timer, msm_otg_id_timer_func,
				(unsigned long) motg);
	motg->ac_detect_count = 0;
	motg->ac_detect_timer.function = ac_detect_expired;
	init_timer(&motg->ac_detect_timer);

	ret = request_irq(motg->irq, msm_otg_irq, IRQF_SHARED,
					"msm_otg", motg);
	if (ret) {
		dev_err(&pdev->dev, "request irq failed\n");
		goto destroy_wlock;
	}

	otg->init = msm_otg_reset;
	otg->set_host = msm_otg_set_host;
	otg->set_peripheral = msm_otg_set_peripheral;
	otg->set_power = msm_otg_set_power;
	otg->set_suspend = msm_otg_set_suspend;
	otg->notify_usb_attached = msm_otg_notify_usb_attached;

	otg->io_ops = &msm_otg_io_ops;

	ret = otg_set_transceiver(&motg->otg);
	if (ret) {
		dev_err(&pdev->dev, "otg_set_transceiver failed\n");
		goto free_irq;
	}

#if 0 /* HTC doesn't use this pin to trigger ID interrupt */
	if (motg->pdata->otg_control == OTG_PMIC_CONTROL) {
		if (motg->pdata->pmic_id_irq) {
			ret = request_irq(motg->pdata->pmic_id_irq,
						msm_pmic_id_irq,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING,
						"msm_otg", motg);
			if (ret) {
				dev_err(&pdev->dev, "request irq failed for PMIC ID\n");
				goto remove_otg;
			}
		} else
			dev_err(&pdev->dev, "PMIC IRQ for ID notifications doesn't exist\n");
	}
#endif

	msm_hsusb_mhl_switch_enable(motg, 1);

	platform_set_drvdata(pdev, motg);
	device_init_wakeup(&pdev->dev, 1);
	motg->mA_port = IUNIT;

	ret = msm_otg_debugfs_init(motg);
	if (ret)
		dev_dbg(&pdev->dev, "mode debugfs file is"
			"not available\n");

	if (motg->pdata->otg_control == OTG_PMIC_CONTROL)
		pm8921_charger_register_vbus_sn(&msm_otg_set_vbus_state);

#if (defined(CONFIG_USB_OTG) && defined(CONFIG_USB_OTG_HOST))
		usb_host_detect_register_notifier(&usb_host_status_notifier);
#endif

	if (motg->pdata->phy_type == SNPS_28NM_INTEGRATED_PHY) {
		if (motg->pdata->otg_control == OTG_PMIC_CONTROL)
			motg->caps = ALLOW_PHY_RETENTION | ALLOW_PHY_COMP_DISABLE;
		if ((motg->pdata->otg_control == OTG_PHY_CONTROL)
			|| motg->pdata->phy_notify_enabled)
			motg->caps = ALLOW_PHY_RETENTION;
	}
	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	motg->xo_handle = msm_xo_get(MSM_XO_PXO, "usb");
	if (IS_ERR(motg->xo_handle)) {
		USBH_ERR(" %s not able to get the handle"
			"to vote for PXO buffer\n", __func__);
		ret = PTR_ERR(motg->xo_handle);
		goto free_irq;
	}

	ret = msm_xo_mode_vote(motg->xo_handle, MSM_XO_MODE_ON);
	if (ret) {
		USBH_ERR("%s failed to vote for PXO"
			"buffer%d\n", __func__, ret);
		msm_xo_put(motg->xo_handle);
		goto free_irq;
	}

	return 0;

free_irq:
	free_irq(motg->irq, motg);
destroy_wlock:
	wake_lock_destroy(&motg->usb_otg_wlock);
	wake_lock_destroy(&motg->cable_detect_wlock);
	clk_disable(motg->core_clk);
#if defined(CONFIG_ARCH_MSM8X60) || defined(CONFIG_ARCH_MSM8960)
	msm_hsusb_ldo_enable(motg, 0);
#endif
free_ldo_init:
#if defined(CONFIG_ARCH_MSM8X60) || defined(CONFIG_ARCH_MSM8960)
	msm_hsusb_ldo_init(motg, 0);
#endif
free_init_vddcx:
	if (motg->pdata->vddcx_name)
		msm_hsusb_init_vddcx(motg, 0);
free_regs:
	iounmap(motg->regs);
put_pclk:
	clk_put(motg->pclk);
put_core_clk:
	if (motg->core_clk)
		clk_put(motg->core_clk);
put_clk:
	clk_put(motg->clk);
put_phy_reset_clk:
	if (!IS_ERR(motg->phy_reset_clk))
		clk_put(motg->phy_reset_clk);
	if (motg->pdata->rpc_connect)
		motg->pdata->rpc_connect(0);
free_motg:
	pm_qos_remove_request(&motg->pm_qos_req_dma);
	kfree(motg);
	return ret;
}

static int __devexit msm_otg_remove(struct platform_device *pdev)
{
	struct msm_otg *motg = platform_get_drvdata(pdev);
	struct otg_transceiver *otg = &motg->otg;
	int cnt = 0;

	if (otg->host || otg->gadget)
		return -EBUSY;

	if (pdev->dev.of_node)
		msm_otg_setup_devices(pdev, motg->pdata->mode, false);
	if (motg->pdata->otg_control == OTG_PMIC_CONTROL)
		pm8921_charger_unregister_vbus_sn(0);
	msm_otg_debugfs_cleanup();
	cancel_delayed_work_sync(&motg->chg_work);
	cancel_work_sync(&motg->sm_work);

	pm_runtime_resume(&pdev->dev);

	device_init_wakeup(&pdev->dev, 0);
	pm_runtime_disable(&pdev->dev);
	wake_lock_destroy(&motg->usb_otg_wlock);
	wake_lock_destroy(&motg->cable_detect_wlock);

	msm_hsusb_mhl_switch_enable(motg, 0);
	if (motg->pdata->pmic_id_irq)
		free_irq(motg->pdata->pmic_id_irq, motg);
	otg_set_transceiver(NULL);
	free_irq(motg->irq, motg);

	/*
	 * Put PHY in low power mode.
	 */
	ulpi_read(otg, 0x14);
	ulpi_write(otg, 0x08, 0x09);

	writel(readl(USB_PORTSC) | PORTSC_PHCD, USB_PORTSC);
	while (cnt < PHY_SUSPEND_TIMEOUT_USEC) {
		if (readl(USB_PORTSC) & PORTSC_PHCD)
			break;
		udelay(1);
		cnt++;
	}
	if (cnt >= PHY_SUSPEND_TIMEOUT_USEC)
		dev_err(otg->dev, "Unable to suspend PHY\n");

	clk_disable(motg->pclk);
	if (motg->core_clk)
		clk_disable(motg->core_clk);

	msm_hsusb_ldo_enable(motg, 0);
	msm_hsusb_ldo_init(motg, 0);
	if (motg->pdata->vddcx_name)
		msm_hsusb_init_vddcx(motg, 0);

	iounmap(motg->regs);
	pm_runtime_set_suspended(&pdev->dev);

	if (!IS_ERR(motg->phy_reset_clk))
		clk_put(motg->phy_reset_clk);
	clk_put(motg->pclk);
	clk_put(motg->clk);
	if (motg->core_clk)
		clk_put(motg->core_clk);
	if (motg->pdata->rpc_connect)
		motg->pdata->rpc_connect(0);

	msm_xo_put(motg->xo_handle);

	pm_qos_remove_request(&motg->pm_qos_req_dma);

	kfree(motg);
	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int msm_otg_runtime_idle(struct device *dev)
{
	struct msm_otg *motg = dev_get_drvdata(dev);
	struct otg_transceiver *otg = &motg->otg;

	dev_dbg(dev, "OTG runtime idle\n");

	if (otg->state == OTG_STATE_UNDEFINED)
		return -EAGAIN;
	else
		return 0;
}

static int msm_otg_runtime_suspend(struct device *dev)
{
	struct msm_otg *motg = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG runtime suspend\n");
	return msm_otg_suspend(motg);
}

static int msm_otg_runtime_resume(struct device *dev)
{
	struct msm_otg *motg = dev_get_drvdata(dev);

	dev_dbg(dev, "OTG runtime resume\n");
	pm_runtime_get_noresume(dev);
	return msm_otg_resume(motg);
}
#endif

#ifdef CONFIG_PM_SLEEP
static int msm_otg_pm_suspend(struct device *dev)
{
	int ret;

	dev_dbg(dev, "OTG PM suspend\n");

#ifdef CONFIG_PM_RUNTIME
	ret = pm_runtime_suspend(dev);
	if (ret > 0)
		ret = 0;
#else
	ret =  msm_otg_suspend(dev_get_drvdata(dev));
#endif
	return ret;
}

static int msm_otg_pm_resume(struct device *dev)
{
	dev_dbg(dev, "OTG PM resume\n");
	return 0;
}
#endif

#ifdef CONFIG_PM
static const struct dev_pm_ops msm_otg_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(msm_otg_pm_suspend, msm_otg_pm_resume)
	SET_RUNTIME_PM_OPS(msm_otg_runtime_suspend, msm_otg_runtime_resume,
				msm_otg_runtime_idle)
};
#endif

static struct of_device_id msm_otg_dt_match[] = {
	{	.compatible = "qcom,hsusb-otg",
	},
	{}
};

static struct platform_driver msm_otg_driver = {
	.remove = __devexit_p(msm_otg_remove),
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm = &msm_otg_dev_pm_ops,
#endif
		.of_match_table = msm_otg_dt_match,
	},
};

static int __init msm_otg_init(void)
{
	return platform_driver_probe(&msm_otg_driver, msm_otg_probe);
}

static void __exit msm_otg_exit(void)
{
	platform_driver_unregister(&msm_otg_driver);
}

module_init(msm_otg_init);
module_exit(msm_otg_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("MSM USB transceiver driver");
