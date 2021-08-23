/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include "ilitek.h"

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"
#define DTS_OF_NAME	"tchip,ilitek"
extern unsigned char CTPM_FW_DEF[];
extern char lcd_info_pr[255];
#include <ontim/ontim_dev_dgb.h>
static char version[40]="0x00";
static char vendor_name[50]="ilitek-ili9881h";
static char lcdname[50]="Ili9881h";
DEV_ATTR_DECLARE(touch_screen)
DEV_ATTR_DEFINE("version",version)
DEV_ATTR_DEFINE("vendor",vendor_name)
DEV_ATTR_DEFINE("lcdvendor",lcdname)
DEV_ATTR_DECLARE_END;
ONTIM_DEBUG_DECLARE_AND_INIT(touch_screen,touch_screen,8);
void ilitek_plat_tp_reset(void)
{
	ipio_info("edge delay = %d\n", idev->rst_edge_delay);

	/* Need accurate power sequence, do not change it to msleep */
	gpio_direction_output(idev->tp_rst, 1);
	mdelay(1);
	gpio_set_value(idev->tp_rst, 0);
	mdelay(5);
	gpio_set_value(idev->tp_rst, 1);
	mdelay(idev->rst_edge_delay);
}

void ilitek_plat_input_register(void)
{
	ipio_info();

	idev->input = input_allocate_device();
	if (ERR_ALLOC_MEM(idev->input)) {
		ipio_err("Failed to allocate touch input device\n");
		input_free_device(idev->input);
		return;
	}

	idev->input->name = idev->hwif->name;
	idev->input->phys = idev->phys;
	idev->input->dev.parent = idev->dev;
	idev->input->id.bustype = idev->hwif->bus_type;

	/* set the supported event type for input device */
	set_bit(EV_ABS, idev->input->evbit);
	set_bit(EV_SYN, idev->input->evbit);
	set_bit(EV_KEY, idev->input->evbit);
	set_bit(BTN_TOUCH, idev->input->keybit);
	set_bit(BTN_TOOL_FINGER, idev->input->keybit);
	set_bit(INPUT_PROP_DIRECT, idev->input->propbit);

	input_set_abs_params(idev->input, ABS_MT_POSITION_X, TOUCH_SCREEN_X_MIN, idev->panel_wid - 1, 0, 0);
	input_set_abs_params(idev->input, ABS_MT_POSITION_Y, TOUCH_SCREEN_Y_MIN, idev->panel_hei - 1, 0, 0);
	input_set_abs_params(idev->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(idev->input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

	if (MT_PRESSURE)
		input_set_abs_params(idev->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	if (MT_B_TYPE) {
#if KERNEL_VERSION(3, 7, 0) <= LINUX_VERSION_CODE
		input_mt_init_slots(idev->input, MAX_TOUCH_NUM, INPUT_MT_DIRECT);
#else
		input_mt_init_slots(idev->input, MAX_TOUCH_NUM);
#endif /* LINUX_VERSION_CODE */
	} else {
		input_set_abs_params(idev->input, ABS_MT_TRACKING_ID, 0, MAX_TOUCH_NUM, 0, 0);
	}

	/* Gesture keys register */
	input_set_capability(idev->input, EV_KEY, KEY_POWER);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_UP);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_LEFT);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_RIGHT);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_O);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_E);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_M);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_W);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_S);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_V);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_C);
	input_set_capability(idev->input, EV_KEY, KEY_GESTURE_F);

	__set_bit(KEY_GESTURE_POWER, idev->input->keybit);
	__set_bit(KEY_GESTURE_UP, idev->input->keybit);
	__set_bit(KEY_GESTURE_DOWN, idev->input->keybit);
	__set_bit(KEY_GESTURE_LEFT, idev->input->keybit);
	__set_bit(KEY_GESTURE_RIGHT, idev->input->keybit);
	__set_bit(KEY_GESTURE_O, idev->input->keybit);
	__set_bit(KEY_GESTURE_E, idev->input->keybit);
	__set_bit(KEY_GESTURE_M, idev->input->keybit);
	__set_bit(KEY_GESTURE_W, idev->input->keybit);
	__set_bit(KEY_GESTURE_S, idev->input->keybit);
	__set_bit(KEY_GESTURE_V, idev->input->keybit);
	__set_bit(KEY_GESTURE_Z, idev->input->keybit);
	__set_bit(KEY_GESTURE_C, idev->input->keybit);
	__set_bit(KEY_GESTURE_F, idev->input->keybit);

	/* register the input device to input sub-system */
	if (input_register_device(idev->input) < 0) {
		ipio_err("Failed to register touch input device\n");
		input_unregister_device(idev->input);
		input_free_device(idev->input);
	}
}

#if REGULATOR_POWER
void ilitek_plat_regulator_power_on(bool status)
{
	ipio_err("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (idev->vdd) {
			if (regulator_enable(idev->vdd) < 0)
				ipio_err("regulator_enable VDD fail\n");
		}
		if (idev->vcc) {
			if (regulator_enable(idev->vcc) < 0)
				ipio_err("regulator_enable VCC fail\n");
		}

	} else {
		if (idev->vdd) {
			if (regulator_disable(idev->vdd) < 0)
				ipio_err("regulator_enable VDD fail\n");
		}
		if (idev->vcc) {
			if (regulator_disable(idev->vcc) < 0)
				ipio_err("regulator_enable VCC fail\n");
		}
	}
	atomic_set(&idev->ice_stat, DISABLE);
	mdelay(5);
}

static void ilitek_plat_regulator_power_init(void)
{
	const char *vdd_name = "vdd";
	const char *vcc_name = "vcc";

	idev->vdd = regulator_get(idev->dev, vdd_name);
	if (ERR_ALLOC_MEM(idev->vdd)) {
		ipio_err("regulator_get VDD fail\n");
		idev->vdd = NULL;
	}
	if (regulator_set_voltage(idev->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
		ipio_err("Failed to set VDD %d\n", VDD_VOLTAGE);

	idev->vcc = regulator_get(idev->dev, vcc_name);
	if (ERR_ALLOC_MEM(idev->vcc)) {
		ipio_err("regulator_get VCC fail.\n");
		idev->vcc = NULL;
	}
	if (regulator_set_voltage(idev->vcc, VCC_VOLTAGE, VCC_VOLTAGE) < 0)
		ipio_err("Failed to set VCC %d\n", VCC_VOLTAGE);
    ipio_err("w vdd power on ini\n");
	ilitek_plat_regulator_power_on(true);
}
#endif

static int ilitek_plat_gpio_register(void)
{
	int ret = 0;
	u32 flag;
	struct device_node *dev_node = idev->dev->of_node;

	idev->tp_int = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	idev->tp_rst = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);

	ipio_info("TP INT: %d\n", idev->tp_int);
	ipio_info("TP RESET: %d\n", idev->tp_rst);

	if (!gpio_is_valid(idev->tp_int)) {
		ipio_err("Invalid INT gpio: %d\n", idev->tp_int);
		return -EBADR;
	}

	if (!gpio_is_valid(idev->tp_rst)) {
		ipio_err("Invalid RESET gpio: %d\n", idev->tp_rst);
		return -EBADR;
	}

	ret = gpio_request(idev->tp_int, "TP_INT");
	if (ret < 0) {
		ipio_err("Request IRQ GPIO failed, ret = %d\n", ret);
		gpio_free(idev->tp_int);
		ret = gpio_request(idev->tp_int, "TP_INT");
		if (ret < 0) {
			ipio_err("Retrying request INT GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

	ret = gpio_request(idev->tp_rst, "TP_RESET");
	if (ret < 0) {
		ipio_err("Request RESET GPIO failed, ret = %d\n", ret);
		gpio_free(idev->tp_rst);
		ret = gpio_request(idev->tp_rst, "TP_RESET");
		if (ret < 0) {
			ipio_err("Retrying request RESET GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

out:
	gpio_direction_input(idev->tp_int);
	return ret;
}

void ilitek_plat_irq_disable(void)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_stat) == DISABLE)
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	disable_irq_nosync(idev->irq_num);
	atomic_set(&idev->irq_stat, DISABLE);
	ipio_debug("Disable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

void ilitek_plat_irq_enable(void)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_stat) == ENABLE)
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	enable_irq(idev->irq_num);
	atomic_set(&idev->irq_stat, ENABLE);
	ipio_debug("Enable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

static irqreturn_t ilitek_plat_isr_top_half(int irq, void *dev_id)
{
	if (irq != idev->irq_num) {
		ipio_err("Incorrect irq number (%d)\n", irq);
		return IRQ_NONE;
	}

	if (atomic_read(&idev->mp_int_check) == ENABLE) {
		atomic_set(&idev->mp_int_check, DISABLE);
		ipio_debug("MP INT detected, ignore\n");
		wake_up(&(idev->inq));
		return IRQ_HANDLED;
	}

	if (idev->prox_near) {
		ipio_info("Proximity event, ignore interrupt!\n");
		return IRQ_HANDLED;
	}

	ipio_debug("report: %d, rst: %d, fw: %d, switch: %d, mp: %d, sleep: %d, esd: %d\n",
			idev->report,
			atomic_read(&idev->tp_reset),
			atomic_read(&idev->fw_stat),
			atomic_read(&idev->tp_sw_mode),
			atomic_read(&idev->mp_stat),
			atomic_read(&idev->tp_sleep),
			atomic_read(&idev->esd_stat));

	if (!idev->report || atomic_read(&idev->tp_reset) ||
		atomic_read(&idev->fw_stat) || atomic_read(&idev->tp_sw_mode) ||
		atomic_read(&idev->mp_stat) || atomic_read(&idev->tp_sleep) ||
		atomic_read(&idev->esd_stat)) {
			ipio_debug("ignore interrupt !\n");
			return IRQ_HANDLED;
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t ilitek_plat_isr_bottom_half(int irq, void *dev_id)
{
	if (mutex_is_locked(&idev->touch_mutex)) {
		ipio_debug("touch is locked, ignore\n");
		return IRQ_HANDLED;
	}
	mutex_lock(&idev->touch_mutex);
	ilitek_tddi_report_handler();
	mutex_unlock(&idev->touch_mutex);
	return IRQ_HANDLED;
}

void ilitek_plat_irq_unregister(void)
{
	devm_free_irq(idev->dev, idev->irq_num, NULL);
}

int ilitek_plat_irq_register(int type)
{
	int ret = 0;
	static bool get_irq_pin;

	atomic_set(&idev->irq_stat, DISABLE);

	if (get_irq_pin == false) {
		idev->irq_num  = gpio_to_irq(idev->tp_int);
		get_irq_pin = true;
	}

	ipio_info("idev->irq_num = %d\n", idev->irq_num);

	ret = devm_request_threaded_irq(idev->dev, idev->irq_num,
				   ilitek_plat_isr_top_half,
				   ilitek_plat_isr_bottom_half,
				   type | IRQF_ONESHOT, "ilitek", NULL);

	if (type == IRQF_TRIGGER_FALLING)
		ipio_info("IRQ TYPE = IRQF_TRIGGER_FALLING\n");
	if (type == IRQF_TRIGGER_RISING)
		ipio_info("IRQ TYPE = IRQF_TRIGGER_RISING\n");

	if (ret != 0)
		ipio_err("Failed to register irq handler, irq = %d, ret = %d\n", idev->irq_num, ret);

	atomic_set(&idev->irq_stat, ENABLE);

	return ret;
}

#if defined(CONFIG_DRM)
static int ilitek_plat_notifier_fb(struct notifier_block *self, unsigned long event, void *data)
{
#if defined(ONTIM_DRM_PANEL)
    ipio_err("wwwwwwONTIM_DRM_PANEL\n");
	struct drm_panel_notifier *evdata = data;
	int blank;
	if (!evdata)
		return 0;
	blank = *(int *) (evdata->data);
	if (evdata->data && event == DRM_PANEL_EVENT_BLANK) {
        switch (blank) {
			case DRM_PANEL_BLANK_UNBLANK:
					if(ilitek_tddi_sleep_handler(TP_RESUME) < 0)
					ipio_err("TP resume failed\n");
				break;
			default:
				break;
		}
	}
	if (evdata->data && event == DRM_PANEL_EARLY_EVENT_BLANK) {
        switch (blank) {
            case DRM_PANEL_BLANK_POWERDOWN:
                if (ilitek_tddi_sleep_handler(TP_SUSPEND) < 0)
                ipio_err("TP suspend failed\n");
                break;
            default:
                break;
        }
    }

	return NOTIFY_OK;

#else

	struct msm_drm_notifier *evdata = data;
    int *blank;
    if (!evdata || evdata->id != 0)
		return 0;
    if (evdata && evdata->data && event == MSM_DRM_EVENT_BLANK) {
             blank = evdata->data;
        if (*blank ==  MSM_DRM_BLANK_UNBLANK){
            ipio_err("MSM_DRM_BLANK_UNBLANK\n");
            if(ilitek_tddi_sleep_handler(TP_RESUME) < 0)
					ipio_err("TP resume failed\n");
		}
    }

    if (evdata && evdata->data && event == MSM_DRM_EARLY_EVENT_BLANK) {
        blank = evdata->data;
        ipio_err("MSM_DRM_EARLY_EVENT_BLANK\n");
        if (*blank == MSM_DRM_BLANK_POWERDOWN){
           ipio_err("MSM_DRM_BLANK_POWERDOWN\n");
           if (ilitek_tddi_sleep_handler(TP_SUSPEND) < 0)
				ipio_err("TP suspend failed\n");

		}
   }

	return NOTIFY_OK;
#endif
}

#else
static int fb_notifier_callback(struct notifier_block *self,
                                unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    ipio_err("wwwwwwfb_notifier_callback\n");
    int *blank = NULL;
    if (!(event == FB_EARLY_EVENT_BLANK || event == FB_EVENT_BLANK)) {
        ipio_err("event(%lu) do not need process\n", event);
        return 0;
    }

    blank = evdata->data;
    switch (*blank) {
    case FB_BLANK_UNBLANK:
        if (FB_EARLY_EVENT_BLANK == event) {
            ipio_err("resume: event = %lu, not care\n", event);
        } else if (FB_EVENT_BLANK == event) {
            if(ilitek_tddi_sleep_handler(TP_RESUME) < 0)
					ipio_err("TP resume failed\n");
        }
        break;
    case FB_BLANK_POWERDOWN:
        if (FB_EARLY_EVENT_BLANK == event) {
             if (ilitek_tddi_sleep_handler(TP_SUSPEND) < 0)
				ipio_err("TP suspend failed\n");
        } else if (FB_EVENT_BLANK == event) {
            ipio_err("suspend: event = %lu, not care\n", event);
        }
        break;
    default:
        ipio_err("FB BLANK(%d) do not need process\n", *blank);
        break;
    }

    return 0;
}
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
static void ilitek_plat_early_suspend(struct early_suspend *h)
{
	if (ilitek_tddi_sleep_handler(TP_SUSPEND) < 0)
		ipio_err("TP suspend failed\n");
}

static void ilitek_plat_late_resume(struct early_suspend *h)
{
	if (ilitek_tddi_sleep_handler(TP_RESUME) < 0)
		ipio_err("TP resume failed\n");
}
#endif

static void ilitek_plat_sleep_init(void)
{
    int ret = 0;
#if defined(CONFIG_DRM)
    idev->notifier_fb.notifier_call = ilitek_plat_notifier_fb;
    #if defined(ONTIM_DRM_PANEL)
    #if 0
    if (active_panel){
		ret= drm_panel_notifier_register(active_panel,&ts_data->fb_notif);
		FTS_ERROR("drm_panel_notifier_register");
    }
	if (ret)
		FTS_ERROR("drm_panel_notifier_register fail: %d", ret);
	#endif
    #else
        ret = msm_drm_register_client(&idev->notifier_fb);
        ipio_err("msm_drm_register_client");
        if (ret)
            ipio_err("msm_drm_register_client fail: %d", ret);

    #endif
#else

    idev->notifier_fb.notifier_call = fb_notifier_callback;
    ret = fb_register_client(idev->notifier_fb);
    if (ret) {
        ipio_err("[FB]Unable to register fb_notifier: %d", ret);
    }
#endif
#if defined(CONFIG_HAS_EARLYSUSPEND)
	ipio_info("Init eqarly_suspend struct\n");
	idev->early_suspend.suspend = ilitek_plat_early_suspend;
	idev->early_suspend.resume = ilitek_plat_late_resume;
	idev->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	register_early_suspend(&idev->early_suspend);
#endif
}

static int ilitek_plat_probe(void)
{
	ipio_info("platform probe\n");
	if(CHECK_THIS_DEV_DEBUG_AREADY_EXIT()==0)
	{
		return -EIO;
	}

#if REGULATOR_POWER
	ilitek_plat_regulator_power_init();
#endif

	if (ilitek_plat_gpio_register() < 0)
		ipio_err("Register gpio failed\n");

	if (ilitek_tddi_init() < 0) {
		ipio_err("platform probe failed\n");
		return -ENODEV;
	}

	ilitek_plat_irq_register(idev->irq_tirgger_type);
	ilitek_plat_sleep_init();
	ipio_info("ILITEK Driver loaded successfully!");
	if(strstr(lcd_info_pr,"ili9881")==0){
	   pr_err("tp ilitech holithecn \n");
	   snprintf(lcdname, sizeof(lcdname),"%s ","ili9881h" );
	}
	snprintf(version, sizeof(version),"0x%x ",CTPM_FW_DEF[65570] );
	snprintf(vendor_name, sizeof(vendor_name),"ilitek_ili9881h" );
	pr_err("wzx name=%s \n",vendor_name);
	REGISTER_AND_INIT_ONTIM_DEBUG_FOR_THIS_DEV();
	return 0;
}

static int ilitek_plat_remove(void)
{
	ipio_info("remove plat dev\n");
	ilitek_tddi_dev_remove();
	return 0;
}

static const struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

static struct ilitek_hwif_info hwif = {
	.bus_type = TDDI_INTERFACE,
	.plat_type = TP_PLAT_QCOM,
	.owner = THIS_MODULE,
	.name = TDDI_DEV_ID,
	.of_match_table = of_match_ptr(tp_match_table),
	.plat_probe = ilitek_plat_probe,
	.plat_remove = ilitek_plat_remove,
};

extern char lcd_info_pr[255];
static int __init ilitek_plat_dev_init(void)
{
    if (strstr(lcd_info_pr, "ili9881") == NULL)
        return -ENODEV;

	ipio_info("ILITEK TP driver init for QCOM\n");
	if (ilitek_tddi_dev_init(&hwif) < 0) {
		ipio_err("Failed to register i2c/spi bus driver\n");
		return -ENODEV;
	}
	return 0;
}

static void __exit ilitek_plat_dev_exit(void)
{
	ipio_info("remove plat dev\n");
	ilitek_tddi_dev_remove();
}

module_init(ilitek_plat_dev_init);
module_exit(ilitek_plat_dev_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
