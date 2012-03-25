/* drivers/video/msm/src/drv/fb/msm_fb.c
 *
 * Core MSM framebuffer driver.
 *
 * Copyright (C) 2007 Google Incorporated
 * Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
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
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/msm_mdp.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <mach/board.h>
#include <linux/uaccess.h>

#include <linux/workqueue.h>
#include <linux/ratelimit.h>
#include <linux/string.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>
#include <linux/debugfs.h>
#include <linux/console.h>
#include <linux/android_pmem.h>
#include <linux/leds.h>
#include <linux/pm_runtime.h>
#include <linux/timer.h>

#define MSM_FB_C
#include "msm_fb.h"
#include "mddihosti.h"
#include "tvenc.h"
#include "mdp.h"
#include "mdp4.h"
#include "mach/debug_display.h"
#include "mach/msm_panel.h"

#define NO_BLT_OFFSET

#ifdef CONFIG_FB_MSM_LOGO
#define INIT_IMAGE_FILE "/initlogo.rle"
#endif

#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_NUM	3
#endif

#define NONE 0
#define SUSPEND_RESUME 0x1
#define FPS 0x2
#define BLIT_TIME 0x4
#define SHOW_UPDATES 0x8

#define DLOG(mask, fmt, args...) \
do { \
	if ((msmfb_debug_mask | FPS) & mask) \
		PR_DISP_INFO(": "fmt, ##args); \
} while (0)

static int msmfb_debug_mask;

static unsigned char *fbram;
static unsigned char *fbram_phys;
static int fbram_size;

static struct platform_device *pdev_list[MSM_FB_MAX_DEV_LIST];
static int pdev_list_cnt;

int vsync_mode = 1;

#define MAX_BLIT_REQ 256

#define MAX_FBI_LIST 32
static struct fb_info *fbi_list[MAX_FBI_LIST];
static int fbi_list_index;

static struct msm_fb_data_type *mfd_list[MAX_FBI_LIST];
static int mfd_list_index;

static u32 msm_fb_pseudo_palette[16] = {
	0x00000000, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
	0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff
};

u32 msm_fb_debug_enabled;

/* Show fps setting infomation at most 5 times in every 2 minutes */
DEFINE_RATELIMIT_STATE(fps_msg_ratelimit, 120 * HZ, 5);

u32 fps_interval;
/* Setting msm_fb_msg_level to 8 prints out ALL messages */
u32 msm_fb_msg_level = 7;

/* Setting mddi_msg_level to 8 prints out ALL messages */
u32 mddi_msg_level = 5;

static int msm_fb_register(struct msm_fb_data_type *mfd);
static int msm_fb_open(struct fb_info *info, int user);
static int msm_fb_release(struct fb_info *info, int user);
static int msm_fb_pan_display(struct fb_var_screeninfo *var,
			      struct fb_info *info);
static int msm_fb_stop_sw_refresher(struct msm_fb_data_type *mfd);
int msm_fb_resume_sw_refresher(struct msm_fb_data_type *mfd);
static int msm_fb_check_var(struct fb_var_screeninfo *var,
			    struct fb_info *info);
static int msm_fb_set_par(struct fb_info *info);
static int msm_fb_blank_sub(int blank_mode, struct fb_info *info,
			    boolean op_enable);
static int msm_fb_suspend_sub(struct msm_fb_data_type *mfd);
static int msm_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg);
static int msm_fb_mmap(struct fb_info *info, struct vm_area_struct *vma);

#ifdef MSM_FB_ENABLE_DBGFS

#define MSM_FB_MAX_DBGFS 1024
#define MAX_BACKLIGHT_BRIGHTNESS 255

int msm_fb_debugfs_file_index;
struct dentry *msm_fb_debugfs_root;
struct dentry *msm_fb_debugfs_file[MSM_FB_MAX_DBGFS];

struct dentry *msm_fb_get_debugfs_root(void)
{
	if (msm_fb_debugfs_root == NULL)
		msm_fb_debugfs_root = debugfs_create_dir("msm_fb", NULL);

	return msm_fb_debugfs_root;
}

void msm_fb_debugfs_file_create(struct dentry *root, const char *name,
				u32 *var)
{
	if (msm_fb_debugfs_file_index >= MSM_FB_MAX_DBGFS)
		return;

	msm_fb_debugfs_file[msm_fb_debugfs_file_index++] =
	    debugfs_create_u32(name, S_IRUGO | S_IWUSR, root, var);
}
#endif

#if defined (CONFIG_FB_MSM_MDP_ABL)
DEFINE_MUTEX(msm_fb_notify_update_sem);
void msmfb_no_update_notify_timer_cb(unsigned long data)
{
        struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)data;
        if (!mfd)
                pr_err("%s mfd NULL\n", __func__);
        complete(&mfd->msmfb_no_update_notify);
}
#endif

#if (defined(CONFIG_USB_FUNCTION_PROJECTOR) || defined(CONFIG_USB_ANDROID_PROJECTOR))
static DEFINE_SPINLOCK(fb_data_lock);
static struct msm_fb_info msm_fb_data;
int msmfb_get_var(struct msm_fb_info *tmp)
{
    unsigned long flags;
    spin_lock_irqsave(&fb_data_lock, flags);
    memcpy(tmp, &msm_fb_data, sizeof(msm_fb_data));
    spin_unlock_irqrestore(&fb_data_lock, flags);
    return 0;
}

/* projector need this, and very much */
int msmfb_get_fb_area(void)
{
    int area;
    unsigned long flags;
    spin_lock_irqsave(&fb_data_lock, flags);
    area = msm_fb_data.msmfb_area;
    spin_unlock_irqrestore(&fb_data_lock, flags);
    return area;
}

static void msmfb_set_var(unsigned char *addr, int area)
{
    unsigned long flags;

    spin_lock_irqsave(&fb_data_lock, flags);
    msm_fb_data.fb_addr = addr;
    msm_fb_data.msmfb_area = area;
    spin_unlock_irqrestore(&fb_data_lock, flags);

}
#endif


int msm_fb_cursor(struct fb_info *info, struct fb_cursor *cursor)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	if (!mfd->cursor_update)
		return -ENODEV;

	return mfd->cursor_update(info, cursor);
}

static int msm_fb_resource_initialized;

#ifndef CONFIG_FB_BACKLIGHT
static int lcd_backlight_registered;
unsigned long auto_bkl_status = 8;


static int cabc_switch(int on)
{

	if (test_bit(CABC_STATE_DCR, &auto_bkl_status) == on)
		return 1;

	if (on) {
		PR_DISP_INFO("turn on DCR\n");
		set_bit(CABC_STATE_DCR, &auto_bkl_status);
	} else {
		PR_DISP_INFO("turn off DCR\n");
		clear_bit(CABC_STATE_DCR, &auto_bkl_status);
	}
	return 1;
}

static ssize_t
auto_backlight_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t
auto_backlight_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count);
#define CABC_ATTR(name) __ATTR(name, 0644, auto_backlight_show, auto_backlight_store)

static struct device_attribute auto_attr = CABC_ATTR(auto);
static ssize_t
auto_backlight_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i = 0;

	i += scnprintf(buf + i, PAGE_SIZE - 1, "%d\n",
				test_bit(CABC_STATE_DCR, &auto_bkl_status));
	return i;
}

static ssize_t
auto_backlight_store(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	int rc;
	unsigned long res;

	rc = strict_strtoul(buf, 10, &res);
	if (rc) {
		PR_DISP_INFO("invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}
	if (cabc_switch(!!res))
		count = -EIO;

err_out:
	return count;
}


static void msm_fb_set_bl_brightness(struct led_classdev *led_cdev,
					enum led_brightness value)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(led_cdev->dev->parent);
	int bl_lvl;

	if (value > MAX_BACKLIGHT_BRIGHTNESS)
		value = MAX_BACKLIGHT_BRIGHTNESS;

	/* This maps android backlight level 0 to 255 into
	   driver backlight level 0 to bl_max with rounding */
	bl_lvl = (2 * value * mfd->panel_info.bl_max + MAX_BACKLIGHT_BRIGHTNESS)
		/(2 * MAX_BACKLIGHT_BRIGHTNESS);

	if (!bl_lvl && value)
		bl_lvl = 1;

	msm_fb_set_backlight(mfd, bl_lvl);
}

static struct led_classdev backlight_led = {
	.name		= "lcd-backlight",
	.brightness	= MAX_BACKLIGHT_BRIGHTNESS,
	.brightness_set	= msm_fb_set_bl_brightness,
};
#endif

static struct msm_fb_platform_data *msm_fb_pdata;
static char panel_name[128];

module_param_string(panel_name, panel_name, sizeof(panel_name) , 0);

int msm_fb_detect_client(const char *name)
{
	int ret = -EPERM;
#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
	u32 id;
#endif

	MSM_FB_DEBUG("\n name = %s, panel_name = %s", name, panel_name);
	if (strlen(panel_name)) {
		if (!strcmp((char *)panel_name, name))
			return 0;
		else
			return -EPERM;
	}

	if (msm_fb_pdata && msm_fb_pdata->detect_client) {
		ret = msm_fb_pdata->detect_client(name);

		/* if it's non mddi panel, we need to pre-scan
		   mddi client to see if we can disable mddi host */

#ifdef CONFIG_FB_MSM_MDDI_AUTO_DETECT
		if (!ret && msm_fb_pdata->mddi_prescan)
			id = mddi_get_client_id();
#endif
	}

	return ret;
}

static ssize_t msm_fb_msm_fb_type(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct msm_fb_panel_data *pdata =
		(struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	switch (pdata->panel_info.type) {
	case NO_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "no panel\n");
		break;
	case MDDI_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "mddi panel\n");
		break;
	case EBI2_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "ebi2 panel\n");
		break;
	case LCDC_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "lcdc panel\n");
		break;
	case EXT_MDDI_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "ext mddi panel\n");
		break;
	case TV_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "tv panel\n");
		break;
	case HDMI_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "hdmi panel\n");
		break;
	case DTV_PANEL:
		ret = snprintf(buf, PAGE_SIZE, "dtv panel\n");
		break;
	default:
		ret = snprintf(buf, PAGE_SIZE, "unknown panel\n");
		break;
	}

	return ret;
}
static ssize_t show_fbdebug(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	struct msm_fb_data_type *mfd = mfd_list[0];
	PR_DISP_INFO("%s(%d) mdp hung dma %d dma_pid %d line %d dmap %d dmap_pid %d line %d\n, ", __func__, __LINE__,
		mfd->dma->busy ?1:0, (mfd->dma->busy_pid & 0xFFFF0000) >> 16, (mfd->dma->busy_pid & 0xFFFF),
		mfd->dma->dmap_busy ?1:0, (mfd->dma->dmap_pid & 0xFFFF0000) >> 16, (mfd->dma->dmap_pid & 0xFFFF));
	return len;
}

static ssize_t show_ratelimit(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
	ssize_t len = 0;
	if (fps_interval)
		len += scnprintf(buf+len, PAGE_SIZE-1, "maximum fps = %d\n", 1000/fps_interval);
	else
		len += scnprintf(buf+len, PAGE_SIZE-1, "maximum fps = unlimited\n");
	return len;
}


static ssize_t store_ratelimit(struct device *dev,
                               struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long fps;

	if (strict_strtoul(buf, 10, &fps))
		return -EINVAL;

	if (fps > 0 && fps < 60)
		fps_interval = 1000/fps;
	else
		fps_interval = 0;

	PR_DISP_INFO("[FPS] Set fps to %ld, fps_interval = %d\n", fps, fps_interval);

	return count;
}

static DEVICE_ATTR(fbdebug, 0644, show_fbdebug, NULL);
static DEVICE_ATTR(msm_fb_type, S_IRUGO, msm_fb_msm_fb_type, NULL);
static DEVICE_ATTR(ratelimit, S_IRUGO|S_IWUSR, show_ratelimit, store_ratelimit);

static struct attribute *msm_fb_attrs[] = {
	&dev_attr_msm_fb_type.attr,
	&dev_attr_ratelimit.attr,
	NULL,
};
static struct attribute_group msm_fb_attr_group = {
	.attrs = msm_fb_attrs,
};

static int msm_fb_create_sysfs(struct platform_device *pdev)
{
	int rc;
	struct msm_fb_data_type *mfd = platform_get_drvdata(pdev);

	rc = sysfs_create_group(&mfd->fbi->dev->kobj, &msm_fb_attr_group);
	if (rc)
		MSM_FB_ERR("%s: sysfs group creation failed, rc=%d\n", __func__,
			rc);
	return rc;
}
static void msm_fb_remove_sysfs(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd = platform_get_drvdata(pdev);
	sysfs_remove_group(&mfd->fbi->dev->kobj, &msm_fb_attr_group);
}

#if defined CONFIG_FB_MSM_SELF_REFRESH
DEFINE_MUTEX(self_refresh_lock);
atomic_t self_refresh_suspend = ATOMIC_INIT(0);
atomic_t self_refresh_on = ATOMIC_INIT(0);
static void self_refresh_do_work(struct work_struct *work)
{
	struct msm_fb_data_type *mfd = container_of(
			work, struct msm_fb_data_type, self_refresh_work);
	struct msm_fb_panel_data *pdata;
	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	if ((pdata) && (pdata->self_refresh_switch) &&
		atomic_read(&self_refresh_suspend) == 0) {
		mutex_lock(&self_refresh_lock);
		if (!atomic_read(&self_refresh_on)) {
			atomic_set(&self_refresh_on, 1);
			pdata->self_refresh_switch(1);
			mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
		}
		mutex_unlock(&self_refresh_lock);
	}
}

static void self_refresh_update(unsigned long data)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)data;

	queue_work(mfd->self_refresh_wq, &mfd->self_refresh_work);
}
#endif
static uint32_t ov_blt_base = 0, ov_blt_size = 0;
static int msm_fb_probe(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;
	struct msm_fb_panel_data *pdata;
	struct fb_info *fbi;
	int rc;
	int err = 0;

	MSM_FB_DEBUG("msm_fb_probe\n");

	if ((pdev->id == 0) && (pdev->num_resources > 0)) {
		msm_fb_pdata = pdev->dev.platform_data;
		fbram_size = pdev->resource[0].end - pdev->resource[0].start + 1;
		fbram_phys = (char *)pdev->resource[0].start;
		fbram = ioremap((unsigned long)fbram_phys, fbram_size);

		if (!fbram) {
			printk(KERN_ERR "fbram ioremap failed!\n");
			return -ENOMEM;
		}
		MSM_FB_INFO("msm_fb_probe:  phy_Addr = 0x%x virt = 0x%x\n",
			     (int)fbram_phys, (int)fbram);

		msm_fb_resource_initialized = 1;
#ifdef CONFIG_FB_MSM_OVERLAY
		/* TODO: find a better way to pass blt_base to overlay */
		if (pdev->num_resources > 1) {
			ov_blt_base = pdev->resource[1].start;
			ov_blt_size = pdev->resource[1].end -
				pdev->resource[1].start + 1;
		}
#endif
		return 0;
	}

	if (!msm_fb_resource_initialized || msm_fb_pdata == NULL)
		return -EPERM;

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (pdev_list_cnt >= MSM_FB_MAX_DEV_LIST)
		return -ENOMEM;

	mfd->panel_info.frame_count = 0;
	mfd->width = msm_fb_pdata->width;
	mfd->height = msm_fb_pdata->height;
#ifdef CONFIG_LCDC_TURN_ON_BL_AFTER_PANEL_ON
	mfd->bl_level = MAX_BACKLIGHT_BRIGHTNESS;
#else
	mfd->bl_level = 0;
#endif
#ifdef CONFIG_FB_MSM_OVERLAY
	mfd->overlay_play_enable = 1;

	/* TODO: find a better way to pass blt_base to overlay */
	err = device_create_file(&pdev->dev, &dev_attr_fbdebug);
	if (err != 0)
		printk(KERN_WARNING "attr_fbdebug failed\n");
	mfd->blt_base = ov_blt_base;
	mfd->blt_size = ov_blt_size;
	if (msm_fb_pdata && mfd->panel_info.type == MIPI_CMD_PANEL) {
		mfd->blt_mode = msm_fb_pdata->blt_mode;
		mfd->panel_info.is_3d_panel = msm_fb_pdata->is_3d_panel;
		PR_DISP_INFO("%s: blt_mode=%d is_3d_pane=%d\n", __func__, mfd->blt_mode, msm_fb_pdata->is_3d_panel);
	}
#endif

	rc = msm_fb_register(mfd);
	if (rc)
		return rc;
	err = pm_runtime_set_active(mfd->fbi->dev);
	if (err < 0)
		printk(KERN_ERR "pm_runtime: fail to set active.\n");
	pm_runtime_enable(mfd->fbi->dev);
#ifdef CONFIG_FB_BACKLIGHT
	msm_fb_config_backlight(mfd);
#else
	/* android supports only one lcd-backlight/lcd for now */
	if (!lcd_backlight_registered) {
		if (led_classdev_register(&pdev->dev, &backlight_led))
			printk(KERN_ERR "led_classdev_register failed\n");
		else {
			lcd_backlight_registered = 1;
			if (device_create_file(backlight_led.dev, &auto_attr))
				PR_DISP_INFO("attr creation failed\n");
		}
	}
#endif
	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
#if defined CONFIG_FB_MSM_SELF_REFRESH
	if ((pdata) && (pdata->self_refresh_switch)) {
		INIT_WORK(&mfd->self_refresh_work, self_refresh_do_work);
		mfd->self_refresh_wq = create_workqueue("self_refresh_wq");
		if (!mfd->self_refresh_wq)
			PR_DISP_ERR("%s: can't create workqueue\n", __func__);
		setup_timer(&mfd->self_refresh_timer, self_refresh_update, (unsigned long)mfd);
	}
#endif
	pdev_list[pdev_list_cnt++] = pdev;
	msm_fb_create_sysfs(pdev);
	if (mfd->panel_info.type != DTV_PANEL &&
		mfd->panel_info.type != TV_PANEL) {
		fbi = mfd->fbi;
		if (msm_fb_blank_sub(FB_BLANK_UNBLANK, fbi, mfd->op_enable)) {
			printk(KERN_ERR "msm_fb_open: can't turn on display!\n");
			return -1;
		}
	}
#if defined (CONFIG_FB_MSM_MDP_ABL)
	mfd->enable_abl = false;
#endif
	return 0;
}

static int msm_fb_remove(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	MSM_FB_DEBUG("msm_fb_remove\n");

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	msm_fb_remove_sysfs(pdev);

	pm_runtime_disable(mfd->fbi->dev);

	if (!mfd)
		return -ENODEV;

	if (mfd->key != MFD_KEY)
		return -EINVAL;

	if (msm_fb_suspend_sub(mfd))
		printk(KERN_ERR "msm_fb_remove: can't stop the device %d\n", mfd->index);

	if (mfd->channel_irq != 0)
		free_irq(mfd->channel_irq, (void *)mfd);

	if (mfd->vsync_width_boundary)
		vfree(mfd->vsync_width_boundary);

	if (mfd->vsync_resync_timer.function)
		del_timer(&mfd->vsync_resync_timer);

	if (mfd->refresh_timer.function)
		del_timer(&mfd->refresh_timer);

	if (mfd->dma_hrtimer.function)
		hrtimer_cancel(&mfd->dma_hrtimer);

#if defined (CONFIG_FB_MSM_MDP_ABL)
	if (mfd->msmfb_no_update_notify_timer.function)
	    del_timer(&mfd->msmfb_no_update_notify_timer);
	complete(&mfd->msmfb_no_update_notify);
	complete(&mfd->msmfb_update_notify);
#endif

	/* remove /dev/fb* */
	unregister_framebuffer(mfd->fbi);

#ifdef CONFIG_FB_BACKLIGHT
	/* remove /sys/class/backlight */
	backlight_device_unregister(mfd->fbi->bl_dev);
#else
	if (lcd_backlight_registered) {
		lcd_backlight_registered = 0;
		led_classdev_unregister(&backlight_led);
	}
#endif

#ifdef MSM_FB_ENABLE_DBGFS
	if (mfd->sub_dir)
		debugfs_remove(mfd->sub_dir);
#endif

	return 0;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int msm_fb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct msm_fb_data_type *mfd;
	int ret = 0;

	MSM_FB_DEBUG("msm_fb_suspend\n");

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	acquire_console_sem();
	fb_set_suspend(mfd->fbi, FBINFO_STATE_SUSPENDED);

	ret = msm_fb_suspend_sub(mfd);
	if (ret != 0) {
		printk(KERN_ERR "msm_fb: failed to suspend! %d\n", ret);
		fb_set_suspend(mfd->fbi, FBINFO_STATE_RUNNING);
	} else {
		pdev->dev.power.power_state = state;
	}

	release_console_sem();
	return ret;
}
#else
#define msm_fb_suspend NULL
#endif

static int msm_fb_suspend_sub(struct msm_fb_data_type *mfd)
{
	int ret = 0;

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

#if defined (CONFIG_FB_MSM_MDP_ABL)
	if (mfd->msmfb_no_update_notify_timer.function)
		del_timer(&mfd->msmfb_no_update_notify_timer);
	complete(&mfd->msmfb_no_update_notify);
#endif

	/*
	 * suspend this channel
	 */
	mfd->suspend.sw_refreshing_enable = mfd->sw_refreshing_enable;
	mfd->suspend.op_enable = mfd->op_enable;
	mfd->suspend.panel_power_on = mfd->panel_power_on;

	if (mfd->op_enable) {
		ret =
		     msm_fb_blank_sub(FB_BLANK_POWERDOWN, mfd->fbi,
				      mfd->suspend.op_enable);
		if (ret) {
			MSM_FB_INFO
			    ("msm_fb_suspend: can't turn off display!\n");
			return ret;
		}
		mfd->op_enable = FALSE;
	}
	/*
	 * try to power down
	 */
	mdp_pipe_ctrl(MDP_MASTER_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);

	/*
	 * detach display channel irq if there's any
	 * or wait until vsync-resync completes
	 */
	if ((mfd->dest == DISPLAY_LCD)) {
		if (mfd->panel_info.lcd.vsync_enable) {
			if (mfd->panel_info.lcd.hw_vsync_mode) {
				if (mfd->channel_irq != 0)
					disable_irq(mfd->channel_irq);
			} else {
				boolean vh_pending;
				do {
					vh_pending = mfd->vsync_handler_pending;
				} while (vh_pending);
			}
		}
	}

	return 0;
}

#ifdef CONFIG_PM
static int msm_fb_resume_sub(struct msm_fb_data_type *mfd)
{
	int ret = 0;

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	/* attach display channel irq if there's any */
	if (mfd->channel_irq != 0)
		enable_irq(mfd->channel_irq);

	/* resume state var recover */
	mfd->sw_refreshing_enable = mfd->suspend.sw_refreshing_enable;
	mfd->op_enable = mfd->suspend.op_enable;

	if (mfd->suspend.panel_power_on) {
		ret =
		     msm_fb_blank_sub(FB_BLANK_UNBLANK, mfd->fbi,
				      mfd->op_enable);
		if (ret)
			MSM_FB_INFO("msm_fb_resume: can't turn on display!\n");
	}

	return ret;
}
#endif

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int msm_fb_resume(struct platform_device *pdev)
{
	/* This resume function is called when interrupt is enabled.
	 */
	int ret = 0;
	struct msm_fb_data_type *mfd;

	MSM_FB_DEBUG("msm_fb_resume\n");

	mfd = (struct msm_fb_data_type *)platform_get_drvdata(pdev);

	if ((!mfd) || (mfd->key != MFD_KEY))
		return 0;

	acquire_console_sem();
	ret = msm_fb_resume_sub(mfd);
	pdev->dev.power.power_state = PMSG_ON;
	fb_set_suspend(mfd->fbi, FBINFO_STATE_RUNNING);
	release_console_sem();

	return ret;
}
#else
#define msm_fb_resume NULL
#endif

static int msm_fb_runtime_suspend(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: suspending...\n");
	return 0;
}

static int msm_fb_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: resuming...\n");
	return 0;
}

static int msm_fb_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "pm_runtime: idling...\n");
	return 0;
}

static struct dev_pm_ops msm_fb_dev_pm_ops = {
	.runtime_suspend = msm_fb_runtime_suspend,
	.runtime_resume = msm_fb_runtime_resume,
	.runtime_idle = msm_fb_runtime_idle,
};

static struct platform_driver msm_fb_driver = {
	.probe = msm_fb_probe,
	.remove = msm_fb_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = msm_fb_suspend,
	.resume = msm_fb_resume,
#endif
	.shutdown = NULL,
	.driver = {
		   /* Driver name must match the device name added in platform.c. */
		   .name = "msm_fb",
		   .pm = &msm_fb_dev_pm_ops,
		   },
};

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(CONFIG_FB_MSM_OVERLAY)
static void memset32_io(u32 __iomem *_ptr, u32 val, size_t count)
{
	count >>= 2;
	while (count--)
		writel(val, _ptr++);
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void msmfb_early_suspend(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd = container_of(h, struct msm_fb_data_type,
						    early_suspend);
#ifdef CONFIG_FB_MSM_OVERLAY
	/*
	* For MDP with overlay, set framebuffer with black pixels
	* to show black screen on HDMI.
	*/
	struct fb_info *fbi = mfd->fbi;
	switch (mfd->fbi->var.bits_per_pixel) {
	case 32:
		memset32_io((void *)fbi->screen_base, 0xFF000000,
							fbi->fix.smem_len);
		break;
	default:
		memset_io(fbi->screen_base, 0x00, fbi->fix.smem_len);
		break;
	}
#endif
	if (mfd->mdp_pdata && mfd->mdp_pdata->mdp_img_stick_wa)
		del_timer_sync(&mfd->frame_update_timer);
	msm_fb_suspend_sub(mfd);
}

static void msmfb_early_resume(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd = container_of(h, struct msm_fb_data_type,
						    early_suspend);
	msm_fb_resume_sub(mfd);
}

#ifdef CONFIG_HTC_ONMODE_CHARGING
static void msmfb_onchg_suspend(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd = container_of(h, struct msm_fb_data_type,
						    onchg_suspend);
#ifdef CONFIG_FB_MSM_OVERLAY
	/*
	* For MDP with overlay, set framebuffer with black pixels
	* to show black screen on HDMI.
	*/
	struct fb_info *fbi = mfd->fbi;
	switch (mfd->fbi->var.bits_per_pixel) {
	case 32:
		memset32_io((void *)fbi->screen_base, 0xFF000000,
							fbi->fix.smem_len);
		break;
	default:
		memset_io(fbi->screen_base, 0x00, fbi->fix.smem_len);
		break;
	}
#endif
	MSM_FB_INFO("%s starts.\n", __func__);
	msm_fb_suspend_sub(mfd);
	MSM_FB_INFO("%s is done.\n", __func__);
}

static void msmfb_onchg_resume(struct early_suspend *h)
{
	struct msm_fb_data_type *mfd = container_of(h, struct msm_fb_data_type,
						    onchg_suspend);
	MSM_FB_INFO("%s starts.\n", __func__);
	msm_fb_resume_sub(mfd);
	MSM_FB_INFO("%s is done.\n", __func__);
}
#endif /* CONFIG_HTC_ONMODE_CHARGING */
#endif

void msm_fb_set_backlight(struct msm_fb_data_type *mfd, __u32 bkl_lvl)
{
	struct msm_fb_panel_data *pdata;

	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	if ((pdata) && (pdata->set_backlight)) {
		if (mfd->bl_level == 0)
			mfd->during_pwr_off_seq = 1;
		mfd->bl_level = bkl_lvl;
		pdata->set_backlight(mfd);
	}
}

void msm_fb_display_on(struct msm_fb_data_type *mfd)
{
	struct msm_fb_panel_data *pdata;
	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	if ((pdata) && (pdata->display_on)) {
		down(&mfd->sem);
		pdata->display_on(mfd);
		up(&mfd->sem);
		if (pdata->bklctrl)
			pdata->bklctrl(1);
		if (pdata->bklswitch)
			pdata->bklswitch(mfd, 1);
		if (pdata->bklenable)
			pdata->bklenable(mfd);
	}
}

static int msm_fb_blank_sub(int blank_mode, struct fb_info *info,
			    boolean op_enable)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct msm_fb_panel_data *pdata = NULL;
	int ret = 0;

	if (!op_enable)
		return -EPERM;

	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
	if ((!pdata) || (!pdata->on) || (!pdata->off)) {
		printk(KERN_ERR "msm_fb_blank_sub: no panel operation detected!\n");
		return -ENODEV;
	}

	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		if (!mfd->panel_power_on) {
#if defined CONFIG_FB_MSM_SELF_REFRESH
			atomic_set(&self_refresh_suspend, 0);
#endif
			ret = pdata->on(mfd->pdev);
			if (ret == 0) {
				mfd->panel_power_on = TRUE;
				mfd->request_display_on = TRUE;
/* ToDo: possible conflict with android which doesn't expect sw refresher */
/*
	  if (!mfd->hw_refresh)
	  {
	    if ((ret = msm_fb_resume_sw_refresher(mfd)) != 0)
	    {
	      MSM_FB_INFO("msm_fb_blank_sub: msm_fb_resume_sw_refresher failed = %d!\n",ret);
	    }
	  }
*/
			}
		}
		break;

	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
	case FB_BLANK_POWERDOWN:
	default:
		if (mfd->panel_power_on) {
			int curr_pwr_state;

			mfd->op_enable = FALSE;
			curr_pwr_state = mfd->panel_power_on;
#if defined CONFIG_FB_MSM_SELF_REFRESH
			if ((pdata) && (pdata->self_refresh_switch)) {
				del_timer_sync(&mfd->self_refresh_timer);
				flush_workqueue(mfd->self_refresh_wq);
				mutex_lock(&self_refresh_lock);
				if (atomic_read(&self_refresh_on)) {
					atomic_set(&self_refresh_on, 0);
					mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
				}
				atomic_set(&self_refresh_suspend, 1);
				mutex_unlock(&self_refresh_lock);
			}
#endif
			if (pdata->bklswitch)
				pdata->bklswitch(mfd, 0);

			ret = pdata->off(mfd->pdev);
			if (ret)
				mfd->panel_power_on = curr_pwr_state;
			else
				mfd->panel_power_on = FALSE;

			mfd->op_enable = TRUE;
		}
		break;
	}

	return ret;
}

static void msm_fb_fillrect(struct fb_info *info,
			    const struct fb_fillrect *rect)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	cfb_fillrect(info, rect);
	if (!mfd->hw_refresh && (info->var.yoffset == 0) &&
		!mfd->sw_currently_refreshing) {
		struct fb_var_screeninfo var;

		var = info->var;
		var.reserved[0] = 0x54445055;
		var.reserved[1] = (rect->dy << 16) | (rect->dx);
		var.reserved[2] = ((rect->dy + rect->height) << 16) |
		    (rect->dx + rect->width);

		msm_fb_pan_display(&var, info);
	}
}

static void msm_fb_copyarea(struct fb_info *info,
			    const struct fb_copyarea *area)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	cfb_copyarea(info, area);
	if (!mfd->hw_refresh && (info->var.yoffset == 0) &&
		!mfd->sw_currently_refreshing) {
		struct fb_var_screeninfo var;

		var = info->var;
		var.reserved[0] = 0x54445055;
		var.reserved[1] = (area->dy << 16) | (area->dx);
		var.reserved[2] = ((area->dy + area->height) << 16) |
		    (area->dx + area->width);

		msm_fb_pan_display(&var, info);
	}
}

static void msm_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	cfb_imageblit(info, image);
	if (!mfd->hw_refresh && (info->var.yoffset == 0) &&
		!mfd->sw_currently_refreshing) {
		struct fb_var_screeninfo var;

		var = info->var;
		var.reserved[0] = 0x54445055;
		var.reserved[1] = (image->dy << 16) | (image->dx);
		var.reserved[2] = ((image->dy + image->height) << 16) |
		    (image->dx + image->width);

		msm_fb_pan_display(&var, info);
	}
}

static int msm_fb_blank(int blank_mode, struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	return msm_fb_blank_sub(blank_mode, info, mfd->op_enable);
}

#if defined (CONFIG_FB_MSM_MDP_ABL)
static int msm_fb_set_lut(struct fb_cmap *cmap, struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	if (!mfd->lut_update)
		return -ENODEV;

	mfd->lut_update(info, cmap);
	return 0;
}

static int msm_fb_get_lut(struct fb_info *info, void __user *p)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct gamma_curvy gc;
	int ret;

	if (!mfd->get_gamma_curvy) {
		return -ENODEV;
	}

	if (copy_from_user(&gc, p, sizeof(struct gamma_curvy))){
		return -EFAULT;
	}

	ret = mfd->get_gamma_curvy(mfd->mdp_pdata->abl_gamma_tbl, &gc, mfd->mdp_pdata->color_enhancment_tbl);

	if (ret)
		return ret;

	ret = copy_to_user(p, &gc, sizeof(struct gamma_curvy));

	return 0;
}
#endif

/*
 * Custom Framebuffer mmap() function for MSM driver.
 * Differs from standard mmap() function by allowing for customized
 * page-protection.
 */
static int msm_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	/* Get frame buffer memory range. */
	unsigned long start = info->fix.smem_start;
	u32 len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.smem_len);
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	if (off >= len) {
		/* memory mapped io */
		off -= len;
		if (info->var.accel_flags) {
			mutex_unlock(&info->lock);
			return -EINVAL;
		}
		start = info->fix.mmio_start;
		len = PAGE_ALIGN((start & ~PAGE_MASK) + info->fix.mmio_len);
	}

	/* Set VM flags. */
	start &= PAGE_MASK;
	if ((vma->vm_end - vma->vm_start + off) > len)
		return -EINVAL;
	off += start;
	vma->vm_pgoff = off >> PAGE_SHIFT;
	/* This is an IO map - tell maydump to skip this VMA */
	vma->vm_flags |= VM_IO | VM_RESERVED;

	/* Set VM page protection */
	if (mfd->mdp_fb_page_protection == MDP_FB_PAGE_PROTECTION_WRITECOMBINE)
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	else if (mfd->mdp_fb_page_protection ==
			MDP_FB_PAGE_PROTECTION_WRITETHROUGHCACHE)
		vma->vm_page_prot = pgprot_writethroughcache(vma->vm_page_prot);
	else if (mfd->mdp_fb_page_protection ==
			MDP_FB_PAGE_PROTECTION_WRITEBACKCACHE)
		vma->vm_page_prot = pgprot_writebackcache(vma->vm_page_prot);
	else if (mfd->mdp_fb_page_protection ==
			MDP_FB_PAGE_PROTECTION_WRITEBACKWACACHE)
		vma->vm_page_prot = pgprot_writebackwacache(vma->vm_page_prot);
	else
		vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	/* Remap the frame buffer I/O range */
	if (io_remap_pfn_range(vma, vma->vm_start, off >> PAGE_SHIFT,
				vma->vm_end - vma->vm_start,
				vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static struct fb_ops msm_fb_ops = {
	.owner = THIS_MODULE,
	.fb_open = msm_fb_open,
	.fb_release = msm_fb_release,
	.fb_read = NULL,
	.fb_write = NULL,
	.fb_cursor = NULL,
	.fb_check_var = msm_fb_check_var,	/* vinfo check */
	.fb_set_par = msm_fb_set_par,	/* set the video mode according to info->var */
	.fb_setcolreg = NULL,	/* set color register */
	.fb_blank = msm_fb_blank,	/* blank display */
	.fb_pan_display = msm_fb_pan_display,	/* pan display */
	.fb_fillrect = msm_fb_fillrect,	/* Draws a rectangle */
	.fb_copyarea = msm_fb_copyarea,	/* Copy data from area to another */
	.fb_imageblit = msm_fb_imageblit,	/* Draws a image to the display */
	.fb_rotate = NULL,
	.fb_sync = NULL,	/* wait for blit idle, optional */
	.fb_ioctl = msm_fb_ioctl,	/* perform fb specific ioctl (optional) */
	.fb_mmap = msm_fb_mmap,
};

static __u32 msm_fb_line_length(__u32 fb_index, __u32 xres, int bpp)
{
	/* The adreno GPU hardware requires that the pitch be aligned to
	   32 pixels for color buffers, so for the cases where the GPU
	   is writing directly to fb0, the framebuffer pitch
	   also needs to be 32 pixel aligned */

	if (fb_index == 0)
		return ALIGN(xres, 32) * bpp;
	else
		return xres * bpp;
}

static int msm_fb_register(struct msm_fb_data_type *mfd)
{
	int ret = -ENODEV;
	int bpp;
	struct msm_panel_info *panel_info = &mfd->panel_info;
	struct fb_info *fbi = mfd->fbi;
	struct fb_fix_screeninfo *fix;
	struct fb_var_screeninfo *var;
	int *id;
	int fbram_offset;

	/*
	 * fb info initialization
	 */
	fix = &fbi->fix;
	var = &fbi->var;

	fix->type_aux = 0;	/* if type == FB_TYPE_INTERLEAVED_PLANES */
	fix->visual = FB_VISUAL_TRUECOLOR;	/* True Color */
	fix->ywrapstep = 0;	/* No support */
	fix->mmio_start = 0;	/* No MMIO Address */
	fix->mmio_len = 0;	/* No MMIO Address */
	fix->accel = FB_ACCEL_NONE;/* FB_ACCEL_MSM needes to be added in fb.h */

	var->xoffset = 0,	/* Offset from virtual to visible */
	var->yoffset = 0,	/* resolution */
	var->grayscale = 0,	/* No graylevels */
	var->nonstd = 0,	/* standard pixel format */
	var->activate = FB_ACTIVATE_VBL,	/* activate it at vsync */
	var->height = mfd->height,	/* height of picture in mm */
	var->width = mfd->width,	/* width of picture in mm */
	var->accel_flags = 0,	/* acceleration flags */
	var->sync = 0,	/* see FB_SYNC_* */
	var->rotate = 0,	/* angle we rotate counter clockwise */
	mfd->op_enable = FALSE;

	switch (mfd->fb_imgType) {
	case MDP_RGB_565:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 0;
		var->green.offset = 5;
		var->red.offset = 11;
		var->blue.length = 5;
		var->green.length = 6;
		var->red.length = 5;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 0;
		bpp = 2;
		break;

	case MDP_RGB_888:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 0;
		var->green.offset = 8;
		var->red.offset = 16;
		var->blue.length = 8;
		var->green.length = 8;
		var->red.length = 8;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 0;
		bpp = 3;
		break;

	case MDP_ARGB_8888:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 0;
		var->green.offset = 8;
		var->red.offset = 16;
		var->blue.length = 8;
		var->green.length = 8;
		var->red.length = 8;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 24;
		var->transp.length = 8;
		bpp = 4;
		break;

	case MDP_RGBA_8888:
		fix->type = FB_TYPE_PACKED_PIXELS;
		fix->xpanstep = 1;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;
		var->blue.offset = 16;
		var->green.offset = 8;
		var->red.offset = 0;
		var->blue.length = 8;
		var->green.length = 8;
		var->red.length = 8;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 8;
		bpp = 4;
		break;

	case MDP_YCRYCB_H2V1:
		/* ToDo: need to check TV-Out YUV422i framebuffer format */
		/*       we might need to create new type define */
		fix->type = FB_TYPE_INTERLEAVED_PLANES;
		fix->xpanstep = 2;
		fix->ypanstep = 1;
		var->vmode = FB_VMODE_NONINTERLACED;

		/* how about R/G/B offset? */
		var->blue.offset = 0;
		var->green.offset = 5;
		var->red.offset = 11;
		var->blue.length = 5;
		var->green.length = 6;
		var->red.length = 5;
		var->blue.msb_right = 0;
		var->green.msb_right = 0;
		var->red.msb_right = 0;
		var->transp.offset = 0;
		var->transp.length = 0;
		bpp = 2;
		break;

	default:
		MSM_FB_ERR("msm_fb_init: fb %d unkown image type!\n",
			   mfd->index);
		return ret;
	}

	if (panel_info->is_3d_panel)
		fix->type = FB_TYPE_3D_PANEL;

	fix->line_length = msm_fb_line_length(mfd->index, panel_info->xres,
					      bpp);
	/* calculate smem_len based on max size of two supplied modes */
	fix->smem_len = roundup(MAX(msm_fb_line_length(mfd->index,
					       ALIGN(panel_info->xres, 32),
					       bpp) *
			    panel_info->yres * mfd->fb_page,
			    msm_fb_line_length(mfd->index,
					       panel_info->mode2_xres,
					       (panel_info->mode2_bpp+7)/8) *
			    panel_info->mode2_yres * mfd->fb_page), PAGE_SIZE);

#if (defined(CONFIG_USB_FUNCTION_PROJECTOR) || defined(CONFIG_USB_ANDROID_PROJECTOR))
	if (mfd->index == 0) {
		msm_fb_data.xres = ALIGN(panel_info->xres, 32);
		msm_fb_data.yres = panel_info->yres;
	}
#endif


	mfd->var_xres = panel_info->xres;
	mfd->var_yres = panel_info->yres;

	var->pixclock = mfd->panel_info.clk_rate;
	mfd->var_pixclock = var->pixclock;

	var->xres = panel_info->xres;
	var->yres = panel_info->yres;
	var->xres_virtual = ALIGN(panel_info->xres, 32);
	var->yres_virtual = panel_info->yres * mfd->fb_page;
	var->bits_per_pixel = bpp * 8;	/* FrameBuffer color depth */
		/*
		 * id field for fb app
		 */
	    id = (int *)&mfd->panel;

#if defined(CONFIG_MSM_MDP22)
	snprintf(fix->id, sizeof(fix->id), "msmfb22_%x", (__u32) *id);
#elif defined(CONFIG_MSM_MDP30)
	snprintf(fix->id, sizeof(fix->id), "msmfb30_%x", (__u32) *id);
#elif defined(CONFIG_MSM_MDP31)
	snprintf(fix->id, sizeof(fix->id), "msmfb31_%x", (__u32) *id);
#elif defined(CONFIG_MSM_MDP40)
	snprintf(fix->id, sizeof(fix->id), "msmfb40_%x", (__u32) *id);
#else
	error CONFIG_FB_MSM_MDP undefined !
#endif
	fbi->fbops = &msm_fb_ops;
	fbi->flags = FBINFO_FLAG_DEFAULT;
	fbi->pseudo_palette = msm_fb_pseudo_palette;

	mfd->ref_cnt = 0;
	mfd->sw_currently_refreshing = FALSE;
	mfd->sw_refreshing_enable = TRUE;
	mfd->panel_power_on = FALSE;

	mfd->pan_waiting = FALSE;
	init_completion(&mfd->pan_comp);
	init_completion(&mfd->refresher_comp);
	sema_init(&mfd->sem, 1);

	init_timer(&mfd->msmfb_no_update_notify_timer);
#if defined (CONFIG_FB_MSM_MDP_ABL)
	mfd->msmfb_no_update_notify_timer.function =
		msmfb_no_update_notify_timer_cb;
	mfd->msmfb_no_update_notify_timer.data = (unsigned long)mfd;
	init_completion(&mfd->msmfb_update_notify);
	init_completion(&mfd->msmfb_no_update_notify);
#endif

	fbram_offset = PAGE_ALIGN((int)fbram)-(int)fbram;
	fbram += fbram_offset;
	fbram_phys += fbram_offset;
	fbram_size -= fbram_offset;

	if (fbram_size < fix->smem_len) {
		printk(KERN_ERR "error: no more framebuffer memory!\n");
		return -ENOMEM;
	}

	fbi->screen_base = fbram;
	fbi->fix.smem_start = (unsigned long)fbram_phys;

#if (defined(CONFIG_USB_FUNCTION_PROJECTOR) || defined(CONFIG_USB_ANDROID_PROJECTOR))
	if (mfd->index == 0)
		msmfb_set_var(fbi->screen_base, 0);
#endif

	memset(fbi->screen_base, 0x0, fix->smem_len);

	mfd->op_enable = TRUE;
	mfd->panel_power_on = FALSE;

	/* cursor memory allocation */
	if (mfd->cursor_update) {
		mfd->cursor_buf = dma_alloc_coherent(NULL,
					MDP_CURSOR_SIZE,
					(dma_addr_t *) &mfd->cursor_buf_phys,
					GFP_KERNEL);
		if (!mfd->cursor_buf)
			mfd->cursor_update = 0;
	}

	if (mfd->lut_update) {
		ret = fb_alloc_cmap(&fbi->cmap, 256, 0);
		if (ret)
			printk(KERN_ERR "%s: fb_alloc_cmap() failed!\n",
					__func__);
	}

	if (register_framebuffer(fbi) < 0) {
		if (mfd->lut_update)
			fb_dealloc_cmap(&fbi->cmap);

		if (mfd->cursor_buf)
			dma_free_coherent(NULL,
				MDP_CURSOR_SIZE,
				mfd->cursor_buf,
				(dma_addr_t) mfd->cursor_buf_phys);

		mfd->op_enable = FALSE;
		return -EPERM;
	}

	fbram += fix->smem_len;
	fbram_phys += fix->smem_len;
	fbram_size -= fix->smem_len;

	MSM_FB_INFO
	    ("FrameBuffer[%d] %dx%d size=%d bytes is registered successfully!\n",
	     mfd->index, fbi->var.xres, fbi->var.yres, fbi->fix.smem_len);

#ifdef CONFIG_FB_MSM_LOGO
	if (!load_565rle_image(INIT_IMAGE_FILE)) ;	/* Flip buffer */
#endif
	ret = 0;

#ifdef CONFIG_HAS_EARLYSUSPEND
	if (mfd->panel_info.type != DTV_PANEL &&
		mfd->panel_info.type != TV_PANEL) {
		mfd->early_suspend.suspend = msmfb_early_suspend;
		mfd->early_suspend.resume = msmfb_early_resume;
		mfd->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 2;
		register_early_suspend(&mfd->early_suspend);
#ifdef CONFIG_HTC_ONMODE_CHARGING
		mfd->onchg_suspend.suspend = msmfb_onchg_suspend;
		mfd->onchg_suspend.resume = msmfb_onchg_resume;
		mfd->onchg_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 2;
		register_onchg_suspend(&mfd->onchg_suspend);
#endif
	}
#endif

#ifdef MSM_FB_ENABLE_DBGFS
	{
		struct dentry *root;
		struct dentry *sub_dir;
		char sub_name[2];

		root = msm_fb_get_debugfs_root();
		if (root != NULL) {
			sub_name[0] = (char)(mfd->index + 0x30);
			sub_name[1] = '\0';
			sub_dir = debugfs_create_dir(sub_name, root);
		} else {
			sub_dir = NULL;
		}

		mfd->sub_dir = sub_dir;

		if (sub_dir) {
			msm_fb_debugfs_file_create(sub_dir, "op_enable",
						   (u32 *) &mfd->op_enable);
			msm_fb_debugfs_file_create(sub_dir, "panel_power_on",
						   (u32 *) &mfd->
						   panel_power_on);
			msm_fb_debugfs_file_create(sub_dir, "ref_cnt",
						   (u32 *) &mfd->ref_cnt);
			msm_fb_debugfs_file_create(sub_dir, "fb_imgType",
						   (u32 *) &mfd->fb_imgType);
			msm_fb_debugfs_file_create(sub_dir,
						   "sw_currently_refreshing",
						   (u32 *) &mfd->
						   sw_currently_refreshing);
			msm_fb_debugfs_file_create(sub_dir,
						   "sw_refreshing_enable",
						   (u32 *) &mfd->
						   sw_refreshing_enable);

			msm_fb_debugfs_file_create(sub_dir, "xres",
						   (u32 *) &mfd->panel_info.
						   xres);
			msm_fb_debugfs_file_create(sub_dir, "yres",
						   (u32 *) &mfd->panel_info.
						   yres);
			msm_fb_debugfs_file_create(sub_dir, "bpp",
						   (u32 *) &mfd->panel_info.
						   bpp);
			msm_fb_debugfs_file_create(sub_dir, "type",
						   (u32 *) &mfd->panel_info.
						   type);
			msm_fb_debugfs_file_create(sub_dir, "wait_cycle",
						   (u32 *) &mfd->panel_info.
						   wait_cycle);
			msm_fb_debugfs_file_create(sub_dir, "pdest",
						   (u32 *) &mfd->panel_info.
						   pdest);
			msm_fb_debugfs_file_create(sub_dir, "backbuff",
						   (u32 *) &mfd->panel_info.
						   fb_num);
			msm_fb_debugfs_file_create(sub_dir, "clk_rate",
						   (u32 *) &mfd->panel_info.
						   clk_rate);
			msm_fb_debugfs_file_create(sub_dir, "frame_count",
						   (u32 *) &mfd->panel_info.
						   frame_count);

			switch (mfd->dest) {
			case DISPLAY_LCD:
				msm_fb_debugfs_file_create(sub_dir,
				"vsync_enable",
				(u32 *)&mfd->panel_info.lcd.vsync_enable);
				msm_fb_debugfs_file_create(sub_dir,
				"refx100",
				(u32 *) &mfd->panel_info.lcd. refx100);
				msm_fb_debugfs_file_create(sub_dir,
				"v_back_porch",
				(u32 *) &mfd->panel_info.lcd.v_back_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"v_front_porch",
				(u32 *) &mfd->panel_info.lcd.v_front_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"v_pulse_width",
				(u32 *) &mfd->panel_info.lcd.v_pulse_width);
				msm_fb_debugfs_file_create(sub_dir,
				"hw_vsync_mode",
				(u32 *) &mfd->panel_info.lcd.hw_vsync_mode);
				msm_fb_debugfs_file_create(sub_dir,
				"vsync_notifier_period", (u32 *)
				&mfd->panel_info.lcd.vsync_notifier_period);
				break;

			case DISPLAY_LCDC:
				msm_fb_debugfs_file_create(sub_dir,
				"h_back_porch",
				(u32 *) &mfd->panel_info.lcdc.h_back_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"h_front_porch",
				(u32 *) &mfd->panel_info.lcdc.h_front_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"h_pulse_width",
				(u32 *) &mfd->panel_info.lcdc.h_pulse_width);
				msm_fb_debugfs_file_create(sub_dir,
				"v_back_porch",
				(u32 *) &mfd->panel_info.lcdc.v_back_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"v_front_porch",
				(u32 *) &mfd->panel_info.lcdc.v_front_porch);
				msm_fb_debugfs_file_create(sub_dir,
				"v_pulse_width",
				(u32 *) &mfd->panel_info.lcdc.v_pulse_width);
				msm_fb_debugfs_file_create(sub_dir,
				"border_clr",
				(u32 *) &mfd->panel_info.lcdc.border_clr);
				msm_fb_debugfs_file_create(sub_dir,
				"underflow_clr",
				(u32 *) &mfd->panel_info.lcdc.underflow_clr);
				msm_fb_debugfs_file_create(sub_dir,
				"hsync_skew",
				(u32 *) &mfd->panel_info.lcdc.hsync_skew);
				break;

			default:
				break;
			}
		}
	}
#endif /* MSM_FB_ENABLE_DBGFS */

	return ret;
}

static int msm_fb_open(struct fb_info *info, int user)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	int result;

	result = pm_runtime_get_sync(info->dev);

	if (result < 0)
		printk(KERN_DEBUG "pm_runtime: fail to wake up %d\n", result);

	if (!mfd->ref_cnt) {
		mdp_set_dma_pan_info(info, NULL, TRUE);

/* To support "powertest 3", remove to msm_fb_init()
		if (msm_fb_blank_sub(FB_BLANK_UNBLANK, info, mfd->op_enable)) {
			printk(KERN_ERR "msm_fb_open: can't turn on display!\n");
			return -1;
		}
*/
	}

	mfd->ref_cnt++;
	return 0;
}

static int msm_fb_release(struct fb_info *info, int user)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	int ret = 0;

	if (!mfd->ref_cnt) {
		MSM_FB_INFO("msm_fb_release: try to close unopened fb %d!\n",
			    mfd->index);
		return -EINVAL;
	}

	mfd->ref_cnt--;

	pm_runtime_put(info->dev);
	return ret;
}

DEFINE_SEMAPHORE(msm_fb_pan_sem);

static int msm_fb_pan_display(struct fb_var_screeninfo *var,
			      struct fb_info *info)
{
	struct mdp_dirty_region dirty;
	struct mdp_dirty_region *dirtyPtr = NULL;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	static unsigned long cur_autobkl_stat = 0, last_autobkl_stat = 0;
	static bool bfirsttime = 1;
	struct msm_fb_panel_data *pdata;
	static uint64_t dt;
	static int64_t frame_count;
	static ktime_t last_sec;
	ktime_t now;

	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	if ((!mfd->op_enable) || (!mfd->panel_power_on))
		return -EPERM;

	if (var->xoffset > (info->var.xres_virtual - info->var.xres))
		return -EINVAL;

	if (var->yoffset > (info->var.yres_virtual - info->var.yres))
		return -EINVAL;

	if (info->fix.xpanstep)
		info->var.xoffset =
		    (var->xoffset / info->fix.xpanstep) * info->fix.xpanstep;

	if (info->fix.ypanstep)
		info->var.yoffset =
		    (var->yoffset / info->fix.ypanstep) * info->fix.ypanstep;

	/* "UPDT" */
	if (var->reserved[0] == 0x54445055) {
		dirty.xoffset = var->reserved[1] & 0xffff;
		dirty.yoffset = (var->reserved[1] >> 16) & 0xffff;

		if ((var->reserved[2] & 0xffff) <= dirty.xoffset)
			return -EINVAL;
		if (((var->reserved[2] >> 16) & 0xffff) <= dirty.yoffset)
			return -EINVAL;

		dirty.width = (var->reserved[2] & 0xffff) - dirty.xoffset;
		dirty.height =
		    ((var->reserved[2] >> 16) & 0xffff) - dirty.yoffset;
		info->var.yoffset = var->yoffset;

		if (dirty.xoffset < 0)
			return -EINVAL;

		if (dirty.yoffset < 0)
			return -EINVAL;

		if ((dirty.xoffset + dirty.width) > info->var.xres)
			return -EINVAL;

		if ((dirty.yoffset + dirty.height) > info->var.yres)
			return -EINVAL;

		if ((dirty.width <= 0) || (dirty.height <= 0))
			return -EINVAL;

		dirtyPtr = &dirty;
	}

#if defined (CONFIG_FB_MSM_MDP_ABL)
	if (mfd->enable_abl) {
		if (waitqueue_active(&mfd->msmfb_update_notify.wait))
			complete(&mfd->msmfb_update_notify);
	} else {
		if (waitqueue_active(&mfd->msmfb_no_update_notify.wait))
			complete(&mfd->msmfb_no_update_notify);
	}
#if 0
	complete(&mfd->msmfb_update_notify);
	mutex_lock(&msm_fb_notify_update_sem);
	if (mfd->msmfb_no_update_notify_timer.function)
		del_timer(&mfd->msmfb_no_update_notify_timer);
	mfd->msmfb_no_update_notify_timer.expires =
		jiffies + ((1000 * HZ) / 1000);
	add_timer(&mfd->msmfb_no_update_notify_timer);
	mutex_unlock(&msm_fb_notify_update_sem);
#endif
#endif

#if (defined(CONFIG_USB_FUNCTION_PROJECTOR) || defined(CONFIG_USB_ANDROID_PROJECTOR))
	if (mfd->index == 0)
		msmfb_set_var(mfd->fbi->screen_base, var->yoffset);
#endif

	down(&msm_fb_pan_sem);
	if (mfd->mdp_pdata && mfd->mdp_pdata->mdp_img_stick_wa) {
		if (atomic_read(&mfd->mdp_pdata->img_stick_on) == 0) {
			if (mfd->mdp_pdata->mdp_img_stick_wa)
					mfd->mdp_pdata->mdp_img_stick_wa(0);
			atomic_set(&mfd->mdp_pdata->img_stick_on, 1);
		}
		mod_timer(&mfd->frame_update_timer, jiffies + msecs_to_jiffies(mfd->mdp_pdata->update_interval));
	}
#if defined CONFIG_FB_MSM_SELF_REFRESH
	if (mfd->request_display_on == 0) {
		if ((pdata) && (pdata->self_refresh_switch)) {
			del_timer_sync(&mfd->self_refresh_timer);
			flush_workqueue(mfd->self_refresh_wq);
			mutex_lock(&self_refresh_lock);
			mod_timer(&mfd->self_refresh_timer, jiffies + msecs_to_jiffies(1000));
			if (atomic_read(&self_refresh_on)) {
				atomic_set(&self_refresh_on, 0);
				mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
				pdata->self_refresh_switch(0);
			}
			mutex_unlock(&self_refresh_lock);
		}
	}
#endif
	mdp_set_dma_pan_info(info, dirtyPtr,
			     (var->activate == FB_ACTIVATE_VBL));
	mdp_dma_pan_update(info);
	up(&msm_fb_pan_sem);

	if (msmfb_debug_mask & FPS) {
		now = ktime_get();
		dt = ktime_to_ns(ktime_sub(now, last_sec));
		frame_count++;
		if (dt > NSEC_PER_SEC) {
			int64_t fps = frame_count * NSEC_PER_SEC * 100;
			frame_count = 0;
			last_sec = ktime_get();
			do_div(fps, dt);
			DLOG(FPS, "pan fps * 100: %llu\n", fps);
		}
	}

	if (mfd->request_display_on) {
		msm_fb_display_on(mfd);
		mfd->request_display_on = 0;
	}

	++mfd->panel_info.frame_count;

	if (mfd->mdp_pdata && mfd->mdp_pdata->dcr_panel_pinfo) {
		if (bfirsttime) {
			last_autobkl_stat = cur_autobkl_stat = auto_bkl_status;
			bfirsttime = 0;
		}
		cur_autobkl_stat = auto_bkl_status;
		if (cur_autobkl_stat != last_autobkl_stat) {
			last_autobkl_stat = cur_autobkl_stat;
			if (test_bit(CABC_STATE_DCR, &auto_bkl_status) == 1) { /* autobkl mode */
				mfd->mdp_pdata->dcr_panel_pinfo->dcr_video_mode(0); /* default mode */
				mfd->mdp_pdata->dcr_panel_pinfo->dcr_power(1);
				mfd->mdp_pdata->dcr_panel_pinfo->bkl_smooth(1);
			} else { /* manual bkl mode */
				mfd->mdp_pdata->dcr_panel_pinfo->bkl_smooth(0);
				mfd->mdp_pdata->dcr_panel_pinfo->dcr_power(0); /* off mode */
				atomic_set(&mfd->mdp_pdata->dcr_panel_pinfo->video_mode, 0);
			}
		}
	}

	if (fps_interval > 16) {
		unsigned int sleep_ms = fps_interval - 16;
		if (__ratelimit(&fps_msg_ratelimit))
			PR_DISP_INFO("[FPS] Current FPS is locked to %d\n", 1000/fps_interval);

		msleep(sleep_ms);
	}

	return 0;
}

static int msm_fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	if (var->rotate != FB_ROTATE_UR)
		return -EINVAL;
	if (var->grayscale != info->var.grayscale)
		return -EINVAL;

	switch (var->bits_per_pixel) {
	case 16:
		if ((var->green.offset != 5) ||
			!((var->blue.offset == 11)
				|| (var->blue.offset == 0)) ||
			!((var->red.offset == 11)
				|| (var->red.offset == 0)) ||
			(var->blue.length != 5) ||
			(var->green.length != 6) ||
			(var->red.length != 5) ||
			(var->blue.msb_right != 0) ||
			(var->green.msb_right != 0) ||
			(var->red.msb_right != 0) ||
			(var->transp.offset != 0) ||
			(var->transp.length != 0))
				return -EINVAL;
		break;

	case 24:
		if ((var->blue.offset != 0) ||
			(var->green.offset != 8) ||
			(var->red.offset != 16) ||
			(var->blue.length != 8) ||
			(var->green.length != 8) ||
			(var->red.length != 8) ||
			(var->blue.msb_right != 0) ||
			(var->green.msb_right != 0) ||
			(var->red.msb_right != 0) ||
			!(((var->transp.offset == 0) &&
				(var->transp.length == 0)) ||
			  ((var->transp.offset == 24) &&
				(var->transp.length == 8))))
				return -EINVAL;
		break;

	case 32:
		/* Figure out if the user meant RGBA or ARGB
		   and verify the position of the RGB components */

		if (var->transp.offset == 24) {
			if ((var->blue.offset != 0) ||
			    (var->green.offset != 8) ||
			    (var->red.offset != 16))
				return -EINVAL;
		} else if (var->transp.offset == 0) {
			if ((var->blue.offset != 16) ||
			    (var->green.offset != 8) ||
			    (var->red.offset != 0))
				return -EINVAL;
		} else
			return -EINVAL;

		/* Check the common values for both RGBA and ARGB */

		if ((var->blue.length != 8) ||
		    (var->green.length != 8) ||
		    (var->red.length != 8) ||
		    (var->transp.length != 8) ||
		    (var->blue.msb_right != 0) ||
		    (var->green.msb_right != 0) ||
		    (var->red.msb_right != 0))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	if ((var->xres_virtual <= 0) || (var->yres_virtual <= 0))
		return -EINVAL;

	if (info->fix.smem_len <
		(var->xres_virtual*var->yres_virtual*(var->bits_per_pixel/8)))
		return -EINVAL;

	if ((var->xres == 0) || (var->yres == 0))
		return -EINVAL;

	if ((var->xres > MAX(mfd->panel_info.xres,
			     mfd->panel_info.mode2_xres)) ||
		(var->yres > MAX(mfd->panel_info.yres,
				 mfd->panel_info.mode2_yres)))
		return -EINVAL;

	if (var->xoffset > (var->xres_virtual - var->xres))
		return -EINVAL;

	if (var->yoffset > (var->yres_virtual - var->yres))
		return -EINVAL;

	return 0;
}

static int msm_fb_set_par(struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct fb_var_screeninfo *var = &info->var;
	int old_imgType;
	int blank = 0;

	old_imgType = mfd->fb_imgType;
	switch (var->bits_per_pixel) {
	case 16:
		if (var->red.offset == 0)
			mfd->fb_imgType = MDP_BGR_565;
		else
			mfd->fb_imgType = MDP_RGB_565;
		break;

	case 24:
		if ((var->transp.offset == 0) && (var->transp.length == 0))
			mfd->fb_imgType = MDP_RGB_888;
		else if ((var->transp.offset == 24) &&
				(var->transp.length == 8)) {
			mfd->fb_imgType = MDP_ARGB_8888;
			info->var.bits_per_pixel = 32;
		}
		break;

	case 32:
		if (var->transp.offset == 24)
			mfd->fb_imgType = MDP_ARGB_8888;
		else
			mfd->fb_imgType = MDP_RGBA_8888;
		break;

	default:
		return -EINVAL;
	}

	if ((mfd->var_pixclock != var->pixclock) ||
		(mfd->hw_refresh && ((mfd->fb_imgType != old_imgType) ||
				(mfd->var_pixclock != var->pixclock) ||
				(mfd->var_xres != var->xres) ||
				(mfd->var_yres != var->yres)))) {
		mfd->var_xres = var->xres;
		mfd->var_yres = var->yres;
		mfd->var_pixclock = var->pixclock;
		blank = 1;
	}
	mfd->fbi->fix.line_length = msm_fb_line_length(mfd->index, var->xres,
						       var->bits_per_pixel/8);

	if (blank) {
		msm_fb_blank_sub(FB_BLANK_POWERDOWN, info, mfd->op_enable);
		msm_fb_blank_sub(FB_BLANK_UNBLANK, info, mfd->op_enable);
	}

	return 0;
}

static int msm_fb_stop_sw_refresher(struct msm_fb_data_type *mfd)
{
	if (mfd->hw_refresh)
		return -EPERM;

	if (mfd->sw_currently_refreshing) {
		down(&mfd->sem);
		mfd->sw_currently_refreshing = FALSE;
		up(&mfd->sem);

		/* wait until the refresher finishes the last job */
		wait_for_completion_killable(&mfd->refresher_comp);
	}

	return 0;
}

int msm_fb_resume_sw_refresher(struct msm_fb_data_type *mfd)
{
	boolean do_refresh;

	if (mfd->hw_refresh)
		return -EPERM;

	down(&mfd->sem);
	if ((!mfd->sw_currently_refreshing) && (mfd->sw_refreshing_enable)) {
		do_refresh = TRUE;
		mfd->sw_currently_refreshing = TRUE;
	} else {
		do_refresh = FALSE;
	}
	up(&mfd->sem);

	if (do_refresh)
		mdp_refresh_screen((unsigned long)mfd);

	return 0;
}

#if defined CONFIG_MSM_MDP31
static int mdp_blit_split_height(struct fb_info *info,
				struct mdp_blit_req *req)
{
	int ret;
	struct mdp_blit_req splitreq;
	int s_x_0, s_x_1, s_w_0, s_w_1, s_y_0, s_y_1, s_h_0, s_h_1;
	int d_x_0, d_x_1, d_w_0, d_w_1, d_y_0, d_y_1, d_h_0, d_h_1;

	splitreq = *req;
	/* break dest roi at height*/
	d_x_0 = d_x_1 = req->dst_rect.x;
	d_w_0 = d_w_1 = req->dst_rect.w;
	d_y_0 = req->dst_rect.y;
	if (req->dst_rect.h % 32 == 3)
		d_h_1 = (req->dst_rect.h - 3) / 2 - 1;
	else if (req->dst_rect.h % 32 == 2)
		d_h_1 = (req->dst_rect.h - 2) / 2 - 6;
	else
		d_h_1 = (req->dst_rect.h - 1) / 2 - 1;
	d_h_0 = req->dst_rect.h - d_h_1;
	d_y_1 = d_y_0 + d_h_0;
	if (req->dst_rect.h == 3) {
		d_h_1 = 2;
		d_h_0 = 2;
		d_y_1 = d_y_0 + 1;
	}

	/* blit first region */
	if (((splitreq.flags & 0x07) == 0x04) ||
		((splitreq.flags & 0x07) == 0x0)) {

		if (splitreq.flags & MDP_ROT_90) {
			s_y_0 = s_y_1 = req->src_rect.y;
			s_h_0 = s_h_1 = req->src_rect.h;
			s_x_0 = req->src_rect.x;
			s_w_1 = (req->src_rect.w * d_h_1) / req->dst_rect.h;
			s_w_0 = req->src_rect.w - s_w_1;
			s_x_1 = s_x_0 + s_w_0;
			if (d_h_1 >= 8 * s_w_1) {
				s_w_1++;
				s_x_1--;
			}
		} else {
			s_x_0 = s_x_1 = req->src_rect.x;
			s_w_0 = s_w_1 = req->src_rect.w;
			s_y_0 = req->src_rect.y;
			s_h_1 = (req->src_rect.h * d_h_1) / req->dst_rect.h;
			s_h_0 = req->src_rect.h - s_h_1;
			s_y_1 = s_y_0 + s_h_0;
			if (d_h_1 >= 8 * s_h_1) {
				s_h_1++;
				s_y_1--;
			}
		}

		splitreq.src_rect.h = s_h_0;
		splitreq.src_rect.y = s_y_0;
		splitreq.dst_rect.h = d_h_0;
		splitreq.dst_rect.y = d_y_0;
		splitreq.src_rect.x = s_x_0;
		splitreq.src_rect.w = s_w_0;
		splitreq.dst_rect.x = d_x_0;
		splitreq.dst_rect.w = d_w_0;
	} else {

		if (splitreq.flags & MDP_ROT_90) {
			s_y_0 = s_y_1 = req->src_rect.y;
			s_h_0 = s_h_1 = req->src_rect.h;
			s_x_0 = req->src_rect.x;
			s_w_1 = (req->src_rect.w * d_h_0) / req->dst_rect.h;
			s_w_0 = req->src_rect.w - s_w_1;
			s_x_1 = s_x_0 + s_w_0;
			if (d_h_0 >= 8 * s_w_1) {
				s_w_1++;
				s_x_1--;
			}
		} else {
			s_x_0 = s_x_1 = req->src_rect.x;
			s_w_0 = s_w_1 = req->src_rect.w;
			s_y_0 = req->src_rect.y;
			s_h_1 = (req->src_rect.h * d_h_0) / req->dst_rect.h;
			s_h_0 = req->src_rect.h - s_h_1;
			s_y_1 = s_y_0 + s_h_0;
			if (d_h_0 >= 8 * s_h_1) {
				s_h_1++;
				s_y_1--;
			}
		}
		splitreq.src_rect.h = s_h_0;
		splitreq.src_rect.y = s_y_0;
		splitreq.dst_rect.h = d_h_1;
		splitreq.dst_rect.y = d_y_1;
		splitreq.src_rect.x = s_x_0;
		splitreq.src_rect.w = s_w_0;
		splitreq.dst_rect.x = d_x_1;
		splitreq.dst_rect.w = d_w_1;
	}
	ret = mdp_ppp_blit(info, &splitreq);
	if (ret)
		return ret;

	/* blit second region */
	if (((splitreq.flags & 0x07) == 0x04) ||
		((splitreq.flags & 0x07) == 0x0)) {
		splitreq.src_rect.h = s_h_1;
		splitreq.src_rect.y = s_y_1;
		splitreq.dst_rect.h = d_h_1;
		splitreq.dst_rect.y = d_y_1;
		splitreq.src_rect.x = s_x_1;
		splitreq.src_rect.w = s_w_1;
		splitreq.dst_rect.x = d_x_1;
		splitreq.dst_rect.w = d_w_1;
	} else {
		splitreq.src_rect.h = s_h_1;
		splitreq.src_rect.y = s_y_1;
		splitreq.dst_rect.h = d_h_0;
		splitreq.dst_rect.y = d_y_0;
		splitreq.src_rect.x = s_x_1;
		splitreq.src_rect.w = s_w_1;
		splitreq.dst_rect.x = d_x_0;
		splitreq.dst_rect.w = d_w_0;
	}
	ret = mdp_ppp_blit(info, &splitreq);
	return ret;
}
#endif

int mdp_blit(struct fb_info *info, struct mdp_blit_req *req)
{
	int ret;
#if defined CONFIG_MSM_MDP31 || defined CONFIG_MSM_MDP30
	unsigned int remainder = 0, is_bpp_4 = 0;
	struct mdp_blit_req splitreq;
	int s_x_0, s_x_1, s_w_0, s_w_1, s_y_0, s_y_1, s_h_0, s_h_1;
	int d_x_0, d_x_1, d_w_0, d_w_1, d_y_0, d_y_1, d_h_0, d_h_1;

	if (req->flags & MDP_ROT_90) {
		if (((req->dst_rect.h == 1) && ((req->src_rect.w != 1) ||
			(req->dst_rect.w != req->src_rect.h))) ||
			((req->dst_rect.w == 1) && ((req->src_rect.h != 1) ||
			(req->dst_rect.h != req->src_rect.w)))) {
			printk(KERN_ERR "mpd_ppp: error scaling when size is 1!\n");
			return -EINVAL;
		}
	} else {
		if (((req->dst_rect.w == 1) && ((req->src_rect.w != 1) ||
			(req->dst_rect.h != req->src_rect.h))) ||
			((req->dst_rect.h == 1) && ((req->src_rect.h != 1) ||
			(req->dst_rect.w != req->src_rect.w)))) {
			printk(KERN_ERR "mpd_ppp: error scaling when size is 1!\n");
			return -EINVAL;
		}
	}
#endif
	if (unlikely(req->src_rect.h == 0 || req->src_rect.w == 0)) {
		printk(KERN_ERR "mpd_ppp: src img of zero size!\n");
		return -EINVAL;
	}
	if (unlikely(req->dst_rect.h == 0 || req->dst_rect.w == 0))
		return 0;

#if defined CONFIG_MSM_MDP31
	/* MDP width split workaround */
	remainder = (req->dst_rect.w)%32;
	ret = mdp_get_bytes_per_pixel(req->dst.format,
					(struct msm_fb_data_type *)info->par);
	if (ret <= 0) {
		printk(KERN_ERR "mdp_ppp: incorrect bpp!\n");
		return -EINVAL;
	}
	is_bpp_4 = (ret == 4) ? 1 : 0;

	if ((is_bpp_4 && (remainder == 6 || remainder == 14 ||
	remainder == 22 || remainder == 30)) || remainder == 3 ||
	(remainder == 1 && req->dst_rect.w != 1) ||
	(remainder == 2 && req->dst_rect.w != 2)) {
		/* make new request as provide by user */
		splitreq = *req;

		/* break dest roi at width*/
		d_y_0 = d_y_1 = req->dst_rect.y;
		d_h_0 = d_h_1 = req->dst_rect.h;
		d_x_0 = req->dst_rect.x;

		if (remainder == 14)
			d_w_1 = (req->dst_rect.w - 14) / 2 + 4;
		else if (remainder == 22)
			d_w_1 = (req->dst_rect.w - 22) / 2 + 10;
		else if (remainder == 30)
			d_w_1 = (req->dst_rect.w - 30) / 2 + 10;
		else if (remainder == 6)
			d_w_1 = req->dst_rect.w / 2 - 1;
		else if (remainder == 3)
			d_w_1 = (req->dst_rect.w - 3) / 2 - 1;
		else if (remainder == 2)
			d_w_1 = (req->dst_rect.w - 2) / 2 - 6;
		else
			d_w_1 = (req->dst_rect.w - 1) / 2 - 1;
		d_w_0 = req->dst_rect.w - d_w_1;
		d_x_1 = d_x_0 + d_w_0;
		if (req->dst_rect.w == 3) {
			d_w_1 = 2;
			d_w_0 = 2;
			d_x_1 = d_x_0 + 1;
		}

		/* blit first region */
		if (((splitreq.flags & 0x07) == 0x07) ||
			((splitreq.flags & 0x07) == 0x0)) {

			if (splitreq.flags & MDP_ROT_90) {
				s_x_0 = s_x_1 = req->src_rect.x;
				s_w_0 = s_w_1 = req->src_rect.w;
				s_y_0 = req->src_rect.y;
				s_h_1 = (req->src_rect.h * d_w_1) /
					req->dst_rect.w;
				s_h_0 = req->src_rect.h - s_h_1;
				s_y_1 = s_y_0 + s_h_0;
				if (d_w_1 >= 8 * s_h_1) {
					s_h_1++;
					s_y_1--;
				}
			} else {
				s_y_0 = s_y_1 = req->src_rect.y;
				s_h_0 = s_h_1 = req->src_rect.h;
				s_x_0 = req->src_rect.x;
				s_w_1 = (req->src_rect.w * d_w_1) /
					req->dst_rect.w;
				s_w_0 = req->src_rect.w - s_w_1;
				s_x_1 = s_x_0 + s_w_0;
				if (d_w_1 >= 8 * s_w_1) {
					s_w_1++;
					s_x_1--;
				}
			}

			splitreq.src_rect.h = s_h_0;
			splitreq.src_rect.y = s_y_0;
			splitreq.dst_rect.h = d_h_0;
			splitreq.dst_rect.y = d_y_0;
			splitreq.src_rect.x = s_x_0;
			splitreq.src_rect.w = s_w_0;
			splitreq.dst_rect.x = d_x_0;
			splitreq.dst_rect.w = d_w_0;
		} else {
			if (splitreq.flags & MDP_ROT_90) {
				s_x_0 = s_x_1 = req->src_rect.x;
				s_w_0 = s_w_1 = req->src_rect.w;
				s_y_0 = req->src_rect.y;
				s_h_1 = (req->src_rect.h * d_w_0) /
					req->dst_rect.w;
				s_h_0 = req->src_rect.h - s_h_1;
				s_y_1 = s_y_0 + s_h_0;
				if (d_w_0 >= 8 * s_h_1) {
					s_h_1++;
					s_y_1--;
				}
			} else {
				s_y_0 = s_y_1 = req->src_rect.y;
				s_h_0 = s_h_1 = req->src_rect.h;
				s_x_0 = req->src_rect.x;
				s_w_1 = (req->src_rect.w * d_w_0) /
					req->dst_rect.w;
				s_w_0 = req->src_rect.w - s_w_1;
				s_x_1 = s_x_0 + s_w_0;
				if (d_w_0 >= 8 * s_w_1) {
					s_w_1++;
					s_x_1--;
				}
			}
			splitreq.src_rect.h = s_h_0;
			splitreq.src_rect.y = s_y_0;
			splitreq.dst_rect.h = d_h_1;
			splitreq.dst_rect.y = d_y_1;
			splitreq.src_rect.x = s_x_0;
			splitreq.src_rect.w = s_w_0;
			splitreq.dst_rect.x = d_x_1;
			splitreq.dst_rect.w = d_w_1;
		}

		if ((splitreq.dst_rect.h % 32 == 3) ||
			((req->dst_rect.h % 32) == 1 && req->dst_rect.h != 1) ||
			((req->dst_rect.h % 32) == 2 && req->dst_rect.h != 2))
			ret = mdp_blit_split_height(info, &splitreq);
		else
			ret = mdp_ppp_blit(info, &splitreq);
		if (ret)
			return ret;
		/* blit second region */
		if (((splitreq.flags & 0x07) == 0x07) ||
			((splitreq.flags & 0x07) == 0x0)) {
			splitreq.src_rect.h = s_h_1;
			splitreq.src_rect.y = s_y_1;
			splitreq.dst_rect.h = d_h_1;
			splitreq.dst_rect.y = d_y_1;
			splitreq.src_rect.x = s_x_1;
			splitreq.src_rect.w = s_w_1;
			splitreq.dst_rect.x = d_x_1;
			splitreq.dst_rect.w = d_w_1;
		} else {
			splitreq.src_rect.h = s_h_1;
			splitreq.src_rect.y = s_y_1;
			splitreq.dst_rect.h = d_h_0;
			splitreq.dst_rect.y = d_y_0;
			splitreq.src_rect.x = s_x_1;
			splitreq.src_rect.w = s_w_1;
			splitreq.dst_rect.x = d_x_0;
			splitreq.dst_rect.w = d_w_0;
		}
		if (((splitreq.dst_rect.h % 32) == 3) ||
			((req->dst_rect.h % 32) == 1 && req->dst_rect.h != 1) ||
			((req->dst_rect.h % 32) == 2 && req->dst_rect.h != 2))
			ret = mdp_blit_split_height(info, &splitreq);
		else
			ret = mdp_ppp_blit(info, &splitreq);
		if (ret)
			return ret;
	} else if ((req->dst_rect.h % 32) == 3 ||
		((req->dst_rect.h % 32) == 1 && req->dst_rect.h != 1) ||
		((req->dst_rect.h % 32) == 2 && req->dst_rect.h != 2))
		ret = mdp_blit_split_height(info, req);
	else
		ret = mdp_ppp_blit(info, req);
	return ret;
#elif defined CONFIG_MSM_MDP30
	/* MDP width split workaround */
	remainder = (req->dst_rect.w)%16;
	ret = mdp_get_bytes_per_pixel(req->dst.format,
					(struct msm_fb_data_type *)info->par);
	if (ret <= 0) {
		printk(KERN_ERR "mdp_ppp: incorrect bpp!\n");
		return -EINVAL;
	}
	is_bpp_4 = (ret == 4) ? 1 : 0;

	if ((is_bpp_4 && (remainder == 6 || remainder == 14))) {

		/* make new request as provide by user */
		splitreq = *req;

		/* break dest roi at width*/
		d_y_0 = d_y_1 = req->dst_rect.y;
		d_h_0 = d_h_1 = req->dst_rect.h;
		d_x_0 = req->dst_rect.x;

		if (remainder == 14 || remainder == 6)
			d_w_1 = req->dst_rect.w / 2;
		else
			d_w_1 = (req->dst_rect.w - 1) / 2 - 1;

		d_w_0 = req->dst_rect.w - d_w_1;
		d_x_1 = d_x_0 + d_w_0;

		/* blit first region */
		if (((splitreq.flags & 0x07) == 0x07) ||
			((splitreq.flags & 0x07) == 0x0)) {

			if (splitreq.flags & MDP_ROT_90) {
				s_x_0 = s_x_1 = req->src_rect.x;
				s_w_0 = s_w_1 = req->src_rect.w;
				s_y_0 = req->src_rect.y;
				s_h_1 = (req->src_rect.h * d_w_1) /
					req->dst_rect.w;
				s_h_0 = req->src_rect.h - s_h_1;
				s_y_1 = s_y_0 + s_h_0;
				if (d_w_1 >= 8 * s_h_1) {
					s_h_1++;
					s_y_1--;
				}
			} else {
				s_y_0 = s_y_1 = req->src_rect.y;
				s_h_0 = s_h_1 = req->src_rect.h;
				s_x_0 = req->src_rect.x;
				s_w_1 = (req->src_rect.w * d_w_1) /
					req->dst_rect.w;
				s_w_0 = req->src_rect.w - s_w_1;
				s_x_1 = s_x_0 + s_w_0;
				if (d_w_1 >= 8 * s_w_1) {
					s_w_1++;
					s_x_1--;
				}
			}

			splitreq.src_rect.h = s_h_0;
			splitreq.src_rect.y = s_y_0;
			splitreq.dst_rect.h = d_h_0;
			splitreq.dst_rect.y = d_y_0;
			splitreq.src_rect.x = s_x_0;
			splitreq.src_rect.w = s_w_0;
			splitreq.dst_rect.x = d_x_0;
			splitreq.dst_rect.w = d_w_0;
		} else {
			if (splitreq.flags & MDP_ROT_90) {
				s_x_0 = s_x_1 = req->src_rect.x;
				s_w_0 = s_w_1 = req->src_rect.w;
				s_y_0 = req->src_rect.y;
				s_h_1 = (req->src_rect.h * d_w_0) /
					req->dst_rect.w;
				s_h_0 = req->src_rect.h - s_h_1;
				s_y_1 = s_y_0 + s_h_0;
				if (d_w_0 >= 8 * s_h_1) {
					s_h_1++;
					s_y_1--;
				}
			} else {
				s_y_0 = s_y_1 = req->src_rect.y;
				s_h_0 = s_h_1 = req->src_rect.h;
				s_x_0 = req->src_rect.x;
				s_w_1 = (req->src_rect.w * d_w_0) /
					req->dst_rect.w;
				s_w_0 = req->src_rect.w - s_w_1;
				s_x_1 = s_x_0 + s_w_0;
				if (d_w_0 >= 8 * s_w_1) {
					s_w_1++;
					s_x_1--;
				}
			}
			splitreq.src_rect.h = s_h_0;
			splitreq.src_rect.y = s_y_0;
			splitreq.dst_rect.h = d_h_1;
			splitreq.dst_rect.y = d_y_1;
			splitreq.src_rect.x = s_x_0;
			splitreq.src_rect.w = s_w_0;
			splitreq.dst_rect.x = d_x_1;
			splitreq.dst_rect.w = d_w_1;
		}

		/* No need to split in height */
		ret = mdp_ppp_blit(info, &splitreq);

		if (ret)
			return ret;

		/* blit second region */
		if (((splitreq.flags & 0x07) == 0x07) ||
			((splitreq.flags & 0x07) == 0x0)) {
			splitreq.src_rect.h = s_h_1;
			splitreq.src_rect.y = s_y_1;
			splitreq.dst_rect.h = d_h_1;
			splitreq.dst_rect.y = d_y_1;
			splitreq.src_rect.x = s_x_1;
			splitreq.src_rect.w = s_w_1;
			splitreq.dst_rect.x = d_x_1;
			splitreq.dst_rect.w = d_w_1;
		} else {
			splitreq.src_rect.h = s_h_1;
			splitreq.src_rect.y = s_y_1;
			splitreq.dst_rect.h = d_h_0;
			splitreq.dst_rect.y = d_y_0;
			splitreq.src_rect.x = s_x_1;
			splitreq.src_rect.w = s_w_1;
			splitreq.dst_rect.x = d_x_0;
			splitreq.dst_rect.w = d_w_0;
		}

		/* No need to split in height ... just width */
		ret = mdp_ppp_blit(info, &splitreq);

		if (ret)
			return ret;

	} else
		ret = mdp_ppp_blit(info, req);
	return ret;
#else
	ret = mdp_ppp_blit(info, req);
	return ret;
#endif
}

typedef void (*msm_dma_barrier_function_pointer) (void *, size_t);

static inline void msm_fb_dma_barrier_for_rect(struct fb_info *info,
			struct mdp_img *img, struct mdp_rect *rect,
			msm_dma_barrier_function_pointer dma_barrier_fp
			)
{
	/*
	 * Compute the start and end addresses of the rectangles.
	 * NOTE: As currently implemented, the data between
	 *       the end of one row and the start of the next is
	 *       included in the address range rather than
	 *       doing multiple calls for each row.
	 */
	unsigned long start;
	size_t size;
	char * const pmem_start = info->screen_base;
	int bytes_per_pixel = mdp_get_bytes_per_pixel(img->format,
					(struct msm_fb_data_type *)info->par);
	if (bytes_per_pixel <= 0) {
		printk(KERN_ERR "%s incorrect bpp!\n", __func__);
		return;
	}
	start = (unsigned long)pmem_start + img->offset +
		(img->width * rect->y + rect->x) * bytes_per_pixel;
	size  = (rect->h * img->width + rect->w) * bytes_per_pixel;
	(*dma_barrier_fp) ((void *) start, size);

}

static inline void msm_dma_nc_pre(void)
{
	dmb();
}
static inline void msm_dma_wt_pre(void)
{
	dmb();
}
static inline void msm_dma_todevice_wb_pre(void *start, size_t size)
{
	dma_cache_pre_ops(start, size, DMA_TO_DEVICE);
}

static inline void msm_dma_fromdevice_wb_pre(void *start, size_t size)
{
	dma_cache_pre_ops(start, size, DMA_FROM_DEVICE);
}

static inline void msm_dma_nc_post(void)
{
	dmb();
}

static inline void msm_dma_fromdevice_wt_post(void *start, size_t size)
{
	dma_cache_post_ops(start, size, DMA_FROM_DEVICE);
}

static inline void msm_dma_todevice_wb_post(void *start, size_t size)
{
	dma_cache_post_ops(start, size, DMA_TO_DEVICE);
}

static inline void msm_dma_fromdevice_wb_post(void *start, size_t size)
{
	dma_cache_post_ops(start, size, DMA_FROM_DEVICE);
}

/*
 * Do the write barriers required to guarantee data is committed to RAM
 * (from CPU cache or internal buffers) before a DMA operation starts.
 * NOTE: As currently implemented, the data between
 *       the end of one row and the start of the next is
 *       included in the address range rather than
 *       doing multiple calls for each row.
*/
static void msm_fb_ensure_memory_coherency_before_dma(struct fb_info *info,
		struct mdp_blit_req *req_list,
		int req_list_count)
{
#ifdef CONFIG_ARCH_QSD8X50
	int i;

	/*
	 * Normally, do the requested barriers for each address
	 * range that corresponds to a rectangle.
	 *
	 * But if at least one write barrier is requested for data
	 * going to or from the device but no address range is
	 * needed for that barrier, then do the barrier, but do it
	 * only once, no matter how many requests there are.
	 */
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	switch (mfd->mdp_fb_page_protection)	{
	default:
	case MDP_FB_PAGE_PROTECTION_NONCACHED:
	case MDP_FB_PAGE_PROTECTION_WRITECOMBINE:
		/*
		 * The following barrier is only done at most once,
		 * since further calls would be redundant.
		 */
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags
				& MDP_NO_DMA_BARRIER_START)) {
				msm_dma_nc_pre();
				break;
			}
		}
		break;

	case MDP_FB_PAGE_PROTECTION_WRITETHROUGHCACHE:
		/*
		 * The following barrier is only done at most once,
		 * since further calls would be redundant.
		 */
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags
				& MDP_NO_DMA_BARRIER_START)) {
				msm_dma_wt_pre();
				break;
			}
		}
		break;

	case MDP_FB_PAGE_PROTECTION_WRITEBACKCACHE:
	case MDP_FB_PAGE_PROTECTION_WRITEBACKWACACHE:
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags &
					MDP_NO_DMA_BARRIER_START)) {

				msm_fb_dma_barrier_for_rect(info,
						&(req_list[i].src),
						&(req_list[i].src_rect),
						msm_dma_todevice_wb_pre
						);

				msm_fb_dma_barrier_for_rect(info,
						&(req_list[i].dst),
						&(req_list[i].dst_rect),
						msm_dma_todevice_wb_pre
						);
			}
		}
		break;
	}
#else
	dmb();
#endif
}


/*
 * Do the write barriers required to guarantee data will be re-read from RAM by
 * the CPU after a DMA operation ends.
 * NOTE: As currently implemented, the data between
 *       the end of one row and the start of the next is
 *       included in the address range rather than
 *       doing multiple calls for each row.
*/
static void msm_fb_ensure_memory_coherency_after_dma(struct fb_info *info,
		struct mdp_blit_req *req_list,
		int req_list_count)
{
#ifdef CONFIG_ARCH_QSD8X50
	int i;

	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	switch (mfd->mdp_fb_page_protection)	{
	default:
	case MDP_FB_PAGE_PROTECTION_NONCACHED:
	case MDP_FB_PAGE_PROTECTION_WRITECOMBINE:
		/*
		 * The following barrier is only done at most once,
		 * since further calls would be redundant.
		 */
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags
				& MDP_NO_DMA_BARRIER_END)) {
				msm_dma_nc_post();
				break;
			}
		}
		break;

	case MDP_FB_PAGE_PROTECTION_WRITETHROUGHCACHE:
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags &
					MDP_NO_DMA_BARRIER_END)) {

				msm_fb_dma_barrier_for_rect(info,
						&(req_list[i].dst),
						&(req_list[i].dst_rect),
						msm_dma_fromdevice_wt_post
						);
			}
		}
		break;
	case MDP_FB_PAGE_PROTECTION_WRITEBACKCACHE:
	case MDP_FB_PAGE_PROTECTION_WRITEBACKWACACHE:
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags &
					MDP_NO_DMA_BARRIER_END)) {

				msm_fb_dma_barrier_for_rect(info,
						&(req_list[i].dst),
						&(req_list[i].dst_rect),
						msm_dma_fromdevice_wb_post
						);
			}
		}
		break;
	}
#else
	dmb();
#endif
}

/*
 * NOTE: The userspace issues blit operations in a sequence, the sequence
 * start with a operation marked START and ends in an operation marked
 * END. It is guranteed by the userspace that all the blit operations
 * between START and END are only within the regions of areas designated
 * by the START and END operations and that the userspace doesnt modify
 * those areas. Hence it would be enough to perform barrier/cache operations
 * only on the START and END operations.
 */
static int msmfb_blit(struct fb_info *info, void __user *p)
{
	/*
	 * CAUTION: The names of the struct types intentionally *DON'T* match
	 * the names of the variables declared -- they appear to be swapped.
	 * Read the code carefully and you should see that the variable names
	 * make sense.
	 */
	const int MAX_LIST_WINDOW = 16;
	struct mdp_blit_req req_list[MAX_LIST_WINDOW];
	struct mdp_blit_req_list req_list_header;

	int count, i, req_list_count;

	/* Get the count size for the total BLIT request. */
	if (copy_from_user(&req_list_header, p, sizeof(req_list_header)))
		return -EFAULT;
	p += sizeof(req_list_header);
	count = req_list_header.count;
	if (count < 0 || count >= MAX_BLIT_REQ)
		return -EINVAL;
	while (count > 0) {
		/*
		 * Access the requests through a narrow window to decrease copy
		 * overhead and make larger requests accessible to the
		 * coherency management code.
		 * NOTE: The window size is intended to be larger than the
		 *       typical request size, but not require more than 2
		 *       kbytes of stack storage.
		 */
		req_list_count = count;
		if (req_list_count > MAX_LIST_WINDOW)
			req_list_count = MAX_LIST_WINDOW;
		if (copy_from_user(&req_list, p,
				sizeof(struct mdp_blit_req)*req_list_count))
			return -EFAULT;

		/*
		 * Ensure that any data CPU may have previously written to
		 * internal state (but not yet committed to memory) is
		 * guaranteed to be committed to memory now.
		 */
		msm_fb_ensure_memory_coherency_before_dma(info,
				req_list, req_list_count);

		/*
		 * Do the blit DMA, if required -- returning early only if
		 * there is a failure.
		 */
		for (i = 0; i < req_list_count; i++) {
			if (!(req_list[i].flags & MDP_NO_BLIT)) {
				/* Do the actual blit. */
				int ret = mdp_blit(info, &(req_list[i]));

				/*
				 * Note that early returns don't guarantee
				 * memory coherency.
				 */
				if (ret)
					return ret;
			}
		}

		/*
		 * Ensure that CPU cache and other internal CPU state is
		 * updated to reflect any change in memory modified by MDP blit
		 * DMA.
		 */
		msm_fb_ensure_memory_coherency_after_dma(info,
				req_list,
				req_list_count);

		/* Go to next window of requests. */
		count -= req_list_count;
		p += sizeof(struct mdp_blit_req)*req_list_count;
	}
	return 0;
}

#ifdef CONFIG_FB_MSM_OVERLAY
static int msmfb_overlay_get(struct fb_info *info, void __user *p)
{
	struct mdp_overlay req;
	int ret;

	if (copy_from_user(&req, p, sizeof(req))) {
		printk(KERN_ERR "%s(%d): copy2user failed \n",
			__func__, __LINE__);
		return -EFAULT;
	}

	ret = mdp4_overlay_get(info, &req);
	if (ret) {
		printk(KERN_ERR "%s: ioctl failed \n",
			__func__);
		return ret;
	}
	if (copy_to_user(p, &req, sizeof(req))) {
		printk(KERN_ERR "%s(%d): copy2user failed \n",
			__func__, __LINE__);
		return -EFAULT;
	}

	return 0;
}

static int msmfb_overlay_set(struct fb_info *info, void __user *p)
{
	struct mdp_overlay req;
	int ret;

	if (copy_from_user(&req, p, sizeof(req)))
		return -EFAULT;

	ret = mdp4_overlay_set(info, &req);
	if (ret) {
		printk(KERN_ERR "%s: ioctl failed, rc=%d\n",
			__func__, ret);
		return ret;
	}

	if (copy_to_user(p, &req, sizeof(req))) {
		printk(KERN_ERR "%s: copy2user failed \n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int msmfb_overlay_unset(struct fb_info *info, unsigned long *argp)
{
	int	ret, ndx;
#if defined CONFIG_FB_MSM_SELF_REFRESH
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct msm_fb_panel_data *pdata;
	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;
#endif
	ret = copy_from_user(&ndx, argp, sizeof(ndx));
	if (ret) {
		printk(KERN_ERR "%s:msmfb_overlay_unset ioctl failed \n",
			__func__);
		return ret;
	}
#if defined CONFIG_FB_MSM_SELF_REFRESH
	if ((pdata) && (pdata->self_refresh_switch) &&
		atomic_read(&self_refresh_suspend) == 0) {
		del_timer_sync(&mfd->self_refresh_timer);
		flush_workqueue(mfd->self_refresh_wq);
		mutex_lock(&self_refresh_lock);
		if (atomic_read(&self_refresh_on)) {
			atomic_set(&self_refresh_on, 0);
			mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
			pdata->self_refresh_switch(0);
		}
		mod_timer(&mfd->self_refresh_timer, jiffies + msecs_to_jiffies(200));
		mutex_unlock(&self_refresh_lock);
	}
#endif
	return mdp4_overlay_unset(info, ndx);
}

static int msmfb_overlay_play_wait(struct fb_info *info, unsigned long *argp)
{
	int ret;
	struct msmfb_overlay_data req;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	if (mfd->overlay_play_enable == 0)      /* nothing to do */
		return 0;

	ret = copy_from_user(&req, argp, sizeof(req));
	if (ret) {
		PR_DISP_ERR("%s:msmfb_overlay_wait ioctl failed", __func__);
		return ret;
	}

	ret = mdp4_overlay_play_wait(info, &req);

	return ret;
}

static int msmfb_overlay_play(struct fb_info *info, unsigned long *argp)
{
	int	ret;
	struct msmfb_overlay_data req;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct file *p_src_file = 0;
	struct msm_fb_panel_data *pdata;
	static uint64_t ovp_dt;
	static int64_t ovp_count;
	static ktime_t ovp_last_sec;
	ktime_t ovp_now;

	pdata = (struct msm_fb_panel_data *)mfd->pdev->dev.platform_data;

	if (mfd->overlay_play_enable == 0)	/* nothing to do */
		return 0;

	ret = copy_from_user(&req, argp, sizeof(req));
	if (ret) {
		printk(KERN_ERR "%s:msmfb_overlay_play ioctl failed \n",
			__func__);
		return ret;
	}

	if (mfd->mdp_pdata && mfd->mdp_pdata->mdp_img_stick_wa) {
		if (atomic_read(&mfd->mdp_pdata->img_stick_on) == 0) {
			if (mfd->mdp_pdata->mdp_img_stick_wa)
				mfd->mdp_pdata->mdp_img_stick_wa(0);
			atomic_set(&mfd->mdp_pdata->img_stick_on, 1);
		}
		mod_timer(&mfd->frame_update_timer, jiffies + msecs_to_jiffies(mfd->mdp_pdata->update_interval));
	}

#if defined CONFIG_FB_MSM_SELF_REFRESH
	if ((pdata) && (pdata->self_refresh_switch)) {
		del_timer_sync(&mfd->self_refresh_timer);
		flush_workqueue(mfd->self_refresh_wq);
		mutex_lock(&self_refresh_lock);
		mod_timer(&mfd->self_refresh_timer, jiffies + msecs_to_jiffies(200));
		if (atomic_read(&self_refresh_on)) {
			atomic_set(&self_refresh_on, 0);
			mdp_pipe_ctrl(MDP_OVERLAY0_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
			pdata->self_refresh_switch(0);
		}
		mutex_unlock(&self_refresh_lock);
	}
#endif

	if (msmfb_debug_mask & FPS) {
		ovp_now = ktime_get();
		ovp_dt = ktime_to_ns(ktime_sub(ovp_now, ovp_last_sec));
		ovp_count++;
		if (ovp_dt > NSEC_PER_SEC) {
			int64_t fps = ovp_count * NSEC_PER_SEC * 100;
			ovp_count = 0;
			ovp_last_sec = ktime_get();
			do_div(fps, ovp_dt);
			DLOG(FPS, "overlay fps * 100: %llu\n", fps);
		}
	}

	ret = mdp4_overlay_play(info, &req, &p_src_file);

#if defined (CONFIG_FB_MSM_MDP_ABL)

       if (mfd->enable_abl) {
               if (waitqueue_active(&mfd->msmfb_update_notify.wait))
                       complete(&mfd->msmfb_update_notify);
       } else {
               if (waitqueue_active(&mfd->msmfb_no_update_notify.wait))
                       complete(&mfd->msmfb_no_update_notify);
       }

#if 0

       complete(&mfd->msmfb_update_notify);
       mutex_lock(&msm_fb_notify_update_sem);
       if (mfd->msmfb_no_update_notify_timer.function)
               del_timer(&mfd->msmfb_no_update_notify_timer);
       mfd->msmfb_no_update_notify_timer.expires =
               jiffies + ((1000 * HZ) / 1000);
       add_timer(&mfd->msmfb_no_update_notify_timer);
       mutex_unlock(&msm_fb_notify_update_sem);
#endif

#endif

#ifdef CONFIG_ANDROID_PMEM
	if (p_src_file)
		put_pmem_file(p_src_file);
#endif

	return ret;
}

static int msmfb_overlay_play_enable(struct fb_info *info, unsigned long *argp)
{
	int	ret, enable;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	ret = copy_from_user(&enable, argp, sizeof(enable));
	if (ret) {
		printk(KERN_ERR "%s:msmfb_overlay_play_enable ioctl failed \n",
			__func__);
		return ret;
	}

	mfd->overlay_play_enable = enable;

	return 0;
}


#ifdef CONFIG_FB_MSM_OVERLAY_WRITEBACK
static int msmfb_overlay_blt(struct fb_info *info, unsigned long *argp)
{
	int     ret;
	struct msmfb_overlay_blt req;

	ret = copy_from_user(&req, argp, sizeof(req));
	if (ret) {
		printk(KERN_ERR "%s:msmfb_overlay_blt ioctl failed\n",
			__func__);
		return ret;
	}

	ret = mdp4_overlay_blt(info, &req);

	return ret;
}

#ifndef NO_BLT_OFFSET
static int msmfb_overlay_blt_off(struct fb_info *info, unsigned long *argp)
{
	int	ret;
	struct msmfb_overlay_blt req;

	ret = mdp4_overlay_blt_offset(info, &req);

	ret = copy_to_user(argp, &req, sizeof(req));
	if (ret)
		printk(KERN_ERR "%s:msmfb_overlay_blt_off ioctl failed\n",
		__func__);

	return ret;
}
#endif /* NO_BLT_OFFSET */

#else
static int msmfb_overlay_blt(struct fb_info *info, unsigned long *argp)
{
	return 0;
}

#ifndef NO_BLT_OFFSET
static int msmfb_overlay_blt_off(struct fb_info *info, unsigned long *argp)
{
	return 0;
}
#endif /* NO_BLT_OFFSET */
#endif /* CONFIG_FB_MSM_OVERLAY_WRITEBACK */

static int msmfb_overlay_3d(struct fb_info *info, unsigned long *argp)
{
	int	ret;
	struct msmfb_overlay_3d req;

	ret = copy_from_user(&req, argp, sizeof(req));
	if (ret) {
		printk(KERN_ERR "%s:msmfb_overlay_3d_ctrl ioctl failed\n",
			__func__);
		return ret;
	}

	ret = mdp4_overlay_3d(info, &req);

	return ret;
}

#endif

DEFINE_SEMAPHORE(msm_fb_ioctl_ppp_sem);
DEFINE_MUTEX(msm_fb_ioctl_lut_sem);
DEFINE_MUTEX(msm_fb_ioctl_hist_sem);

/* Set color conversion matrix from user space */

#ifndef CONFIG_MSM_MDP40
static void msmfb_set_color_conv(struct mdp_ccs *p)
{
	int i;

	if (p->direction == MDP_CCS_RGB2YUV) {
		/* MDP cmd block enable */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

		/* RGB->YUV primary forward matrix */
		for (i = 0; i < MDP_CCS_SIZE; i++)
			writel(p->ccs[i], MDP_CSC_PFMVn(i));

		#ifdef CONFIG_MSM_MDP31
		for (i = 0; i < MDP_BV_SIZE; i++)
			writel(p->bv[i], MDP_CSC_POST_BV2n(i));
		#endif

		/* MDP cmd block disable */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	} else {
		/* MDP cmd block enable */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);

		/* YUV->RGB primary reverse matrix */
		for (i = 0; i < MDP_CCS_SIZE; i++)
			writel(p->ccs[i], MDP_CSC_PRMVn(i));
		for (i = 0; i < MDP_BV_SIZE; i++)
			writel(p->bv[i], MDP_CSC_PRE_BV1n(i));

		/* MDP cmd block disable */
		mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF, FALSE);
	}
}
#endif

#if defined (CONFIG_FB_MSM_MDP_ABL)
static int msmfb_notify_update(struct fb_info *info, unsigned long *argp)
{
	int ret, notify;
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;

	ret = copy_from_user(&notify, argp, sizeof(int));
	if (ret) {
		pr_err("%s:ioctl failed\n", __func__);
		return ret;
	}

	if (notify >= NOTIFY_NUM)
		return -EINVAL;

	if (notify == NOTIFY_UPDATE_START) {
		INIT_COMPLETION(mfd->msmfb_update_notify);
		ret = wait_for_completion_interruptible(&mfd->msmfb_update_notify);
	} else if (notify == NOTIFY_UPDATE_STOP){
		INIT_COMPLETION(mfd->msmfb_no_update_notify);
		ret = wait_for_completion_interruptible(&mfd->msmfb_no_update_notify);
	} else {
		complete(&mfd->msmfb_no_update_notify);
		complete(&mfd->msmfb_update_notify);
	}

	return ret;
}
#endif

static int msm_fb_ioctl(struct fb_info *info, unsigned int cmd,
			unsigned long arg)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	void __user *argp = (void __user *)arg;
	struct fb_cursor cursor;
#if defined (CONFIG_FB_MSM_MDP_ABL)
	struct fb_cmap cmap;
	struct mdp_histogram hist;
#endif
#ifndef CONFIG_MSM_MDP40
	struct mdp_ccs ccs_matrix;
#endif
	struct mdp_page_protection fb_page_protection;
	int ret = 0;

	switch (cmd) {
#ifdef CONFIG_FB_MSM_OVERLAY
	case MSMFB_OVERLAY_GET:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_get(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_SET:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_set(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_UNSET:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_unset(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_PLAY:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_play(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_PLAY_ENABLE:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_play_enable(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_PLAY_WAIT:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_play_wait(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_BLT:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_blt(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
	case MSMFB_OVERLAY_3D:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_overlay_3d(info, argp);
		up(&msm_fb_ioctl_ppp_sem);
		break;
#endif
	case MSMFB_BLIT:
		down(&msm_fb_ioctl_ppp_sem);
		ret = msmfb_blit(info, argp);
		up(&msm_fb_ioctl_ppp_sem);

		break;

	/* Ioctl for setting ccs matrix from user space */
	case MSMFB_SET_CCS_MATRIX:
#ifndef CONFIG_MSM_MDP40
		ret = copy_from_user(&ccs_matrix, argp, sizeof(ccs_matrix));
		if (ret) {
			printk(KERN_ERR
				"%s:MSMFB_SET_CCS_MATRIX ioctl failed \n",
				__func__);
			return ret;
		}

		down(&msm_fb_ioctl_ppp_sem);
		if (ccs_matrix.direction == MDP_CCS_RGB2YUV)
			mdp_ccs_rgb2yuv = ccs_matrix;
		else
			mdp_ccs_yuv2rgb = ccs_matrix;

		msmfb_set_color_conv(&ccs_matrix) ;
		up(&msm_fb_ioctl_ppp_sem);
#else
		ret = -EINVAL;
#endif

		break;

	/* Ioctl for getting ccs matrix to user space */
	case MSMFB_GET_CCS_MATRIX:
#ifndef CONFIG_MSM_MDP40
		ret = copy_from_user(&ccs_matrix, argp, sizeof(ccs_matrix)) ;
		if (ret) {
			printk(KERN_ERR
				"%s:MSMFB_GET_CCS_MATRIX ioctl failed \n",
				 __func__);
			return ret;
		}

		down(&msm_fb_ioctl_ppp_sem);
		if (ccs_matrix.direction == MDP_CCS_RGB2YUV)
			ccs_matrix = mdp_ccs_rgb2yuv;
		 else
			ccs_matrix =  mdp_ccs_yuv2rgb;

		ret = copy_to_user(argp, &ccs_matrix, sizeof(ccs_matrix));

		if (ret)	{
			printk(KERN_ERR
				"%s:MSMFB_GET_CCS_MATRIX ioctl failed \n",
				 __func__);
			return ret ;
		}
		up(&msm_fb_ioctl_ppp_sem);
#else
		ret = -EINVAL;
#endif

		break;

	case MSMFB_GRP_DISP:
#ifdef CONFIG_MSM_MDP22
		{
			unsigned long grp_id;

			ret = copy_from_user(&grp_id, argp, sizeof(grp_id));
			if (ret)
				return ret;

			mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_ON, FALSE);
			writel(grp_id, MDP_FULL_BYPASS_WORD43);
			mdp_pipe_ctrl(MDP_CMD_BLOCK, MDP_BLOCK_POWER_OFF,
				      FALSE);
			break;
		}
#else
		return -EFAULT;
#endif
	case MSMFB_SUSPEND_SW_REFRESHER:
		if (!mfd->panel_power_on)
			return -EPERM;

		mfd->sw_refreshing_enable = FALSE;
		ret = msm_fb_stop_sw_refresher(mfd);
		break;

	case MSMFB_RESUME_SW_REFRESHER:
		if (!mfd->panel_power_on)
			return -EPERM;

		mfd->sw_refreshing_enable = TRUE;
		ret = msm_fb_resume_sw_refresher(mfd);
		break;

	case MSMFB_CURSOR:
		ret = copy_from_user(&cursor, argp, sizeof(cursor));
		if (ret)
			return ret;

		ret = msm_fb_cursor(info, &cursor);
		break;

#if defined (CONFIG_FB_MSM_MDP_ABL)
    case MSMFB_NOTIFY_UPDATE:
		ret = msmfb_notify_update(info, argp);
		break;

	case MSMFB_SET_LUT:
		ret = copy_from_user(&cmap, argp, sizeof(cmap));
		if (ret)
			return ret;

		mutex_lock(&msm_fb_ioctl_lut_sem);
		ret = msm_fb_set_lut(&cmap, info);
		mutex_unlock(&msm_fb_ioctl_lut_sem);
		break;

	case MSMFB_HISTOGRAM:
		if (!mfd->panel_power_on)
			return -EPERM;

		if (!mfd->do_histogram)
			return -ENODEV;

		ret = copy_from_user(&hist, argp, sizeof(hist));
		if (ret)
			return ret;

		mutex_lock(&msm_fb_ioctl_hist_sem);
		ret = mfd->do_histogram(info, &hist, mfd);
		mutex_unlock(&msm_fb_ioctl_hist_sem);
		break;

	case MSMFB_HISTOGRAM_START:
		if (!mfd->do_histogram)
			return -ENODEV;
		ret = mdp_start_histogram(info);
		break;

	case MSMFB_HISTOGRAM_STOP:
		if (!mfd->do_histogram)
			return -ENODEV;
		ret = mdp_stop_histogram(info);
		break;

	case MSMFB_GET_GAMMA_CURVY:
		mutex_lock(&msm_fb_ioctl_lut_sem);
		ret = msm_fb_get_lut(info, argp);
		mutex_unlock(&msm_fb_ioctl_lut_sem);
		break;
#endif

	case MSMFB_GET_PAGE_PROTECTION:
		fb_page_protection.page_protection
			= mfd->mdp_fb_page_protection;
		ret = copy_to_user(argp, &fb_page_protection,
				sizeof(fb_page_protection));
		if (ret)
				return ret;
		break;

	case MSMFB_SET_PAGE_PROTECTION:
#if defined CONFIG_ARCH_QSD8X50 || defined CONFIG_ARCH_MSM8X60
		ret = copy_from_user(&fb_page_protection, argp,
				sizeof(fb_page_protection));
		if (ret)
				return ret;

		/* Validate the proposed page protection settings. */
		switch (fb_page_protection.page_protection)	{
		case MDP_FB_PAGE_PROTECTION_NONCACHED:
		case MDP_FB_PAGE_PROTECTION_WRITECOMBINE:
		case MDP_FB_PAGE_PROTECTION_WRITETHROUGHCACHE:
		/* Write-back cache (read allocate)  */
		case MDP_FB_PAGE_PROTECTION_WRITEBACKCACHE:
		/* Write-back cache (write allocate) */
		case MDP_FB_PAGE_PROTECTION_WRITEBACKWACACHE:
			mfd->mdp_fb_page_protection =
				fb_page_protection.page_protection;
			break;
		default:
			ret = -EINVAL;
			break;
		}
#else
		/*
		 * Don't allow caching until 7k DMA cache operations are
		 * available.
		 */
		ret = -EINVAL;
#endif
		break;

	default:
		MSM_FB_INFO("MDP: unknown ioctl (cmd=%x) received!\n", cmd);
		ret = -EINVAL;
		break;
	}

	return ret;
}

static int msm_fb_register_driver(void)
{
	return platform_driver_register(&msm_fb_driver);
}

struct platform_device *msm_fb_add_device(struct platform_device *pdev)
{
	struct msm_fb_panel_data *pdata;
	struct platform_device *this_dev = NULL;
	struct fb_info *fbi;
	struct msm_fb_data_type *mfd = NULL;
	u32 type, id, fb_num;

	if (!pdev)
		return NULL;
	id = pdev->id;

	pdata = pdev->dev.platform_data;
	if (!pdata)
		return NULL;
	type = pdata->panel_info.type;

#if defined MSM_FB_NUM
	/*
	 * over written fb_num which defined
	 * at panel_info
	 *
	 */
	if (type == HDMI_PANEL || type == DTV_PANEL || type == TV_PANEL)
		pdata->panel_info.fb_num = 1;
	else
		pdata->panel_info.fb_num = MSM_FB_NUM;

	MSM_FB_INFO("setting pdata->panel_info.fb_num to %d. type: %d\n",
			pdata->panel_info.fb_num, type);
#endif
	fb_num = pdata->panel_info.fb_num;

	if (fb_num <= 0)
		return NULL;

	if (fbi_list_index >= MAX_FBI_LIST) {
		printk(KERN_ERR "msm_fb: no more framebuffer info list!\n");
		return NULL;
	}
	/*
	 * alloc panel device data
	 */
	this_dev = msm_fb_device_alloc(pdata, type, id);

	if (!this_dev) {
		printk(KERN_ERR
		"%s: msm_fb_device_alloc failed!\n", __func__);
		return NULL;
	}

	/*
	 * alloc framebuffer info + par data
	 */
	fbi = framebuffer_alloc(sizeof(struct msm_fb_data_type), NULL);
	if (fbi == NULL) {
		platform_device_put(this_dev);
		printk(KERN_ERR "msm_fb: can't alloca framebuffer info data!\n");
		return NULL;
	}

	mfd = (struct msm_fb_data_type *)fbi->par;
	mfd->key = MFD_KEY;
	mfd->fbi = fbi;
	mfd->panel.type = type;
	mfd->panel.id = id;
	mfd->fb_page = fb_num;
	mfd->index = fbi_list_index;
	mfd->mdp_fb_page_protection = MDP_FB_PAGE_PROTECTION_WRITECOMBINE;

	/* link to the latest pdev */
	mfd->pdev = this_dev;

	mfd_list[mfd_list_index++] = mfd;
	fbi_list[fbi_list_index++] = fbi;

	/*
	 * set driver data
	 */
	platform_set_drvdata(this_dev, mfd);

	if (platform_device_add(this_dev)) {
		printk(KERN_ERR "msm_fb: platform_device_add failed!\n");
		platform_device_put(this_dev);
		framebuffer_release(fbi);
		fbi_list_index--;
		return NULL;
	}
	return this_dev;
}
EXPORT_SYMBOL(msm_fb_add_device);

int get_fb_phys_info(unsigned long *start, unsigned long *len, int fb_num)
{
	struct fb_info *info;

	if (fb_num >= MAX_FBI_LIST)
		return -1;

	info = fbi_list[fb_num];
	if (!info)
		return -1;

	*start = info->fix.smem_start;
	*len = info->fix.smem_len;
	return 0;
}
EXPORT_SYMBOL(get_fb_phys_info);

int __init msm_fb_init(void)
{
	int rc = -ENODEV;

	if (msm_fb_register_driver())
		return rc;

#ifdef MSM_FB_ENABLE_DBGFS
	{
		struct dentry *root;
		root = msm_fb_get_debugfs_root();
		if (root != NULL) {
			msm_fb_debugfs_file_create(root,
						   "msm_fb_msg_printing_level",
						   (u32 *) &msm_fb_msg_level);
			msm_fb_debugfs_file_create(root,
						   "mddi_msg_printing_level",
						   (u32 *) &mddi_msg_level);
			msm_fb_debugfs_file_create(root, "msm_fb_debug_enabled",
						   (u32 *) &msm_fb_debug_enabled);

			msm_fb_debugfs_file_create(root, "debug_mask",
						   (u32 *) &msmfb_debug_mask);
		}
	}
#endif

	return 0;
}

module_init(msm_fb_init);
