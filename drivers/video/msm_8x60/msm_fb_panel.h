/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MSM_FB_PANEL_H
#define MSM_FB_PANEL_H

#include "msm_fb_def.h"

struct msm_fb_data_type;

typedef void (*msm_fb_vsync_handler_type) (void *arg);

/* panel id type */
typedef struct panel_id_s {
	uint16 id;
	uint16 type;
} panel_id_type;

/* panel type list */
#define NO_PANEL		0xffff	/* No Panel */
#define MDDI_PANEL		1	/* MDDI */
#define EBI2_PANEL		2	/* EBI2 */
#define LCDC_PANEL		3	/* internal LCDC type */
#define EXT_MDDI_PANEL		4	/* Ext.MDDI */
#define TV_PANEL		5	/* TV */
#define HDMI_PANEL		6	/* HDMI TV */
#define DTV_PANEL		7	/* DTV */
#define MIPI_VIDEO_PANEL	8	/* MIPI */
#define MIPI_CMD_PANEL		9	/* MIPI */

/* panel class */
typedef enum {
	DISPLAY_LCD = 0,	/* lcd = ebi2/mddi */
	DISPLAY_LCDC,		/* lcdc */
	DISPLAY_TV,		/* TV Out */
	DISPLAY_EXT_MDDI,	/* External MDDI */
} DISP_TARGET;

/* panel device locaiton */
typedef enum {
	DISPLAY_1 = 0,		/* attached as first device */
	DISPLAY_2,		/* attached on second device */
	MAX_PHYS_TARGET_NUM,
} DISP_TARGET_PHYS;

/* panel info type */
struct lcd_panel_info {
	__u32 vsync_enable;
	__u32 refx100;
	__u32 v_back_porch;
	__u32 v_front_porch;
	__u32 v_pulse_width;
	__u32 hw_vsync_mode;
	__u32 vsync_notifier_period;
	__u32 rev;
};

struct lcdc_panel_info {
	__u32 h_back_porch;
	__u32 h_front_porch;
	__u32 h_pulse_width;
	__u32 v_back_porch;
	__u32 v_front_porch;
	__u32 v_pulse_width;
	__u32 border_clr;
	__u32 underflow_clr;
	__u32 hsync_skew;
};

struct mddi_panel_info {
	__u32 vdopkt;
};

/* DSI PHY configuration */
struct mipi_dsi_phy_ctrl {
	uint32 regulator[4];
	uint32 timing[12];
	uint32 ctrl[4];
	uint32 strength[4];
	uint32 pll[21];
};

struct mipi_panel_info {
	char mode;		/* video/cmd */
	char interleave_mode;
	char crc_check;
	char ecc_check;
	char dst_format;	/* shared by video and command */
	char data_lane0;
	char data_lane1;
	char data_lane2;
	char data_lane3;
	char dlane_swap;	/* data lane swap */
	char rgb_swap;
	char b_sel;
	char g_sel;
	char r_sel;
	char rx_eot_ignore;
	char tx_eot_append;
	char t_clk_post; /* 0xc0, DSI_CLKOUT_TIMING_CTRL */
	char t_clk_pre;  /* 0xc0, DSI_CLKOUT_TIMING_CTRL */
	char vc;	/* virtual channel */
	struct mipi_dsi_phy_ctrl *dsi_phy_db;
	/* video mode */
	char pulse_mode_hsa_he;
	char hfp_power_stop;
	char hbp_power_stop;
	char hsa_power_stop;
	char eof_bllp_power_stop;
	char bllp_power_stop;
	char traffic_mode;
#ifdef CONFIG_MSM_DSI_CLK_AUTO_CALCULATE
	char frame_rate;
	uint32 dsi_pclk_rate;
#endif
	/* command mode */
	char interleave_max;
	char insert_dcs_cmd;
	char wr_mem_continue;
	char wr_mem_start;
	char te_sel;
	char stream;	/* 0 or 1 */
	char mdp_trigger;
	char dma_trigger;
};

struct msm_panel_info {
	__u32 xres;
	__u32 yres;
	__u32 bpp;
	__u32 mode2_xres;
	__u32 mode2_yres;
	__u32 mode2_bpp;
	__u32 type;
	__u32 wait_cycle;
	DISP_TARGET_PHYS pdest;
	__u32 bl_max;
	__u32 bl_min;
	__u32 fb_num;
	__u32 clk_rate;
	__u32 clk_min;
	__u32 clk_max;
	__u32 frame_count;
	bool  is_3d_panel;


	union {
		struct mddi_panel_info mddi;
	};

	union {
		struct lcd_panel_info lcd;
		struct lcdc_panel_info lcdc;
	};

	struct mipi_panel_info mipi;
};

#define MSM_FB_SINGLE_MODE_PANEL(pinfo)		\
	do {					\
		(pinfo)->mode2_xres = 0;	\
		(pinfo)->mode2_yres = 0;	\
		(pinfo)->mode2_bpp = 0;		\
	} while (0)

struct msm_fb_panel_data {
	struct msm_panel_info panel_info;
	void (*set_rect) (int x, int y, int xres, int yres);
	void (*set_vsync_notifier) (msm_fb_vsync_handler_type, void *arg);
	void (*set_backlight) (struct msm_fb_data_type *);
	void (*display_on) (struct msm_fb_data_type *);

	/* function entry chain */
	int (*on) (struct platform_device *pdev);
	int (*off) (struct platform_device *pdev);
	void (*bklswitch) (struct msm_fb_data_type *, bool on);
	void (*bklctrl) (bool on);
	void (*bklenable) (struct msm_fb_data_type *);
	void (*panel_type_detect) (void);
	struct platform_device *next;
	int (*clk_func) (int enable);
#if defined CONFIG_FB_MSM_SELF_REFRESH
	void (*self_refresh_switch) (int enable);
#endif
};

/*===========================================================================
  FUNCTIONS PROTOTYPES
============================================================================*/
struct platform_device *msm_fb_device_alloc(struct msm_fb_panel_data *pdata,
						u32 type, u32 id);
int panel_next_on(struct platform_device *pdev);
int panel_next_off(struct platform_device *pdev);

int lcdc_device_register(struct msm_panel_info *pinfo);

int mddi_toshiba_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel);

#endif /* MSM_FB_PANEL_H */
