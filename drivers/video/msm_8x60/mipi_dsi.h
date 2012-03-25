/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
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

#ifndef MIPI_DSI_H
#define MIPI_DSI_H

#include <mach/scm-io.h>

#ifdef BIT
#undef BIT
#endif

#define BIT(x)  (1<<(x))

#define MMSS_CC_BASE_PHY 0x04000000	/* mmss clcok control */
#define MMSS_SFPB_BASE_PHY 0x05700000	/* mmss SFPB CFG */

#define MIPI_DSI_BASE mipi_dsi_base
#define DSI_VIDEO_BASE	0xE0000

#ifdef CONFIG_MSM_SECURE_IO
#define MIPI_OUTP(addr, data) secure_writel((data), (addr))
#define MIPI_INP(addr) secure_readl(addr)
#else
#define MIPI_OUTP(addr, data) writel((data), (addr))
#define MIPI_INP(addr) readl(addr)
#endif

#define MIPI_DSI_PRIM 1
#define MIPI_DSI_SECD 2

#define MIPI_DSI_PANEL_VGA	0
#define MIPI_DSI_PANEL_WVGA	1
#define MIPI_DSI_PANEL_WVGA_PT	2

#define DSI_PANEL_MAX	2

enum {		/* mipi dsi panel */
	DSI_VIDEO_MODE,
	DSI_CMD_MODE,
};

#define DSI_NON_BURST_SYNCH_PULSE	0
#define DSI_NON_BURST_SYNCH_EVENT	1
#define DSI_BURST_MODE			2


#define DSI_RGB_SWAP_RGB	0
#define DSI_RGB_SWAP_RBG	1
#define DSI_RGB_SWAP_BGR	2
#define DSI_RGB_SWAP_BRG	3
#define DSI_RGB_SWAP_GRB	4
#define DSI_RGB_SWAP_GBR	5

#define DSI_VIDEO_DST_FORMAT_RGB565		0
#define DSI_VIDEO_DST_FORMAT_RGB666		1
#define DSI_VIDEO_DST_FORMAT_RGB666_LOOSE	2
#define DSI_VIDEO_DST_FORMAT_RGB888		3

#define DSI_CMD_DST_FORMAT_RGB111	0
#define DSI_CMD_DST_FORMAT_RGB332	3
#define DSI_CMD_DST_FORMAT_RGB444	4
#define DSI_CMD_DST_FORMAT_RGB565	6
#define DSI_CMD_DST_FORMAT_RGB666	7
#define DSI_CMD_DST_FORMAT_RGB888	8

#define DSI_INTR_ERROR_MASK		BIT(25)
#define DSI_INTR_ERROR			BIT(24)
#define DSI_INTR_VIDEO_DONE_MASK	BIT(17)
#define DSI_INTR_VIDEO_DONE		BIT(16)
#define DSI_INTR_CMD_MDP_DONE_MASK	BIT(9)
#define DSI_INTR_CMD_MDP_DONE		BIT(8)
#define DSI_INTR_CMD_DMA_DONE_MASK	BIT(1)
#define DSI_INTR_CMD_DMA_DONE		BIT(0)

#define DSI_CMD_TRIGGER_NONE           0x0     /* mdp trigger */
#define DSI_CMD_TRIGGER_TE		0x02
#define DSI_CMD_TRIGGER_SW		0x04
#define DSI_CMD_TRIGGER_SW_SEOF		0x05	/* cmd dma only */
#define DSI_CMD_TRIGGER_SW_TE		0x06

extern struct device dsi_dev;

struct dsi_clk_desc {
	uint32 src;
	uint32 m;
	uint32 n;
	uint32 d;
#ifdef CONFIG_MSM_DSI_CLK_AUTO_CALCULATE
	uint32 mnd_mode;
	uint32 pre_div_func;
#endif
};

#define DSI_HOST_HDR_SIZE	4
#define DSI_HDR_LAST		BIT(31)
#define DSI_HDR_LONG_PKT	BIT(30)
#define DSI_HDR_BTA		BIT(29)
#define DSI_HDR_VC(vc)		(((vc) & 0x03) << 22)
#define DSI_HDR_DTYPE(dtype)	(((dtype) & 0x03f) << 16)
#define DSI_HDR_DATA2(data)	(((data) & 0x0ff) << 8)
#define DSI_HDR_DATA1(data)	((data) & 0x0ff)
#define DSI_HDR_WC(wc)		((wc) & 0x0ffff)

#define DSI_BUF_SIZE	1024
#define MIPI_DSI_MRPS	0x04  /* Maximum Return Packet Size */

#define MIPI_DSI_REG_LEN 16 /* 4 x 4 bytes register */

struct dsi_buf {
	uint32 *hdr;	/* dsi host header */
	char *start;	/* buffer start addr */
	char *end;	/* buffer end addr */
	int size;	/* size of buffer */
	char *data;	/* buffer */
	int len;	/* data length */
	dma_addr_t dmap; /* mapped dma addr */
};

/* dcs read/write */
#define DTYPE_VSYNC_START	0x01	/* short write, 0 parameter */
#define DTYPE_DCS_WRITE		0x05	/* short write, 0 parameter */
#define DTYPE_DCS_WRITE1	0x15	/* short write, 1 parameter */
#define DTYPE_HSYNC_START	0x21	/* short write, 0 parameter */
#define DTYPE_DCS_READ		0x06	/* read */
#define DTYPE_DCS_LWRITE	0x39	/* long write */

/* generic read/write */
#define DTYPE_GEN_WRITE		0x03	/* short write, 0 parameter */
#define DTYPE_GEN_WRITE1	0x13	/* short write, 1 parameter */
#define DTYPE_GEN_WRITE2	0x23	/* short write, 2 parameter */
#define DTYPE_GEN_LWRITE	0x29	/* long write */
#define DTYPE_GEN_READ		0x04	/* long read, 0 parameter */
#define DTYPE_GEN_READ1		0x14	/* long read, 1 parameter */
#define DTYPE_GEN_READ2		0x24	/* long read, 2 parameter */

#define DTYPE_TEAR_ON          0x35    /* set tear on */
#define DTYPE_MAX_PKTSIZE	0x37	/* set max packet size */
#define DTYPE_NULL_PKT		0x09	/* null packet, no data */
#define DTYPE_BLANK_PKT		0x19	/* blankiing packet, no data */

#define DTYPE_CM_ON		0x02	/* color mode off */
#define DTYPE_CM_OFF		0x12	/* color mode on */
#define DTYPE_PERIPHERAL_OFF	0x22
#define DTYPE_PERIPHERAL_ON	0x32


struct dsi_cmd_desc {
	int dtype;
	int last;
	int vc;
	int ack;	/* ask ACK from peripheral */
	int wait;
	int dlen;
	char *payload;
};


/* MIPI_DSI_MRPS, Maximum Return Packet Size */
extern char max_pktsize[2]; /* defined at mipi_dsi.c */

char *mipi_dsi_buf_reserve_hdr(struct dsi_buf *dp, int hlen);
char *mipi_dsi_buf_init(struct dsi_buf *dp);
void mipi_dsi_init(void);
int mipi_dsi_buf_alloc(struct dsi_buf *, int size);
int mipi_dsi_cmd_dma_add(struct dsi_buf *dp, struct dsi_cmd_desc *cm);
int mipi_dsi_cmds_tx(struct dsi_buf *dp, struct dsi_cmd_desc *cmds, int cnt);
int mipi_dsi_cmd_dma_tx(struct dsi_buf *dp);
int mipi_dsi_cmd_reg_tx(uint32 data);
int mipi_dsi_cmds_rx(struct dsi_buf *tp, struct dsi_buf *rp,
				struct dsi_cmd_desc *cmds, int len);
int mipi_dsi_cmd_dma_rx(struct dsi_buf *tp, int rlen);
void mipi_dsi_host_init(struct mipi_panel_info *pinfo);
void mipi_dsi_op_mode_config(int mode);
void mipi_dsi_cmd_mode_ctrl(int enable);
void mdp4_dsi_cmd_trigger(void);
void mipi_dsi_cmd_mdp_sw_trigger(void);
void mipi_dsi_cmd_bta_sw_trigger(void);
void mipi_dsi_ack_err_status(void);
void mipi_dsi_set_tear_on(void);
void mipi_dsi_set_tear_off(void);
irqreturn_t mipi_dsi_isr(int irq, void *ptr);
void dsi_mutex_lock(void);
void dsi_busy_check(void);
void dsi_mutex_unlock(void);
void mipi_dsi_enable_irq(void);
void mipi_dsi_disable_irq(void);
void mipi_dsi_read_status_reg(void);
void mipi_set_tx_power_mode(int mode);
void mipi_dsi_controller_cfg(int enable, int cmd, int video);
int mipi_dsi_controller_on(void);
void mipi_dsi_sw_reset(void);
int mipi_dsi_reset_read(void);
void mipi_dsi_reset_set(int);

extern struct mutex cmdlock;

extern int panel_type;

enum {
	PANEL_SHARP_QHD,
	PANEL_SHARP_WVGA,
	PANEL_UNKNOWN,
};

#endif /* MIPI_DSI_H */
