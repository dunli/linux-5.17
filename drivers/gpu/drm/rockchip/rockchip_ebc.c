// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Rockchip Electronics Co. Ltd.
 * Copyright (c) 2021 Samuel Holland <samuel@sholland.org>
 *
 * Author: Zorro Liu <zorro.liu@rock-chips.com>
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/dma-mapping.h>
#include <linux/iio/consumer.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "pvi_waveform.h"

#define UPDATE(x, h, l)			(((x) << (l)) & GENMASK((h), (l)))

#define EBC_DSP_START			0x0000 // Frame statrt register
#define		DSP_OUT_LOW			BIT(31)
#define		DSP_SDCE_WIDTH(x)		UPDATE(x, 25, 16)
#define		DSP_EINK_MODE			BIT(13)
#define		DSP_SW_BURST_CTRL		BIT(12)
#define		DSP_FRM_TOTAL(x)		UPDATE(x, 9, 2)
#define		DSP_FRM_TOTAL_MASK		GENMASK(9, 2)
#define		DSP_FRM_START			BIT(0)
#define EBC_EPD_CTRL			0x0004 // EPD control register
#define		EINK_MODE_SWAP			BIT(31)
#define		EINK_MODE_FRM_SEL		BIT(30)
		/* GCLK falling edge point (SCLK), which counts from the rising edge of hsync */
#define		DSP_GD_END(x)			UPDATE(x, 26, 16)
		/* GCLK rising edge point (SCLK), which counts from the rising edge of hsync
		 * (spec is wrong, count from rising edge of hsync, not falling edge of hsync) */
#define		DSP_GD_ST(x)			UPDATE(x, 15, 8)
		/* 0: LUT mode or direct mode; 1: three win mode */
#define		DSP_THREE_WIN_MODE		BIT(7)
		/* 0: 8 bit data output; 1: 16 bit data output */
#define		DSP_SDDW_MODE			BIT(6)
		/* 0: EINK; 1:AUO */
#define		EPD_AUO				BIT(5)
#define		EPD_PWR(x)			UPDATE(x, 4, 2)
		/* Gate scanning direction: 0: top to bottom; 1: bottom to top */
#define		EPD_GDRL			BIT(1)
		/* Source scanning direction 0: left to right; 1: right to left */
#define		EPD_SDSHR			BIT(0)
#define EBC_DSP_CTRL			0x0008 // Display control register
#define		DSP_SWAP_MODE(x)		UPDATE(x, 31, 30)
		/* From the U-Boot driver: 0: EINK_UPDATE_NORMAL; 1: EINK_UPDATE_DIFF */
#define		DSP_UPDATE_MODE			BIT(29)
		/* From the U-Boot driver: 0: DIRECT_MODE; 1: LUT_MODE */
#define		DSP_DISPLAY_MODE		BIT(28)
#define		DSP_VCOM_MODE			BIT(27)
#define		DSP_SDCLK_DIV(x)		UPDATE(x, 19, 16)
#define		DSP_SDCLK_DIV_MASK		GENMASK(19, 16)
#define EBC_DSP_HTIMING0		0x000c // H-Timing setting register 0
#define		DSP_HTOTAL(x)			UPDATE(x, 27, 16)
#define		DSP_HS_END(x)			UPDATE(x, 7, 0)
#define EBC_DSP_HTIMING1		0x0010 // H-Timing setting register 1
#define		DSP_HACT_END(x)			UPDATE(x, 26, 16)
#define		DSP_HACT_ST(x)			UPDATE(x, 7, 0)
#define EBC_DSP_VTIMING0		0x0014 // V-Timing setting register 0
#define		DSP_VTOTAL(x)			UPDATE(x, 26, 16)
#define		DSP_VS_END(x)			UPDATE(x, 7, 0)
#define EBC_DSP_VTIMING1		0x0018 // V-Timing setting register 1
#define		DSP_VACT_END(x)			UPDATE(x, 26, 16)
#define		DSP_VACT_ST(x)			UPDATE(x, 7, 0)
#define EBC_DSP_ACT_INFO		0x001c // ACTIVE width/height
#define		DSP_HEIGHT(x)			UPDATE(x, 26, 16)
#define		DSP_WIDTH(x)			UPDATE(x, 11, 0)
#define EBC_WIN_CTRL			0x0020 // Window ctrl
#define		WIN2_FIFO_ALMOST_FULL_LEVEL(x)	UPDATE(x, 27, 19)
#define		WIN_EN(x)			UPDATE(x, 18, 18)
#define		BURST_REG(x)			UPDATE(x, 12, 10)
#define		WIN_FIFO_ALMOST_FULL_LEVEL(x)	UPDATE(x, 9, 2)
#define		WIN_FMT(x)			UPDATE(x, 1, 0)
#define EBC_WIN_MST0			0x0024 // Current win memory start
#define EBC_WIN_MST1			0x0028 // Next win memory start
#define EBC_WIN_VIR			0x002c // Window vir width/height
#define		WIN_VIR_HEIGHT(x)		UPDATE(x, 31, 16)
#define		WIN_VIR_WIDTH(x)		UPDATE(x, 15, 0)
#define EBC_WIN_ACT			0x0030 // Window act width/height
#define		WIN_ACT_HEIGHT(x)		UPDATE(x, 26, 16)
#define		WIN_ACT_WIDTH(x)		UPDATE(x, 11, 0)
#define EBC_WIN_DSP			0x0034 // Window dsp width/height
#define		WIN_DSP_HEIGHT(x)		UPDATE(x, 26, 16)
#define		WIN_DSP_WIDTH(x)		UPDATE(x, 11, 0)
#define EBC_WIN_DSP_ST			0x0038 // Window display start piont
#define		WIN_DSP_YST(x)			UPDATE(x, 26, 16)
#define		WIN_DSP_XST(x)			UPDATE(x, 11, 0)
#define EBC_INT_STATUS			0x003c // Interrupt register
#define		DSP_FRM_INT_NUM(x)		UPDATE(x, 19, 12)
#define		LINE_FLAG_INT_CLR		BIT(11)
#define		DSP_FRM_INT_CLR			BIT(10)
#define		DSP_END_INT_CLR			BIT(9)
#define		FRM_END_INT_CLR			BIT(8)
#define		LINE_FLAG_INT_MASK		BIT(7)
#define		DSP_FRM_INT_MASK		BIT(6)
#define		DSP_END_INT_MASK		BIT(5)
#define		FRM_END_INT_MASK		BIT(4)
#define		LINE_FLAG_INT			BIT(3)
#define		DSP_FRM_INT			BIT(2)
#define		DSP_END_INT			BIT(1)
#define		FRM_END_INT			BIT(0)
#define EBC_VCOM0			0x0040 // VCOM setting register 0
#define EBC_VCOM1			0x0044 // VCOM setting register 1
#define EBC_VCOM2			0x0048 // VCOM setting register 2
#define EBC_VCOM3			0x004c // VCOM setting register 3
#define EBC_CONFIG_DONE			0x0050 // Config done register
#define		REG_LOAD_GLOBAL_EN		BIT(0)
#define EBC_VNUM			0x0054 // Line flag num
#define EBC_WIN_MST2			0x0058 // Frame count memory start
#define EBC_LUT_DATA_ADDR		0x1000 // LUT data address

enum ebc_win_data_fmt {
	Y_DATA_4BPP	= 0,
	Y_DATA_8BPP	= 1,
	RGB888		= 2,
	RGB565		= 3,
};

#define EBC_DMA_SIZE(ebc)		((ebc)->pixels / 4) /* 2bpp */
#define EBC_COUNT_SIZE(ebc)		((ebc)->pixels / 2) /* 8b/2px */
#define EBC_FRAME_SIZE(ebc)		((ebc)->pixels / 2) /* 4bpp */
#define EBC_NEXT_SIZE(ebc)		((ebc)->pixels / 2) /* 4bpp */
#define EBC_PREV_SIZE(ebc)		((ebc)->pixels / 2) /* 4bpp */

#define EBC_NUM_SUPPLIES		3

enum rockchip_ebc_panel_state {
	EBC_PANEL_BOOT,
	EBC_PANEL_INIT,
	EBC_PANEL_ON,
	EBC_PANEL_OFF,
};

struct rockchip_ebc_panel {
	u32	width;
	u32	height;
	u32	vir_width;
	u32	vir_height;
	u32	sdck;
	u32	lsl;
	u32	lbl;
	u32	ldl;
	u32	lel;
	u32	gdck_sta;
	u32	lgonl;
	u32	fsl;
	u32	fbl;
	u32	fdl;
	u32	fel;
	u32	mirror;
	u32	panel_16bit;
	u32	panel_color;
	u32	width_mm;
	u32	height_mm;
};

struct rockchip_ebc {
	struct clk			*dclk;
	struct clk			*hclk;
	struct completion		frame_complete;
	struct drm_connector		connector;
	struct drm_device		drm;
	struct drm_simple_display_pipe	pipe;
	struct iio_channel		*temp_chan;
	struct pvi_wf_ctx		wf;
	struct regmap			*regmap;
	struct regulator_bulk_data	supplies[EBC_NUM_SUPPLIES];
	struct rockchip_ebc_panel	panel;
	struct work_struct		refresh_work;
	void				*count_buffer;
	void				*frame_buffer;
	void				*prev_buffer;
	void				*next_buffer;
	void				*dma_buffer;
	dma_addr_t			dma_handle;
	enum rockchip_ebc_panel_state	panel_state;
	u32				pixels;
	u32				temperature;
};

static bool force_refresh = false;
module_param(force_refresh, bool, 0644);
MODULE_PARM_DESC(force_refresh, "trigger a forced refresh");

static int lut_type = WF_TYPE_GC16;
module_param(lut_type, int, 0644);
MODULE_PARM_DESC(lut_type, "default LUT type");

static inline struct rockchip_ebc *
connector_to_ebc(struct drm_connector *connector)
{
	return container_of(connector, struct rockchip_ebc, connector);
}

static inline struct rockchip_ebc *
pipe_to_ebc(struct drm_simple_display_pipe *pipe)
{
	return container_of(pipe, struct rockchip_ebc, pipe);
}

static inline unsigned int ebc_read(struct rockchip_ebc *ebc, unsigned int reg)
{
	unsigned int value;

	regmap_read(ebc->regmap, reg, &value);

	return value;
}

static inline void ebc_update_bits(struct rockchip_ebc *ebc, unsigned int reg,
				    unsigned int mask, unsigned int val)
{
	regmap_update_bits(ebc->regmap, reg, mask, val);
}

static inline void ebc_write(struct rockchip_ebc *ebc, unsigned int reg,
			      unsigned int value)
{
	regmap_write(ebc->regmap, reg, value);
}

static inline void ebc_cfg_done(struct rockchip_ebc *ebc)
{
	regmap_write(ebc->regmap, EBC_CONFIG_DONE, REG_LOAD_GLOBAL_EN);
}

static void rockchip_ebc_configure(struct rockchip_ebc *ebc)
{
	struct rockchip_ebc_panel *panel = &ebc->panel;

	/* Write panel timing and window info. */
	ebc_write(ebc, EBC_DSP_HTIMING0,
		  DSP_HTOTAL(panel->lsl + panel->lbl + panel->ldl + panel->lel) |
		  DSP_HS_END(panel->lsl + 2));
	ebc_write(ebc, EBC_DSP_HTIMING1,
		  DSP_HACT_END(panel->lsl + panel->lbl + panel->ldl) |
		  DSP_HACT_ST(panel->lsl + panel->lbl - 1));
	ebc_write(ebc, EBC_DSP_VTIMING0,
		  DSP_VTOTAL(panel->fsl + panel->fbl + panel->fdl + panel->fel) |
		  DSP_VS_END(panel->fsl));
	ebc_write(ebc, EBC_DSP_VTIMING1,
		  DSP_VACT_END(panel->fsl + panel->fbl + panel->fdl) |
		  DSP_VACT_ST(panel->fsl + panel->fbl));
	ebc_write(ebc, EBC_DSP_ACT_INFO,
		  DSP_HEIGHT(panel->height) |
		  DSP_WIDTH(panel->width));
	ebc_write(ebc, EBC_WIN_VIR,
		  WIN_VIR_HEIGHT(panel->vir_height) |
		  WIN_VIR_WIDTH(panel->vir_width));
	ebc_write(ebc, EBC_WIN_ACT,
		  WIN_ACT_HEIGHT(panel->height) |
		  WIN_ACT_WIDTH(panel->width));
	ebc_write(ebc, EBC_WIN_DSP,
		  WIN_DSP_HEIGHT(panel->height) |
		  WIN_DSP_WIDTH(panel->width));
	ebc_write(ebc, EBC_WIN_DSP_ST,
		  WIN_DSP_YST(panel->fsl + panel->fbl) |
		  WIN_DSP_XST(panel->lsl + panel->lbl));

	/*
	 * win2 fifo is 512x128, win fifo is 256x128, we set fifo almost value (fifo_size - 16)
	 * burst_reg = 7 mean ahb burst is incr16
	 */
	ebc_write(ebc, EBC_WIN_CTRL,
		  WIN2_FIFO_ALMOST_FULL_LEVEL(496) |
		  WIN_EN(1) |
		  BURST_REG(7) |
		  WIN_FIFO_ALMOST_FULL_LEVEL(240) |
		  WIN_FMT(Y_DATA_4BPP));

	ebc_write(ebc, EBC_EPD_CTRL,
		  EINK_MODE_SWAP |
		  DSP_GD_END(panel->lsl + panel->gdck_sta + panel->lgonl) |
		  DSP_GD_ST(panel->lsl + panel->gdck_sta) |
		  (panel->panel_16bit ? DSP_SDDW_MODE : 0) |
		  EPD_GDRL |
		  EPD_SDSHR);
	ebc_write(ebc, EBC_DSP_START,
		  DSP_SDCE_WIDTH(panel->ldl) |
		  DSP_SW_BURST_CTRL);
	ebc_write(ebc, EBC_DSP_CTRL,
		  DSP_SWAP_MODE(panel->panel_16bit ? 2 : 3) |
		  DSP_VCOM_MODE |
		  DSP_SDCLK_DIV(panel->panel_16bit ? 7 : 3));

	ebc_write(ebc, EBC_WIN_MST0, 0);
	ebc_write(ebc, EBC_WIN_MST1, 0);
	ebc_write(ebc, EBC_WIN_MST2, 0);
}

static void rockchip_ebc_frame_start(struct rockchip_ebc *ebc, int frames)
{
	/* Always set the frame start bit to 0 before the real frame start. */
	ebc_update_bits(ebc, EBC_DSP_START,
			DSP_FRM_TOTAL_MASK | DSP_FRM_START,
			DSP_FRM_TOTAL(frames - 1));
	ebc_cfg_done(ebc);

	ebc_update_bits(ebc, EBC_DSP_START, DSP_FRM_START, DSP_FRM_START);
}

static bool rockchip_ebc_generate_frame(struct rockchip_ebc *ebc,
					u32 *out_buffer,
					u8 global_count)
{
	const u8 *wf_table = ebc->wf.wf_table;
	u64 *count_buffer = ebc->count_buffer;
	u64 *frame_buffer = ebc->frame_buffer;
	u64 *prev_buffer = ebc->prev_buffer;
	u64 *next_buffer = ebc->next_buffer;
	unsigned int byte, stride, x, y;
	bool ret = false;

	/* Default to not driving any pixels either way. */
	memset(out_buffer, 0, EBC_DMA_SIZE(ebc));

	/* Operate on 8-byte groups of 4-bit pixels. */
	stride = ebc->panel.width / sizeof(u64) / 2;

	for (y = 0; y < ebc->panel.height; y++) {
		for (x = 0; x < stride; x++) {
			unsigned int pos = y * stride + x;
			// XXX currently only using one count per group.
			u64 count = global_count ?: count_buffer[pos] & 0xff;
			u64 prev = prev_buffer[pos];
			u64 next = next_buffer[pos];
			u32 output = 0;
			u8 this_frame;

			/* If no waveform in progress, check for framebuffer updates. */
			if (!count) {
				next = frame_buffer[pos];
				if (next == prev)
					continue;
				next_buffer[pos] = next;
				count = ebc->wf.frame_num;
			}

			this_frame = ebc->wf.frame_num - count;
			for (byte = 0; byte < sizeof(u64); byte++) {
				u8 prev_byte = prev >> (8 * byte);
				u8 next_byte = next >> (8 * byte);

				output |= wf_table[this_frame << 16 |
						   prev_byte  << 8  |
						   next_byte] << (4 * byte);
			}
			out_buffer[pos] = output;

			/* Update the previous buffer when the waveform completes. */
			if (!--count)
				prev_buffer[pos] = next;
			/* Save the decremented frame counter. */
			if (!global_count)
				count_buffer[pos] = count;

			ret = true;
		}
	}

	return ret;
}

static void rockchip_ebc_refresh(struct rockchip_ebc *ebc,
				 enum epd_lut_type refresh_mode,
				 bool global_refresh)
{
	struct device *dev = ebc->drm.dev;
	u32 frames = 0, offset = 0;
	int ret, temperature;
	u8 global_count;

	/* Temperature updates should be done asynchronously... */
	ret = iio_read_channel_processed(ebc->temp_chan, &temperature);
	if (ret < 0)
		DRM_DEV_ERROR(dev, "Failed to get temperature: %d\n", ret);
	else
		ebc->temperature = temperature / 1000;

	ret = pvi_wf_get_lut(&ebc->wf, refresh_mode, ebc->temperature);
	if (ret)
		DRM_DEV_ERROR(dev, "Failed to get LUT: %d\n", ret);

	global_count = global_refresh ? ebc->wf.frame_num : 0;
	DRM_DEV_DEBUG_DRIVER(dev, "Starting refresh: global=%d mode=%d temp=%d -> frames=%d\n",
			     global_refresh, refresh_mode, ebc->temperature, ebc->wf.frame_num);

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0) {
		DRM_DEV_ERROR(dev, "Failed to resume: %d\n", ret);
		return;
	}

	/* Loop until there are no pending pixel updates. */
	while (rockchip_ebc_generate_frame(ebc, ebc->dma_buffer + offset, global_count)) {
		dma_sync_single_for_device(dev, ebc->dma_handle + offset,
					   EBC_DMA_SIZE(ebc), DMA_TO_DEVICE);

		if (frames)
			wait_for_completion_timeout(&ebc->frame_complete,
						    msecs_to_jiffies(50));

		ebc_write(ebc, EBC_WIN_MST0, ebc->dma_handle + offset);
		rockchip_ebc_frame_start(ebc, 1);

		frames++;
		offset = EBC_DMA_SIZE(ebc) - offset;

		if (global_refresh && !--global_count)
			break;
	}
	if (frames)
		wait_for_completion_timeout(&ebc->frame_complete,
					    msecs_to_jiffies(50));

	pm_runtime_mark_last_busy(dev);
	pm_runtime_put_autosuspend(dev);

	DRM_DEV_DEBUG_DRIVER(dev, "Finished refresh: frames=%d\n", frames);
}

static void rockchip_ebc_refresh_work(struct work_struct *work)
{
	struct rockchip_ebc *ebc = container_of(work, struct rockchip_ebc, refresh_work);

	switch (ebc->panel_state) {
	case EBC_PANEL_BOOT:
	case EBC_PANEL_OFF:
		/* Wait for the CRTC to be enabled before doing anything. */
		return;
	case EBC_PANEL_INIT:
		/* The INIT waveform ends with all white pixels. */
		memset(ebc->prev_buffer, 0xff, EBC_PREV_SIZE(ebc));
		memset(ebc->next_buffer, 0xff, EBC_NEXT_SIZE(ebc));
		rockchip_ebc_refresh(ebc, WF_TYPE_RESET, true);
		ebc->panel_state = EBC_PANEL_ON;
		break;
	case EBC_PANEL_ON:
		if (force_refresh) {
			force_refresh = false;
			rockchip_ebc_refresh(ebc, WF_TYPE_GC16, true);
		}
		break;
	}

	rockchip_ebc_refresh(ebc, lut_type, false);
}

static irqreturn_t rockchip_ebc_irq(int irq, void *dev_id)
{
	struct rockchip_ebc *ebc = dev_id;
	unsigned int status = ebc_read(ebc, EBC_INT_STATUS);

	if (status & DSP_END_INT) {
		ebc_write(ebc, EBC_INT_STATUS, status | DSP_END_INT_CLR);
		complete(&ebc->frame_complete);
	}

	return IRQ_HANDLED;
}

DEFINE_DRM_GEM_FOPS(rockchip_ebc_fops);

static const struct drm_driver rockchip_ebc_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	.fops			= &rockchip_ebc_fops,
	DRM_GEM_SHMEM_DRIVER_OPS,
	.name			= "rockchip_ebc",
	.desc			= "Rockchip E-Book Controller",
	.date			= "20220101",
	.major			= 0,
	.minor			= 1,
};

static const struct drm_mode_config_funcs rockchip_ebc_mode_config_funcs = {
	.fb_create		= drm_gem_fb_create_with_dirty,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

static int rockchip_ebc_connector_helper_get_modes(struct drm_connector *connector)
{
	struct rockchip_ebc *ebc = connector_to_ebc(connector);
	struct rockchip_ebc_panel *panel = &ebc->panel;
	struct drm_display_mode *mode;

	mode = drm_mode_create(&ebc->drm);
	if (!mode)
		return 0;

	*mode = (struct drm_display_mode) {
		DRM_SIMPLE_MODE(panel->width, panel->height,
				panel->width_mm, panel->height_mm)
	};
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;

	return 1;
}

static const struct drm_connector_helper_funcs rockchip_ebc_connector_helper_funcs = {
	.get_modes		= rockchip_ebc_connector_helper_get_modes,
};

static const struct drm_connector_funcs rockchip_ebc_connector_funcs = {
	.reset			= drm_atomic_helper_connector_reset,
	.fill_modes		= drm_helper_probe_single_connector_modes,
	.destroy		= drm_connector_cleanup,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
};

static void rockchip_ebc_enable(struct drm_simple_display_pipe *pipe,
				struct drm_crtc_state *crtc_state,
				struct drm_plane_state *plane_state)
{
	struct rockchip_ebc *ebc = pipe_to_ebc(pipe);
	struct rockchip_ebc_panel *panel = &ebc->panel;
	struct device *dev = ebc->drm.dev;
	int ret;

	DRM_DEV_DEBUG_DRIVER(dev, "CRTC enable...\n");

	ret = clk_set_rate(ebc->dclk, panel->sdck * ((panel->panel_16bit ? 7 : 3) + 1));
	if (ret)
		DRM_DEV_ERROR(dev, "Failed to set pixel clock rate: %d\n", ret);

	rockchip_ebc_configure(ebc);

	if (ebc->panel_state == EBC_PANEL_BOOT)
		ebc->panel_state = EBC_PANEL_INIT;
	if (ebc->panel_state == EBC_PANEL_OFF)
		ebc->panel_state = EBC_PANEL_ON;
	queue_work(system_long_wq, &ebc->refresh_work);
}

static void rockchip_ebc_disable(struct drm_simple_display_pipe *pipe)
{
	struct rockchip_ebc *ebc = pipe_to_ebc(pipe);
	struct device *dev = ebc->drm.dev;

	DRM_DEV_DEBUG_DRIVER(dev, "CRTC disable...\n");

	flush_work(&ebc->refresh_work);
	ebc->panel_state = EBC_PANEL_OFF;
}

static u8 rockchip_ebc_apply_damage(struct rockchip_ebc *ebc,
				    struct drm_framebuffer *fb, void *vaddr,
				    struct drm_rect *clip)
{
	unsigned int x, y;
	u8 damage;

	for (y = clip->y1; y < clip->y2; y++) {
		u32 *src = vaddr + fb->pitches[0] * y;
		u8 *dst = ebc->frame_buffer + fb->width / 2 * (ebc->panel.height - y);

		for (x = clip->x1 & ~1U; x < clip->x2; x += 2) {
			u32 rgb_even = src[x + 0];
			u32 rgb_odd  = src[x + 1];
			u8 gray4;

			/* Truncate the RGB values to 5 bits each. */
			rgb_even &= 0x00f8f8f8U; rgb_odd &= 0x00f8f8f8U;
			/* This puts 2R+5G+B in bits 24-31. */
			rgb_even *= 0x0020a040U; rgb_odd *= 0x0020a040U;
			/* Unbias the rounding when taking the top 4 bits. */
			rgb_even += 0x07000000U; rgb_odd += 0x07000000U;

			gray4 = (rgb_even >> 28) | ((rgb_odd >> 28) << 4);
			damage |= dst[x / 2] ^ gray4;
			dst[x / 2] = gray4;
		}
	}

	return damage;
}

static void rockchip_ebc_update(struct drm_simple_display_pipe *pipe,
				struct drm_plane_state *old_plane_state)
{
	struct drm_plane_state *plane_state = pipe->plane.state;
	struct drm_framebuffer *fb = plane_state->fb;
	struct rockchip_ebc *ebc = pipe_to_ebc(pipe);
	struct drm_atomic_helper_damage_iter iter;
	struct device *dev = ebc->drm.dev;
	struct drm_gem_shmem_object *shmem;
	struct drm_gem_object *obj;
	bool need_refresh = false;
	struct dma_buf_map map;
	struct drm_rect clip;
	int ret;

	if (!fb)
		return;

	obj = drm_gem_fb_get_obj(fb, 0);
	shmem = to_drm_gem_shmem_obj(obj);
	ret = drm_gem_shmem_vmap(shmem, &map);
	if (ret) {
		DRM_DEV_ERROR(dev, "Failed to map FB shmem: %d\n", ret);
		return;
	}

	drm_atomic_helper_damage_iter_init(&iter, old_plane_state, plane_state);
	drm_atomic_for_each_plane_damage(&iter, &clip) {
		if (rockchip_ebc_apply_damage(ebc, fb, map.vaddr, &clip))
			need_refresh = true;
	}

	if (need_refresh && ebc->panel_state == EBC_PANEL_ON)
		queue_work(system_long_wq, &ebc->refresh_work);

	drm_gem_shmem_vunmap(shmem, &map);
}

static const struct drm_simple_display_pipe_funcs rockchip_ebc_pipe_funcs = {
	.enable		= rockchip_ebc_enable,
	.disable	= rockchip_ebc_disable,
	.update		= rockchip_ebc_update,
};

static const u32 rockchip_ebc_formats[] = {
	DRM_FORMAT_XRGB8888,
};

static const u64 rockchip_ebc_format_modifiers[] = {
	DRM_FORMAT_MOD_LINEAR,
	DRM_FORMAT_MOD_INVALID
};

static int rockchip_ebc_of_parse(struct rockchip_ebc *ebc,
				 struct device_node *np)
{
	struct rockchip_ebc_panel *panel = &ebc->panel;

	of_property_read_u32(np, "panel,width", &panel->width);
	of_property_read_u32(np, "panel,height", &panel->height);
	of_property_read_u32(np, "panel,vir_width", &panel->vir_width);
	of_property_read_u32(np, "panel,vir_height", &panel->vir_height);
	of_property_read_u32(np, "panel,sdck", &panel->sdck);
	of_property_read_u32(np, "panel,lsl", &panel->lsl);
	of_property_read_u32(np, "panel,lbl", &panel->lbl);
	of_property_read_u32(np, "panel,ldl", &panel->ldl);
	of_property_read_u32(np, "panel,lel", &panel->lel);
	of_property_read_u32(np, "panel,gdck-sta", &panel->gdck_sta);
	of_property_read_u32(np, "panel,lgonl", &panel->lgonl);
	of_property_read_u32(np, "panel,fsl", &panel->fsl);
	of_property_read_u32(np, "panel,fbl", &panel->fbl);
	of_property_read_u32(np, "panel,fdl", &panel->fdl);
	of_property_read_u32(np, "panel,fel", &panel->fel);
	of_property_read_u32(np, "panel,mirror", &panel->mirror);
	of_property_read_u32(np, "panel,panel_16bit", &panel->panel_16bit);
	of_property_read_u32(np, "panel,panel_color", &panel->panel_color);
	of_property_read_u32(np, "panel,width-mm", &panel->width_mm);
	of_property_read_u32(np, "panel,height-mm", &panel->height_mm);

	ebc->pixels = panel->width * panel->height;

	return 0;
}

static void rockchip_ebc_release_firmware(void *data)
{
	release_firmware(data);
}

static bool rockchip_ebc_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case EBC_INT_STATUS:
	case EBC_CONFIG_DONE:
	case EBC_VNUM:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config rockchip_ebc_regmap_config = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.volatile_reg	= rockchip_ebc_volatile_reg,
	.max_register	= EBC_WIN_MST2, /* LUT registers are not used */
	.cache_type	= REGCACHE_FLAT,
};

static const char *const rockchip_ebc_supplies[EBC_NUM_SUPPLIES] = {
	"panel",
	"vcom",
	"vdrive",
};

static int rockchip_ebc_runtime_suspend(struct device *dev)
{
	struct rockchip_ebc *ebc = dev_get_drvdata(dev);

	DRM_DEV_DEBUG_DRIVER(dev, "Suspending...\n");

	/* Ensure frame start is not set, and drive the output pins low. */
	ebc_update_bits(ebc, EBC_DSP_START,
			DSP_OUT_LOW | DSP_FRM_START,
			DSP_OUT_LOW);

	regcache_cache_only(ebc->regmap, true);

	/* Return the pins to normal operation during resume. */
	ebc_update_bits(ebc, EBC_DSP_START, DSP_OUT_LOW, 0);

	clk_disable_unprepare(ebc->dclk);
	clk_disable_unprepare(ebc->hclk);
	regulator_bulk_disable(EBC_NUM_SUPPLIES, ebc->supplies);

	return 0;
}

static int rockchip_ebc_runtime_resume(struct device *dev)
{
	struct rockchip_ebc *ebc = dev_get_drvdata(dev);
	int ret;

	DRM_DEV_DEBUG_DRIVER(dev, "Resuming...\n");

	ret = regulator_bulk_enable(EBC_NUM_SUPPLIES, ebc->supplies);
	if (ret)
		return ret;

	ret = clk_prepare_enable(ebc->hclk);
	if (ret)
		goto err_disable_supplies;

	ret = clk_prepare_enable(ebc->dclk);
	if (ret)
		goto err_disable_hclk;

	regcache_cache_only(ebc->regmap, false);
	regcache_mark_dirty(ebc->regmap);
	regcache_sync(ebc->regmap);

	/* Clear DSP_END_INT and mask the other interrupts. */
	ebc_write(ebc, EBC_INT_STATUS,
		  DSP_END_INT_CLR |
		  LINE_FLAG_INT_MASK |
		  DSP_FRM_INT_MASK |
		  FRM_END_INT_MASK);

	return 0;

err_disable_hclk:
	clk_disable_unprepare(ebc->hclk);
err_disable_supplies:
	regulator_bulk_disable(EBC_NUM_SUPPLIES, ebc->supplies);

	return ret;
}

static int rockchip_ebc_suspend(struct device *dev)
{
	struct rockchip_ebc *ebc = dev_get_drvdata(dev);

	return drm_mode_config_helper_suspend(&ebc->drm);
}

static int rockchip_ebc_resume(struct device *dev)
{
	struct rockchip_ebc *ebc = dev_get_drvdata(dev);

	return drm_mode_config_helper_resume(&ebc->drm);
}

static const struct dev_pm_ops rockchip_ebc_pm_ops = {
	SET_RUNTIME_PM_OPS(rockchip_ebc_runtime_suspend,
			   rockchip_ebc_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(rockchip_ebc_suspend, rockchip_ebc_resume)
};

static int rockchip_ebc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct firmware *fw;
	struct rockchip_ebc *ebc;
	struct drm_device *drm;
	void __iomem *base;
	int i, ret;

	ebc = devm_drm_dev_alloc(dev, &rockchip_ebc_drm_driver,
				 struct rockchip_ebc, drm);
	if (IS_ERR(ebc))
		return PTR_ERR(ebc);

	platform_set_drvdata(pdev, ebc);

	init_completion(&ebc->frame_complete);
	INIT_WORK(&ebc->refresh_work, rockchip_ebc_refresh_work);

	ret = rockchip_ebc_of_parse(ebc, dev->of_node);
	if (ret)
		return ret;

	/* Allocate all of the necessary buffers. */
	ebc->count_buffer = devm_kzalloc(dev, EBC_COUNT_SIZE(ebc), GFP_KERNEL);
	ebc->frame_buffer = devm_kzalloc(dev, EBC_FRAME_SIZE(ebc), GFP_KERNEL);
	ebc->prev_buffer = devm_kmalloc(dev, EBC_PREV_SIZE(ebc), GFP_KERNEL);
	ebc->next_buffer = devm_kmalloc(dev, EBC_NEXT_SIZE(ebc), GFP_KERNEL);
	ebc->dma_buffer = dmam_alloc_attrs(dev, 2 * EBC_DMA_SIZE(ebc),
					   &ebc->dma_handle, GFP_KERNEL,
					   DMA_ATTR_WRITE_COMBINE);
	if (!ebc->count_buffer || !ebc->frame_buffer || !ebc->prev_buffer ||
	    !ebc->next_buffer  || !ebc->dma_buffer)
		return -ENOMEM;

	ret = request_firmware(&fw, "waveform.bin", dev);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, rockchip_ebc_release_firmware,
				       (void *)fw);
	if (ret)
		return ret;

	ret = pvi_wf_ctx_init(&ebc->wf, fw);
	if (ret)
		return ret;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	ebc->regmap = devm_regmap_init_mmio(dev, base, &rockchip_ebc_regmap_config);
	if (IS_ERR(ebc->regmap))
		return PTR_ERR(ebc->regmap);

	regcache_cache_only(ebc->regmap, true);

	ebc->dclk = devm_clk_get(dev, "dclk");
	if (IS_ERR(ebc->dclk))
		return dev_err_probe(dev, PTR_ERR(ebc->dclk), "failed to get dclk\n");

	ebc->hclk = devm_clk_get(dev, "hclk");
	if (IS_ERR(ebc->hclk))
		return dev_err_probe(dev, PTR_ERR(ebc->hclk), "failed to get hclk\n");

	ebc->temp_chan = devm_iio_channel_get(dev, NULL);
	if (IS_ERR(ebc->temp_chan))
		return dev_err_probe(dev, PTR_ERR(ebc->temp_chan),
				     "failed to get temperature I/O channel\n");

	ret = devm_request_irq(dev, platform_get_irq(pdev, 0),
			       rockchip_ebc_irq, 0, dev_name(dev), ebc);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	for (i = 0; i < EBC_NUM_SUPPLIES; ++i)
		ebc->supplies[i].supply = rockchip_ebc_supplies[i];

	ret = devm_regulator_bulk_get(dev, EBC_NUM_SUPPLIES, ebc->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	pm_runtime_set_autosuspend_delay(dev, 250);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_enable(dev);

	drm = &ebc->drm;
	ret = drmm_mode_config_init(drm);
	if (ret)
		goto err_disable_pm;

	/* These match the window register bit field sizes. */
	drm->mode_config.max_width = 4095;
	drm->mode_config.max_height = 2047;
	drm->mode_config.funcs = &rockchip_ebc_mode_config_funcs;

	ret = drm_connector_init(drm, &ebc->connector,
				 &rockchip_ebc_connector_funcs,
				 DRM_MODE_CONNECTOR_Unknown);
	if (ret)
		goto err_disable_pm;

	drm_connector_helper_add(&ebc->connector, &rockchip_ebc_connector_helper_funcs);

	ret = drm_simple_display_pipe_init(drm, &ebc->pipe,
					   &rockchip_ebc_pipe_funcs,
					   rockchip_ebc_formats,
					   ARRAY_SIZE(rockchip_ebc_formats),
					   rockchip_ebc_format_modifiers,
					   &ebc->connector);
	if (ret)
		goto err_disable_pm;

	drm_plane_enable_fb_damage_clips(&ebc->pipe.plane);
	drm_mode_config_reset(drm);

	ret = drm_dev_register(drm, 0);
	if (ret)
		goto err_disable_pm;

	drm_fbdev_generic_setup(drm, 0);

	return 0;

err_disable_pm:
	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		rockchip_ebc_runtime_suspend(dev);

	return ret;
}

static int rockchip_ebc_remove(struct platform_device *pdev)
{
	struct rockchip_ebc *ebc = platform_get_drvdata(pdev);
	struct drm_device *drm = &ebc->drm;
	struct device *dev = &pdev->dev;

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);

	pm_runtime_disable(dev);
	if (!pm_runtime_status_suspended(dev))
		rockchip_ebc_runtime_suspend(dev);

	return 0;
}

static void rockchip_ebc_shutdown(struct platform_device *pdev)
{
	struct rockchip_ebc *ebc = platform_get_drvdata(pdev);
	struct drm_device *drm = &ebc->drm;
	struct device *dev = &pdev->dev;

	drm_atomic_helper_shutdown(drm);

	if (!pm_runtime_status_suspended(dev))
		rockchip_ebc_runtime_suspend(dev);
}

static const struct of_device_id rockchip_ebc_of_match[] = {
	{ .compatible = "rockchip,rk3568-ebc-tcon" },
	{}
};
MODULE_DEVICE_TABLE(of, rockchip_ebc_of_match);

static struct platform_driver rockchip_ebc_driver = {
	.probe		= rockchip_ebc_probe,
	.remove		= rockchip_ebc_remove,
	.shutdown	= rockchip_ebc_shutdown,
	.driver		= {
		.name		= "rockchip-ebc",
		.of_match_table	= rockchip_ebc_of_match,
		.pm		= &rockchip_ebc_pm_ops,
	},
};
module_platform_driver(rockchip_ebc_driver);

MODULE_AUTHOR("Zorro Liu <zorro.liu@rock-chips.com>");
MODULE_DESCRIPTION("Rockchip EBC driver");
MODULE_FIRMWARE("waveform.bin");
MODULE_LICENSE("GPL v2");
