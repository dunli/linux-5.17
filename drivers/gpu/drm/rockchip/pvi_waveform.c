// SPDX-License-Identifier: GPL-2.0+

#include <linux/module.h>
#include <linux/printk.h>
#include <linux/vmalloc.h>

#include "pvi_waveform.h"

enum pvi_wf_mode {
	PVI_WF_RESET,
	PVI_WF_DU,
	PVI_WF_DU4,
	PVI_WF_GC16,
	PVI_WF_GL16,
	PVI_WF_GLR16,
	PVI_WF_GLD16,
	PVI_WF_A2,
	PVI_WF_GCC16,
	PVI_WF_MAX
};

struct pvi_wf_file {
	__le32	checksum;		// 0x00
	__le32	file_length;		// 0x04
	__le32	serial;			// 0x08
	u8	run_type;		// 0x0c
	u8	fpl_platform;		// 0x0d
	__le16	fpl_lot;		// 0x0e
	u8	mode_version;		// 0x10
	u8	wf_version;		// 0x11
	u8	wf_subversion;		// 0x12
	u8	wf_type;		// 0x13
	u8	panel_size;		// 0x14
	u8	amepd_part_number;	// 0x15
	u8	wfm_rev;		// 0x16
	u8	frame_rate;		// 0x17
	__le32	reserved;		// 0x18
	u8	vcom_offset;		// 0x1c
	u8	xwia[3];		// 0x1d
	u8	sc1;			// 0x20
	u8	unknown1[4];		// 0x21
	u8	modenumber;		// 0x25
	// number of temperature table entries
	u8	temperaturenumber;	// 0x26
	u8	unknown2[9];		// 0x27
	u8	data[];			// 0x30
};

// new structure added for the table below
struct pvi_wf_version_info {
	// some index tables are used by two versions
	u8	wf_version_a;
	u8	wf_version_b;
	u8	wf_modes[PVI_WF_MAX];
};

// from get_wf_mode_index(), skipped the default for "unknown" versions
static const struct pvi_wf_version_info pvi_wf_version_table[] = {
	{
		.wf_version_a	= 0x09,
		.wf_modes	= {
			[PVI_WF_RESET]	= 0,
			[PVI_WF_DU]	= 1,
			[PVI_WF_DU4]	= 1,
			[PVI_WF_GC16]	= 2,
			[PVI_WF_GL16]	= 3,
			[PVI_WF_GLR16]	= 3,
			[PVI_WF_GLD16]	= 3,
			[PVI_WF_A2]	= 4,
			[PVI_WF_GCC16]	= 3,
		},
	},
	{
		.wf_version_a	= 0x12,
		.wf_modes	= {
			[PVI_WF_RESET]	= 0,
			[PVI_WF_DU]	= 1,
			[PVI_WF_DU4]	= 7,
			[PVI_WF_GC16]	= 3,
			[PVI_WF_GL16]	= 3,
			[PVI_WF_GLR16]	= 5,
			[PVI_WF_GLD16]	= 6,
			[PVI_WF_A2]	= 4,
			[PVI_WF_GCC16]	= 5,
		},
	},
	{
		.wf_version_a	= 0x16,
		.wf_modes	= {
			[PVI_WF_RESET]	= 0,
			[PVI_WF_DU]	= 1,
			[PVI_WF_DU4]	= 1,
			[PVI_WF_GC16]	= 2,
			[PVI_WF_GL16]	= 3,
			[PVI_WF_GLR16]	= 4,
			[PVI_WF_GLD16]	= 4,
			[PVI_WF_A2]	= 6,
			[PVI_WF_GCC16]	= 5,
		},
	},
	{
		.wf_version_a	= 0x18,
		.wf_version_b	= 0x20,
		.wf_modes	= {
			[PVI_WF_RESET]	= 0,
			[PVI_WF_DU]	= 1,
			[PVI_WF_DU4]	= 1,
			[PVI_WF_GC16]	= 2,
			[PVI_WF_GL16]	= 3,
			[PVI_WF_GLR16]	= 4,
			[PVI_WF_GLD16]	= 5,
			[PVI_WF_A2]	= 6,
			[PVI_WF_GCC16]	= 4,
		},
	},
	{
		.wf_version_a	= 0x19,
		.wf_version_b	= 0x43,
		.wf_modes	= {
			[PVI_WF_RESET]	= 0,
			[PVI_WF_DU]	= 1,
			[PVI_WF_DU4]	= 7,
			[PVI_WF_GC16]	= 2,
			[PVI_WF_GL16]	= 3,
			[PVI_WF_GLR16]	= 4,
			[PVI_WF_GLD16]	= 5,
			[PVI_WF_A2]	= 6,
			[PVI_WF_GCC16]	= 4,
		},
	},
	{
		.wf_version_a	= 0x23,
		.wf_modes	= {
			[PVI_WF_RESET]	= 0,
			[PVI_WF_DU]	= 1,
			[PVI_WF_DU4]	= 5,
			[PVI_WF_GC16]	= 2,
			[PVI_WF_GL16]	= 3,
			[PVI_WF_GLR16]	= 3,
			[PVI_WF_GLD16]	= 3,
			[PVI_WF_A2]	= 4,
			[PVI_WF_GCC16]	= 3,
		},
	},
	{
		.wf_version_a	= 0x54,
		.wf_modes	= {
			[PVI_WF_RESET]	= 0,
			[PVI_WF_DU]	= 1,
			[PVI_WF_DU4]	= 1,
			[PVI_WF_GC16]	= 2,
			[PVI_WF_GL16]	= 3,
			[PVI_WF_GLR16]	= 4,
			[PVI_WF_GLD16]	= 4,
			[PVI_WF_A2]	= 5,
			[PVI_WF_GCC16]	= 4,
		},
	},
};

// from get_wf_mode_index(), complete
static int pvi_wf_get_mode_index(struct pvi_wf_ctx *wf_ctx,
				 enum epd_lut_type lut_type)
{
	const struct pvi_wf_file *wf_file = wf_ctx->wf_file;
	const struct pvi_wf_version_info *info;
	size_t i;

	for (i = 0; i < ARRAY_SIZE(pvi_wf_version_table); ++i) {
		info = &pvi_wf_version_table[i];

		if (info->wf_version_a == wf_file->mode_version ||
		    info->wf_version_b == wf_file->mode_version)
			break;
	}
	if (i == ARRAY_SIZE(pvi_wf_version_table)) {
		pr_err("PVI: unsupported waveform version: %#x\n", wf_file->wf_version);
		return -EINVAL;
	}

	switch (lut_type) {
	case WF_TYPE_RESET:
		return info->wf_modes[PVI_WF_RESET];
	case WF_TYPE_GRAY16:
		return info->wf_modes[PVI_WF_GC16];
	case WF_TYPE_GRAY4:
		return info->wf_modes[PVI_WF_DU4];
	case WF_TYPE_GRAY2:
		return info->wf_modes[PVI_WF_DU];
	case WF_TYPE_AUTO:
		return info->wf_modes[PVI_WF_GC16];
	case WF_TYPE_A2:
		return info->wf_modes[PVI_WF_A2];
	case WF_TYPE_GC16:
		return info->wf_modes[PVI_WF_GC16];
	case WF_TYPE_GL16:
		return info->wf_modes[PVI_WF_GL16];
	case WF_TYPE_GLR16:
		return info->wf_modes[PVI_WF_GLR16];
	case WF_TYPE_GLD16:
		return info->wf_modes[PVI_WF_GLD16];
	case WF_TYPE_GCC16:
	default:
		pr_err("PVI: unsupported waveform type: %d\n", lut_type);
		return -EINVAL;
	}
}

static int pvi_wf_get_temp_index(struct pvi_wf_ctx *wf_ctx,
				 int temperature)
{
	const struct pvi_wf_file *wf_file = wf_ctx->wf_file;
	const u8 *temp_threshold_table = &wf_file->data[0];
	int i;

	// Avoid unsigned undeflow. (move this to the file validity check function?)
	if (!wf_file->temperaturenumber) {
		pr_err("PVI: missing temperature threshold table\n");
		return -EINVAL;
	}

	for (i = 0; i < wf_file->temperaturenumber; ++i)
		if (temperature < temp_threshold_table[i])
			return i;

	return wf_file->temperaturenumber - 1;
}

// from decodewaveform, complete
static int pvi_wf_decode_waveform(struct pvi_wf_ctx *wf_ctx,
				  const u8 *waveform_data, u32 maxpic)
{
	u8 *wf_table = wf_ctx->wf_table;
	u32 format, offset, x, y;
	int frame, frame_number;

	if (maxpic != 0x10 && maxpic != 0x20) {
		pr_err("PVI: bad maxpic value %#x\n", maxpic);
		return -EINVAL;
	}

	offset = 0;
	format = 1;
	frame = 0;
	x = 0;
	y = 0;
	while (frame < 0x100) {
		u8 token = waveform_data[offset++];
		int iters = 1;

		// End of input stream
		if (token == 0xff)
			break;

		// Switch formats
		//   - format 0: sequence of input tokens
		//   - format 1: sequence of [token, copies] pairs
		if (token == 0xfc) {
			format = !format;
			token = waveform_data[offset++];
		}

		// If in format 1, read the number of additional copies
		if (format)
			iters += waveform_data[offset++];

		while (iters--) {
			wf_table[wf_table_cell(frame, x + 0, y)] = token >> 0 & 0x3;
			wf_table[wf_table_cell(frame, x + 1, y)] = token >> 2 & 0x3;
			wf_table[wf_table_cell(frame, x + 2, y)] = token >> 4 & 0x3;
			wf_table[wf_table_cell(frame, x + 3, y)] = token >> 6 & 0x3;

			x += 4;
			if (x >= maxpic) {
				x = 0;
				y++;
				if (y >= maxpic) {
					y = 0;
					frame++;
				}
			}
		}
	}

	frame_number = frame;
	pr_info("PVI: read %d frames\n",
		frame_number);

	// Didn't find the end of the input?
	if (frame_number == 0x100)
		pr_err("PVI: failed to decode waveform\n");

	if (maxpic == 0x20) {
		for (frame = 0; frame < frame_number; ++frame) {
			for (x = 0; x < maxpic; x += 2) {
				for (y = 0; y < maxpic; y += 2) {
					wf_table[wf_table_cell(frame, x / 2, y / 2)] =
						wf_table[wf_table_cell(frame, x, y)];
				}
			}
		}
	}

	return frame_number;
}

// from get_wf_frm_num(), complete
static int pvi_wf_get_frame_number(struct pvi_wf_ctx *wf_ctx,
				   int mode_index, int temp_index)
{
	const struct pvi_wf_file *wf_file = wf_ctx->wf_file;
	const u8 *buf = (const u8 *)wf_file;
	const u8 *mode_table, *temp_table, *waveform_data;
	u32 temp_table_offset, waveform_data_offset;
	int maxpic; // decodewaveform parameter name

	mode_table = buf + wf_file->sc1 + 4 * mode_index;
	if (mode_table[3] != (u8)(mode_table[0] + mode_table[1] + mode_table[2])) {
		pr_err("PVI: bad mode table entry at %#lx\n",
		       mode_table - buf);
		return -EINVAL;
	}

	temp_table_offset = (u32)mode_table[2] << 16 |
			    (u32)mode_table[1] <<  8 |
			    (u32)mode_table[0];
	temp_table = buf + temp_table_offset + 4 * temp_index;
	if (temp_table[3] != (u8)(temp_table[2] + temp_table[1] + temp_table[0])) {
		pr_err("PVI: bad temperature table entry at %#lx\n",
		       temp_table - buf);
		return -EINVAL;
	}

	waveform_data_offset = (u32)temp_table[2] << 16 |
			       (u32)temp_table[1] <<  8 |
			       (u32)temp_table[0];
	waveform_data = buf + waveform_data_offset;

	pr_info("PVI: mode %#04lx => temp %#05lx => data %#07lx\n",
		mode_table - buf, temp_table - buf, waveform_data - buf);

	switch (wf_file->mode_version) {
	case 0x16:
	case 0x18:
	case 0x19:
	case 0x20:
	case 0x43:
		maxpic = 0x20;
		break;
	default:
		maxpic = 0x10;
		break;
	}

	return pvi_wf_decode_waveform(wf_ctx, waveform_data, maxpic);
}

// complete
int pvi_wf_get_lut(struct pvi_wf_ctx *wf_ctx,
		   enum epd_lut_type lut_type, int temperature)
{
	int frame, frame_number, gray2_frame_number, mode_index, temp_index;
	u8 *wf_table = wf_ctx->wf_table;
	u32 x, y;

	// Do nothing if the LUT type and temperature have not changed.
	if (lut_type == wf_ctx->lut_type &&
	    temperature / 3 == wf_ctx->temperature / 3)
		return 0;

	// I'm not sure why this is needed? It's in the mode tables...
	if (lut_type == WF_TYPE_GRAY4) {
		pr_err("PVI: unsupported waveform type GRAY4\n");
		return -EINVAL;
	}

	mode_index = pvi_wf_get_mode_index(wf_ctx, lut_type);
	if (mode_index < 0)
		return mode_index;

	temp_index = pvi_wf_get_temp_index(wf_ctx, temperature);
	if (temp_index < 0)
		return temp_index;

	frame_number = pvi_wf_get_frame_number(wf_ctx, mode_index, temp_index);
	if (frame_number < 1) {
		pr_err("PVI: failed to get frame number: %d\n", frame_number);
		return frame_number;
	}

	wf_ctx->frame_num = frame_number;

	if (lut_type == WF_TYPE_AUTO) {
		mode_index = pvi_wf_get_mode_index(wf_ctx, WF_TYPE_GRAY2);
		if (mode_index < 0)
			return mode_index;

		gray2_frame_number = pvi_wf_get_frame_number(wf_ctx, mode_index, temp_index);
		if (gray2_frame_number < 1) {
			pr_err("PVI: failed to get GRAY2 frame number: %d\n", gray2_frame_number);
			return gray2_frame_number;
		}
	}

	// Descramble(?) the waveform data
	for (frame = 0; frame < frame_number; ++frame) {
		for (x = 0; x < 0x100; ++x) {
			u32 x_hi = x >> 4;
			u32 x_lo = x & 0xf;

			for (y = 0; y < 0x100; ++y) {
				u32 y_hi = y >> 4;
				u32 y_lo = y & 0xf;

				wf_table[wf_table_cell(frame, x, y)] =
					((wf_table[wf_table_cell(frame, x_hi, y_hi)] & 3) << 2) |
					((wf_table[wf_table_cell(frame, x_lo, y_lo)] & 3) << 0);
			}
		}
	}

	wf_ctx->lut_type = lut_type;
	wf_ctx->temperature = temperature;

	return 0;
}
EXPORT_SYMBOL_GPL(pvi_wf_get_lut);

int pvi_wf_ctx_init(struct pvi_wf_ctx *wf_ctx,
		    const struct firmware *fw)
{

	if (fw->size < sizeof(struct pvi_wf_file))
		return -EINVAL;

	wf_ctx->frame_num	= 0;
	wf_ctx->lut_type	= 0;
	wf_ctx->temperature	= 0;
	wf_ctx->wf_file		= (const void *)fw->data;
	// FIXME: leaks
	wf_ctx->wf_table	= vmalloc(0x1000000);

	if (!wf_ctx->wf_table)
		return -ENOMEM;

	return 0;
}
EXPORT_SYMBOL_GPL(pvi_wf_ctx_init);

MODULE_AUTHOR("Samuel Holland <samuel@sholland.org>");
MODULE_DESCRIPTION("PVI waveform parser");
MODULE_LICENSE("GPL");
