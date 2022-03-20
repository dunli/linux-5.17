// SPDX-License-Identifier: GPL-2.0+

#ifndef _PVI_WAVEFORM_H_
#define _PVI_WAVEFORM_H_

#include <linux/firmware.h>

#define wf_table_cell(_frame, _row, _col) (0x10000 * (_frame) + 0x100 * (_row) + (_col))

enum epd_lut_type {
	WF_TYPE_RESET = 1,
	WF_TYPE_GRAY16,
	WF_TYPE_GRAY4,
	WF_TYPE_GRAY2,
	WF_TYPE_AUTO,
	WF_TYPE_A2,
	WF_TYPE_GC16,
	WF_TYPE_GL16,
	WF_TYPE_GLR16,
	WF_TYPE_GLD16,
	WF_TYPE_GCC16,
	WF_TYPE_MAX
};

static const char *const epd_lut_type_names[WF_TYPE_MAX] = {
	"(invalid)",
	"WF_TYPE_RESET",
	"WF_TYPE_GRAY16",
	"WF_TYPE_GRAY4",
	"WF_TYPE_GRAY2",
	"WF_TYPE_AUTO",
	"WF_TYPE_A2",
	"WF_TYPE_GC16",
	"WF_TYPE_GL16",
	"WF_TYPE_GLR16",
	"WF_TYPE_GLD16",
	"WF_TYPE_GCC16",
};

struct pvi_wf_file;
struct pvi_wf_ctx {
	const struct pvi_wf_file	*wf_file;
	u8				*wf_table;
	u32				frame_num;
	enum epd_lut_type		lut_type;
	int				temperature;
};

int pvi_wf_ctx_init(struct pvi_wf_ctx *wf_ctx,
		    const struct firmware *fw);
int pvi_wf_get_lut(struct pvi_wf_ctx *wf_ctx,
		   enum epd_lut_type lut_type, int temperature);

#endif /* _PVI_WAVEFORM_H_ */
