// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for Arducam Pivariety Cameras
 * Copyright (C) 2022 Arducam Technology co., Ltd.
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 *
 * I2C read and write method is taken from the OV9281 driver
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/rk-camera-module.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-preisp.h>
#include "../platform/rockchip/isp/rkisp_tb_helper.h"
#include "arducam-pivariety.h"
#include "../platform/rockchip/isp/rkisp_tb_helper.h"

static int debug = 0;
module_param(debug, int, 0644);

#define PIVARIETY_NAME			"pivariety"
#define DRIVER_VERSION			KERNEL_VERSION(0, 0x00, 0x10)

/* regulator supplies */
static const char * const pivariety_supply_name[] = {
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

/* The supported raw formats. */
static const u32 codes[] = {
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_Y8_1X8,

	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_Y10_1X10,

	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_Y12_1X12,
};

#define ARDUCAM_NUM_SUPPLIES ARRAY_SIZE(pivariety_supply_name)

#define ARDUCAM_XCLR_MIN_DELAY_US	10000
#define ARDUCAM_XCLR_DELAY_RANGE_US	1000

#define MAX_CTRLS 32

struct pivariety {
	struct v4l2_subdev sd;
	struct media_pad pad;

	//struct v4l2_mbus_config_mipi_csi2 bus;
	struct clk *xclk;
	u32 xclk_freq;

	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARDUCAM_NUM_SUPPLIES];

	struct arducam_format *supported_formats;
	int num_supported_formats;
	int current_format_idx;
	int current_resolution_idx;
	int lanes;
	int bayer_order_volatile;
	bool wait_until_free;

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *ctrls[MAX_CTRLS];
	/* V4L2 Controls */
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;

	struct v4l2_rect crop;
	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;
	bool power_on;
	bool is_thunderboot;
	bool is_thunderboot_ng;

	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

static const s64 link_freq_menu_items[] = {
	456000000,
};

static inline struct pivariety *to_pivariety(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct pivariety, sd);
}

/* Write registers up to 4 at a time */
static int pivariety_write_reg(struct i2c_client *client, u16 reg, u32 val)
{
	unsigned int len = sizeof(u32);
	u32 buf_i, val_i = 0;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

/* Read registers up to 4 at a time */
static int pivariety_read_reg(struct i2c_client *client, u16 reg, u32 *val)
{
	struct i2c_msg msgs[2];
	unsigned int len = sizeof(u32);
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = data_be_p;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int
pivariety_read(struct pivariety *pivariety, u16 addr, u32 *value)
{
	struct v4l2_subdev *sd = &pivariety->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret, count = 0;

	while (count++ < I2C_READ_RETRY_COUNT) {
		ret = pivariety_read_reg(client, addr, value);
		if (!ret) {
			v4l2_dbg(2, debug, sd, "%s: 0x%02x 0x%04x\n",
				 __func__, addr, *value);
			return ret;
		}
	}

	v4l2_err(sd, "%s: Reading register 0x%02x failed\n",
		 __func__, addr);

	return ret;
}

static int pivariety_write(struct pivariety *pivariety, u16 addr, u32 value)
{
	struct v4l2_subdev *sd = &pivariety->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret, count = 0;

	while (count++ < I2C_WRITE_RETRY_COUNT) {
		ret = pivariety_write_reg(client, addr, value);
		if (!ret)
			return ret;
	}

	v4l2_err(sd, "%s: Write 0x%04x to register 0x%02x failed\n",
		 __func__, value, addr);

	return ret;
}

static int wait_for_free(struct pivariety *pivariety, int interval)
{
	u32 value;
	u32 count = 0;

	while (count++ < (1000 / interval)) {
		int ret = pivariety_read(pivariety, SYSTEM_IDLE_REG, &value);

		if (!ret && !value)
			break;
		msleep(interval);
	}

	v4l2_dbg(2, debug, &pivariety->sd, "%s: End wait, Count: %d.\n",
		 __func__, count);

	return 0;
}

static int is_raw(int pixformat)
{
	return pixformat >= 0x28 && pixformat <= 0x2D;
}

static u32 bayer_to_mbus_code(int data_type, int bayer_order)
{
	const u32 depth8[] = {
		MEDIA_BUS_FMT_SBGGR8_1X8,
		MEDIA_BUS_FMT_SGBRG8_1X8,
		MEDIA_BUS_FMT_SGRBG8_1X8,
		MEDIA_BUS_FMT_SRGGB8_1X8,
		MEDIA_BUS_FMT_Y8_1X8,
	};

	const u32 depth10[] = {
		MEDIA_BUS_FMT_SBGGR10_1X10,
		MEDIA_BUS_FMT_SGBRG10_1X10,
		MEDIA_BUS_FMT_SGRBG10_1X10,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		MEDIA_BUS_FMT_Y10_1X10,
	};

	const u32 depth12[] = {
		MEDIA_BUS_FMT_SBGGR12_1X12,
		MEDIA_BUS_FMT_SGBRG12_1X12,
		MEDIA_BUS_FMT_SGRBG12_1X12,
		MEDIA_BUS_FMT_SRGGB12_1X12,
		MEDIA_BUS_FMT_Y12_1X12,
	};

	if (bayer_order < 0 || bayer_order > 4)
		return 0;

	switch (data_type) {
	case IMAGE_DT_RAW8:
		return depth8[bayer_order];
	case IMAGE_DT_RAW10:
		return depth10[bayer_order];
	case IMAGE_DT_RAW12:
		return depth12[bayer_order];
	}

	return 0;
}

static u32 yuv422_to_mbus_code(int data_type, int order)
{
	const u32 depth8[] = {
		MEDIA_BUS_FMT_YUYV8_1X16,
		MEDIA_BUS_FMT_YVYU8_1X16,
		MEDIA_BUS_FMT_UYVY8_1X16,
		MEDIA_BUS_FMT_VYUY8_1X16,
	};

	const u32 depth10[] = {
		MEDIA_BUS_FMT_YUYV10_1X20,
		MEDIA_BUS_FMT_YVYU10_1X20,
		MEDIA_BUS_FMT_UYVY10_1X20,
		MEDIA_BUS_FMT_VYUY10_1X20,
	};

	if (order < 0 || order > 3)
		return 0;

	switch (data_type) {
	case IMAGE_DT_YUV422_8:
		return depth8[order];
	case IMAGE_DT_YUV422_10:
		return depth10[order];
	}

	return 0;
}

static u32 data_type_to_mbus_code(int data_type, int bayer_order)
{
	if (is_raw(data_type))
		return bayer_to_mbus_code(data_type, bayer_order);

	switch (data_type) {
	case IMAGE_DT_YUV422_8:
	case IMAGE_DT_YUV422_10:
		return yuv422_to_mbus_code(data_type, bayer_order);
	case IMAGE_DT_RGB565:
		return MEDIA_BUS_FMT_RGB565_2X8_LE;
	case IMAGE_DT_RGB888:
		return MEDIA_BUS_FMT_RGB888_1X24;
	}

	return 0;
}

/* Get bayer order based on flip setting. */
static u32 pivariety_get_format_code(struct pivariety *pivariety,
				     struct arducam_format *format)
{
	unsigned int order, origin_order;

	lockdep_assert_held(&pivariety->mutex);

	/*
	 * Only the bayer format needs to transform the format.
	 */
	if (!is_raw(format->data_type) ||
	    !pivariety->bayer_order_volatile ||
	    format->bayer_order == BAYER_ORDER_GRAY)
		return data_type_to_mbus_code(format->data_type,
					      format->bayer_order);

	order = format->bayer_order;

	origin_order = order;

	order = (pivariety->hflip && pivariety->hflip->val ? order ^ 1 : order);
	order = (pivariety->vflip && pivariety->vflip->val ? order ^ 2 : order);

	v4l2_dbg(1, debug, &pivariety->sd, "%s: before: %d, after: %d.\n",
		 __func__, origin_order, order);

	return data_type_to_mbus_code(format->data_type, order);
}

/* Power/clock management functions */
static int pivariety_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct pivariety *pivariety = to_pivariety(sd);
	int ret;

	ret = regulator_bulk_enable(ARDUCAM_NUM_SUPPLIES,
				    pivariety->supplies);
	if (ret) {
		dev_err(dev, "%s: failed to enable regulators\n",
			__func__);
		return ret;
	}

	ret = clk_prepare_enable(pivariety->xclk);
	if (ret) {
		dev_err(dev, "%s: failed to enable clock\n",
			__func__);
		goto reg_off;
	}

	gpiod_set_value_cansleep(pivariety->reset_gpio, 1);
	usleep_range(ARDUCAM_XCLR_MIN_DELAY_US,
		     ARDUCAM_XCLR_MIN_DELAY_US + ARDUCAM_XCLR_DELAY_RANGE_US);

	return 0;

reg_off:
	regulator_bulk_disable(ARDUCAM_NUM_SUPPLIES, pivariety->supplies);

	return ret;
}

static int pivariety_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct pivariety *pivariety = to_pivariety(sd);

	gpiod_set_value_cansleep(pivariety->reset_gpio, 0);
	regulator_bulk_disable(ARDUCAM_NUM_SUPPLIES, pivariety->supplies);
	clk_disable_unprepare(pivariety->xclk);

	return 0;
}

static int pivariety_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct pivariety *pivariety = to_pivariety(sd);
	struct v4l2_mbus_framefmt *try_fmt =
		v4l2_subdev_get_try_format(sd, fh->pad, 0);
	struct arducam_format *def_fmt = &pivariety->supported_formats[0];

	/* Initialize try_fmt */
	try_fmt->width = def_fmt->resolution_set->width;
	try_fmt->height = def_fmt->resolution_set->height;
	try_fmt->code = def_fmt->mbus_code;
	try_fmt->field = V4L2_FIELD_NONE;

	return 0;
}

static int pivariety_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret, i;
	struct pivariety *pivariety =
		container_of(ctrl->handler, struct pivariety,
			     ctrl_handler);
	struct arducam_format *supported_fmts = pivariety->supported_formats;
	int num_supported_formats = pivariety->num_supported_formats;

	v4l2_dbg(3, debug, &pivariety->sd, "%s: cid = (0x%X), value = (%d).\n",
		 __func__, ctrl->id, ctrl->val);

	ret = pivariety_write(pivariety, CTRL_ID_REG, ctrl->id);
	ret += pivariety_write(pivariety, CTRL_VALUE_REG, ctrl->val);
	if (ret < 0)
		return -EINVAL;

	/* When flip is set, modify all bayer formats */
	if (ctrl->id == V4L2_CID_VFLIP || ctrl->id == V4L2_CID_HFLIP) {
		for (i = 0; i < num_supported_formats; i++) {
			supported_fmts[i].mbus_code =
				pivariety_get_format_code(pivariety,
							  &supported_fmts[i]);
		}
	}

	/*
	 * When starting streaming, controls are set in batches,
	 * and the short interval will cause some controls to be unsuccessfully
	 * set.
	 */
	if (pivariety->wait_until_free)
		wait_for_free(pivariety, 1);
	else
		usleep_range(200, 210);

	return 0;
}

static const struct v4l2_ctrl_ops pivariety_ctrl_ops = {
	.s_ctrl = pivariety_s_ctrl,
};

static int pivariety_enum_mbus_code(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	struct pivariety *pivariety = to_pivariety(sd);
	struct arducam_format *supported_formats = pivariety->supported_formats;
	int num_supported_formats = pivariety->num_supported_formats;

	v4l2_dbg(1, debug, sd, "%s: index = (%d)\n", __func__, code->index);

	if (code->index >= num_supported_formats)
		return -EINVAL;

	code->code = supported_formats[code->index].mbus_code;

	return 0;
}

static int pivariety_enum_framesizes(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_frame_size_enum *fse)
{
	int i;
	struct pivariety *pivariety = to_pivariety(sd);
	struct arducam_format *supported_formats = pivariety->supported_formats;
	int num_supported_formats = pivariety->num_supported_formats;
	struct arducam_format *format;
	struct arducam_resolution *resolution;

	v4l2_dbg(1, debug, sd, "%s: code = (0x%X), index = (%d)\n",
		 __func__, fse->code, fse->index);

	for (i = 0; i < num_supported_formats; i++) {
		format = &supported_formats[i];
		if (fse->code == format->mbus_code) {
			if (fse->index >= format->num_resolution_set)
				return -EINVAL;

			resolution = &format->resolution_set[fse->index];
			fse->min_width = resolution->width;
			fse->max_width = resolution->width;
			fse->min_height = resolution->height;
			fse->max_height = resolution->height;

			return 0;
		}
	}

	return -EINVAL;
}

static int pivariety_enum_frame_interval(struct v4l2_subdev *sd,
				       struct v4l2_subdev_pad_config *cfg,
				       struct v4l2_subdev_frame_interval_enum *fie)
{
	int i;
	struct pivariety *pivariety = to_pivariety(sd);
	struct arducam_format *supported_formats = pivariety->supported_formats;
	int num_supported_formats = pivariety->num_supported_formats;
	struct arducam_format *format;
	struct arducam_resolution *resolution;

	struct v4l2_fract tmp = {
		.numerator = 10000,
		.denominator = 600000,
	};

	v4l2_dbg(1, debug, sd, "%s: code = (0x%X), index = (%d)\n",
		 __func__, fie->code, fie->index);
		 
	for (i = 0; i < num_supported_formats; i++) {
		format = &supported_formats[i];
		if (fie->code == format->mbus_code) {
			// if (fie->index >= format->num_resolution_set)
			// 	return -EINVAL;
			fie->code = MEDIA_BUS_FMT_SRGGB10_1X10;
			resolution = &format->resolution_set[fie->index];
			fie->width = resolution->width;
			fie->height = resolution->height;
			fie->interval = tmp;
			fie->reserved[0] = NO_HDR;

			return 0;
		}
	}

	return -EINVAL;
}

static int pivariety_get_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct pivariety *pivariety = to_pivariety(sd);
	struct arducam_format *current_format = 
		&pivariety->supported_formats[pivariety->current_format_idx];
	struct v4l2_mbus_framefmt *fmt = &format->format;
	int cur_res_idx = pivariety->current_resolution_idx;

	// if (format->pad != 0)
	// 	return -EINVAL;

	// mutex_lock(&pivariety->mutex);

	// current_format =
	// 	&pivariety->supported_formats[pivariety->current_format_idx];
	// cur_res_idx = pivariety->current_resolution_idx;
	format->format.width =
		current_format->resolution_set[cur_res_idx].width;
	format->format.height =
		current_format->resolution_set[cur_res_idx].height;
	format->format.code = current_format->mbus_code;
	format->format.field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);

	v4l2_dbg(1, debug, sd, "%s: width: (%d) height: (%d) code: (0x%X)\n",
		 __func__, format->format.width, format->format.height,
		 format->format.code);

	// mutex_unlock(&pivariety->mutex);
	return 0;
}

static int pivariety_get_fmt_idx_by_code(struct pivariety *pivariety,
					 u32 mbus_code)
{
	int i;
	u32 data_type;
	struct arducam_format *formats = pivariety->supported_formats;

	for (i = 0; i < pivariety->num_supported_formats; i++) {
		if (formats[i].mbus_code == mbus_code)
			return i;
	}

	/*
	 * If the specified format is not found in the list of supported
	 * formats, try to find a format of the same data type.
	 */
	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == mbus_code)
			break;

	if (i >= ARRAY_SIZE(codes))
		return -EINVAL;

	data_type = i / 5 + IMAGE_DT_RAW8;

	for (i = 0; i < pivariety->num_supported_formats; i++) {
		if (formats[i].data_type == data_type)
			return i;
	}

	return -EINVAL;
}

static struct v4l2_ctrl *get_control(struct pivariety *pivariety,
				     u32 id)
{
	int index = 0;

	while (index < MAX_CTRLS && pivariety->ctrls[index]) {
		if (pivariety->ctrls[index]->id == id)
			return pivariety->ctrls[index];
		index++;
	}

	return NULL;
}

static int update_control(struct pivariety *pivariety, u32 id)
{
	struct v4l2_subdev *sd = &pivariety->sd;
	struct v4l2_ctrl *ctrl;
	u32 min, max, step, def, id2;
	int ret = 0;

	pivariety_write(pivariety, CTRL_ID_REG, id);
	pivariety_read(pivariety, CTRL_ID_REG, &id2);

	v4l2_dbg(1, debug, sd, "%s: Write ID: 0x%08X Read ID: 0x%08X\n",
		 __func__, id, id2);

	pivariety_write(pivariety, CTRL_VALUE_REG, 0);
	wait_for_free(pivariety, 1);

	ret += pivariety_read(pivariety, CTRL_MAX_REG, &max);
	ret += pivariety_read(pivariety, CTRL_MIN_REG, &min);
	ret += pivariety_read(pivariety, CTRL_DEF_REG, &def);
	ret += pivariety_read(pivariety, CTRL_STEP_REG, &step);

	if (ret < 0)
		goto err;

	if (id == NO_DATA_AVAILABLE || max == NO_DATA_AVAILABLE ||
	    min == NO_DATA_AVAILABLE || def == NO_DATA_AVAILABLE ||
	    step == NO_DATA_AVAILABLE)
		goto err;

	v4l2_dbg(1, debug, sd, "%s: min: %d, max: %d, step: %d, def: %d\n",
		 __func__, min, max, step, def);

	ctrl = get_control(pivariety, id);
	return __v4l2_ctrl_modify_range(ctrl, min, max, step, def);

err:
	return -EINVAL;
}

static int update_controls(struct pivariety *pivariety)
{
	int ret = 0;
	int index = 0;

	wait_for_free(pivariety, 5);

	while (index < MAX_CTRLS && pivariety->ctrls[index]) {
		ret += update_control(pivariety, pivariety->ctrls[index]->id);
		index++;
	}

	return ret;
}

static int pivariety_set_fmt(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	int i, j;
	struct pivariety *pivariety = to_pivariety(sd);
	struct arducam_format *supported_formats = pivariety->supported_formats;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&pivariety->mutex);

	format->format.colorspace = V4L2_COLORSPACE_RAW;
	format->format.field = V4L2_FIELD_NONE;

	v4l2_dbg(1, debug, sd, "%s: code: 0x%X, width: %d, height: %d\n",
		 __func__, format->format.code, format->format.width,
		 format->format.height);

	i = pivariety_get_fmt_idx_by_code(pivariety, format->format.code);
	if (i < 0)
		i = 0;

	format->format.code = supported_formats[i].mbus_code;

	for (j = 0; j < supported_formats[i].num_resolution_set; j++) {
		if (supported_formats[i].resolution_set[j].width ==
						format->format.width &&
			supported_formats[i].resolution_set[j].height ==
						format->format.height) {
			v4l2_dbg(1, debug, sd,
				 "%s: format match.\n", __func__);
			v4l2_dbg(1, debug, sd,
				 "%s: set format to device: %d %d.\n",
				 __func__, supported_formats[i].index, j);

			pivariety_write(pivariety, PIXFORMAT_INDEX_REG,
					supported_formats[i].index);
			pivariety_write(pivariety, RESOLUTION_INDEX_REG, j);

			pivariety->current_format_idx = i;
			pivariety->current_resolution_idx = j;

			update_controls(pivariety);

			goto unlock;
		}
	}

	format->format.width = supported_formats[i].resolution_set[0].width;
	format->format.height = supported_formats[i].resolution_set[0].height;

	pivariety_write(pivariety, PIXFORMAT_INDEX_REG,
			supported_formats[i].index);
	pivariety_write(pivariety, RESOLUTION_INDEX_REG, 0);

	pivariety->current_format_idx = i;
	pivariety->current_resolution_idx = 0;
	update_controls(pivariety);

unlock:

	mutex_unlock(&pivariety->mutex);

	return 0;
}

static int pivariety_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	// struct pivariety *pivariety = to_pivariety(sd);

	struct v4l2_fract tmp = {
		.numerator = 10000,
		.denominator = 300000,
	};
	// mutex_lock(&pivariety->mutex);
	fi->interval = tmp;
	// mutex_unlock(&pivariety->mutex);

	return 0;
}

/* Start streaming */
static int pivariety_start_streaming(struct pivariety *pivariety)
{
	int ret;

	/* set stream on register */
	ret = pivariety_write(pivariety, MODE_SELECT_REG,
			      ARDUCAM_MODE_STREAMING);

	if (ret)
		return ret;

	wait_for_free(pivariety, 2);

	/*
	 * When starting streaming, controls are set in batches,
	 * and the short interval will cause some controls to be unsuccessfully
	 * set.
	 */
	pivariety->wait_until_free = true;
	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(pivariety->sd.ctrl_handler);

	pivariety->wait_until_free = false;
	if (ret)
		return ret;

	wait_for_free(pivariety, 2);

	return ret;
}

static int pivariety_read_sel(struct pivariety *pivariety,
			      struct v4l2_rect *rect)
{
	int ret = 0;

	ret += pivariety_read(pivariety, IPC_SEL_TOP_REG, &rect->top);
	ret += pivariety_read(pivariety, IPC_SEL_LEFT_REG, &rect->left);
	ret += pivariety_read(pivariety, IPC_SEL_WIDTH_REG, &rect->width);
	ret += pivariety_read(pivariety, IPC_SEL_HEIGHT_REG, &rect->height);

	if (ret || rect->top == NO_DATA_AVAILABLE ||
	    rect->left == NO_DATA_AVAILABLE ||
	    rect->width == NO_DATA_AVAILABLE ||
	    rect->height == NO_DATA_AVAILABLE) {
		v4l2_err(&pivariety->sd, "%s: Failed to read selection.\n",
			 __func__);
		return -EINVAL;
		}

	return 0;
}

static const struct v4l2_rect *
__pivariety_get_pad_crop(struct pivariety *pivariety,
			 struct v4l2_subdev_pad_config *cfg,
			 unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	int ret;

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&pivariety->sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		ret = pivariety_read_sel(pivariety, &pivariety->crop);
		if (ret)
			return NULL;
		return &pivariety->crop;
	}

	return NULL;
}

static int pivariety_get_selection(struct v4l2_subdev *sd,
				   struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_selection *sel)
{
	int ret = 0;
	struct v4l2_rect rect;
	struct pivariety *pivariety = to_pivariety(sd);

	ret = pivariety_write(pivariety, IPC_SEL_TARGET_REG, sel->target);
	if (ret) {
		v4l2_err(sd, "%s: Write register 0x%02x failed\n",
			 __func__, IPC_SEL_TARGET_REG);
		return -EINVAL;
	}

	wait_for_free(pivariety, 2);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		mutex_lock(&pivariety->mutex);
		sel->r = *__pivariety_get_pad_crop(pivariety, cfg,
						   sel->pad,
						   sel->which);
		mutex_unlock(&pivariety->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		ret = pivariety_read_sel(pivariety, &rect);
		if (ret)
			return -EINVAL;

		sel->r = rect;
		return 0;
	}

	return -EINVAL;
}

/* Stop streaming */
static int pivariety_stop_streaming(struct pivariety *pivariety)
{
	int ret;

	/* set stream off register */
	ret = pivariety_write(pivariety, MODE_SELECT_REG, ARDUCAM_MODE_STANDBY);
	if (ret)
		v4l2_err(&pivariety->sd, "%s failed to set stream\n", __func__);

	/*
	 * Return success even if it was an error, as there is nothing the
	 * caller can do about it.
	 */
	return 0;
}

static int pivariety_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct pivariety *pivariety = to_pivariety(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&pivariety->mutex);
	// if (pivariety->streaming == enable) {
	// 	mutex_unlock(&pivariety->mutex);
	// 	return 0;
	// }
	printk("arducam pivariety set stream - %d",enable);
	if (enable) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto err_unlock;
		}

		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = pivariety_start_streaming(pivariety);
		printk("pivariety_start_streaming");

		if (ret)
			goto err_rpm_put;
	} else {
		pivariety_stop_streaming(pivariety);
		printk("pivariety_stop_streaming");

		pm_runtime_put(&client->dev);
	}

	pivariety->streaming = enable;

	/*
	 * vflip and hflip cannot change during streaming
	 * Pivariety may not implement flip control.
	 */
	if (pivariety->vflip)
		__v4l2_ctrl_grab(pivariety->vflip, enable);

	if (pivariety->hflip)
		__v4l2_ctrl_grab(pivariety->hflip, enable);

	mutex_unlock(&pivariety->mutex);

	return ret;

err_rpm_put:
	pm_runtime_put(&client->dev);
err_unlock:
	mutex_unlock(&pivariety->mutex);

	return ret;
}

static int __maybe_unused pivariety_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct pivariety *pivariety = to_pivariety(sd);

	if (pivariety->streaming)
		pivariety_stop_streaming(pivariety);

	return 0;
}

static int __maybe_unused pivariety_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct pivariety *pivariety = to_pivariety(sd);
	int ret;

	if (pivariety->streaming) {
		ret = pivariety_start_streaming(pivariety);
		if (ret)
			goto error;
	}

	return 0;

error:
	pivariety_stop_streaming(pivariety);
	pivariety->streaming = 0;
	return ret;
}

static int pivariety_get_regulators(struct pivariety *pivariety)
{
	struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);
	int i;

	for (i = 0; i < ARDUCAM_NUM_SUPPLIES; i++)
		pivariety->supplies[i].supply = pivariety_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       ARDUCAM_NUM_SUPPLIES,
				       pivariety->supplies);
}

static int pivariety_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				     struct v4l2_mbus_config *cfg)
{
	struct pivariety *pivariety = to_pivariety(sd);
	u32 val = 0;

	// if (pivariety->lanes > pivariety->bus.num_data_lanes)
	// 	return -EINVAL;

	val = 1 << (pivariety->lanes - 1) |
	      V4L2_MBUS_CSI2_CHANNEL_0 |
	      V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	val |= V4L2_MBUS_CSI2_CHANNEL_1;
	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->flags = val;
	// cfg->bus.mipi_csi2.flags = pivariety->bus.flags;
	// cfg->bus.mipi_csi2.num_data_lanes = pivariety->lanes;

	return 0;
}

static void pivariety_get_module_inf(struct pivariety *pivariety,
				  struct rkmodule_inf *inf)
{
	// struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);

	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, PIVARIETY_NAME, sizeof(inf->base.sensor));
	strlcpy(inf->base.module, pivariety->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, pivariety->len_name, sizeof(inf->base.lens));

	// v4l2_dbg(1, debug, pivariety->sd,"%s: get_module_inf:%s, %s, %s.\n", __func__,
	// 	inf->base.sensor, inf->base.module, inf->base.lens);
}

static int pivariety_get_channel_info(struct pivariety *pivariety, struct rkmodule_channel_info *ch_info)
{
	struct arducam_format *current_format;
	int cur_res_idx;

	if (ch_info->index < PAD0 || ch_info->index >= PAD_MAX)
		return -EINVAL;

	current_format =
		&pivariety->supported_formats[pivariety->current_format_idx];
	cur_res_idx = pivariety->current_resolution_idx;
	ch_info->width = current_format->resolution_set[cur_res_idx].width;
	ch_info->height =
		current_format->resolution_set[cur_res_idx].height;
	ch_info->bus_fmt = current_format->mbus_code;
	ch_info->vc = V4L2_MBUS_CSI2_CHANNEL_0;
	 // pivariety->cur_mode->vc[ch_info->index];

	return 0;
}

static long pivariety_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct pivariety *pivariety = to_pivariety(sd);
	struct rkmodule_channel_info *ch_info;
	long ret = 0;
	printk("%s: cmd:[%u] ",__func__,cmd);
	switch (cmd)
	{
	case RKMODULE_GET_MODULE_INFO:
		pivariety_get_module_inf(pivariety, (struct rkmodule_inf *)arg);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = (struct rkmodule_channel_info *)arg;
		ret = pivariety_get_channel_info(pivariety, ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long pivariety_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	struct rkmodule_awb_cfg *cfg;
	struct rkmodule_channel_info *ch_info;
	long ret;
	// printk("%s: cmd:[%u] ",__func__,cmd);

	switch (cmd)
	{
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = pivariety_ioctl(sd, cmd, inf);
		if(!ret) {
			if (copy_to_user(up, inf, sizeof(*inf))) {
				kfree (inf);
				return -EFAULT;
			}
		}
		kfree(inf);
		break;
	case RKMODULE_AWB_CFG:
		cfg = kzalloc(sizeof(*cfg), GFP_KERNEL);
		if (!cfg) {
			ret = -ENOMEM;
			return ret;
		}

		if (copy_from_user(cfg, up, sizeof(*cfg))) {
			kfree(cfg);
			return -EFAULT;
		}
		ret = pivariety_ioctl(sd, cmd, cfg);
		kfree(cfg);
		break;
	case RKMODULE_GET_CHANNEL_INFO:
		ch_info = kzalloc(sizeof(*ch_info), GFP_KERNEL);
		if (!ch_info) {
			ret = -ENOMEM;
			return ret;
		}

		ret = pivariety_ioctl(sd, cmd, ch_info);
		if (!ret) {
			ret = copy_to_user(up, ch_info, sizeof(*ch_info));
			if (ret)
				ret = -EFAULT;
		}
		kfree(ch_info);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}	
	return ret;
}
#endif

static const struct v4l2_subdev_core_ops pivariety_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	// .unsubscribe_event = v4l2_event_subdev_unsubscribe,
	.ioctl	= pivariety_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = pivariety_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops pivariety_video_ops = {
	.s_stream = pivariety_set_stream,
	.g_frame_interval = pivariety_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops pivariety_pad_ops = {
	.enum_mbus_code = pivariety_enum_mbus_code,
	.get_fmt = pivariety_get_fmt,
	.set_fmt = pivariety_set_fmt,
	.enum_frame_interval = pivariety_enum_frame_interval,
	.enum_frame_size = pivariety_enum_framesizes,
	.get_selection = pivariety_get_selection,
	.get_mbus_config = pivariety_get_mbus_config,
};

static const struct v4l2_subdev_ops pivariety_subdev_ops = {
	.core = &pivariety_core_ops,
	.video = &pivariety_video_ops,
	.pad = &pivariety_pad_ops,
};

static const struct v4l2_subdev_internal_ops pivariety_internal_ops = {
	.open = pivariety_open,
};

static void pivariety_free_controls(struct pivariety *pivariety)
{
	v4l2_ctrl_handler_free(pivariety->sd.ctrl_handler);
	mutex_destroy(&pivariety->mutex);
}

static int pivariety_get_length_of_set(struct pivariety *pivariety,
				       u16 idx_reg, u16 val_reg)
{
	int ret;
	int index = 0;
	u32 val;

	while (1) {
		ret = pivariety_write(pivariety, idx_reg, index);
		ret += pivariety_read(pivariety, val_reg, &val);

		if (ret < 0)
			return -1;

		if (val == NO_DATA_AVAILABLE)
			break;
		index++;
	}
	pivariety_write(pivariety, idx_reg, 0);
	return index;
}

static int pivariety_enum_resolution(struct pivariety *pivariety,
				     struct arducam_format *format)
{
	struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);
	int index = 0;
	u32 width, height;
	int num_resolution = 0;
	int ret;

	num_resolution = pivariety_get_length_of_set(pivariety,
						     RESOLUTION_INDEX_REG,
						     FORMAT_WIDTH_REG);
	if (num_resolution < 0)
		goto err;

	format->resolution_set = devm_kzalloc(&client->dev,
					      sizeof(*format->resolution_set) *
								num_resolution,
					      GFP_KERNEL);
	while (1) {
		ret = pivariety_write(pivariety, RESOLUTION_INDEX_REG, index);
		ret += pivariety_read(pivariety, FORMAT_WIDTH_REG, &width);
		ret += pivariety_read(pivariety, FORMAT_HEIGHT_REG, &height);

		if (ret < 0)
			goto err;

		if (width == NO_DATA_AVAILABLE || height == NO_DATA_AVAILABLE)
			break;

		format->resolution_set[index].width = width;
		format->resolution_set[index].height = height;

		index++;
	}

	format->num_resolution_set = index;
	pivariety_write(pivariety, RESOLUTION_INDEX_REG, 0);
	return 0;
err:
	return -ENODEV;
}

static int pivariety_enum_pixformat(struct pivariety *pivariety)
{
	int ret = 0;
	u32 mbus_code = 0;
	int pixfmt_type;
	int bayer_order;
	int bayer_order_not_volatile;
	int lanes;
	int index = 0;
	int num_pixformat = 0;
	struct arducam_format *arducam_fmt;
	struct i2c_client *client = v4l2_get_subdevdata(&pivariety->sd);

	num_pixformat = pivariety_get_length_of_set(pivariety,
						    PIXFORMAT_INDEX_REG,
						    PIXFORMAT_TYPE_REG);

	if (num_pixformat < 0)
		goto err;

	ret = pivariety_read(pivariety, FLIPS_DONT_CHANGE_ORDER_REG,
			     &bayer_order_not_volatile);
	if (bayer_order_not_volatile == NO_DATA_AVAILABLE)
		pivariety->bayer_order_volatile = 1;
	else
		pivariety->bayer_order_volatile = !bayer_order_not_volatile;

	if (ret < 0)
		goto err;

	pivariety->supported_formats =
		devm_kzalloc(&client->dev,
			     sizeof(*pivariety->supported_formats) *
								num_pixformat,
			     GFP_KERNEL);

	while (1) {
		ret = pivariety_write(pivariety, PIXFORMAT_INDEX_REG, index);
		ret += pivariety_read(pivariety, PIXFORMAT_TYPE_REG,
				      &pixfmt_type);

		if (pixfmt_type == NO_DATA_AVAILABLE)
			break;

		ret += pivariety_read(pivariety, MIPI_LANES_REG, &lanes);
		if (lanes == NO_DATA_AVAILABLE)
			break;

		ret += pivariety_read(pivariety, PIXFORMAT_ORDER_REG,
				      &bayer_order);
		if (ret < 0)
			goto err;

		mbus_code = data_type_to_mbus_code(pixfmt_type, bayer_order);
		arducam_fmt = &pivariety->supported_formats[index];
		arducam_fmt->index = index;
		arducam_fmt->mbus_code = mbus_code;
		arducam_fmt->bayer_order = bayer_order;
		arducam_fmt->data_type = pixfmt_type;
		if (pivariety_enum_resolution(pivariety, arducam_fmt))
			goto err;

		index++;
	}

	pivariety_write(pivariety, PIXFORMAT_INDEX_REG, 0);
	pivariety->num_supported_formats = index;
	pivariety->current_format_idx = 0;
	pivariety->current_resolution_idx = 0;
	pivariety->lanes = lanes;

	return 0;

err:
	return -ENODEV;
}

static const char *pivariety_ctrl_get_name(u32 id)
{
	switch (id) {
	case V4L2_CID_ARDUCAM_EXT_TRI:
		return "trigger_mode";
	case V4L2_CID_ARDUCAM_IRCUT:
		return "ircut";
	case V4L2_CID_ARDUCAM_STROBE_SHIFT:
		return "strobe_shift";
	case V4L2_CID_ARDUCAM_STROBE_WIDTH:
		return "strobe_width";
	case V4L2_CID_ARDUCAM_MODE:
		return "mode";
	default:
		return NULL;
	}
}

enum v4l2_ctrl_type pivariety_get_v4l2_ctrl_type(u32 id)
{
	switch (id) {
	case V4L2_CID_ARDUCAM_EXT_TRI:
		return V4L2_CTRL_TYPE_BOOLEAN;
	case V4L2_CID_ARDUCAM_IRCUT:
		return V4L2_CTRL_TYPE_BOOLEAN;
	default:
		return V4L2_CTRL_TYPE_INTEGER;
	}
}

static struct v4l2_ctrl *v4l2_ctrl_new_arducam(struct v4l2_ctrl_handler *hdl,
					       const struct v4l2_ctrl_ops *ops,
					       u32 id, s64 min, s64 max,
					       u64 step, s64 def)
{
	struct v4l2_ctrl_config ctrl_cfg = {
		.ops = ops,
		.id = id,
		.name = NULL,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = 0,
		.min = min,
		.max = max,
		.def = def,
		.step = step,
	};

	ctrl_cfg.name = pivariety_ctrl_get_name(id);
	ctrl_cfg.type = pivariety_get_v4l2_ctrl_type(id);

	return v4l2_ctrl_new_custom(hdl, &ctrl_cfg, NULL);
}

static int pivariety_enum_controls(struct pivariety *pivariety)
{
	struct v4l2_subdev *sd = &pivariety->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_ctrl_handler *ctrl_hdlr = &pivariety->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl **ctrl = pivariety->ctrls;
	int ret, index, num_ctrls;
	u32 id, min, max, def, step;

	num_ctrls = pivariety_get_length_of_set(pivariety, CTRL_INDEX_REG,
						CTRL_ID_REG);
	if (num_ctrls < 0)
		goto err;

	v4l2_dbg(1, debug, sd, "%s: num_ctrls = %d\n",
		 __func__, num_ctrls);

	ret = v4l2_ctrl_handler_init(ctrl_hdlr, num_ctrls);
	if (ret)
		return ret;

	index = 0;
	while (1) {
		ret = pivariety_write(pivariety, CTRL_INDEX_REG, index);
		pivariety_write(pivariety, CTRL_VALUE_REG, 0);
		wait_for_free(pivariety, 1);

		ret += pivariety_read(pivariety, CTRL_ID_REG, &id);
		ret += pivariety_read(pivariety, CTRL_MAX_REG, &max);
		ret += pivariety_read(pivariety, CTRL_MIN_REG, &min);
		ret += pivariety_read(pivariety, CTRL_DEF_REG, &def);
		ret += pivariety_read(pivariety, CTRL_STEP_REG, &step);
		if (ret < 0)
			goto err;

		if (id == NO_DATA_AVAILABLE || max == NO_DATA_AVAILABLE ||
		    min == NO_DATA_AVAILABLE || def == NO_DATA_AVAILABLE ||
		    step == NO_DATA_AVAILABLE)
			break;

		v4l2_dbg(1, debug, sd,
			 "%s: index = %d, id = 0x%x, max = %d, min = %d, def = %d, step = %d\n",
			 __func__, index, id, max, min, def, step);

		if (v4l2_ctrl_get_name(id)) {
			*ctrl = v4l2_ctrl_new_std(ctrl_hdlr,
						  &pivariety_ctrl_ops,
						  id, min,
						  max, step,
						  def);
			v4l2_dbg(1, debug, sd, "%s: ctrl: 0x%p\n",
				 __func__, *ctrl);
		} else if (pivariety_ctrl_get_name(id)) {
			*ctrl = v4l2_ctrl_new_arducam(ctrl_hdlr,
						      &pivariety_ctrl_ops,
						      id, min, max, step, def);

			v4l2_dbg(1, debug, sd,
				 "%s: new custom ctrl, ctrl: 0x%p.\n",
				 __func__, *ctrl);
		} else {
			index++;
			continue;
		}

		if (!*ctrl)
			goto err;

		switch (id) {
		case V4L2_CID_HFLIP:
			pivariety->hflip = *ctrl;
			if (pivariety->bayer_order_volatile)
				pivariety->hflip->flags |=
						V4L2_CTRL_FLAG_MODIFY_LAYOUT;
			break;

		case V4L2_CID_VFLIP:
			pivariety->vflip = *ctrl;
			if (pivariety->bayer_order_volatile)
				pivariety->vflip->flags |=
						V4L2_CTRL_FLAG_MODIFY_LAYOUT;
			break;

		case V4L2_CID_HBLANK:
			(*ctrl)->flags |= V4L2_CTRL_FLAG_READ_ONLY;
			break;
		}

		ctrl++;
		index++;
	}
	/* freq */
	v4l2_ctrl_new_int_menu(ctrl_hdlr, NULL, V4L2_CID_LINK_FREQ,
			       0, 0, link_freq_menu_items);

	pivariety_write(pivariety, CTRL_INDEX_REG, 0);

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto err;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr,
					      &pivariety_ctrl_ops,
					      &props);
	if (ret)
		goto err;

	pivariety->sd.ctrl_handler = ctrl_hdlr;
	v4l2_ctrl_handler_setup(ctrl_hdlr);
	return 0;
err:
	return -ENODEV;
}

static int pivariety_parse_dt(struct pivariety *pivariety, struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	/* Get CSI2 bus config */
	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	//pivariety->bus = ep_cfg.bus.mipi_csi2;

	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int pivariety_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct pivariety *pivariety;
	char facing[2];
	u32 device_id, firmware_version;
	int ret;

	dev_info(dev, "driver version: %02x.%02x.%02x",
		DRIVER_VERSION >> 16,
		(DRIVER_VERSION & 0xff00) >> 8,
		DRIVER_VERSION & 0x00ff);
	
	pivariety = devm_kzalloc(&client->dev, sizeof(*pivariety), GFP_KERNEL);
	if (!pivariety)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &pivariety->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &pivariety->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &pivariety->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &pivariety->len_name);
	/* Initialize subdev */
	v4l2_i2c_subdev_init(&pivariety->sd, client,
			     &pivariety_subdev_ops);

	if (pivariety_parse_dt(pivariety, dev))
		return -EINVAL;	

	/* Get system clock (xclk) */
	pivariety->xclk = devm_clk_get(dev, "xclk");
	if (IS_ERR(pivariety->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(pivariety->xclk);
	}

	pivariety->xclk_freq = clk_get_rate(pivariety->xclk);
	if (pivariety->xclk_freq != 24000000) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			pivariety->xclk_freq);
		return -EINVAL;
	}

	ret = pivariety_get_regulators(pivariety);
	if (ret)
		return ret;

	/* Request optional enable pin */
	pivariety->reset_gpio = devm_gpiod_get_optional(dev, "reset",
							GPIOD_OUT_HIGH);

	ret = pivariety_power_on(dev);
	if (ret)
		return ret;

	ret = pivariety_read(pivariety, DEVICE_ID_REG, &device_id);
	if (ret || device_id != DEVICE_ID) {
		dev_err(dev, "probe failed\n");
		ret = -ENODEV;
		goto error_power_off;
	}

	ret = pivariety_read(pivariety, DEVICE_VERSION_REG, &firmware_version);
	if (ret)
		dev_err(dev, "read firmware version failed\n");

	dev_info(dev, "firmware version: 0x%04X\n", firmware_version);

	if (pivariety_enum_pixformat(pivariety)) {
		dev_err(dev, "enum pixformat failed.\n");
		ret = -ENODEV;
		goto error_power_off;
	}

	if (pivariety_enum_controls(pivariety)) {
		dev_err(dev, "enum controls failed.\n");
		ret = -ENODEV;
		goto error_power_off;
	}
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	/* Initialize subdev */
	pivariety->sd.internal_ops = &pivariety_internal_ops;
	pivariety->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
					V4L2_SUBDEV_FL_HAS_EVENTS;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	/* Initialize source pad */
	pivariety->pad.flags = MEDIA_PAD_FL_SOURCE;
	pivariety->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ret = media_entity_pads_init(&pivariety->sd.entity, 1, &pivariety->pad);
	if (ret)
		goto error_handler_free;
#endif

	memset(facing, 0, sizeof(facing));
	if (strcmp(pivariety->module_facing, "back") == 0)
		facing[0] = 'b';
	else
		facing[0] = 'f';

	snprintf(pivariety->sd.name, sizeof(pivariety->sd.name), 
		 "m%02d_%s_%s %s", pivariety->module_index, facing,
		 PIVARIETY_NAME, dev_name(pivariety->sd.dev));
	ret = v4l2_async_register_subdev_sensor_common(&pivariety->sd);
	if (ret < 0)
		goto error_media_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&pivariety->sd.entity);

error_handler_free:
	pivariety_free_controls(pivariety);

error_power_off:
	pivariety_power_off(dev);

	return ret;
}

static int pivariety_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct pivariety *pivariety = to_pivariety(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	pivariety_free_controls(pivariety);

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	return 0;
}

static const struct dev_pm_ops pivariety_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pivariety_suspend, pivariety_resume)
	SET_RUNTIME_PM_OPS(pivariety_power_off, pivariety_power_on, NULL)
};

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id arducam_pivariety_dt_ids[] = {
	{ .compatible = "arducam,arducam-pivariety" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, arducam_pivariety_dt_ids);
#endif

// static const struct i2c_device_id arducam_pivariety_match_id[] = {
// 	{ "arducam,pivariety", 0 },
// 	{ },
// };

static struct i2c_driver arducam_pivariety_i2c_driver = {
	.driver = {
		.name = PIVARIETY_NAME,
		.of_match_table	= of_match_ptr(arducam_pivariety_dt_ids),
		.pm = &pivariety_pm_ops,
	},
	.probe = pivariety_probe,
	.remove = pivariety_remove,
	// .id_table = arducam_pivariety_match_id,
};

// module_i2c_driver(arducam_pivariety_i2c_driver);

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&arducam_pivariety_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&arducam_pivariety_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_AUTHOR("Lee Jackson <info@arducam.com>");
MODULE_DESCRIPTION("Arducam Pivariety v4l2 driver");
MODULE_LICENSE("GPL v2");
