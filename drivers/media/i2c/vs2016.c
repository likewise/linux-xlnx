#ifndef DEBUG
#define DEBUG 

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/media.h>
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/videodev2.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-image-sizes.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-mediabus.h>

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-2)");

#define DRIVER_NAME "VS2016"

struct vs2016_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_wb;
		struct v4l2_ctrl *blue_balance;
		struct v4l2_ctrl *red_balance;
	};
	struct {
		struct v4l2_ctrl *hflip;
		struct v4l2_ctrl *vflip;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *sharpness;
	struct v4l2_ctrl *light_freq;
};

struct vs2016_framesize {
	u16 width;
	u16 height;
	u16 max_exp_lines;
	const u8 *regs;
};

struct stvs2016 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	enum v4l2_mbus_type bus_type;
	struct i2c_client *client;
	struct vs2016_ctrls ctrls;
	const struct vs2016_framesize *frame_size;
	struct v4l2_mbus_framefmt format;
};

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct stvs2016, ctrls.handler)->sd;
}

static inline struct stvs2016 *to_vs2016(struct v4l2_subdev *sd)
{
	return container_of(sd, struct stvs2016, sd);
}


static int vs2016_g_volatile_ctrl (struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct stvs2016 *vs2016 = to_vs2016(sd);
	int ret = -EINVAL;

	v4l2_dbg(1, debug, sd, "g_ctrl: %s, value: %d. \n",
		 ctrl->name, ctrl->val);

	//pr_alert("%s : Enter \n", __func__);

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		pr_alert("%s : get V4L2_CID_AUTO_WHITE_BALANCE \n", __func__);
		break;

	case V4L2_CID_BRIGHTNESS:
		pr_alert("%s : get V4L2_CID_BRIGHTNESS \n", __func__);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		pr_alert("%s : get V4L2_CID_EXPOSURE_AUTO \n", __func__);
		break;

	case V4L2_CID_AUTOGAIN:
		pr_alert("%s : get V4L2_CID_AUTOGAIN \n", __func__);
		break;

	case V4L2_CID_HFLIP:
		pr_alert("%s : get V4L2_CID_HFLIP \n", __func__);
		break;

	case V4L2_CID_POWER_LINE_FREQUENCY:
		pr_alert("%s : get V4L2_CID_POWER_LINE_FREQUENCY \n ", __func__);
		break;

	case V4L2_CID_SATURATION:
		pr_alert("%s : get V4L2_CID_SATURATION \n", __func__);
		break;

	case V4L2_CID_SHARPNESS:
		pr_alert("%s : get V4L2_CID_SHARPNESS \n", __func__);
		break;

	case V4L2_CID_TEST_PATTERN:
		pr_alert("%s : get V4L2_CID_TEST_PATTERN \n", __func__);
		break;
	}

	return ret;
}

static int vs2016_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct stvs2016 *vs2016 = to_vs2016(sd);
	int ret = -EINVAL;

	v4l2_dbg(1, debug, sd, "s_ctrl: %s, value: %d.\n",
		 ctrl->name, ctrl->val);

	//pr_alert("%s : Enter \n", __func__);

	switch (ctrl->id) {
	case V4L2_CID_AUTO_WHITE_BALANCE:
		pr_alert("%s : set V4L2_CID_AUTO_WHITE_BALANCE \n", __func__);
		break;

	case V4L2_CID_BRIGHTNESS:
		pr_alert("%s : set V4L2_CID_BRIGHTNESS \n", __func__);
		break;

	case V4L2_CID_EXPOSURE_AUTO:
		pr_alert("%s : set V4L2_CID_EXPOSURE_AUTO \n", __func__);
		break;

	case V4L2_CID_AUTOGAIN:
		pr_alert("%s : set V4L2_CID_AUTOGAIN \n", __func__);
		break;

	case V4L2_CID_HFLIP:
		pr_alert("%s : set V4L2_CID_HFLIP \n", __func__);
		break;

	case V4L2_CID_POWER_LINE_FREQUENCY:
		pr_alert("%s : set V4L2_CID_POWER_LINE_FREQUENCY \n ", __func__);
		break;

	case V4L2_CID_SATURATION:
		pr_alert("%s : set V4L2_CID_SATURATION \n", __func__);
		break;

	case V4L2_CID_SHARPNESS:
		pr_alert("%s : set V4L2_CID_SHARPNESS \n", __func__);
		break;

	case V4L2_CID_TEST_PATTERN:
		pr_alert("%s : set V4L2_CID_TEST_PATTERN \n", __func__);
		break;
	}

	return ret;
}


static int vs2016_get_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct stvs2016 *vs2016 = to_vs2016(sd);
	struct v4l2_mbus_framefmt *mf;

	//pr_alert("%s : Enter \n", __func__);
#if 0
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		mf = v4l2_subdev_get_try_format(sd, cfg, 0);
		fmt->format = *mf;
		return 0;
	}
#endif
	fmt->format = vs2016->format;
	return 0;
}


static int vs2016_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct stvs2016 *vs2016 = to_vs2016(sd);
	int ret = 0;

	//pr_alert("%s : Enter \n", __func__);
	//pr_alert("%s : width = %d height = %d colorspace = %d field = %d code = %d \n", 
	//		__func__, mf->width, mf->height, mf->colorspace, mf->field, mf->code);

	vs2016->format = fmt->format;

	return ret;
}

static int vs2016_s_stream(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct stvs2016 *vs2016 = to_vs2016(sd);
	struct vs2016_ctrls *ctrls = &vs2016->ctrls;
	int ret = 0;

	//pr_alert("%s : %s \n", __func__, on ? "Stream Start" : "Stream Stop");

	return 0;
}

static int vs2016_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *mf = v4l2_subdev_get_try_format(sd, fh->pad, 0);

	//pr_alert("%s : Enter \n", __func__);
	return 0;
}

static int vs2016_close(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_mbus_framefmt *mf = v4l2_subdev_get_try_format(sd, fh->pad, 0);

	//pr_alert("%s : Enter \n", __func__);
	return 0;
}
/* -----------------------------------------------------------------------------
 * Media Operations
 */

static const struct media_entity_operations vs2016_media_ops = {
	.link_validate = v4l2_subdev_link_validate
};



static const struct v4l2_subdev_pad_ops vs2016_pad_ops = {
	/*.enum_mbus_code = vs2016_enum_mbus_code,
	.enum_frame_size = vs2016_enum_frame_sizes, */
	.get_fmt = vs2016_get_fmt,
	.set_fmt = vs2016_set_fmt,
};

static const struct v4l2_subdev_video_ops vs2016_video_ops = {
	.s_stream = vs2016_s_stream,
	/*.g_frame_interval = vs2016_g_frame_interval,
	.s_frame_interval = vs2016_s_frame_interval, */

};

static const struct v4l2_subdev_internal_ops vs2016_sd_internal_ops = {
	.open = vs2016_open,
	.close = vs2016_close
};

static const struct v4l2_subdev_core_ops vs2016_core_ops = {
	/* .s_power = vs2016_s_power,
	.log_status = v4l2_ctrl_subdev_log_status,
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,*/
};

static const struct v4l2_subdev_ops vs2016_subdev_ops = {
	.core = &vs2016_core_ops,
	.pad = &vs2016_pad_ops,
	.video = &vs2016_video_ops,
};

static const struct v4l2_ctrl_ops vs2016_ctrl_ops = {
	.g_volatile_ctrl = vs2016_g_volatile_ctrl,
	.s_ctrl	= vs2016_s_ctrl,
};

static const struct vs2016_framesize vs2016_framesizes[] = {
	{
		.width		= 1920,
		.height		= 1080,
	}
};


static int vs2016_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct v4l2_subdev *sd;
	struct stvs2016 *vs2016;
	int ret;

	pr_alert("%s : Enter \n", __func__);

	vs2016 = devm_kzalloc(&client->dev, sizeof(*vs2016), GFP_KERNEL);
	if (!vs2016) {
		pr_alert("%s : No mem \n", __func__);
		return -ENOMEM;
	}

	vs2016->client = client;

	sd = &vs2016->sd;
	v4l2_i2c_subdev_init(sd, client, &vs2016_subdev_ops);
	strlcpy(sd->name, DRIVER_NAME, sizeof(sd->name));

	//v4l2_set_subdevdata(sd, vs2016);

	sd->internal_ops = &vs2016_sd_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
		     V4L2_SUBDEV_FL_HAS_EVENTS;

	vs2016->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	//sd->entity.ops = &vs2016_media_ops;

	ret = media_entity_pads_init(&sd->entity, 1, &vs2016->pad);
	if (ret < 0) {
		pr_alert("%s : media_entity_init failed %d \n", __func__, ret);
		return ret;
	}

	ret = v4l2_ctrl_handler_init(&vs2016->ctrls.handler, 1);
	if (ret < 0) {
		pr_alert("%s : ctrl handler init failed \n", __func__);
		goto err_me;
	}

	vs2016->ctrls.auto_wb = v4l2_ctrl_new_std(&vs2016->ctrls.handler, &vs2016_ctrl_ops,
					V4L2_CID_AUTO_WHITE_BALANCE, 0, 1, 1, 1);

	vs2016->ctrls.auto_wb->flags |= V4L2_CTRL_FLAG_VOLATILE;
	/* Skipping stuff */
	vs2016->sd.ctrl_handler = &vs2016->ctrls.handler;


	vs2016->format.width = vs2016_framesizes[0].width;
	vs2016->format.height = vs2016_framesizes[0].height;
	vs2016->format.field = V4L2_FIELD_NONE;
	vs2016->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
	vs2016->format.colorspace = V4L2_COLORSPACE_SRGB;

	ret = v4l2_async_register_subdev(sd);
	if (ret < 0) {
		pr_alert("%s : v4l2_async_register_subdev failed %d \n", __func__, ret);
		return ret;
	}

	pr_alert("%s : VS2016 probe Success Done \n", __func__ );
	return 0;

err_ctrls:
	v4l2_ctrl_handler_free(sd->ctrl_handler);
err_me:
	media_entity_cleanup(&sd->entity);
	return ret;
}

static int vs2016_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	pr_alert("%s : Enter \n", __func__);

	v4l2_device_unregister_subdev(sd);
	v4l2_ctrl_handler_free(sd->ctrl_handler);
	media_entity_cleanup(&sd->entity);

	return 0;
}

static const struct of_device_id vs2016_of_id_table[] = {
	{ .compatible = "vs,vs2016" },
	{ }
};
MODULE_DEVICE_TABLE(of, vs2016_of_id_table);


static const struct i2c_device_id vs2016_id[] = {
	{ "VS2016", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, vs2016_id);


static struct i2c_driver vs2016_i2c_driver = {
	.driver = {
		.name	= DRIVER_NAME,
		.of_match_table	= vs2016_of_id_table,
	},
	.probe		= vs2016_probe,
	.remove		= vs2016_remove,
	.id_table	= vs2016_id,
};

module_i2c_driver(vs2016_i2c_driver);

MODULE_AUTHOR("Vishal Sagar <vsagar@xilinx.com>");
MODULE_DESCRIPTION("Dummy driver based on OV9650/OV9652 CMOS Image Sensor driver");
MODULE_LICENSE("GPL");

#undef DEBUG
#endif
