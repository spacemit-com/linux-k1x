// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2023 Spacemit Co., Ltd.
 *
 */

#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/hdmi.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/component.h>
#include <linux/pm_runtime.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_of.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>

#include "spacemit_hdmi.h"
#include "spacemit_lib.h"
#include "spacemit_dpu.h"

#define SPACEMIT_HDMI_DDC_OTHER_MASK        BIT(31)
#define SPACEMIT_HDMI_DDC_DONE_MASK         BIT(30)
#define SPACEMIT_HDMI_HPD_IQR_MASK          BIT(29)

#define SPACEMIT_HDMI_DDC_NACK              BIT(15)
#define SPACEMIT_HDMI_DDC_DONE              BIT(14)
#define SPACEMIT_HDMI_HPD_IQR               BIT(13)

#define SPACEMIT_HDMI_HPD_STATUS            BIT(12)

#define SPACEMIT_HDMI_PHY_STATUS            0xC


struct hdmi_data_info {
	uint8_t edid[256];
	int vic;
	bool sink_has_audio;
	unsigned int enc_in_format;
	unsigned int enc_out_format;
	unsigned int colorimetry;
};

struct spacemit_hdmi_i2c {
	u8 segment_addr;

	struct mutex lock;
	struct completion cmp;
};

struct spacemit_hdmi {
	struct device *dev;
	struct drm_device *drm_dev;

	int irq;
	struct clk *pclk;
	void __iomem *regs;

	struct drm_connector	connector;
	struct drm_encoder encoder;

	struct reset_control *hdmi_reset;
	struct clk *hdmi_mclk;

	unsigned int tmds_rate;

	struct mutex lock;
	bool suspended;
	struct hdmi_data_info *hdmi_data;
	struct drm_display_mode previous_mode;
};

#define encoder_to_spacemit_hdmi(encoder) \
	container_of(encoder, struct spacemit_hdmi, encoder)

#define connector_to_spacemit_hdmi(connector) \
	container_of(connector, struct spacemit_hdmi, connector)

static inline u32 hdmi_readb(struct spacemit_hdmi *hdmi, u16 offset)
{
	return readl_relaxed(hdmi->regs + (offset));
}

static inline void hdmi_writeb(struct spacemit_hdmi *hdmi, u16 offset, u32 val)
{
	writel_relaxed(val, hdmi->regs + (offset));
}

static int hdmi_get_plug_in_status(struct spacemit_hdmi *hdmi)
{
	u32 value;

	value = readl_relaxed(hdmi->regs + SPACEMIT_HDMI_PHY_STATUS) & SPACEMIT_HDMI_HPD_STATUS;

	return !!value;
}

static void spacemit_hdmi_set_pwr_mode(struct spacemit_hdmi *hdmi, int mode)
{
	//normal/ low power
}

static void spacemit_hdmi_reset(struct spacemit_hdmi *hdmi)
{
}

static int spacemit_hdmi_config_video_vsi(struct spacemit_hdmi *hdmi,
				      struct drm_display_mode *mode)
{
	union hdmi_infoframe frame;
	int rc;

	rc = drm_hdmi_vendor_infoframe_from_display_mode(&frame.vendor.hdmi,
							 &hdmi->connector,
							 mode);

	return 0;
}

static int spacemit_hdmi_upload_frame(struct spacemit_hdmi *hdmi, int setup_rc,
				  union hdmi_infoframe *frame, u32 frame_index,
				  u32 mask, u32 disable, u32 enable)
{
	if (setup_rc >= 0) {
		u8 packed_frame[0x11];
		ssize_t rc;

		rc = hdmi_infoframe_pack(frame, packed_frame,
					 sizeof(packed_frame));
		if (rc < 0)
			return rc;
	}

	return setup_rc;
}

static int spacemit_hdmi_config_video_avi(struct spacemit_hdmi *hdmi,
				      struct drm_display_mode *mode)
{
	union hdmi_infoframe frame;
	int rc;

	rc = drm_hdmi_avi_infoframe_from_display_mode(&frame.avi,
						      &hdmi->connector,
						      mode);

	if (hdmi->hdmi_data->enc_out_format == HDMI_COLORSPACE_YUV444)
		frame.avi.colorspace = HDMI_COLORSPACE_YUV444;
	else if (hdmi->hdmi_data->enc_out_format == HDMI_COLORSPACE_YUV422)
		frame.avi.colorspace = HDMI_COLORSPACE_YUV422;
	else
		frame.avi.colorspace = HDMI_COLORSPACE_RGB;

	return spacemit_hdmi_upload_frame(hdmi, rc, &frame, INFOFRAME_AVI, 0, 0, 0);
}

static int spacemit_hdmi_config_video_timing(struct spacemit_hdmi *hdmi,
					 struct drm_display_mode *mode)
{
	return 0;
}

enum bit_depth {
	EIGHT_BPP = 0,
	TEN_BPP = 1,
	TWELVE_BPP = 2,
};

int power_of_two(int n)
{
	int result = 1;

	for (int i = 0; i < n; ++i)
		result <<= 1;

	return result;
}

int pll8_bit_5_6(int bit_clock, int n)
{
	int ret = 0;

	bit_clock = bit_clock / n;

	if (bit_clock < 425)
		ret = 3;
	else if (bit_clock < 850)
		ret = 2;
	else if (bit_clock < 1700)
		ret = 1;
	else
		ret = 0;

	return ret;
}

int pll6_bit_4_5(int bit_clock, int n)
{
	int ret = 0;

	bit_clock = bit_clock / n;

	if (bit_clock <= 337)
		ret = 0;
	else if (bit_clock < 425)
		ret = 1;
	else if (bit_clock < 675)
		ret = 0;
	else if (bit_clock < 850)
		ret = 1;
	else if (bit_clock < 1350)
		ret = 0;
	else if (bit_clock < 1700)
		ret = 1;
	else
		ret = 0;

	return ret;
}

int pll5_bit_0_2(int bit_clock, int n)
{
	int value =  bit_clock * power_of_two(pll8_bit_5_6(bit_clock, n)) / n;
	int ret;

	if (value < 1830)
		ret = 0;
	else if (value < 2030)
		ret = 1;
	else if (value < 2275)
		ret = 2;
	else if (value < 2520)
		ret = 3;
	else if (value < 2765)
		ret = 4;
	else if (value < 3015)
		ret = 5;
	else if (value < 3260)
		ret = 6;
	else
		ret = 7;

	return ret;
}

int PLL9_BIT0_1[3] = {0x0, 0x1, 0x2};

void pll_reg_cal(int bit_clock, int ref_clock, int n, int *integer_part, u32 *hmdi_e8_reg)
{
	long long int_para = 1000000000;
	long long value = (power_of_two(pll8_bit_5_6(bit_clock, n))) * bit_clock * int_para / (n * (pll6_bit_4_5(bit_clock, n) + 1) * ref_clock);
	long long integer = (power_of_two(pll8_bit_5_6(bit_clock, n))) * bit_clock / (n * (pll6_bit_4_5(bit_clock, n) + 1) * ref_clock) * int_para;
	long long fraction = value - integer;
	bool negative = false;
	int bit = 0;
	int frac_20bit = 0;
	int pll2_reg = 0;
	int pll1_reg = 0;
	int pll0_reg = 0;

	negative =  fraction > 500000000 ? true : false;
	fraction = negative ? 1000000000 - fraction : fraction;
	*integer_part = negative ? integer/int_para + 1 : integer/int_para;


	for (int i = 0; i < 20; i++) {
		if (bit >= int_para) {
			frac_20bit |= 1 << (19 - i);
			fraction -= int_para;
		}
		fraction *= 2;
		bit = fraction;
	}

	if (!negative) {
		pll2_reg = ((frac_20bit & 0xF0000) >> 16) | (1 << 5);
	} else {
		frac_20bit = (~frac_20bit + 1) & 0xfffff;
		pll2_reg = 0x10 | ((frac_20bit & 0xF0000) >> 16) | (1 << 5);
	}

	pll1_reg = (frac_20bit & 0xFF00) >> 8;
	pll0_reg = frac_20bit & 0xFF;
	*hmdi_e8_reg = (0x20 << 24) | (pll2_reg << 16) | (pll1_reg << 8) | pll0_reg;
}

int pll_reg(struct spacemit_hdmi *hdmi, int pixel_clock, int bit_depth)
{
	int pll9_reg = 0, pll8_reg = 0, pll7_reg = 0, pll6_reg = 0, pll5_reg = 0, pll4_reg = 0;
	int n = 100;
	int ref_clock = 24;
	int hdmi_ec_reg = 0;
	int hdmi_f0_reg = 0;
	int hdmi_e8_reg = 0;
	int pow = 0;
	int bit_clock = bit_depth == EIGHT_BPP ? pixel_clock : pixel_clock * 125 / 100;

	int integer_part = 0;

	DRM_DEBUG("%s()\n", __func__);

	pll_reg_cal(bit_clock, ref_clock, n, &integer_part, &hdmi_e8_reg);

	pll9_reg = 2 << 2 | PLL9_BIT0_1[bit_depth];
	pll8_reg = (0 << 7) | (pll8_bit_5_6(bit_clock, n) << 5) | 1;
	pll7_reg = 0x50;
	pll6_reg = 0xD | (pll6_bit_4_5(bit_clock, n) << 4) | (2 << 6);
	pll5_reg = 0x40 | pll5_bit_0_2(bit_clock, n);

	pow = (pll8_bit_5_6(bit_clock, n));

	pll4_reg = integer_part;

	hdmi_ec_reg = (pll7_reg << 24) | (pll6_reg << 16) | (pll5_reg << 8) | pll4_reg;
	hdmi_f0_reg = (pll9_reg << 8) | pll8_reg;

	writel(hdmi_e8_reg, hdmi->regs + 0xe8);
	DRM_DEBUG("%s() hdmi 0xe8 0x%x\n", __func__, hdmi_e8_reg);

	writel(hdmi_ec_reg, hdmi->regs + 0xec);
	DRM_DEBUG("%s() hdmi 0xec 0x%x\n", __func__, hdmi_ec_reg);

	writel(hdmi_f0_reg, hdmi->regs + 0xf0);
	DRM_DEBUG("%s() hdmi 0xf0 0x%x\n", __func__, hdmi_f0_reg);

	return 0;
}

void hdmi_write_bits(struct spacemit_hdmi *hdmi, u16 offset, u32 value, u32 mask, u32 shifts)
{
	u32 reg_val;

	reg_val = readl_relaxed(hdmi->regs + (offset));
	reg_val &= ~(mask << shifts);
	reg_val |= (value << shifts);
	writel_relaxed(reg_val, hdmi->regs + (offset));
}

void hdmi_init(struct spacemit_hdmi *hdmi, int pixel_clock, int bit_depth)
{
	u32 value = 0;
	int color_depth = bit_depth == EIGHT_BPP ? 4 : 5;

	u32 good_phase = 0x00;
	u32 bias_current = 0x01;
	u32 bias_risistor = 0x07;

	DRM_DEBUG("%s()\n", __func__);

	writel(0xAE5C410F, hdmi->regs + 0xe0);
	hdmi_write_bits(hdmi, 0xe0, bias_current, 0x03, 29);
	hdmi_write_bits(hdmi, 0xe0, bias_risistor, 0x0F, 18);
	hdmi_write_bits(hdmi, 0xe0, good_phase, 0x03, 14);
	// writel(0xEE40410F, hdmi->regs + 0xe0);
	// value = readl_relaxed(hdmi->regs + 0xe0);
	// DRM_DEBUG("%s() hdmi 0xe0 0x%x\n", __func__, value);

	value = 0x0000000d | (color_depth << 4);
	writel(value, hdmi->regs + 0x34);
	DRM_DEBUG("%s() hdmi 0x34 0x%x\n", __func__, value);

	pll_reg(hdmi, pixel_clock, bit_depth);
	writel(0x03, hdmi->regs + 0xe4);
	value = readl_relaxed(hdmi->regs + 0xe4);
	DRM_DEBUG("%s() hdmi pll lock status 0x%x\n", __func__, value);
	// while ( (value & 0x10000) != 0) {
	// 	value = readl_relaxed(hdmi->regs + 0xe4);
	// }
	udelay(100);

	// value = 0x3018C000 | bit_depth;
	value = 0x1C208000 | bit_depth;
	writel(value, hdmi->regs + 0x28);
	DRM_DEBUG("%s() hdmi 0x28 0x%x\n", __func__, value);
}

static int spacemit_hdmi_setup(struct spacemit_hdmi *hdmi,
			   struct drm_display_mode *mode)
{
	void __iomem *ciu = (void __iomem *)ioremap(0xD4282C00, 0x200);
	struct hdmi_data_info *hdmi_data = hdmi->hdmi_data;
	int bit_depth = TEN_BPP;

	DRM_DEBUG("%s() \n", __func__);

	hdmi_init(hdmi, hdmi->previous_mode.clock, bit_depth);

	spacemit_hdmi_config_video_timing(hdmi, mode);
	spacemit_hdmi_config_video_avi(hdmi, mode);
	spacemit_hdmi_config_video_vsi(hdmi, mode);

	iounmap(ciu);

	return 0;
}

static void spacemit_hdmi_encoder_mode_set(struct drm_encoder *encoder,
				       struct drm_display_mode *mode,
				       struct drm_display_mode *adj_mode)
{
	struct spacemit_hdmi *hdmi = encoder_to_spacemit_hdmi(encoder);
	DRM_DEBUG("%s() \n", __func__);

	/* Store the display mode for plugin/DPMS poweron events */
	drm_mode_copy(&hdmi->previous_mode, adj_mode);
}

static void spacemit_hdmi_encoder_enable(struct drm_encoder *encoder)
{
	struct spacemit_hdmi *hdmi = encoder_to_spacemit_hdmi(encoder);
	DRM_INFO("%s()\n", __func__);

	spacemit_hdmi_set_pwr_mode(hdmi, NORMAL);
	spacemit_hdmi_setup(hdmi, &hdmi->previous_mode);
}

static void spacemit_hdmi_encoder_disable(struct drm_encoder *encoder)
{
	struct spacemit_hdmi *hdmi = encoder_to_spacemit_hdmi(encoder);
	struct spacemit_dpu *dpu = crtc_to_dpu(encoder->crtc);
	DRM_INFO("%s()\n", __func__);

	spacemit_dpu_stop(dpu);
	writel(0x00, hdmi->regs + 0xe4);
	udelay(100);
	spacemit_hdmi_set_pwr_mode(hdmi, LOWER_PWR);
}

static bool spacemit_hdmi_encoder_mode_fixup(struct drm_encoder *encoder,
					 const struct drm_display_mode *mode,
					 struct drm_display_mode *adj_mode)
{
	return true;
}

static int
spacemit_hdmi_encoder_atomic_check(struct drm_encoder *encoder,
			       struct drm_crtc_state *crtc_state,
			       struct drm_connector_state *conn_state)
{
	DRM_DEBUG("%s()\n", __func__);
	return 0;
}

static struct drm_encoder_helper_funcs spacemit_hdmi_encoder_helper_funcs = {
	.enable     = spacemit_hdmi_encoder_enable,
	.disable    = spacemit_hdmi_encoder_disable,
	.mode_fixup = spacemit_hdmi_encoder_mode_fixup,
	.mode_set   = spacemit_hdmi_encoder_mode_set,
	.atomic_check = spacemit_hdmi_encoder_atomic_check,
};

/* spacemit_hdmi_register_client - register a client notifier */
int spacemit_hdmi_register_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&hdmi_notifier_list, nb);
}
EXPORT_SYMBOL(spacemit_hdmi_register_client);

/* spacemit_hdmi_unregister_client - unregister a client notifier */
int spacemit_hdmi_unregister_client(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&hdmi_notifier_list, nb);
}
EXPORT_SYMBOL(spacemit_hdmi_unregister_client);

/* spacemit_hdmi_notifier_call_chain - notify clients of hdmi status events */
int spacemit_hdmi_notifier_call_chain(unsigned long val, void *v)
{
	return blocking_notifier_call_chain(&hdmi_notifier_list, val, v);
}
EXPORT_SYMBOL_GPL(spacemit_hdmi_notifier_call_chain);

static enum drm_connector_status
spacemit_hdmi_connector_detect(struct drm_connector *connector, bool force)
{
	struct spacemit_hdmi *hdmi = connector_to_spacemit_hdmi(connector);
	int ret;
	enum drm_connector_status status;

	DRM_DEBUG("%s() \n", __func__);

	mutex_lock(&hdmi->lock);
	if (hdmi->suspended) {
		DRM_DEBUG("%s() hdmi is suspended\n", __func__);
		mutex_unlock(&hdmi->lock);
		return connector_status_disconnected;
	}
	mutex_unlock(&hdmi->lock);

	ret = pm_runtime_get_sync(hdmi->dev);
	if (ret < 0) {
		DRM_INFO("%s() pm_runtime_get_sync failed\n", __func__);
		return connector_status_connected;
	}

	if (hdmi_get_plug_in_status(hdmi)) {
		mdelay(2);
		if (hdmi_get_plug_in_status(hdmi)) {
			DRM_INFO("%s() hdmi status connected\n", __func__);
			spacemit_hdmi_notifier_call_chain(DRM_HDMI_EVENT_CONNECTED, "status");
			status = connector_status_connected;
		} else {
			DRM_INFO("%s() hdmi status disconnected\n", __func__);
			spacemit_hdmi_notifier_call_chain(DRM_HDMI_EVENT_DISCONNECTED, "status");
			status = connector_status_disconnected;
		}
	} else {
		DRM_INFO("%s() hdmi status disconnected\n", __func__);
		spacemit_hdmi_notifier_call_chain(DRM_HDMI_EVENT_DISCONNECTED, "status");
		status = connector_status_disconnected;
	}

	pm_runtime_put(hdmi->dev);

	return status;
}

static int spacemit_hdmi_connector_get_modes(struct drm_connector *connector)
{
	DRM_DEBUG("%s() \n", __func__);

	return drm_add_modes_noedid(connector, 1920, 1080);
}

static enum drm_mode_status
spacemit_hdmi_connector_mode_valid(struct drm_connector *connector,
			       struct drm_display_mode *mode)
{
	return MODE_OK;
}

static int
spacemit_hdmi_probe_single_connector_modes(struct drm_connector *connector,
				       uint32_t maxX, uint32_t maxY)
{
	return drm_helper_probe_single_connector_modes(connector, 1920, 1080);
}

static void spacemit_hdmi_connector_destroy(struct drm_connector *connector)
{
	struct spacemit_hdmi *hdmi = connector_to_spacemit_hdmi(connector);

	kfree(hdmi->hdmi_data);
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs spacemit_hdmi_connector_funcs = {
	.fill_modes = spacemit_hdmi_probe_single_connector_modes,
	.detect = spacemit_hdmi_connector_detect,
	.destroy = spacemit_hdmi_connector_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_connector_helper_funcs spacemit_hdmi_connector_helper_funcs = {
	.get_modes = spacemit_hdmi_connector_get_modes,
	.mode_valid = spacemit_hdmi_connector_mode_valid,
};

static int spacemit_hdmi_register(struct drm_device *drm, struct spacemit_hdmi *hdmi)
{
	struct drm_encoder *encoder = &hdmi->encoder;
	struct device *dev = hdmi->dev;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm, dev->of_node);

	/*
	 * If we failed to find the CRTC(s) which this encoder is
	 * supposed to be connected to, it's because the CRTC has
	 * not been registered yet.  Defer probing, and hope that
	 * the required CRTC is added later.
	 */
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	drm_encoder_helper_add(encoder, &spacemit_hdmi_encoder_helper_funcs);
	drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_TMDS);

	hdmi->connector.polled = DRM_CONNECTOR_POLL_HPD;

	drm_connector_helper_add(&hdmi->connector,
				 &spacemit_hdmi_connector_helper_funcs);

	drm_connector_init(drm, &hdmi->connector,
				 &spacemit_hdmi_connector_funcs,
				 DRM_MODE_CONNECTOR_HDMIA);

	drm_connector_attach_encoder(&hdmi->connector, encoder);

	return 0;
}

static irqreturn_t spacemit_hdmi_hardirq(int irq, void *dev_id)
{
	struct spacemit_hdmi *hdmi = dev_id;
	irqreturn_t ret = IRQ_NONE;
	uint32_t value;

	value = hdmi_readb(hdmi, SPACEMIT_HDMI_PHY_STATUS);
	if (value & SPACEMIT_HDMI_HPD_IQR) {
		value |= SPACEMIT_HDMI_HPD_IQR;
		hdmi_writeb(hdmi, 0xc, value);
		ret = IRQ_WAKE_THREAD;
	}

	return ret;
}

static irqreturn_t spacemit_hdmi_irq(int irq, void *dev_id)
{
	struct spacemit_hdmi *hdmi = dev_id;

	drm_helper_hpd_irq_event(hdmi->connector.dev);

	return IRQ_HANDLED;
}

static int spacemit_hdmi_bind(struct device *dev, struct device *master,
				 void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = data;
	struct spacemit_hdmi *hdmi;
	int irq;
	int ret;

	DRM_DEBUG("%s() \n", __func__);

	hdmi = devm_kzalloc(dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi)
		return -ENOMEM;

	hdmi->hdmi_data = devm_kzalloc(dev, sizeof(*(hdmi->hdmi_data)), GFP_KERNEL);
	if (!hdmi->hdmi_data)
		return -ENOMEM;

	hdmi->dev = dev;
	hdmi->drm_dev = drm;

	hdmi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(hdmi->regs))
		return PTR_ERR(hdmi->regs);

	hdmi->hdmi_reset = devm_reset_control_get_optional_shared(&pdev->dev, "hdmi_reset");
	if (IS_ERR_OR_NULL(hdmi->hdmi_reset))
		DRM_INFO("Failed to found hdmi_reset\n");

	hdmi->hdmi_mclk = of_clk_get_by_name(dev->of_node, "hmclk");
	if (IS_ERR(hdmi->hdmi_mclk))
		DRM_INFO("Failed to found hdmi mclk\n");

	dev_set_drvdata(dev, hdmi);

	pm_runtime_enable(&pdev->dev);

	if (!IS_ERR_OR_NULL(hdmi->hdmi_reset)) {
		ret = reset_control_deassert(hdmi->hdmi_reset);
		if (ret < 0)
			DRM_INFO("Failed to deassert hdmi_reset\n");
	}

	pm_runtime_get_sync(&pdev->dev);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		DRM_ERROR("%s() get hdmi phd irq failed\n", __func__);
		return -ENOENT;
	}
	DRM_DEBUG("%s() hdmi hpd irq %d\n", __func__, irq);

	spacemit_hdmi_reset(hdmi);

	hdmi->suspended = false;
	mutex_init(&hdmi->lock);

	ret = spacemit_hdmi_register(drm, hdmi);

	ret = devm_request_threaded_irq(dev, irq, spacemit_hdmi_hardirq,
					spacemit_hdmi_irq, IRQF_SHARED,
					dev_name(dev), hdmi);
	if (ret < 0)
		goto irq_err;

	return 0;

irq_err:
	hdmi->connector.funcs->destroy(&hdmi->connector);
	hdmi->encoder.funcs->destroy(&hdmi->encoder);

	return ret;
}

static void spacemit_hdmi_unbind(struct device *dev, struct device *master,
			     void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct spacemit_hdmi *hdmi = dev_get_drvdata(dev);
	int ret;

	DRM_DEBUG("%s() \n", __func__);

	hdmi->connector.funcs->destroy(&hdmi->connector);
	hdmi->encoder.funcs->destroy(&hdmi->encoder);

	mutex_destroy(&hdmi->lock);

	pm_runtime_put_sync(&pdev->dev);
	if (!IS_ERR_OR_NULL(hdmi->hdmi_reset)) {
		ret = reset_control_assert(hdmi->hdmi_reset);
		if (ret < 0)
			DRM_INFO("Failed to assert hdmi_reset\n");
	}
	pm_runtime_disable(dev);
}

static const struct component_ops spacemit_hdmi_ops = {
	.bind	= spacemit_hdmi_bind,
	.unbind	= spacemit_hdmi_unbind,
};

static int spacemit_hdmi_probe(struct platform_device *pdev)
{
	DRM_DEBUG("%s() \n", __func__);

	return component_add(&pdev->dev, &spacemit_hdmi_ops);
}

static void spacemit_hdmi_remove(struct platform_device *pdev)
{
	DRM_DEBUG("%s() \n", __func__);

	component_del(&pdev->dev, &spacemit_hdmi_ops);
}

static int hdmi_rt_pm_resume(struct device *dev)
{
	struct spacemit_hdmi *hdmi = dev_get_drvdata(dev);
	uint64_t clk_val;

	DRM_DEBUG("%s()\n", __func__);

	clk_prepare_enable(hdmi->hdmi_mclk);

	clk_val = clk_get_rate(hdmi->hdmi_mclk);
	DRM_DEBUG("get hdmi mclk=%lld\n", clk_val);
	if (clk_val != DPU_MCLK_DEFAULT) {
		clk_val = clk_round_rate(hdmi->hdmi_mclk, DPU_MCLK_DEFAULT);
		clk_set_rate(hdmi->hdmi_mclk, clk_val);
		DRM_INFO("set hdmi mclk=%lld\n", clk_val);
	}

	return 0;
}

static int hdmi_rt_pm_suspend(struct device *dev)
{
	struct spacemit_hdmi *hdmi = dev_get_drvdata(dev);

	DRM_DEBUG("%s()\n", __func__);

	clk_disable_unprepare(hdmi->hdmi_mclk);

	return 0;
}

#ifdef CONFIG_PM_SLEEP

static int hdmi_drv_pm_suspend(struct device *dev)
{
	struct spacemit_hdmi *hdmi = dev_get_drvdata(dev);
	uint32_t value;

	DRM_DEBUG("%s()\n", __func__);

	mutex_lock(&hdmi->lock);
	hdmi->suspended = true;

	value = hdmi_readb(hdmi, SPACEMIT_HDMI_PHY_STATUS);
	value &= (~SPACEMIT_HDMI_HPD_IQR_MASK);
	value |= SPACEMIT_HDMI_HPD_IQR;
	hdmi_writeb(hdmi, SPACEMIT_HDMI_PHY_STATUS, value);
	udelay(5);

	clk_disable_unprepare(hdmi->hdmi_mclk);
	mutex_unlock(&hdmi->lock);

	return 0;
}

static int hdmi_drv_pm_resume(struct device *dev)
{
	struct spacemit_hdmi *hdmi = dev_get_drvdata(dev);
	uint32_t value;

	DRM_DEBUG("%s()\n", __func__);

	mutex_lock(&hdmi->lock);
	clk_prepare_enable(hdmi->hdmi_mclk);
	udelay(5);

	value = hdmi_readb(hdmi, SPACEMIT_HDMI_PHY_STATUS);
	value |= SPACEMIT_HDMI_HPD_IQR_MASK;
	hdmi_writeb(hdmi, SPACEMIT_HDMI_PHY_STATUS, value);

	hdmi->suspended = false;
	mutex_unlock(&hdmi->lock);

	if (hdmi_get_plug_in_status(hdmi))
		drm_helper_hpd_irq_event(hdmi->connector.dev);

	return 0;
}

#endif

static const struct dev_pm_ops hdmi_pm_ops = {
	SET_RUNTIME_PM_OPS(hdmi_rt_pm_suspend,
			hdmi_rt_pm_resume,
			NULL)
	SET_SYSTEM_SLEEP_PM_OPS(hdmi_drv_pm_suspend,
				hdmi_drv_pm_resume)
};

static const struct of_device_id spacemit_hdmi_dt_ids[] = {
	{ .compatible = "spacemit,hdmi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, spacemit_hdmi_dt_ids);

struct platform_driver spacemit_hdmi_driver = {
	.probe  = spacemit_hdmi_probe,
	.remove = spacemit_hdmi_remove,
	.driver = {
		.name = "spacemit-hdmi-drv",
		.of_match_table = spacemit_hdmi_dt_ids,
		.pm = &hdmi_pm_ops,
	},
};

// module_platform_driver(spacemit_hdmi_driver);

static int spacemit_hdmi_driver_init(void)
{
       return platform_driver_register(&spacemit_hdmi_driver);
}
late_initcall(spacemit_hdmi_driver_init);

MODULE_DESCRIPTION("Spacemit HDMI Driver");
MODULE_LICENSE("GPL v2");
