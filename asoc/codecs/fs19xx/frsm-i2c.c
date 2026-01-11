// SPDX-License-Identifier: GPL-2.0+
/**
 * Copyright (C) Shanghai FourSemi Semiconductor Co.,Ltd 2016-2023. All rights reserved.
 * 2023-06-14 File created.
 */

//#define DEBUG
#include "frsm-dev.h"
#include <linux/module.h>
#include <linux/delay.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
#include "internal.h"
#include "frsm-regmap.h"
#include "frsm-firmware.h"
#if KERNEL_VERSION_HIGHER(4, 17, 0)
#include "frsm-codec.h"
#else
#include "frsm-codec-v1.h"
#endif
#include "frsm-sysfs.h"
#include "frsm-monitor.h"
#include "fs19v1-ops.h"
#include "fs19v2-ops.h"
#include "fs19v4-ops.h"
#include "fs18v1-ops.h"
#include "fs18v4-ops.h"
#if IS_ENABLED(CONFIG_SND_SOC_FRSM_AMP)
#include "frsm-i2c-amp.h"
#endif
//+P86801AA1, zhouweijie.lux, ADD, 2025/09/08, add smartpa info
#include <linux/hardware_info.h>
extern int smartpa_type;
//-P86801AA1, zhouweijie.lux, ADD, 2025/09/08, add smartpa info
#define FRSM_EVENT_ACTION(event, action) \
	[event] = { ENUM_TO_STR(event), action }

static int g_frsm_ndev;
static DEFINE_MUTEX(g_frsm_mutex);

static void frsm_mutex_lock(void)
{
	mutex_lock(&g_frsm_mutex);
}

static void frsm_mutex_unlock(void)
{
	mutex_unlock(&g_frsm_mutex);
}

static int frsm_i2c_reg_write(struct frsm_dev *frsm_dev,
		uint8_t reg, uint16_t val)
{
	struct i2c_msg msgs[1];
	uint8_t buffer[3];
	int ret;

	if (!frsm_dev || !frsm_dev->i2c)
		return -EINVAL;

	buffer[0] = reg;
	buffer[1] = (val >> 8) & 0x00ff;
	buffer[2] = val & 0x00ff;
	msgs[0].addr = frsm_dev->i2c->addr;
	msgs[0].flags = 0;
	msgs[0].len = sizeof(buffer);
	msgs[0].buf = &buffer[0];

	mutex_lock(&frsm_dev->io_lock);
	ret = i2c_transfer(frsm_dev->i2c->adapter, &msgs[0], ARRAY_SIZE(msgs));
	mutex_unlock(&frsm_dev->io_lock);

	if (ret != ARRAY_SIZE(msgs)) {
		dev_err(frsm_dev->dev, "write %02x transfer error: %d\n",
				reg, ret);
		return -EIO;
	}

	return 0;
}

static int frsm_stub_dev_init(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (test_bit(EVENT_DEV_INIT, &frsm_dev->state))
		return 0;

	if (frsm_dev->ops.dev_init == NULL)
		return -ENOTSUPP;

	if (frsm_dev->tbl_scene == NULL)
		frsm_dev->tbl_scene = frsm_get_fwm_table(frsm_dev, INDEX_SCENE);

	ret = frsm_dev->ops.dev_init(frsm_dev);
	if (!ret)
		set_bit(EVENT_DEV_INIT, &frsm_dev->state);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_set_scene(struct frsm_dev *frsm_dev)
{
	struct scene_table *scene;
	const char *name = NULL;
	bool on_state;
	int ret = 0;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (test_bit(EVENT_SET_SCENE, &frsm_dev->state))
		return 0;

	if (frsm_dev->ops.set_scene == NULL)
		return -ENOTSUPP;

	if (frsm_dev->tbl_scene == NULL) {
		dev_err(frsm_dev->dev, "Scene table is null\n");
		return -EINVAL;
	}

	scene = (struct scene_table *)frsm_dev->tbl_scene->buf;
	scene += frsm_dev->next_scene + 1;
	if (scene == frsm_dev->cur_scene) {
		set_bit(EVENT_SET_SCENE, &frsm_dev->state);
		return 0;
	}

	on_state = test_bit(EVENT_STREAM_ON, &frsm_dev->state);
	if (on_state) {
		ret |= frsm_stub_mute(frsm_dev);
		ret |= frsm_stub_shut_down(frsm_dev);
	}

	ret = frsm_get_fwm_string(frsm_dev, scene->name, &name);
	if (!ret && name != NULL)
		dev_info(frsm_dev->dev, "Set scene: %s\n", name);

	ret |= frsm_dev->ops.set_scene(frsm_dev, scene);
	if (!ret) {
		frsm_dev->cur_scene = scene;
		set_bit(EVENT_SET_SCENE, &frsm_dev->state);
	}

	if (on_state) {
		FRSM_DELAY_MS(5);
		ret |= frsm_stub_start_up(frsm_dev);
		ret |= frsm_stub_unmute(frsm_dev);
		ret |= frsm_stub_tsmute(frsm_dev, false);
	}

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_hw_params(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (test_bit(EVENT_HW_PARAMS, &frsm_dev->state))
		return 0;

	if (frsm_dev->ops.hw_params == NULL)
		return -ENOTSUPP;

	ret = frsm_dev->ops.hw_params(frsm_dev);
	if (!ret)
		set_bit(EVENT_HW_PARAMS, &frsm_dev->state);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_set_spkr_prot(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (frsm_dev->ops.set_spkr_prot == NULL)
		return 0;

	ret = frsm_dev->ops.set_spkr_prot(frsm_dev);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_start_up(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (test_bit(EVENT_START_UP, &frsm_dev->state))
		return 0;

	if (frsm_dev->ops.start_up == NULL)
		return -ENOTSUPP;

	ret = frsm_dev->ops.start_up(frsm_dev);
	if (!ret)
		set_bit(EVENT_START_UP, &frsm_dev->state);

	ret |= frsm_stub_set_spkr_prot(frsm_dev);
	ret |= frsm_stub_set_volume(frsm_dev);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_unmute(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (test_bit(EVENT_STREAM_ON, &frsm_dev->state))
		return 0;

	if (frsm_dev->ops.set_mute == NULL)
		return -ENOTSUPP;

	ret = frsm_dev->ops.set_mute(frsm_dev, 0);
	if (!ret)
		set_bit(EVENT_STREAM_ON, &frsm_dev->state);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_tsmute(struct frsm_dev *frsm_dev, bool mute)
{
	bool status;
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (frsm_dev->ops.set_tsmute == NULL || !frsm_dev->state_ts)
		return 0;

	status = test_bit(STATE_TS_ON, &frsm_dev->state);
	if ((status ^ mute) != 0)
		return 0;

	if (!mute)
		FRSM_DELAY_MS(15);

	ret = frsm_dev->ops.set_tsmute(frsm_dev, mute);
	if (mute) {
		frsm_dev->state_ts = false;
		clear_bit(STATE_TS_ON, &frsm_dev->state);
	} else {
		set_bit(STATE_TS_ON, &frsm_dev->state);
	}

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_mute(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (!test_bit(EVENT_STREAM_ON, &frsm_dev->state))
		return 0;

	ret = frsm_stub_tsmute(frsm_dev, true);

	if (frsm_dev->ops.set_mute == NULL)
		return -ENOTSUPP;

	ret |= frsm_dev->ops.set_mute(frsm_dev, 1);
	if (!ret)
		clear_bit(EVENT_STREAM_ON, &frsm_dev->state);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_mntr_switch(struct frsm_dev *frsm_dev, bool enable)
{
	bool stream_on;
	bool mntr_on;

	if (frsm_dev == NULL)
		return -EINVAL;

	mntr_on = test_bit(EVENT_STAT_MNTR, &frsm_dev->state);
	if ((mntr_on ^ enable) == 0)
		return 0;

	if (enable) {
		stream_on = test_bit(EVENT_STREAM_ON, &frsm_dev->state);
		if (!stream_on)
			return 0;
		set_bit(EVENT_STAT_MNTR, &frsm_dev->state);
		frsm_mntr_switch(frsm_dev, true);
		set_bit(EVENT_STAT_MNTR, &frsm_dev->event);
	} else {
		clear_bit(EVENT_STAT_MNTR, &frsm_dev->event);
		frsm_mntr_switch(frsm_dev, false);
		clear_bit(EVENT_STAT_MNTR, &frsm_dev->state);
	}

	return 0;
}

static int frsm_stub_shut_down(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (!test_bit(EVENT_START_UP, &frsm_dev->state))
		return 0;

	if (frsm_dev->ops.shut_down == NULL)
		return -ENOTSUPP;

	ret = frsm_dev->ops.shut_down(frsm_dev);
	if (!ret)
		clear_bit(EVENT_START_UP, &frsm_dev->state);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_set_idle(struct frsm_dev *frsm_dev)
{
	if (frsm_dev == NULL)
		return -EINVAL;

	if (!test_bit(EVENT_HW_PARAMS, &frsm_dev->state))
		return 0;

	clear_bit(EVENT_HW_PARAMS, &frsm_dev->state);

	return 0;
}

static int frsm_stub_set_channel(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (!(frsm_dev->func & FRSM_SWAP_CHN))
		return -ENOTSUPP;

	if (frsm_dev->ops.set_channel == NULL)
		return -ENOTSUPP;

	ret = frsm_dev->ops.set_channel(frsm_dev);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_set_volume(struct frsm_dev *frsm_dev)
{
	int volume, smask, batt_vol;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	if (frsm_dev->ops.set_volume == NULL)
		return -ENOTSUPP;

	volume = frsm_dev->volume;
	batt_vol = frsm_dev->batt_vol;

	if (frsm_dev->pdata->bsg_volume_v2)
		batt_vol += volume;

	smask = frsm_dev->pdata->mntr_scenes & BIT(frsm_dev->next_scene);
	if (!smask)
		batt_vol = FRSM_VOLUME_MAX;

	if (volume > batt_vol)
		volume = batt_vol;

	if (volume > frsm_dev->safe_vol)
		volume = frsm_dev->safe_vol;

	ret = frsm_dev->ops.set_volume(frsm_dev, volume);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_stub_stat_monitor(struct frsm_dev *frsm_dev)
{
	int ret = -ENOTSUPP;

	if (frsm_dev == NULL)
		return -EINVAL;

	if (frsm_dev->ops.stat_monitor == NULL)
		return -ENOTSUPP;

	if (!test_bit(EVENT_STREAM_ON, &frsm_dev->state))
		return 0;

	ret = frsm_stub_tsmute(frsm_dev, false);

	if (frsm_dev && frsm_dev->ops.stat_monitor)
		ret = frsm_dev->ops.stat_monitor(frsm_dev);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static struct frsm_action_map g_action_map[EVENT_MAX] = {
	FRSM_EVENT_ACTION(EVENT_DEV_INIT,   frsm_stub_dev_init),
	FRSM_EVENT_ACTION(EVENT_SET_SCENE,  frsm_stub_set_scene),
	FRSM_EVENT_ACTION(EVENT_HW_PARAMS,  frsm_stub_hw_params),
	FRSM_EVENT_ACTION(EVENT_START_UP,   frsm_stub_start_up),
	FRSM_EVENT_ACTION(EVENT_STREAM_ON,  frsm_stub_unmute),
	FRSM_EVENT_ACTION(EVENT_STAT_MNTR,  frsm_stub_stat_monitor),
	FRSM_EVENT_ACTION(EVENT_STREAM_OFF, frsm_stub_mute),
	FRSM_EVENT_ACTION(EVENT_SHUT_DOWN,  frsm_stub_shut_down),
	FRSM_EVENT_ACTION(EVENT_SET_IDLE,   frsm_stub_set_idle),
	FRSM_EVENT_ACTION(EVENT_SET_CHANN,  frsm_stub_set_channel),
	FRSM_EVENT_ACTION(EVENT_SET_VOL,    frsm_stub_set_volume),
};

static int frsm_send_normal_event(struct frsm_dev *frsm_dev,
		enum frsm_event event)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (g_action_map[event].ops_action == NULL)
		return 0;

	ret = g_action_map[event].ops_action(frsm_dev);

	return ret;
}

static int frsm_send_pmu_event(struct frsm_dev *frsm_dev, enum frsm_event event)
{
	enum frsm_event vid;
	int ret;

	if (event <= EVENT_SET_SCENE)
		clear_bit(event, &frsm_dev->state);
	else if (test_bit(event, &frsm_dev->event))
		return 0;

	set_bit(event, &frsm_dev->event);

//#pragma clang loop unroll(full)
	for (vid = EVENT_DEV_INIT; vid <= EVENT_STREAM_ON; vid++) {
		if (!test_bit(vid, &frsm_dev->event))
			return 0;
		ret = g_action_map[vid].ops_action(frsm_dev);
		if (ret)
			return ret;
	}

	return 0;
}

static int frsm_send_pmd_event(struct frsm_dev *frsm_dev, enum frsm_event event)
{
	enum frsm_event vid, on_event;
	int ret;

	on_event = (EVENT_STAT_MNTR * 2) - event;
	if (!test_bit(on_event, &frsm_dev->event))
		return 0;

//#pragma clang loop unroll(full)
	for (vid = EVENT_STREAM_OFF; vid <= event; vid++) {
		ret = g_action_map[vid].ops_action(frsm_dev);
		if (ret)
			return ret;
	}

	clear_bit(on_event, &frsm_dev->event);

	return 0;
}

static int frsm_send_event(struct frsm_dev *frsm_dev, enum frsm_event event)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	if (event < 0 || event >= EVENT_MAX) {
		dev_err(frsm_dev->dev, "Unknow event:%d\n", event);
		return -EINVAL;
	}

	dev_dbg(frsm_dev->dev, "event[%d]: %s\n",
			event, g_action_map[event].event_name);

#ifdef CONFIG_FRSM_TUNING_SUPPORT
	/* ONLY for tuning in userdebug version */
	if (test_bit(STATE_TUNING, &frsm_dev->state)) {
		dev_info(frsm_dev->dev, "It's tuning mode, skip event\n");
		return 0;
	}
#endif

	if (event <= EVENT_STREAM_ON) {
		frsm_mutex_lock();
		ret = frsm_send_pmu_event(frsm_dev, event);
		frsm_mutex_unlock();
		frsm_stub_mntr_switch(frsm_dev, true);
	} else if (event >= EVENT_STREAM_OFF && event <= EVENT_SET_IDLE) {
		frsm_stub_mntr_switch(frsm_dev, false);
		frsm_mutex_lock();
		ret = frsm_send_pmd_event(frsm_dev, event);
		frsm_mutex_unlock();
	} else {
		frsm_mutex_lock();
		ret = frsm_send_normal_event(frsm_dev, event);
		frsm_mutex_unlock();
	}

	dev_dbg(frsm_dev->dev, "mask: %04lx-%04lx\n",
			frsm_dev->event, frsm_dev->state);

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return ret;
}

static int frsm_parse_dts_pins(struct frsm_dev *frsm_dev)
{
	struct frsm_platform_data *pdata;
	struct gpio_desc *gpiod;
	struct frsm_gpio *gpio;
	static char reprobe[FRSM_DEV_MAX+1];
	int ret;

	pdata = frsm_dev->pdata;
	gpio = pdata->gpio + FRSM_PIN_SDZ;
	gpiod = devm_gpiod_get(frsm_dev->dev, "sdz", GPIOD_OUT_LOW);
	if (IS_ERR_OR_NULL(gpiod)) {
		gpio->gpiod = NULL;
		ret = PTR_ERR(gpiod);
		if (ret == -EBUSY) {
			if (reprobe[pdata->spkr_id]++ < FRSM_REPROBE_MAX)
				return -EPROBE_DEFER;
			dev_info(frsm_dev->dev, "Assuming shared sdz pin\n");
		} else {
			dev_info(frsm_dev->dev, "Assuming unused sdz pin\n");
		}
	} else {
		gpio->gpiod = gpiod;
	}

	gpio = pdata->gpio + FRSM_PIN_INTZ;
	gpiod = devm_gpiod_get(frsm_dev->dev, "intz", GPIOD_IN);
	if (IS_ERR_OR_NULL(gpiod)) {
		gpio->gpiod = NULL;
		ret = PTR_ERR(gpiod);
		if (ret == -EBUSY)
			dev_info(frsm_dev->dev, "Assuming shared intz pin\n");
		else
			dev_info(frsm_dev->dev, "Assuming unused intz pin\n");
	} else {
		gpio->gpiod = gpiod;
	}

	return 0;
}

static int frsm_parse_dts(struct frsm_dev *frsm_dev)
{
	struct frsm_platform_data *pdata;
	char str[FRSM_STRING_MAX];
	const char *prefix;
	struct device_node *np;
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	np = frsm_dev->dev->of_node;
	pdata = frsm_dev->pdata;

	/* spkr id, clockwise: 1 -> 12 */
	ret = of_property_read_s32(np, "spkr-id", &pdata->spkr_id);
	if (ret)
		pdata->spkr_id = 0;

	ret = frsm_parse_dts_pins(frsm_dev);
	if (ret)
		return ret;

	/* channel id: 0=mono 1=left 2=right */
	ret = of_property_read_s32(np, "rx-channel", &pdata->rx_channel);
	if (ret)
		pdata->rx_channel = -1; /* Use firmware config */

	/* firmware name */
	ret = of_property_read_string(np, "fwm-name", &pdata->fwm_name);
	if (ret)
		pdata->fwm_name = "frsm-params.bin";

	dev_info(frsm_dev->dev, "channel:%d fwm_name: %s\n",
			pdata->rx_channel, pdata->fwm_name);

	ret = of_property_read_string(np, "sound-name-prefix", &prefix);
	if (ret && pdata->spkr_id > 0) {
		snprintf(str, sizeof(str), "SPK%d", pdata->spkr_id);
		pdata->name_prefix =
				devm_kstrdup(frsm_dev->dev, str, GFP_KERNEL);
	}

	dev_info(frsm_dev->dev, "spkr-id:%d prefix:%s\n", pdata->spkr_id,
			pdata->name_prefix ? pdata->name_prefix : "");

	/* virtual address */
	ret = of_property_read_s32(np, "vrtl-addr", &pdata->vrtl_addr);
	if (ret)
		pdata->vrtl_addr = frsm_dev->i2c->addr;

	ret = of_property_read_u32(np, "ref-rdc", &pdata->ref_rdc);
	if (ret)
		pdata->ref_rdc = 0;

	dev_info(frsm_dev->dev, "vrtl-addr:%x ref-rdc:%d\n",
			pdata->vrtl_addr, pdata->ref_rdc);

	ret = of_property_read_u32(np, "soft-reset", &pdata->soft_reset);
	if (ret)
		pdata->soft_reset = 0x0;

	pdata->rx_volume_v2 = of_property_read_bool(np, "rx-volume-v2");

	return 0;
}

static int frsm_detect_device(struct frsm_dev *frsm_dev)
{
	uint16_t devid;
	int ret;

	if (frsm_dev == NULL || frsm_dev->dev == NULL)
		return -EINVAL;

	ret = frsm_reg_read(frsm_dev, FRSM_REG_DEVID, &devid);
	if (ret)
		return ret;

	switch (HIGH8(devid)) {
//	case FRSM_DEVID_FS19NH:
//		dev_info(frsm_dev->dev, "Detect FS1958/FS1962!\n");
//		break;
	case FRSM_DEVID_FS19AM:
		dev_info(frsm_dev->dev, "Detect FS194x!\n");
		fs19v1_dev_ops(frsm_dev);
		break;
	case FRSM_DEVID_FS19MS:
		dev_info(frsm_dev->dev, "Detect FS199x!\n");
		fs19v2_dev_ops(frsm_dev);
		break;
	case FRSM_DEVID_FS18DH:
		frsm_dev->dev_id = devid;
		ret = fs19v2_dev_ops(frsm_dev);
		if (ret)
			break;
		frsm_dev->dev_id = FRSM_DEVID_FS19MS << 8; // DEVID[15..8]
		dev_info(frsm_dev->dev, "Detect FS199x!\n");
		return ret;
	case FRSM_DEVID_FS18YN:
	case FRSM_DEVID_FS18HS:
		dev_info(frsm_dev->dev, "Detect FS1816/FS1832/FS1838!\n");
		fs18v1_dev_ops(frsm_dev);
		break;
	case FRSM_DEVID_FS19TB:
		dev_info(frsm_dev->dev, "Detect FS1925!\n");
		fs19v4_dev_ops(frsm_dev);
		break;
	case FRSM_DEVID_FS18LH:
		dev_info(frsm_dev->dev, "Detect FS1827!\n");
		fs18v4_dev_ops(frsm_dev);
		break;
	default:
		dev_err(frsm_dev->dev, "Unknow DEVID: 0x%04X!\n", devid);
		return -ENODEV;
	}

	frsm_dev->dev_id = devid;

	FRSM_FUNC_EXIT(frsm_dev->dev, ret);
	return 0;
}

static int frsm_soft_reset(struct frsm_dev *frsm_dev)
{
	struct frsm_platform_data *pdata;
	uint8_t reg, addr, addr_bak;
	static int bus_nr = -1;
	uint16_t val;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	pdata = frsm_dev->pdata;
	if (pdata->soft_reset == 0x0)
		return 0;

	if (bus_nr == frsm_dev->i2c->adapter->nr)
		return 0;

	addr_bak = frsm_dev->i2c->addr;
	bus_nr = frsm_dev->i2c->adapter->nr;

	reg = (pdata->soft_reset >> 16) & 0xff;
	val = pdata->soft_reset & 0xffff;
	for (addr = 0x34; addr <= 0x37; addr++) {
		frsm_dev->i2c->addr = addr;
		frsm_i2c_reg_write(frsm_dev, reg, val);
	}

	frsm_dev->i2c->addr = addr_bak;
	FRSM_DELAY_MS(5);

	dev_dbg(frsm_dev->dev, "Soft reseted!\n");

	return 0;
}

static int frsm_ext_reset(struct frsm_dev *frsm_dev)
{
	struct frsm_gpio *gpio;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	gpio = frsm_dev->pdata->gpio + FRSM_PIN_SDZ;
	if (!IS_ERR_OR_NULL(gpio->gpiod)) {
		gpiod_set_value(gpio->gpiod, 0);
		FRSM_DELAY_MS(10);
		gpiod_set_value(gpio->gpiod, 1);
		FRSM_DELAY_MS(10);
	}

	frsm_soft_reset(frsm_dev);

	return 0;
}

static int frsm_i2c_dev_init(struct frsm_dev *frsm_dev)
{
	int ret;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return -EINVAL;

	ret = frsm_ext_reset(frsm_dev);
	if (ret)
		return ret;

	ret = frsm_regmap_init(frsm_dev);
	if (ret)
		return ret;

	ret = frsm_detect_device(frsm_dev);
	if (ret)
		return ret;

	ret = frsm_codec_init(frsm_dev);
	if (ret)
		return ret;

	ret = frsm_sysfs_init(frsm_dev);
	if (ret)
		return ret;

	frsm_dev->volume = FRSM_VOLUME_MAX;
	frsm_dev->safe_vol = FRSM_VOLUME_MAX;
	frsm_dev->batt_vol = FRSM_VOLUME_MAX;
	if (frsm_dev->pdata->bsg_volume_v2)
		frsm_dev->batt_vol = 0; /* not attenuate volume */
	frsm_dev->hw_params.bclk = 0;
	frsm_dev->hw_params.format = 0xFF;
	frsm_dev->amp_enable = false;
	frsm_dev->stream_unmute = false;

	return 0;
}

static void frsm_i2c_pins_deinit(struct frsm_dev *frsm_dev)
{
	struct frsm_gpio *gpio;
	int idx;

	if (frsm_dev == NULL || frsm_dev->pdata == NULL)
		return;

	gpio = frsm_dev->pdata->gpio;
	for (idx = 0; idx < FRSM_PIN_MAX; idx++, gpio++) {
		if (IS_ERR_OR_NULL(gpio->gpiod))
			continue;
		devm_gpiod_put(frsm_dev->dev, gpio->gpiod);
	}
}

#if IS_ENABLED(CONFIG_SND_SOC_FRSM_AMP)
int frsm_i2c_notify_callback(int event, void *buf, int buf_size)
{
	return frsm_notify_callback(event, buf, buf_size);
}
#endif

static void frsm_i2c_dev_deinit(struct frsm_dev *frsm_dev)
{
	if (frsm_dev == NULL)
		return;

	frsm_sysfs_deinit(frsm_dev);
	frsm_codec_deinit(frsm_dev);
	frsm_i2c_pins_deinit(frsm_dev);
}

static int frsm_i2c_probe(struct i2c_client *i2c
#if !KERNEL_VERSION_HIGHER(6, 3, 0)
		, const struct i2c_device_id *id
#endif
		)
{
	struct frsm_dev *frsm_dev;
	int ret;

	dev_dbg(&i2c->dev, "Version: %s\n", FRSM_I2C_VERSION);

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "Failed to check I2C_FUNC_I2C\n");
		return -EIO;
	}

	frsm_dev = devm_kzalloc(&i2c->dev, sizeof(struct frsm_dev), GFP_KERNEL);
	if (frsm_dev == NULL)
		return -ENOMEM;

	frsm_dev->pdata = devm_kzalloc(&i2c->dev,
			sizeof(struct frsm_platform_data), GFP_KERNEL);
	if (frsm_dev->pdata == NULL)
		return -ENOMEM;

	frsm_dev->dev = &i2c->dev;
	frsm_dev->i2c = i2c;
	i2c_set_clientdata(i2c, frsm_dev);
	mutex_init(&frsm_dev->io_lock);

	frsm_mutex_lock();
	ret = frsm_parse_dts(frsm_dev);
	if (ret) {
		dev_err(&i2c->dev, "Failed to parse DTS: %d\n", ret);
		frsm_i2c_pins_deinit(frsm_dev);
		frsm_mutex_unlock();
		return ret;
	}

	ret = frsm_i2c_dev_init(frsm_dev);
	if (ret) {
		dev_err(&i2c->dev, "Failed to init DEVICE: %d\n", ret);
		frsm_i2c_pins_deinit(frsm_dev);
		frsm_mutex_unlock();
		return ret;
	}

#if IS_ENABLED(CONFIG_SND_SOC_FRSM_AMP)
	frsm_i2c_amp_init(frsm_dev);
	frsm_amp_register_notify_callback(frsm_i2c_notify_callback);
#endif

	g_frsm_ndev++;
	//+P86801AA1, zhouweijie.lux, ADD, 2025/09/08, add smartpa info
	if (HIGH8(frsm_dev->dev_id) == FRSM_DEVID_FS19AM) {
		pr_info("%s audiock fsm detect device seccess\n",__func__);
		smartpa_type = snd_soc_set_smartpa_type("frsm_i2c_probe", FS16XX);
	} else {
		hardwareinfo_set_prop(HARDWARE_SMARTPA, "unknown");
	}
	//-P86801AA1, zhouweijie.lux, ADD, 2025/09/08, add smartpa info
	dev_info(&i2c->dev, "i2c probed\n");
	frsm_mutex_unlock();

	return 0;
}

static i2c_remove_type frsm_i2c_remove(struct i2c_client *i2c)
{
	struct frsm_dev *frsm_dev;

	frsm_dev = i2c_get_clientdata(i2c);
	if (frsm_dev)
		frsm_i2c_dev_deinit(frsm_dev);

	dev_info(&i2c->dev, "i2c removed\n");

	return i2c_remove_val;
}

static void frsm_i2c_shutdown(struct i2c_client *i2c)
{
	struct frsm_dev *frsm_dev;
	struct frsm_gpio *gpio;

	frsm_dev = i2c_get_clientdata(i2c);
	if (frsm_dev == NULL)
		return;

	frsm_send_event(frsm_dev, EVENT_SET_IDLE);
	gpio = frsm_dev->pdata->gpio + FRSM_PIN_SDZ;
	if (!IS_ERR_OR_NULL(gpio->gpiod))
		gpiod_set_value(gpio->gpiod, 0);

	dev_info(&i2c->dev, "i2c shutdown\n");
}

#ifdef CONFIG_FRSM_COMPAT_EXPORT
int exfrsm_i2c_probe(struct i2c_client *i2c,
		const struct i2c_device_id *id)
{
	return frsm_i2c_probe(i2c, id);
}
EXPORT_SYMBOL_GPL(exfrsm_i2c_probe);

void exfrsm_i2c_remove(struct i2c_client *i2c)
{
	frsm_i2c_remove(i2c);
}
EXPORT_SYMBOL_GPL(exfrsm_i2c_remove);

void exfrsm_i2c_shutdown(struct i2c_client *i2c)
{
	return frsm_i2c_shutdown(i2c);
}
EXPORT_SYMBOL_GPL(exfrsm_i2c_shutdown);

#else
static const struct i2c_device_id frsm_i2c_id[] = {
	{ "fs19xx", 0 },
	{ "fs199x", 0 },
	{ "fs183x", 0 },
	{ "fs1827", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, frsm_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id frsm_of_id[] = {
	{ .compatible = "foursemi,fs19xx", },
	{ .compatible = "foursemi,fs199x", },
	{ .compatible = "foursemi,fs183x", },
	{ .compatible = "foursemi,fs1827", },
	{}
};
MODULE_DEVICE_TABLE(of, frsm_of_id);
#endif

struct i2c_driver frsm_i2c_driver = {
	.driver = {
		.name = FRSM_I2C_NAME,
#ifdef CONFIG_OF
		.of_match_table = frsm_of_id,
#endif
		.owner = THIS_MODULE,
	},
	.probe = frsm_i2c_probe,
	.remove = frsm_i2c_remove,
	.shutdown = frsm_i2c_shutdown,
	.id_table = frsm_i2c_id,
};

#ifndef FRSM_DRV_2IN1_SUPPORT
module_i2c_driver(frsm_i2c_driver);

MODULE_AUTHOR("FourSemi SW <support@foursemi.com>");
MODULE_DESCRIPTION("ASoC FourSemi Audio Amplifier Driver");
MODULE_VERSION(FRSM_I2C_VERSION);
MODULE_LICENSE("GPL");
#endif // ndef FRSM_DRV_2IN1_SUPPORT
#endif // ndef CONFIG_FRSM_COMPAT_EXPORT
