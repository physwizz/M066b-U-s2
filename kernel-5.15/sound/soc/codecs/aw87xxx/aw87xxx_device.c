// SPDX-License-Identifier: GPL-2.0
/* aw87xxx_device.c  aw87xxx pa module
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/timer.h>
#include "aw87xxx.h"
#include "aw87xxx_device.h"
#include "aw87xxx_dsp.h"
#include "aw87xxx_log.h"

#ifdef AW_ALGO_AUTH_DSP
static DEFINE_MUTEX(g_algo_auth_dsp_lock);
int g_algo_auth_st;
#endif

/*************************************************************************
 * aw87xxx variable
 ************************************************************************/
const char *g_aw_pid_9b_product[] = {
	"aw87319",
	"AW87319",
};
const char *g_aw_pid_18_product[] = {
	"aw87358",
	"AW87358",
};

const char *g_aw_pid_39_product[] = {
	"aw87329",
	"AW87329",
	"aw87339",
	"AW87339",
	"aw87349",
	"AW87349",
};

const char *g_aw_pid_59_3x9_product[] = {
	"aw87359",
	"AW87359",
	"aw87389",
	"AW87389",
};

const char *g_aw_pid_59_5x9_product[] = {
	"aw87509",
	"AW87509",
	"aw87519",
	"AW87519",
	"aw87529",
	"AW87529",
	"aw87539",
	"AW87539",
};

const char *g_aw_pid_5a_product[] = {
	"aw87549",
	"AW87549",
	"aw87559",
	"AW87559",
	"aw87569",
	"AW87569",
	"aw87579",
	"AW87579",
	"aw81509",
	"AW81509",
	"aw87579G",
	"AW87579G",
};

const char *g_aw_pid_76_product[] = {
	"aw87390",
	"AW87390",
	"aw87320",
	"AW87320",
	"aw87401",
	"AW87401",
	"aw87360",
	"AW87360",
	"aw87390G",
	"AW87390G",
};

const char *g_aw_pid_60_product[] = {
	"aw87560",
	"AW87560",
	"aw87561",
	"AW87561",
	"aw87562",
	"AW87562",
	"aw87501",
	"AW87501",
	"aw87550",
	"AW87550",
};

const char *g_aw_pid_c1_product[] = {
	"aw87391",
	"AW87391",
	"aw87392",
	"AW87392",
};

const char *g_aw_pid_c2_product[] = {
	"aw87565",
	"AW87565",
	"aw87566",
	"AW87566",
	"aw81564",
	"AW81564",
	"aw87567",
	"AW87567",
	"aw87568",
	"AW87568",
	"aw87564",
	"AW87564",
};

static int aw87xxx_dev_get_chipid(struct aw_device *aw_dev);

/***************************************************************************
 *
 * reading and writing of I2C bus
 *
 ***************************************************************************/
int aw87xxx_dev_i2c_write_byte(struct aw_device *aw_dev,
			uint8_t reg_addr, uint8_t reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_write_byte_data(aw_dev->i2c, reg_addr, reg_data);
		if (ret < 0)
			AW_DEV_LOGE(aw_dev->dev, "i2c_write cnt=%d error=%d",
				cnt, ret);
		else
			break;

		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

int aw87xxx_dev_i2c_read_byte(struct aw_device *aw_dev,
			uint8_t reg_addr, uint8_t *reg_data)
{
	int ret = -1;
	unsigned char cnt = 0;

	while (cnt < AW_I2C_RETRIES) {
		ret = i2c_smbus_read_byte_data(aw_dev->i2c, reg_addr);
		if (ret < 0) {
			AW_DEV_LOGE(aw_dev->dev, "i2c_read cnt=%d error=%d",
				cnt, ret);
		} else {
			*reg_data = ret;
			break;
		}
		cnt++;
		msleep(AW_I2C_RETRY_DELAY);
	}

	return ret;
}

int aw87xxx_dev_i2c_read_msg(struct aw_device *aw_dev,
	uint8_t reg_addr, uint8_t *data_buf, uint32_t data_len)
{
	int ret = -1;

	struct i2c_msg msg[] = {
	[0] = {
		.addr = aw_dev->i2c_addr,
		.flags = 0,
		.len = sizeof(uint8_t),
		.buf = &reg_addr,
		},
	[1] = {
		.addr = aw_dev->i2c_addr,
		.flags = I2C_M_RD,
		.len = data_len,
		.buf = data_buf,
		},
	};

	ret = i2c_transfer(aw_dev->i2c->adapter, msg, ARRAY_SIZE(msg));
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "transfer failed");
		return ret;
	} else if (ret != AW_I2C_READ_MSG_NUM) {
		AW_DEV_LOGE(aw_dev->dev, "transfer failed(size error)");
		return -ENXIO;
	}

	return 0;
}

int aw87xxx_dev_i2c_write_bits(struct aw_device *aw_dev,
	uint8_t reg_addr, uint8_t mask, uint8_t reg_data)
{
	int ret = -1;
	unsigned char reg_val = 0;

	ret = aw87xxx_dev_i2c_read_byte(aw_dev, reg_addr, &reg_val);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "i2c read error, ret=%d", ret);
		return ret;
	}
	reg_val &= mask;
	reg_val |= (reg_data & (~mask));
	ret = aw87xxx_dev_i2c_write_byte(aw_dev, reg_addr, reg_val);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "i2c write error, ret=%d", ret);
		return ret;
	}

	return 0;
}

/************************************************************************
 *
 * aw87xxx device update profile data to registers
 *
 ************************************************************************/
static void aw87xxx_dev_reg_mute_bits_set(struct aw_device *aw_dev,
				uint8_t *reg_val, bool enable)
{
	if (enable) {
		*reg_val &= aw_dev->mute_desc.mask;
		*reg_val |= aw_dev->mute_desc.enable;
	} else {
		*reg_val &= aw_dev->mute_desc.mask;
		*reg_val |= aw_dev->mute_desc.disable;
	}
}

static int aw87xxx_dev_reg_update(struct aw_device *aw_dev,
			struct aw_data_container *profile_data)
{
	int i = 0;
	int ret = -1;
	uint8_t reg_addr = 0;
	uint8_t reg_val = 0;

	if (profile_data == NULL)
		return -EINVAL;

	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGE(aw_dev->dev, "hwen is off, can not update reg");
		return -EINVAL;
	}

	for (i = 0; i < profile_data->len; i = i + 2) {
		reg_addr = profile_data->data[i];
		reg_val = profile_data->data[i + 1];

		AW_DEV_LOGI(aw_dev->dev, "reg=0x%02x, val = 0x%02x", reg_addr, reg_val);
		/*delay ms*/
		if (reg_addr == AW87XXX_DELAY_REG_ADDR) {
			AW_DEV_LOGI(aw_dev->dev, "delay %d ms", reg_val);
			usleep_range(reg_val * AW87XXX_REG_DELAY_TIME, reg_val * AW87XXX_REG_DELAY_TIME + 10);
			continue;
		}

		if ((aw_dev->mute_desc.addr != AW_REG_NONE) &&
			(reg_addr == aw_dev->mute_desc.addr)) {
			aw87xxx_dev_reg_mute_bits_set(aw_dev, &reg_val, true);
			AW_DEV_LOGD(aw_dev->dev, "change mute_mask, val = 0x%02x", reg_val);
		}

		ret = aw87xxx_dev_i2c_write_byte(aw_dev, reg_addr, reg_val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/************************************************************************
 *
 * aw87xxx device hadware and soft contols
 *
 ************************************************************************/
static bool aw87xxx_dev_gpio_is_valid(struct aw_device *aw_dev)
{
	if (gpio_is_valid(aw_dev->rst_gpio))
		return true;
	else
		return false;
}

void aw87xxx_dev_hw_pwr_ctrl(struct aw_device *aw_dev, bool enable)
{
	if (aw_dev->hwen_status == AW_DEV_HWEN_INVALID) {
		AW_DEV_LOGD(aw_dev->dev, "product not have reset-pin,hardware pwd control invalid");
		return;
	}
	if (enable) {
		if (aw87xxx_dev_gpio_is_valid(aw_dev)) {
			gpio_set_value_cansleep(aw_dev->rst_gpio, AW_GPIO_LOW_LEVEL);
			mdelay(2);
			gpio_set_value_cansleep(aw_dev->rst_gpio, AW_GPIO_HIGHT_LEVEL);
			mdelay(2);
			aw_dev->hwen_status = AW_DEV_HWEN_ON;
			AW_DEV_LOGI(aw_dev->dev, "hw power on");
		} else {
			AW_DEV_LOGI(aw_dev->dev, "hw already power on");
		}
	} else {
		if (aw87xxx_dev_gpio_is_valid(aw_dev)) {
			gpio_set_value_cansleep(aw_dev->rst_gpio, AW_GPIO_LOW_LEVEL);
			mdelay(2);
			aw_dev->hwen_status = AW_DEV_HWEN_OFF;
			AW_DEV_LOGI(aw_dev->dev, "hw power off");
		} else {
			AW_DEV_LOGI(aw_dev->dev, "hw already power off");
		}
	}
	//add for debug
	mdelay(2);
	AW_DEV_LOGI(aw_dev->dev, "awinic rst gpio :%d",gpio_get_value_cansleep(aw_dev->rst_gpio));
}

static int aw87xxx_dev_mute_ctrl(struct aw_device *aw_dev, bool enable)
{
	int ret = 0;

	if (enable) {
		ret = aw87xxx_dev_i2c_write_bits(aw_dev, aw_dev->mute_desc.addr,
				aw_dev->mute_desc.mask, aw_dev->mute_desc.enable);
		if (ret < 0)
			return ret;
		AW_DEV_LOGI(aw_dev->dev, "set mute down");
	} else {
		ret = aw87xxx_dev_i2c_write_bits(aw_dev, aw_dev->mute_desc.addr,
				aw_dev->mute_desc.mask, aw_dev->mute_desc.disable);
		if (ret < 0)
			return ret;
		AW_DEV_LOGI(aw_dev->dev, "close mute down");
	}

	return 0;
}

void aw87xxx_dev_soft_reset(struct aw_device *aw_dev)
{
	int ret = -1;

	AW_DEV_LOGD(aw_dev->dev, "enter");

	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGE(aw_dev->dev, "hw is off,can not softrst");
		return;
	}

	ret = aw87xxx_dev_i2c_write_byte(aw_dev, AW87XXX_CHIPIDL_REG, AW87XXX_SW_RESET_PASSWORD);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "write failed, ret = %d", ret);
		return;
	}

	AW_DEV_LOGD(aw_dev->dev, "down");
}


int aw87xxx_dev_default_pwr_off(struct aw_device *aw_dev,
		struct aw_data_container *profile_data)
{
	int ret = 0;

	AW_DEV_LOGD(aw_dev->dev, "enter");
	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGE(aw_dev->dev, "hwen is already off");
		return 0;
	}

	if (aw_dev->soft_off_enable && profile_data) {
		ret = aw87xxx_dev_reg_update(aw_dev, profile_data);
		if (ret < 0) {
			AW_DEV_LOGE(aw_dev->dev, "update profile[Off] fw config failed");
			goto reg_off_update_failed;
		}
	}

	if (aw_dev->delay_desc.power_off_delay_ms > 0)
		mdelay(aw_dev->delay_desc.power_off_delay_ms);

	aw87xxx_dev_hw_pwr_ctrl(aw_dev, false);
	AW_DEV_LOGD(aw_dev->dev, "down");
	return 0;

reg_off_update_failed:
	aw87xxx_dev_hw_pwr_ctrl(aw_dev, false);
	return ret;
}


/************************************************************************
 *
 * aw87xxx device power on process function
 *
 ************************************************************************/
int aw87xxx_dev_default_pwr_on(struct aw_device *aw_dev,
			struct aw_data_container *profile_data)
{
	int ret = 0;

	/*hw power on*/
	aw87xxx_dev_hw_pwr_ctrl(aw_dev, true);

	if (aw_dev->delay_desc.power_on_delay_ms > 0)
		mdelay(aw_dev->delay_desc.power_on_delay_ms);

	if (aw_dev->mute_desc.addr != AW_REG_NONE) {
		/* open the mute */
		ret = aw87xxx_dev_mute_ctrl(aw_dev, true);
		if (ret < 0)
			return ret;
	}

	ret = aw87xxx_dev_reg_update(aw_dev, profile_data);
	if (ret < 0)
		return ret;

	if (aw_dev->mute_desc.addr != AW_REG_NONE) {
		/* close the mute */
		ret = aw87xxx_dev_mute_ctrl(aw_dev, false);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/****************************************************************************
 *
 * aw87xxx chip esd status check
 *
 ****************************************************************************/
int aw87xxx_dev_esd_reg_status_check(struct aw_device *aw_dev)
{
	int ret;
	unsigned char reg_val = 0;
	struct aw_esd_check_desc *esd_desc = &aw_dev->esd_desc;

	AW_DEV_LOGD(aw_dev->dev, "enter");

	if (!esd_desc->first_update_reg_addr) {
		AW_DEV_LOGE(aw_dev->dev, "esd check info if not init,please check");
		return -EINVAL;
	}

	ret = aw87xxx_dev_i2c_read_byte(aw_dev, esd_desc->first_update_reg_addr,
			&reg_val);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "read reg 0x%02x failed",
			esd_desc->first_update_reg_addr);
		return ret;
	}

	AW_DEV_LOGD(aw_dev->dev, "0x%02x:default val=0x%02x real val=0x%02x",
		esd_desc->first_update_reg_addr,
		esd_desc->first_update_reg_val, reg_val);

	if (reg_val == esd_desc->first_update_reg_val) {
		AW_DEV_LOGE(aw_dev->dev, "reg status check failed");
		return -EINVAL;
	}
	return 0;
}

int aw87xxx_dev_check_reg_is_rec_mode(struct aw_device *aw_dev)
{
	int ret;
	unsigned char reg_val = 0;
	struct aw_rec_mode_desc *rec_desc = &aw_dev->rec_desc;

	if (!rec_desc->addr) {
		AW_DEV_LOGE(aw_dev->dev, "rec check info if not init,please check");
		return -EINVAL;
	}

	ret = aw87xxx_dev_i2c_read_byte(aw_dev, rec_desc->addr, &reg_val);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "read reg 0x%02x failed",
			rec_desc->addr);
		return ret;
	}

	if (rec_desc->enable) {
		if (reg_val & ~(rec_desc->mask)) {
			AW_DEV_LOGI(aw_dev->dev, "reg status is receiver mode");
			aw_dev->is_rec_mode = AW_IS_REC_MODE;
		} else {
			aw_dev->is_rec_mode = AW_NOT_REC_MODE;
		}
	} else {
		if (!(reg_val & ~(rec_desc->mask))) {
			AW_DEV_LOGI(aw_dev->dev, "reg status is receiver mode");
			aw_dev->is_rec_mode = AW_IS_REC_MODE;
		} else {
			aw_dev->is_rec_mode = AW_NOT_REC_MODE;
		}
	}
	return 0;
}

/****************************************************************************
 *
 * aw87xxx algo_encryption
 *
 ****************************************************************************/
int aw87xxx_dev_get_encrypted_value(struct aw_device *aw_dev,
			unsigned int in, unsigned int *out)
{
	int ret = 0;
	struct aw_auth_desc *desc = &aw_dev->auth_desc;
	uint8_t out_l = 0;
	uint8_t out_h = 0;

	if ((desc->reg_in_l == AW_REG_NONE) || (desc->reg_in_h == AW_REG_NONE) ||
		(desc->reg_out_l == AW_REG_NONE) || (desc->reg_out_h == AW_REG_NONE)) {
		AW_DEV_LOGD(aw_dev->dev, "Missing encryption register");
		return -EINVAL;
	}

	ret = aw87xxx_dev_i2c_write_byte(aw_dev, desc->reg_in_l, (in & 0xFF));
	if (ret < 0)
		return ret;

	ret = aw87xxx_dev_i2c_write_byte(aw_dev, desc->reg_in_h, ((in >> 8) & 0xFF));
	if (ret < 0)
		return ret;

	ret = aw87xxx_dev_i2c_read_byte(aw_dev, desc->reg_out_l, &out_l);
	if (ret < 0)
		return ret;

	ret = aw87xxx_dev_i2c_read_byte(aw_dev, desc->reg_out_h, &out_h);

	*out = out_l | (out_h << 8);

	return ret;
}

int aw87xxx_dev_algo_auth_mode(struct aw_device *aw_dev, struct algo_auth_data *algo_data)
{
	int ret = 0;
	unsigned int encrypted_out = 0;

	AW_DEV_LOGD(aw_dev->dev, "algo auth mode: %d", algo_data->auth_mode);

	aw_dev->auth_desc.auth_mode = algo_data->auth_mode;
	aw_dev->auth_desc.random = algo_data->random;
	aw_dev->auth_desc.chip_id = AW_ALGO_AUTH_MAGIC_ID;
	aw_dev->auth_desc.check_result = algo_data->check_result;

	switch (algo_data->auth_mode) {
	case AW_ALGO_AUTH_MODE_MAGIC_ID:
		aw_dev->auth_desc.reg_crc = algo_data->reg_crc;
		break;
	case AW_ALGO_AUTH_MODE_REG_CRC:
		ret = aw87xxx_dev_get_encrypted_value(aw_dev, algo_data->random, &encrypted_out);
		if (ret < 0)
			AW_DEV_LOGE(aw_dev->dev, "get encrypted value failed");
		aw_dev->auth_desc.reg_crc = encrypted_out;
		break;
	default:
		AW_DEV_LOGE(aw_dev->dev, "unsupport auth mode[%d]", algo_data->auth_mode);
		ret = -EINVAL;
	}
	return ret;
}

#ifdef AW_ALGO_AUTH_DSP
int aw87xxx_dev_algo_auth_dsp_mode(struct aw_device *aw_dev, struct algo_auth_data *algo_data)
{
	int ret = 0;
	unsigned int encrypted_out = 0;

	AW_DEV_LOGD(aw_dev->dev, "algo auth mode: %d", algo_data->auth_mode);

	algo_data->chip_id = AW_ALGO_AUTH_MAGIC_ID;

	if (algo_data->auth_mode == AW_ALGO_AUTH_MODE_REG_CRC) {
		ret = aw87xxx_dev_get_encrypted_value(aw_dev, algo_data->random, &encrypted_out);
		if (ret < 0)
			AW_DEV_LOGE(aw_dev->dev, "get encrypted value failed");
		algo_data->reg_crc = encrypted_out;
	}

	return ret;
}

void aw87xxx_dev_algo_authentication(struct aw_device *aw_dev)
{
	int ret = 0;
	struct algo_auth_data algo_data;

	mutex_lock(&g_algo_auth_dsp_lock);

	AW_DEV_LOGD(aw_dev->dev, "g_algo_auth_st=%d", g_algo_auth_st);

	if (g_algo_auth_st == AW_ALGO_AUTH_OK) {
		AW_DEV_LOGD(aw_dev->dev, "algo auth complete");
		goto exit;
	}

	ret = aw87xxx_dsp_get_algo_auth_data(aw_dev, (char *)&algo_data, sizeof(struct algo_auth_data));
	if (ret < 0)
		goto exit;

	ret = aw87xxx_dev_algo_auth_dsp_mode(aw_dev, &algo_data);
	if (ret < 0)
		goto exit;

	ret = aw87xxx_dsp_set_algo_auth_data(aw_dev, (char *)&algo_data, sizeof(struct algo_auth_data));
	if (ret < 0)
		goto exit;

	g_algo_auth_st = AW_ALGO_AUTH_OK;

	AW_DEV_LOGI(aw_dev->dev, "g_algo_auth_st=%d", g_algo_auth_st);

	AW_DEV_LOGD(aw_dev->dev, "mode=%d,reg_crc=0x%x,random=0x%x,id=0x%x,res=%d",
		algo_data.auth_mode, algo_data.reg_crc, algo_data.random,
		algo_data.chip_id, algo_data.check_result);

exit:
	mutex_unlock(&g_algo_auth_dsp_lock);
}
#endif

static void aw_dev_auth_reg_none(struct aw_device *aw_dev)
{
	/*encryption info*/
	aw_dev->auth_desc.reg_in_l = AW_REG_NONE;
	aw_dev->auth_desc.reg_in_h = AW_REG_NONE;
	aw_dev->auth_desc.reg_out_l = AW_REG_NONE;
	aw_dev->auth_desc.reg_out_h = AW_REG_NONE;
}

/****************************************************************************
 *
 * aw87xxx product attributes init info
 *
 ****************************************************************************/
static int aw87xxx_dev_common_init(struct aw_device *aw_dev,
	const struct aw_dev_property *property)
{
	int ret = 0;

	aw_dev->product_tab = property->product;
	aw_dev->product_cnt = property->product_cnt;

	aw_dev->reg_max_addr = property->max_addr;

	aw_dev->soft_off_enable = property->soft_off_enabled;

	aw_dev->delay_desc.power_on_delay_ms = property->power_on_delay_ms;
	aw_dev->delay_desc.power_off_delay_ms = property->power_off_delay_ms;

	aw_dev->rec_desc.addr = AW87XXX_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_EN_RCV_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_EN_RCV_ENABLE;
	aw_dev->rec_desc.mask = (uint8_t)AW87XXX_EN_RCV_MASK;

	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_ESD_REG;
	aw_dev->esd_desc.first_update_reg_val = property->esd_default;

	aw_dev->ops.pwr_on_func = property->ops.pwr_on_func;
	aw_dev->ops.pwr_off_func = property->ops.pwr_off_func;

	if (property->sw_enabled) {
		aw_dev->mute_desc.addr = AW87XXX_SYSCTRL_REG;
		aw_dev->mute_desc.mask = AW87XXX_EN_SW_MASK;
		aw_dev->mute_desc.enable = AW87XXX_EN_SW_DISABLE_VALUE;
		aw_dev->mute_desc.disable = AW87XXX_EN_SW_ENABLE_VALUE;
	} else {
		aw_dev->mute_desc.addr = AW_REG_NONE;
	}

	if (property->ipeak_enabled) {
		aw_dev->ipeak_desc.reg = AW87XXX_BSTCTRL_REG;
		aw_dev->ipeak_desc.mask = AW87XXX_BST_IPEAK_MASK;
	} else {
		aw_dev->ipeak_desc.reg = AW_REG_NONE;
	}

	aw_dev->vol_desc.addr = property->vol_enabled ? AW87XXX_CPOVP_REG : AW_REG_NONE;

	if (property->auth_enabled) {
		aw_dev->auth_desc.reg_in_l = AW87XXX_VCINL_REG;
		aw_dev->auth_desc.reg_in_h = AW87XXX_VCINH_REG;
		aw_dev->auth_desc.reg_out_l = AW87XXX_VCOUTL_REG;
		aw_dev->auth_desc.reg_out_h = AW87XXX_VCOUTH_REG;
	} else {
		aw_dev_auth_reg_none(aw_dev);
	}

	return ret;
}

/********************** aw87xxx_pid_9A attributes ***************************/

static int aw_dev_pid_9b_reg_update(struct aw_device *aw_dev,
			struct aw_data_container *profile_data)
{
	int i = 0;
	int ret = -1;
	uint8_t reg_val = 0;

	if (profile_data == NULL)
		return -EINVAL;

	if (aw_dev->hwen_status == AW_DEV_HWEN_OFF) {
		AW_DEV_LOGE(aw_dev->dev, "dev is pwr_off,can not update reg");
		return -EINVAL;
	}

	if (profile_data->len != AW_PID_9B_BIN_REG_CFG_COUNT) {
		AW_DEV_LOGE(aw_dev->dev, "reg_config count of bin is error,can not update reg");
		return -EINVAL;
	}
	ret = aw87xxx_dev_i2c_write_byte(aw_dev, AW87XXX_PID_9B_ENCRYPTION_REG,
		AW87XXX_PID_9B_ENCRYPTION_BOOST_OUTPUT_SET);
	if (ret < 0)
		return ret;

	for (i = 1; i < AW_PID_9B_BIN_REG_CFG_COUNT; i++) {
		AW_DEV_LOGI(aw_dev->dev, "reg=0x%02x, val = 0x%02x",
			i, profile_data->data[i]);
		/*delay ms*/
		if (profile_data->data[i] == AW87XXX_DELAY_REG_ADDR) {
			AW_DEV_LOGI(aw_dev->dev, "delay %d ms", profile_data->data[i + 1]);
			usleep_range(profile_data->data[i + 1] * AW87XXX_REG_DELAY_TIME,
				profile_data->data[i + 1] * AW87XXX_REG_DELAY_TIME + 10);
			continue;
		}

		reg_val = profile_data->data[i];
		if (i == AW87XXX_PID_9B_SYSCTRL_REG) {
			aw87xxx_dev_reg_mute_bits_set(aw_dev, &reg_val, true);
			AW_DEV_LOGD(aw_dev->dev, "change mute_mask, val = 0x%02x",
				reg_val);
		}

		ret = aw87xxx_dev_i2c_write_byte(aw_dev, i, reg_val);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int aw_dev_pid_9b_pwr_on(struct aw_device *aw_dev, struct aw_data_container *data)
{
	int ret = 0;

	/*hw power on*/
	aw87xxx_dev_hw_pwr_ctrl(aw_dev, true);

	/* open the mute */
	ret = aw87xxx_dev_mute_ctrl(aw_dev, true);
	if (ret < 0)
		return ret;

	/* Update scene parameters in mute mode */
	ret = aw_dev_pid_9b_reg_update(aw_dev, data);
	if (ret < 0)
		return ret;

	/* close the mute */
	ret = aw87xxx_dev_mute_ctrl(aw_dev, false);
	if (ret < 0)
		return ret;

	return 0;
}

static int aw_dev_pid_9b_init(struct aw_device *aw_dev)
{
	int ret = 0;

	/* Product register permission info */
	aw_dev->reg_max_addr = AW87XXX_PID_9B_REG_MAX;

	aw_dev->mute_desc.addr = AW87XXX_PID_9B_SYSCTRL_REG;
	aw_dev->mute_desc.mask = AW87XXX_PID_9B_REG_EN_SW_MASK;
	aw_dev->mute_desc.enable = AW87XXX_PID_9B_REG_EN_SW_DISABLE_VALUE;
	aw_dev->mute_desc.disable = AW87XXX_PID_9B_REG_EN_SW_ENABLE_VALUE;
	aw_dev->ops.pwr_on_func = aw_dev_pid_9b_pwr_on;

	/* Whether to allow register operation to power off */
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_DISENABLE;

	aw_dev->product_tab = g_aw_pid_9b_product;
	aw_dev->product_cnt = sizeof(g_aw_pid_9b_product) / sizeof(char *);

	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_9B_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_9B_POWER_OFF_DELAY_MS;

	aw_dev->rec_desc.addr = AW87XXX_PID_9B_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_9B_SPK_MODE_ENABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_9B_SPK_MODE_DISABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_9B_SPK_MODE_MASK;

	/* esd reg info */
	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_9B_SYSCTRL_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_9B_SYSCTRL_DEFAULT;

	aw_dev->ipeak_desc.reg = AW_REG_NONE;

	aw_dev->vol_desc.addr = AW_REG_NONE;

	aw_dev_auth_reg_none(aw_dev);

	return ret;
}

static int aw_dev_pid_9a_init(struct aw_device *aw_dev)
{
	int ret = 0;

	ret = aw87xxx_dev_i2c_write_byte(aw_dev, AW87XXX_PID_9B_ENCRYPTION_REG,
		AW87XXX_PID_9B_ENCRYPTION_BOOST_OUTPUT_SET);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "write 0x64=0x2C error");
		return -EINVAL;
	}

	ret = aw87xxx_dev_get_chipid(aw_dev);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "read chipid is failed,ret=%d", ret);
		return ret;
	}

	if (aw_dev->chipid == AW_DEV_CHIPID_9B) {
		AW_DEV_LOGI(aw_dev->dev, "product is pid_9B class");
		aw_dev_pid_9b_init(aw_dev);
	} else {
		AW_DEV_LOGE(aw_dev->dev, "product is not pid_9B class，not support");
		return -EINVAL;
	}

	return 0;
}

/********************** aw87xxx_pid_9b attributes end ***********************/

/********************** aw87xxx_pid_18 attributes ***************************/
static int aw_dev_pid_18_init(struct aw_device *aw_dev)
{
	int ret = 0;

	/* Product register permission info */
	aw_dev->reg_max_addr = AW87XXX_PID_18_REG_MAX;

	aw_dev->mute_desc.addr = AW87XXX_PID_18_SYSCTRL_REG;
	aw_dev->mute_desc.mask = AW87XXX_PID_18_REG_EN_SW_MASK;
	aw_dev->mute_desc.enable = AW87XXX_PID_18_REG_EN_SW_DISABLE_VALUE;
	aw_dev->mute_desc.disable = AW87XXX_PID_18_REG_EN_SW_ENABLE_VALUE;

	/* Whether to allow register operation to power off */
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_ENABLE;

	aw_dev->product_tab = g_aw_pid_18_product;
	aw_dev->product_cnt = sizeof(g_aw_pid_18_product) / sizeof(char *);

	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_18_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_18_POWER_OFF_DELAY_MS;

	aw_dev->rec_desc.addr = AW87XXX_PID_18_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_18_REG_REC_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_18_REG_REC_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_18_REG_REC_MODE_MASK;

	/* esd reg info */
	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_18_CLASSD_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_18_CLASSD_DEFAULT;

	aw_dev->ipeak_desc.reg = AW_REG_NONE;

	aw_dev->vol_desc.addr = AW87XXX_PID_18_CPOC_REG;

	aw_dev_auth_reg_none(aw_dev);

	return ret;
}
/********************** aw87xxx_pid_18 attributes end ***********************/

/********************** aw87xxx_pid_39 attributes ***************************/
static int aw_dev_pid_39_init(struct aw_device *aw_dev)
{
	int ret = 0;

	/* Product register permission info */
	aw_dev->reg_max_addr = AW87XXX_PID_39_REG_MAX;

	/* Whether to allow register operation to power off */
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_ENABLE;

	aw_dev->product_tab = g_aw_pid_39_product;
	aw_dev->product_cnt = sizeof(g_aw_pid_39_product) / sizeof(char *);

	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_39_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_39_POWER_OFF_DELAY_MS;

	aw_dev->mute_desc.addr = AW_REG_NONE;

	aw_dev->rec_desc.addr = AW87XXX_PID_39_REG_MODECTRL;
	aw_dev->rec_desc.disable = AW87XXX_PID_39_REC_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_39_REC_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_39_REC_MODE_MASK;

	/* esd reg info */
	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_39_REG_MODECTRL;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_39_MODECTRL_DEFAULT;

	aw_dev->ipeak_desc.reg = AW_REG_NONE;

	aw_dev->vol_desc.addr = AW87XXX_PID_39_REG_CPOVP;

	aw_dev_auth_reg_none(aw_dev);

	return ret;
}
/********************* aw87xxx_pid_39 attributes end *************************/


/********************* aw87xxx_pid_59 attributes *************************/
static int aw_dev_pid_59_5x9_init(struct aw_device *aw_dev)
{
	int ret = 0;

	/* Product register permission info */
	aw_dev->reg_max_addr = AW87XXX_PID_59_5X9_REG_MAX;

	/* Whether to allow register operation to power off */
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_ENABLE;

	aw_dev->product_tab = g_aw_pid_59_5x9_product;
	aw_dev->product_cnt = sizeof(g_aw_pid_59_5x9_product) / sizeof(char *);

	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_59_5X9_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_59_5X9_POWER_OFF_DELAY_MS;

	aw_dev->mute_desc.addr = AW_REG_NONE;

	aw_dev->rec_desc.addr = AW87XXX_PID_59_5X9_REG_SYSCTRL;
	aw_dev->rec_desc.disable = AW87XXX_PID_59_5X9_REC_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_59_5X9_REC_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_59_5X9_REC_MODE_MASK;

	/* esd reg info */
	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_59_5X9_REG_ENCR;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_59_5X9_ENCRY_DEFAULT;

	aw_dev->ipeak_desc.reg = AW_REG_NONE;

	aw_dev->vol_desc.addr = AW_REG_NONE;

	aw_dev_auth_reg_none(aw_dev);

	return ret;
}

static int aw_dev_pid_59_3x9_init(struct aw_device *aw_dev)
{
	int ret = 0;

	/* Product register permission info */
	aw_dev->reg_max_addr = AW87XXX_PID_59_3X9_REG_MAX;

	/* Whether to allow register operation to power off */
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_ENABLE;

	aw_dev->product_tab = g_aw_pid_59_3x9_product;
	aw_dev->product_cnt = sizeof(g_aw_pid_59_3x9_product) / sizeof(char *);

	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_59_3X9_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_59_3X9_POWER_OFF_DELAY_MS;

	aw_dev->mute_desc.addr = AW_REG_NONE;

	aw_dev->rec_desc.addr = AW87XXX_PID_59_3X9_REG_MDCRTL;
	aw_dev->rec_desc.disable = AW87XXX_PID_59_3X9_SPK_MODE_ENABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_59_3X9_SPK_MODE_DISABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_59_3X9_SPK_MODE_MASK;

	/* esd reg info */
	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_59_3X9_REG_ENCR;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_59_3X9_ENCR_DEFAULT;

	aw_dev->ipeak_desc.reg = AW_REG_NONE;

	aw_dev->vol_desc.addr = AW87XXX_PID_59_3X9_REG_CPOVP;

	aw_dev_auth_reg_none(aw_dev);

	return ret;
}

static int aw_dev_pid_59_init(struct aw_device *aw_dev)
{
	int ret = 0;

	if (aw87xxx_dev_gpio_is_valid(aw_dev))
		ret = aw_dev_pid_59_5x9_init(aw_dev);
	else
		ret = aw_dev_pid_59_3x9_init(aw_dev);

	return ret;
}
/******************* aw87xxx_pid_59 attributes end ***********************/

/********************** aw87xxx_pid_5a attributes ****************************/
static int aw_dev_pid_5a_init(struct aw_device *aw_dev)
{
	int ret = 0;

	/* Product register permission info */
	aw_dev->reg_max_addr = AW87XXX_PID_5A_REG_MAX;

	/* Whether to allow register operation to power off */
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_ENABLE;

	aw_dev->product_tab = g_aw_pid_5a_product;
	aw_dev->product_cnt = sizeof(g_aw_pid_5a_product) / sizeof(char *);

	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_5A_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_5A_POWER_OFF_DELAY_MS;

	aw_dev->mute_desc.addr = AW_REG_NONE;

	aw_dev->rec_desc.addr = AW87XXX_PID_5A_REG_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_5A_REG_RCV_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_5A_REG_RCV_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_5A_REG_RCV_MODE_MASK;

	/* esd reg info */
	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_5A_REG_DFT3R_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_5A_DFT3R_DEFAULT;

	aw_dev->ipeak_desc.reg = AW87XXX_PID_5A_REG_BSTCPR2_REG;
	aw_dev->ipeak_desc.mask = AW87XXX_PID_5A_REG_BST_IPEAK_MASK;

	aw_dev->vol_desc.addr = AW_REG_NONE;

	aw_dev_auth_reg_none(aw_dev);

	return ret;
}
/********************** aw87xxx_pid_5a attributes end ************************/

/********************** aw87xxx_pid_76 attributes ****************************/
static int aw_dev_pid_76_init(struct aw_device *aw_dev)
{
	int ret = 0;

	/* Product register permission info */
	aw_dev->reg_max_addr = AW87XXX_PID_76_REG_MAX;

	/* software power off control info */
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_ENABLE;

	aw_dev->product_tab = g_aw_pid_76_product;
	aw_dev->product_cnt = sizeof(g_aw_pid_76_product) / sizeof(char *);

	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_76_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_76_POWER_OFF_DELAY_MS;

	aw_dev->mute_desc.addr = AW_REG_NONE;

	aw_dev->rec_desc.addr = AW87XXX_PID_76_MDCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_76_EN_SPK_ENABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_76_EN_SPK_DISABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_76_EN_SPK_MASK;

	/* esd reg info */
	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_76_DFT_ADP1_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_76_DFT_ADP1_CHECK;

	aw_dev->ipeak_desc.reg = AW_REG_NONE;

	aw_dev->vol_desc.addr = AW87XXX_PID_76_CPOVP_REG;

	aw_dev_auth_reg_none(aw_dev);

	return ret;
}
/********************** aw87xxx_pid_76 attributes end ************************/

/********************** aw87xxx_pid_60 attributes ****************************/
static int aw_dev_pid_60_init(struct aw_device *aw_dev)
{
	int ret = 0;

	/* Product register permission info */
	aw_dev->reg_max_addr = AW87XXX_PID_60_REG_MAX;

	/* software power off control info */
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_ENABLE;

	aw_dev->product_tab = g_aw_pid_60_product;
	aw_dev->product_cnt = sizeof(g_aw_pid_60_product) / sizeof(char *);

	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_60_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_60_POWER_OFF_DELAY_MS;

	aw_dev->mute_desc.addr = AW_REG_NONE;

	aw_dev->rec_desc.addr = AW87XXX_PID_60_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_60_RCV_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_60_RCV_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_60_RCV_MODE_MASK;

	/* esd reg info */
	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_60_NG3_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_60_ESD_REG_VAL;

	aw_dev->ipeak_desc.reg = AW_REG_NONE;

	aw_dev->vol_desc.addr = AW_REG_NONE;

	aw_dev_auth_reg_none(aw_dev);

	return ret;
}
/********************** aw87xxx_pid_60 attributes end ************************/

/********************** aw87xxx_pid_c1 attributes ****************************/
static int aw_dev_pid_c1_init(struct aw_device *aw_dev)
{
	int ret = 0;

	/* Product register permission info */
	aw_dev->reg_max_addr = AW87XXX_PID_C1_REG_MAX;

	/* software power off control info */
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_ENABLE;

	aw_dev->product_tab = g_aw_pid_c1_product;
	aw_dev->product_cnt = sizeof(g_aw_pid_c1_product) / sizeof(char *);

	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_C1_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_C1_POWER_OFF_DELAY_MS;

	aw_dev->mute_desc.addr = AW_REG_NONE;

	aw_dev->rec_desc.addr = AW87XXX_PID_C1_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_C1_EN_SPK_SPK_MODE_ENABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_C1_EN_SPK_SPK_MODE_DISABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_C1_EN_SPK_MASK;

	/* esd reg info */
	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_C1_DFT_THGEN1_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_C1_DFT_THGEN1_CHECK;

	aw_dev->vol_desc.addr = AW_REG_NONE;

	aw_dev->ipeak_desc.reg = AW_REG_NONE;

	/*encryption info*/
	aw_dev->auth_desc.reg_in_l = AW87XXX_PID_C1_TESTIN1_REG;
	aw_dev->auth_desc.reg_in_h = AW87XXX_PID_C1_TESTIN2_REG;
	aw_dev->auth_desc.reg_out_l = AW87XXX_PID_C1_TESTOUT1_REG;
	aw_dev->auth_desc.reg_out_h = AW87XXX_PID_C1_TESTOUT2_REG;

	return ret;
}
/********************** aw87xxx_pid_c1 attributes end ************************/

/********************** aw87xxx_pid_c2 attributes ****************************/
static int aw_dev_pid_c2_init(struct aw_device *aw_dev)
{
	int ret = 0;

	/* Product register permission info */
	aw_dev->reg_max_addr = AW87XXX_PID_C2_REG_MAX;

	/* software power off control info */
	aw_dev->soft_off_enable = AW_DEV_SOFT_OFF_ENABLE;

	aw_dev->product_tab = g_aw_pid_c2_product;
	aw_dev->product_cnt = sizeof(g_aw_pid_c2_product) / sizeof(char *);

	aw_dev->delay_desc.power_on_delay_ms = AW87XXX_PID_C2_POWER_ON_DELAY_MS;
	aw_dev->delay_desc.power_off_delay_ms = AW87XXX_PID_C2_POWER_OFF_DELAY_MS;

	aw_dev->mute_desc.addr = AW_REG_NONE;

	aw_dev->rec_desc.addr = AW87XXX_PID_C2_SYSCTRL_REG;
	aw_dev->rec_desc.disable = AW87XXX_PID_C2_RCV_MODE_DISABLE;
	aw_dev->rec_desc.enable = AW87XXX_PID_C2_RCV_MODE_ENABLE;
	aw_dev->rec_desc.mask = AW87XXX_PID_C2_RCV_MODE_MASK;

	/* esd reg info */
	aw_dev->esd_desc.first_update_reg_addr = AW87XXX_PID_C2_CP_REG;
	aw_dev->esd_desc.first_update_reg_val = AW87XXX_PID_C2_CP_CHECK;

	aw_dev->ipeak_desc.reg = AW87XXX_PID_C2_PEAKLIMIT_REG;
	aw_dev->ipeak_desc.mask = AW87XXX_PID_C2_BST_IPEAK_MASK;

	aw_dev->vol_desc.addr = AW_REG_NONE;

	/*encryption info*/
	aw_dev->auth_desc.reg_in_l = AW87XXX_PID_C2_TESTIN1_REG;
	aw_dev->auth_desc.reg_in_h = AW87XXX_PID_C2_TESTIN2_REG;
	aw_dev->auth_desc.reg_out_l = AW87XXX_PID_C2_CRCOUT0_REG;
	aw_dev->auth_desc.reg_out_h = AW87XXX_PID_C2_CRCOUT1_REG;

	return ret;
}
/********************** aw87xxx_pid_c2 attributes end ************************/

const struct aw_dev_property g_aw_dev_property_registry[] = {
	{
		.id = AW_DEV_CHIPID_9A,
		.dev_init_func = aw_dev_pid_9a_init,
	},
	{
		.id = AW_DEV_CHIPID_9B,
		.dev_init_func = aw_dev_pid_9b_init,
	},
	{
		.id = AW_DEV_CHIPID_18,
		.dev_init_func = aw_dev_pid_18_init,
	},
	{
		.id = AW_DEV_CHIPID_39,
		.dev_init_func = aw_dev_pid_39_init,
	},
	{
		.id = AW_DEV_CHIPID_59,
		.dev_init_func = aw_dev_pid_59_init,
	},
	{
		.id = AW_DEV_CHIPID_5A,
		.dev_init_func = aw_dev_pid_5a_init,
	},
	{
		.id = AW_DEV_CHIPID_76,
		.dev_init_func = aw_dev_pid_76_init,
	},
	{
		.id = AW_DEV_CHIPID_60,
		.dev_init_func = aw_dev_pid_60_init,
	},
	{
		.id = AW_DEV_CHIPID_C1,
		.dev_init_func = aw_dev_pid_c1_init,
	},
	{
		.id = AW_DEV_CHIPID_C2,
		.dev_init_func = aw_dev_pid_c2_init,
	},
};

static int aw_dev_check_chip_model(struct aw_device *aw_dev)
{
	int ret  = 0;
	int i = 0;

	for (i = 0; i < sizeof(g_aw_dev_property_registry) / sizeof(struct aw_dev_property); i++) {
		if (aw_dev->chipid == g_aw_dev_property_registry[i].id) {
			if (g_aw_dev_property_registry[i].dev_init_func != NULL)
				ret = g_aw_dev_property_registry[i].dev_init_func(aw_dev);
			else
				ret = aw87xxx_dev_common_init(aw_dev, &g_aw_dev_property_registry[i]);

			if (ret < 0)
				AW_DEV_LOGE(aw_dev->dev, "product is pid_%x init failed",
					g_aw_dev_property_registry[i].id);
			else
				AW_DEV_LOGI(aw_dev->dev, "product is pid_%x class",
					g_aw_dev_property_registry[i].id);

			return 0;
		}
	}

	return -EINVAL;
}

static int aw_dev_chip_init(struct aw_device *aw_dev)
{
	int ret  = 0;

	ret = aw_dev_check_chip_model(aw_dev);
	if (ret == 0)
		return 0;

	aw_dev->chipid &= 0xFF;
	ret = aw_dev_check_chip_model(aw_dev);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "unsupported device revision [0x%x]",
			aw_dev->chipid);
		return -EINVAL;
	}

	return 0;
}

static int aw87xxx_dev_get_chipid(struct aw_device *aw_dev)
{
	int ret = -1;
	unsigned int cnt = 0;
	unsigned char reg_val_l = 0;
	unsigned char reg_val_h = 0;

	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		ret = aw87xxx_dev_i2c_read_byte(aw_dev, AW87XXX_CHIPIDL_REG, &reg_val_l);
		if (ret < 0) {
			AW_DEV_LOGE(aw_dev->dev, "[%d] read low id is failed, ret=%d",
				cnt, ret);
			continue;
		}
		break;
	}
	if (cnt == AW_READ_CHIPID_RETRIES) {
		AW_DEV_LOGE(aw_dev->dev, "read low id is failed");
		return -EINVAL;
	}

	for (cnt = 0; cnt < AW_READ_CHIPID_RETRIES; cnt++) {
		ret = aw87xxx_dev_i2c_read_byte(aw_dev, AW87XXX_CHIPIDH_REG, &reg_val_h);
		if (ret < 0) {
			AW_DEV_LOGE(aw_dev->dev, "[%d] read high id is failed, ret=%d",
				cnt, ret);
			continue;
		}
		break;
	}
	if (cnt == AW_READ_CHIPID_RETRIES) {
		AW_DEV_LOGE(aw_dev->dev, "read high id is failed");
		return -EINVAL;
	}

	aw_dev->chipid = (int)reg_val_l | (int)reg_val_h << 8;
	AW_DEV_LOGI(aw_dev->dev, "read chipid[0x%x] succeed", aw_dev->chipid);

	return 0;
}

int aw87xxx_dev_init(struct aw_device *aw_dev)
{
	int ret = -1;

	ret = aw87xxx_dev_get_chipid(aw_dev);
	if (ret < 0) {
		AW_DEV_LOGE(aw_dev->dev, "read chipid is failed,ret=%d", ret);
		return ret;
	}

	ret = aw_dev_chip_init(aw_dev);

	return ret;
}


