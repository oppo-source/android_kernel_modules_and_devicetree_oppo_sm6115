/* SPDX-License-Identifier: GPL-2.0
 * aw87xxx_pid_9b_reg.h
 *
 * Copyright (c) 2021 AWINIC Technology CO., LTD
 *
 * Author: Barry <zhaozhongbo@awinic.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __AW87XXX_PID_9B_REG_H__
#define __AW87XXX_PID_9B_REG_H__

#define AW87XXX_PID_9B_CHIPID_REG		(0x00)
#define AW87XXX_PID_9B_SYSCTRL_REG		(0x01)
#define AW87XXX_PID_9B_BATSAFE_REG		(0x02)
#define AW87XXX_PID_9B_BOV_REG			(0x03)
#define AW87XXX_PID_9B_BP_REG			(0x04)
#define AW87XXX_PID_9B_GAIN_REG			(0x05)
#define AW87XXX_PID_9B_AGC3_PO_REG		(0x06)
#define AW87XXX_PID_9B_AGC3_REG			(0x07)
#define AW87XXX_PID_9B_AGC2_REG			(0x08)
#define AW87XXX_PID_9B_AGC1_REG			(0x09)

#define AW87XXX_PID_9B_SYSCTRL_DEFAULT		(0x03)

/********************************************
 * soft control info
 * If you need to update this file, add this information manually
 *******************************************/
unsigned char aw87xxx_pid_9b_softrst_access[2] = {0x00, 0xaa};

/********************************************
 * Register Access
 *******************************************/
#define AW87XXX_PID_9B_REG_MAX			(0x64)

#define REG_NONE_ACCESS		(0)
#define REG_RD_ACCESS		(1 << 0)
#define REG_WR_ACCESS		(1 << 1)

const unsigned char aw87xxx_pid_9b_reg_access[AW87XXX_PID_9B_REG_MAX] = {
	[AW87XXX_PID_9B_CHIPID_REG]	= (REG_RD_ACCESS),
	[AW87XXX_PID_9B_SYSCTRL_REG]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[AW87XXX_PID_9B_BATSAFE_REG]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[AW87XXX_PID_9B_BOV_REG]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[AW87XXX_PID_9B_BP_REG]		= (REG_RD_ACCESS | REG_WR_ACCESS),
	[AW87XXX_PID_9B_GAIN_REG]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[AW87XXX_PID_9B_AGC3_PO_REG]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[AW87XXX_PID_9B_AGC3_REG]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[AW87XXX_PID_9B_AGC2_REG]	= (REG_RD_ACCESS | REG_WR_ACCESS),
	[AW87XXX_PID_9B_AGC1_REG]	= (REG_RD_ACCESS | REG_WR_ACCESS),
};


#define AW87XXX_PID_9B_ENCRYPTION_REG		(0x64)
#define AW87XXX_PID_9B_ENCRYPTION_BOOST_OUTPUT_SET	(0x2C)

/* REG_EN_SW bit 2 (SYSCTRL 0x01) */
#define AW87XXX_PID_9B_REG_EN_SW_START_BIT	(2)
#define AW87XXX_PID_9B_REG_EN_SW_BITS_LEN	(1)
#define AW87XXX_PID_9B_REG_EN_SW_MASK	\
	(~(((1<<AW87XXX_PID_9B_REG_EN_SW_BITS_LEN)-1) << AW87XXX_PID_9B_REG_EN_SW_START_BIT))

#define AW87XXX_PID_9B_REG_EN_SW_DISABLE	(0)
#define AW87XXX_PID_9B_REG_EN_SW_DISABLE_VALUE	\
	(AW87XXX_PID_9B_REG_EN_SW_DISABLE << AW87XXX_PID_9B_REG_EN_SW_START_BIT)

#define AW87XXX_PID_9B_REG_EN_SW_ENABLE		(1)
#define AW87XXX_PID_9B_REG_EN_SW_ENABLE_VALUE	\
	(AW87XXX_PID_9B_REG_EN_SW_ENABLE << AW87XXX_PID_9B_REG_EN_SW_START_BIT)

#define AW87XXX_PID_9B_REG_EN_SW_DEFAULT_VALUE	(1)
#define AW87XXX_PID_9B_REG_EN_SW_DEFAULT	\
	(AW87XXX_PID_9B_REG_EN_SW_DEFAULT_VALUE << AW87XXX_PID_9B_REG_EN_SW_START_BIT)

/* SPK_MODE bit 0 (SYSCTRL 0x01) */
#define AW87XXX_PID_9B_SPK_MODE_START_BIT	(0)
#define AW87XXX_PID_9B_SPK_MODE_BITS_LEN	(1)
#define AW87XXX_PID_9B_SPK_MODE_MASK	\
	(~(((1<<AW87XXX_PID_9B_SPK_MODE_BITS_LEN)-1) << AW87XXX_PID_9B_SPK_MODE_START_BIT))

#define AW87XXX_PID_9B_SPK_MODE_DISABLE	(0)
#define AW87XXX_PID_9B_SPK_MODE_DISABLE_VALUE	\
	(AW87XXX_PID_9B_SPK_MODE_DISABLE << AW87XXX_PID_9B_SPK_MODE_START_BIT)

#define AW87XXX_PID_9B_SPK_MODE_ENABLE	(1)
#define AW87XXX_PID_9B_SPK_MODE_ENABLE_VALUE	\
	(AW87XXX_PID_9B_SPK_MODE_ENABLE << AW87XXX_PID_9B_SPK_MODE_START_BIT)

#endif
