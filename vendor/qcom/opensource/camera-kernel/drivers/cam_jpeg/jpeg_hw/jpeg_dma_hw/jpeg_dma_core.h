/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2017-2019, The Linux Foundation. All rights reserved.
 */

#ifndef CAM_JPEG_DMA_CORE_H
#define CAM_JPEG_DMA_CORE_H

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "cam_jpeg_hw_intf.h"

struct cam_jpeg_dma_device_hw_info {
	uint32_t reserved;
};

enum cam_jpeg_dma_core_state {
	CAM_JPEG_DMA_CORE_NOT_READY,
	CAM_JPEG_DMA_CORE_READY,
	CAM_JPEG_DMA_CORE_RESETTING,
	CAM_JPEG_DMA_CORE_STATE_MAX,
};

struct cam_jpeg_dma_device_core_info {
	enum cam_jpeg_dma_core_state core_state;
	struct cam_jpeg_dma_device_hw_info *jpeg_dma_hw_info;
	uint32_t cpas_handle;
	struct cam_jpeg_set_irq_cb irq_cb;
	int32_t ref_count;
	struct mutex core_mutex;
};

int cam_jpeg_dma_init_hw(void *device_priv,
	void *init_hw_args, uint32_t arg_size);
int cam_jpeg_dma_deinit_hw(void *device_priv,
	void *init_hw_args, uint32_t arg_size);
int cam_jpeg_dma_process_cmd(void *device_priv, uint32_t cmd_type,
	void *cmd_args, uint32_t arg_size);
irqreturn_t cam_jpeg_dma_irq(int irq_num, void *data);

/**
 * @brief : API to register JPEG DMA hw to platform framework.
 * @return struct platform_device pointer on on success, or ERR_PTR() on error.
 */
int cam_jpeg_dma_init_module(void);

/**
 * @brief : API to remove JPEG DMA Hw from platform framework.
 */
void cam_jpeg_dma_exit_module(void);
#endif /* CAM_JPEG_DMA_CORE_H */
