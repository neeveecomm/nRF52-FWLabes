/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <hal/nrf_gpio.h>
#include "dth11_driver.h"

int main(void)
{
	uint16_t temperature = 0;
	uint16_t humidity = 0;
	DHTxx_ErrorCode error_check;

	while (1)
	{
		temperature = 0;
		humidity = 0;
		error_check = DHTxx_Read(&temperature, &humidity);
		printk("Data read, temp:%d, hum:%d\n", temperature, humidity);
		// printk("error:%d\n",error_check);
	}
	return 0;
}
