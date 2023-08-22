/**
 * @file  hardware.c
 * @brief Low-level cowstick-ums hardware configuration
 *
 * @author Saint-Genest Gwenael <gwen@cowlab.fr>
 * @copyright Agilack (c) 2023
 *
 * @page License
 * Cowstick-ums firmware is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * version 3 as published by the Free Software Foundation. You should have
 * received a copy of the GNU Lesser General Public License along with this
 * program, see LICENSE.md file for more details.
 * This program is distributed WITHOUT ANY WARRANTY.
 */
#include "hardware.h"

/**
 * @brief Initialize processor, clocks and some peripherals
 *
 * This function should be called on startup for clocks and IOs configuration.
 */
void hw_init(void)
{
	int i;

	// Enable GPIO ports
	reg_set(RCC_AHB2ENR1(RCC), (1 << 1)); /* GPIO-B */
	// RCC : Reset GPIOB
	reg_wr(RCC_AHB2RSTR1(RCC), (1 << 1));
	for (i = 0; i < 16; i++)
		asm volatile("nop");
	reg_wr(RCC_AHB2RSTR1(RCC), 0);
}
/* EOF */
