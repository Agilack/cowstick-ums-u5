/**
 * @file  main.c
 * @brief Entry point of the firmware and main application loop
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
 * @brief Entry point of the C code
 *
 */
int main(void)
{
	u32 v;

	// Board init
	hw_init();

	// Configure PB7 as output
	v = reg_rd(GPIO_MODER(GPIOB));
	v &= ~(u32)(3 << 14); // Clear curernt mode
	v |=  (u32)(1 << 14); // Set general purpose output mode
	reg_wr(GPIO_MODER(GPIOB), v);

	// Set PB7 to '1'
	reg_wr(GPIO_BSRR(GPIOB), (1 << 7));

	while(1);
}
/* EOF */
