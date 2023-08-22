/**
 * @file  hardware.h
 * @brief Headers and definitions for low-level hardware handling
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
#ifndef HARDWARE_H
#define HARDWARE_H

// Main bus start addresses (non-secure)
#define APB1_NS 0x40000000
#define APB2_NS 0x40012C00
#define AHB1_NS 0x40020000
#define AHB2_NS 0x42020000
#define APB3_NS 0X46000400
#define AHB3_NS 0X46020000
// Main bus start addresses (secure)
#define APB1_S 0x50000000
#define APB2_S 0x50012C00
#define AHB1_S 0x50020000
#define AHB2_S 0x52020000
#define APB3_S 0X56000400
#define AHB3_S 0X56020000

// Peripherals addresses (secure)
#define GPIOA_S  (AHB2_S + 0x0000)
#define GPIOB_S  (AHB2_S + 0x0400)
#define GPIOC_S  (AHB2_S + 0x0800)
#define GPIOD_S  (AHB2_S + 0x0C00)
#define GPIOE_S  (AHB2_S + 0x1000)
#define GPIOF_S  (AHB2_S + 0x1400)
#define GPIOG_S  (AHB2_S + 0x1800)
#define GPIOH_S  (AHB2_S + 0x1C00)
#define GPIOI_S  (AHB2_S + 0x2000)
#define GPIOJ_S  (AHB2_S + 0x2400)
#define SPI4_S   (APB2_S + 0x4C00)
#define RCC_S    (AHB3_S + 0X0C00)
// Peripherals addresses (non-secure)
#define RCC_NS    (AHB3_NS + 0X0C00)
#define GPIOA_NS  (AHB2_NS + 0x0000)
#define GPIOB_NS  (AHB2_NS + 0x0400)
#define GPIOC_NS  (AHB2_NS + 0x0800)
#define GPIOD_NS  (AHB2_NS + 0x0C00)
#define GPIOE_NS  (AHB2_NS + 0x1000)
#define GPIOF_NS  (AHB2_NS + 0x1400)
#define GPIOG_NS  (AHB2_NS + 0x1800)
#define GPIOH_NS  (AHB2_NS + 0x1C00)
#define GPIOI_NS  (AHB2_NS + 0x2000)
#define GPIOJ_NS  (AHB2_NS + 0x2400)

#define GPIOB  GPIOB_S
#define RCC    RCC_S

// RCC registers
#define RCC_AHB1RSTR(x)  (x + 0x60)
#define RCC_AHB2RSTR1(x) (x + 0x64)
#define RCC_AHB2RSTR2(x) (x + 0x68)
#define RCC_AHB3RSTR(x)  (x + 0x6C)
#define RCC_AHB1ENR(x)   (x + 0x88)
#define RCC_AHB2ENR1(x)  (x + 0x8C)
#define RCC_AHB2ENR2(x)  (x + 0x90)
#define RCC_AHB3ENR(x)   (x + 0x94)
#define RCC_APB1LENR(x)  (x + 0x9C)
#define RCC_APB1HENR(x)  (x + 0xA0)
#define RCC_APB2ENR(x)   (x + 0xA4)
#define RCC_APB3ENR(x)   (x + 0xA8)

// GPIO registers
#define GPIO_MODER(x)   (x + 0x00)
#define GPIO_OTYPER(x)  (x + 0x04)
#define GPIO_OSPEEDR(x) (x + 0x08)
#define GPIO_PUPDR(x)   (x + 0x0C)
#define GPIO_IDR(x)     (x + 0x10)
#define GPIO_ODR(x)     (x + 0x14)
#define GPIO_BSRR(x)    (x + 0x18)
#define GPIO_LCKR(x)    (x + 0x1C)
#define GPIO_AFRL(x)    (x + 0x20)
#define GPIO_AFRH(x)    (x + 0x24)
#define GPIO_BRR(x)     (x + 0x28)
#define GPIO_HSLVR(x)   (x + 0x2C)
#define GPIO_SECCFGR(x) (x + 0x30)

void hw_init(void);

#include "types.h"

/* -------------------------------------------------------------------------- */
/*                        Low level register functions                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Write a 32 bits value to a memory mapped register
 *
 * @param reg   Address of the register to update
 * @param value New value to write into the register
 */
inline void reg_wr(u32 addr, u32 value)
{
	*(volatile u32 *)addr = value;
}

/**
 * @brief Write a 16 bits value to a memory mapped register
 *
 * @param reg   Address of the register to update
 * @param value New value to write into the register
 */
inline void reg16_wr(u32 addr, u16 value)
{
	*(volatile u16 *)addr = value;
}

/**
 * @brief Write a 8 bits value to a memory mapped register
 *
 * @param reg   Address of the register to update
 * @param value New value to write into the register
 */
inline void reg8_wr(u32 addr, u8 value)
{
	*(volatile u8 *)addr = value;
}

/**
 * @brief Read the value of a 32 bits memory mapped register
 *
 * @param  reg Address of the register to read
 * @return u32 Value of the register (32 bits)
 */
inline u32 reg_rd(u32 reg)
{
	return( *(volatile u32 *)reg );
}

/**
 * @brief Read the value of a 16 bits memory mapped register
 *
 * @param  addr Address of the register to read
 * @return u16  Value of the register (16 bits)
 */
inline u16 reg16_rd(u32 addr)
{
	return( *(volatile u16 *)addr );
}

/**
 * @brief Read the value of a 8 bits memory mapped register
 *
 * @param  addr Address of the register to read
 * @return u8   Value of the register
 */
inline u8 reg8_rd(u32 addr)
{
	return( *(volatile u8 *)addr );
}

/**
 * @brief Modify a 32 bits memory mapped register by clearing some bits
 *
 * @param addr  Address of the register to write
 * @param value Mask of the bits to clear
 */
inline void reg_clr(u32 addr, u32 value)
{
	*(volatile u32 *)addr = ( *(volatile u32 *)addr & ~value );
}

/**
 * @brief Modify a 16 bits memory mapped register by clearing some bits
 *
 * @param addr  Address of the register to write
 * @param value Mask of the bits to clear
 */
inline void reg16_clr(u32 addr, u16 value)
{
	*(volatile u16 *)addr = (u16)( *(volatile u16 *)addr & ~value );
}

/**
 * @brief Modify a 8 bits memory mapped register by clearing some bits
 *
 * @param addr  Address of the register to write
 * @param value Mask of the bits to clear
 */
inline void reg8_clr(u32 addr, u8 value)
{
	*(volatile u8 *)addr = (u8)( *(volatile u8 *)addr & ~value );
}

/**
 * @brief Modify a 32 bits memory mapped register by setting some bits
 *
 * @param addr  Address of the register to write
 * @param value Mask of the bits to set
 */
inline void reg_set(u32 addr, u32 value)
{
	*(volatile u32 *)addr = ( *(volatile u32 *)addr | value );
}

/**
 * @brief Modify a 16 bits memory mapped register by setting some bits
 *
 * @param addr  Address of the register to modify
 * @param value Mask of the bits to set
 */
inline void reg16_set(u32 addr, u16 value)
{
	*(volatile u16 *)addr = ( *(volatile u16 *)addr | value );
}

/**
 * @brief Modify a 8 bits memory mapped register by setting some bits
 *
 * @param addr  Address of the register to modify
 * @param value Mask of the bits to set
 */
inline void reg8_set(u32 addr, u8 value)
{
	*(volatile u8 *)addr = ( *(volatile u8 *)addr | value );
}
#endif
