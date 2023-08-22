/**
 * @file  startup.s
 * @brief Processor vector table and interrupts handlers (including reset)
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
    .syntax unified
    .arch armv6-m

/* -- Stack and Head sections ---------------------------------------------- */
    .section .stack
    .align 2
    .equ    Stack_Size, 0x1FFC
    .globl    __StackTop
    .globl    __StackLimit
__StackLimit:
    .space    Stack_Size
    .size __StackLimit, . - __StackLimit
__StackTop:
    .size __StackTop, . - __StackTop

/* -- Vector Table --------------------------------------------------------- */

    .section .isr_vector
    .align 2
    .globl __isr_vector
__isr_vector:
    /* Cortex M33 Handlers */
    .long   _estack                     /* Top of Stack                       */
    .long   Reset_Handler               /* Reset handler                      */
    .long   NMI_Handler                 /* NMI handler                        */
    .long   HardFault_Handler           /* (Non/)secure Hard Fault handler    */
    .long   MemManage_Handler           /* Memory Management                  */
    .long   BusFault_Handler            /* Pre-fetch or memory access fault   */
    .long   Usage_Fault                 /* Undefined instruction              */
    .long   SecureFault_Handler         /* Secure Fault                       */
    .long   0                           /* Reserved                           */
    .long   0                           /* Reserved                           */
    .long   0                           /* Reserved                           */
    .long   SVC_Handler                 /* SVCall handler                     */
    .long   Debug_Handler               /* Debug Monitor handler              */
    .long   0                           /* Reserved                           */
    .long   PendSV_Handler              /* PendSV handler                     */
    .long   SysTick_Handler             /* SysTick handler                    */
    /* STM32U5 Peripherals Handlers */
    .long   WWDG_Handler                /* Window Watchdog                    */
    .long   PVD_Handler                 /* Power Voltage Detector             */
    .long   RTC_Handler                 /* Real-Time Clock non-secure         */
    .long   RTC_S_Handler               /* Real-Time Clock secure             */
    .long   TAMP_Handler                /* Tamper global interrupt            */
    .long   RAMCFG_Handler              /* RAM configuration                  */
    .long   FLASH_Handler               /* Internal Flash non-secure          */
    .long   FLASH_S_Handler             /* Internal Flash secure              */
    .long   GTZC_Handler                /* Global TrustZone Controler         */
    .long   RCC_Handler                 /* Reset and Clocks non-secure        */
    .long   RCC_S_Handler               /* Reset and Clocks secure            */
    .long   EXTI0_Handler               /* External Interrupt line 0          */
    .long   EXTI1_Handler               /* External Interrupt line 1          */
    .long   EXTI2_Handler               /* External Interrupt line 2          */
    .long   EXTI3_Handler               /* External Interrupt line 3          */
    .long   EXTI4_Handler               /* External Interrupt line 4          */
    .long   EXTI5_Handler               /* External Interrupt line 5          */
    .long   EXTI6_Handler               /* External Interrupt line 6          */
    .long   EXTI7_Handler               /* External Interrupt line 7          */
    .long   EXTI8_Handler               /* External Interrupt line 8          */
    .long   EXTI9_Handler               /* External Interrupt line 9          */
    .long   EXTI10_Handler              /* External Interrupt line 10         */
    .long   EXTI11_Handler              /* External Interrupt line 11         */
    .long   EXTI12_Handler              /* External Interrupt line 12         */
    .long   EXTI13_Handler              /* External Interrupt line 13         */
    .long   EXTI14_Handler              /* External Interrupt line 14         */
    .long   EXTI15_Handler              /* External Interrupt line 15         */
    .long   IWDG_Handler                /* Independent Watchdog interrupt     */
    .long   SAES_Handler                /* Secure AES                         */
    .long   GPDMA1_CH0_Handler          /* General Purpose DMA 1 channel 0    */
    .long   GPDMA1_CH1_Handler          /* General Purpose DMA 1 channel 1    */
    .long   GPDMA1_CH2_Handler          /* General Purpose DMA 1 channel 2    */
    .long   GPDMA1_CH3_Handler          /* General Purpose DMA 1 channel 3    */
    .long   GPDMA1_CH4_Handler          /* General Purpose DMA 1 channel 4    */
    .long   GPDMA1_CH5_Handler          /* General Purpose DMA 1 channel 5    */
    .long   GPDMA1_CH6_Handler          /* General Purpose DMA 1 channel 6    */
    .long   GPDMA1_CH7_Handler          /* General Purpose DMA 1 channel 7    */
    .long   ADC12_Handler               /* Analog to Digital Converter 1      */
    .long   DAC1_Handler                /* Digital to Analog Converter 1      */
    .long   FDCAN1_IT0_Handler          /* Flexible Datarate CAN 1            */
    .long   FDCAN1_IT1_Handler          /* Flexible Datarate CAN 1            */
    .long   TIM1_Handler                /* Timer 1 : Break/Error              */
    .long   TIM1_UP_Handler             /* Timer 1 : Update                   */
    .long   TIM1_TRG_Handler            /* Timer 1 : Trigger/Commutation/Dir  */
    .long   TIM1_CC_Handler             /* Timer 1 : Capture and Compare      */
    .long   TIM2_Handler                /* Timer 2 (all events)               */
    .long   TIM3_Handler                /* Timer 3 (all events)               */
    .long   TIM4_Handler                /* Timer 4 (all events)               */
    .long   TIM5_Handler                /* Timer 5 (all events)               */
    .long   TIM6_Handler                /* Timer 6 (all events)               */
    .long   TIM7_Handler                /* Timer 7 (all events)               */
    .long   TIM8_Handler                /* Timer 8 : Break/Error              */
    .long   TIM8_UP_Handler             /* Timer 8 : Update                   */
    .long   TIM8_TRG_Handler            /* Timer 8 : Trigger/Commutation/Dir  */
    .long   TIM8_CC_Handler             /* Timer 8 : Capture and Compare      */
    .long   I2C1_EV_Handler             /* I2C controller 1 Events            */
    .long   I2C1_ER_Handler             /* I2C controller 1 Errors            */
    .long   I2C2_EV_Handler             /* I2C controller 2 Events            */
    .long   I2C2_ER_Handler             /* I2C controller 2 Errors            */
    .long   SPI1_Handler                /* SPI1 (all events)                  */
    .long   SPI2_Handler                /* SPI2 (all events)                  */
    .long   USART1_Handler              /* USART1 (all events)                */
    .long   USART2_Handler              /* USART2 (all events)                */
    .long   USART3_Handler              /* USART3 (all events)                */
    .long   UART4_Handler               /* UART4 (all events)                 */
    .long   UART5_Handler               /* UART5 (all events)                 */
    .long   LPUART1_Handler             /* Low Power UART1                    */
    .long   LPTIM1_Handler              /* Low Power Timer 1                  */
    .long   LPTIM2_Handler              /* Low Power Timer 2                  */
    .long   TIM15_Handler               /* Timer 15 (all events)              */
    .long   TIM16_Handler               /* Timer 16 (all events)              */
    .long   TIM17_Handler               /* Timer 17 (all events)              */
    .long   COMP_Handler                /* COMP1/COMP2                        */
    .long   USB_Handler                 /* USB global interrupt               */
    .long   CRS_Handler                 /* Clock Recovery System              */
    .long   FMC_Handler                 /* Flexible Memory COntroller         */
    .long   OCTOSPI1_Handler            /* Octo-SPI 1 interface               */
    .long   PWR_S3WU_Handler            /* PWR wake-up from Stop 3 interrupt  */
    .long   SDMMC1_Handler              /* SD/MMC 1 controller                */
    .long   SDMMC2_Handler              /* SD/MMC 2 controller                */
    .long   GPDMA1_CH8_Handler          /* General Purpose DMA 1 channel 8    */
    .long   GPDMA1_CH9_Handler          /* General Purpose DMA 1 channel 9    */
    .long   GPDMA1_CH10_Handler         /* General Purpose DMA 1 channel 10   */
    .long   GPDMA1_CH11_Handler         /* General Purpose DMA 1 channel 11   */
    .long   GPDMA1_CH12_Handler         /* General Purpose DMA 1 channel 12   */
    .long   GPDMA1_CH13_Handler         /* General Purpose DMA 1 channel 13   */
    .long   GPDMA1_CH14_Handler         /* General Purpose DMA 1 channel 14   */
    .long   GPDMA1_CH15_Handler         /* General Purpose DMA 1 channel 15   */
    .long   I2C3_EV_Handler             /* I2C controller 3 Events            */
    .long   I2C3_ER_Handler             /* I2C controller 3 Errors            */
    .long   SAI1_Handler                /* Serial Audio Interface 1           */
    .long   SAI2_Handler                /* Serial Audio Interface 2           */
    .long   TSC_Handler                 /* Touch Sensing Controller (TSC)     */
    .long   AES_Handler                 /* AES coprocessor                    */
    .long   RNG_Handler                 /* Random Number Generator            */
    .long   FPU_Handler                 /* Floating Point Unit                */
    .long   HASH_Handler                /* HASH coprocessor                   */
    .long   PKA_Handler                 /* Public Key Accelerator             */
    .long   LPTIM3_Handler              /* Low Power Timer 3                  */
    .long   SPI3_Handler                /* SPI3 (all events)                  */
    .long   I2C4_EV_Handler             /* I2C controller 4 Events            */
    .long   I2C4_ER_Handler             /* I2C controller 4 Errors            */
    .long   MDF1_FLT0_Handler           /* Multi-function Digital Filter 0    */
    .long   MDF1_FLT1_Handler           /* Multi-function Digital Filter 1    */
    .long   MDF1_FLT2_Handler           /* Multi-function Digital Filter 2    */
    .long   MDF1_FLT3_Handler           /* Multi-function Digital Filter 3    */
    .long   UCPD1_Handler               /* USB-C Power Delivery               */
    .long   ICACHE_Handler              /* Instruction cache global interrupt */
    .long   OTFDEC1_Handler             /* On-the-fly Decryption Engine 1     */
    .long   OTFDEC2_Handler             /* On-the-fly Decryption Engine 2     */
    .long   LPTIM4_Handler              /* Low Power Timer 4                  */
    .long   DCACHE1_Handler             /* Data cache global interrupt        */
    .long   ADF1_FLT0_Handler           /* Audio Digital Filter               */
    .long   ADC4_Handler                /* Analog to Digital Converter 4      */
    .long   LPDMA1_CH0_Handler          /* Low Power DMA                      */
    .long   LPDMA1_CH1_Handler          /* Low Power DMA                      */
    .long   LPDMA1_CH2_Handler          /* Low Power DMA                      */
    .long   LPDMA1_CH3_Handler          /* Low Power DMA                      */
    .long   DMA2D_Handler               /* Chrom-ART Accelerator              */
    .long   DCMI_PSSI_Handler           /* Digital Camera and Parallel Slave  */
    .long   OCTOSPI2_Handler            /* Octo-SPI 2 interface               */
    .long   MDF1_FLT4_Handler           /* Multi-function Digital Filter 4    */
    .long   MDF1_FLT5_Handler           /* Multi-function Digital Filter 5    */
    .long   CORDIC_Handler              /* Cordic coprocessor (math)          */
    .long   FMA_Handler                 /* Filter Math Accelerator            */
    .long   LSECSS_Handler              /* MSECSS / LSI_PLL                   */
    .long   USART6_Handler              /* USART6 (all events)                */
    .long   I2C5_EV_Handler             /* I2C controller 5 Events            */
    .long   I2C5_ER_Handler             /* I2C controller 5 Errors            */
    .long   I2C6_EV_Handler             /* I2C controller 6 Events            */
    .long   I2C6_ER_Handler             /* I2C controller 6 Errors            */
    .long   HSPI_Handler                /* Hexa-deca SPI interface            */
    .long   GPU2D_IRQ_Handler           /* Neo-Chrom Graphic Accelerator      */
    .long   GPU2D_IRQSYS_Handler        /* Neo-Chrom Graphic Accelerator      */
    .long   GFXMMU_Handler              /* Chrom-GRC                          */
    .long   LCD_TFT_Handler             /* LCD-TFT Display Controller (LTDC)  */
    .long   LCD_TFT_ERR_Handler         /* LTDC Errors                        */
    .long   DSIHOST_Handler             /* Display Serial Interface (DSI)     */
    .long   DCACHE2_Handler             /* Data cache for port0 of GPU2D      */
    .long   GFXTIM_Handler              /* Graphic Timer                      */
    .long   JPEG_Handler                /* JPEG Codec                         */
    .size    __isr_vector, . - __isr_vector

    .section .fw_version
    .align 4
    .globl __fw_version
__fw_version:
    .long   0x00000001 /* Firmware version */
    .long   0x00000000 /* Rfu */
    .long   0x00000000 /* Serial Number (rfu) */
    .long   0x00000000 /* Serial Number (rfu) */

/**
 * @brief Reset handler called after a reset
 *
 */
    .text
    .thumb
    .thumb_func
    .align 2
    .globl    Reset_Handler
    .type    Reset_Handler, %function
Reset_Handler:
    /* Copy datas from flash to SRAM */
    ldr    r1, =_sidata
    ldr    r2, =__data_start__
    ldr    r3, =__data_end__
    subs    r3, r2
    ble    .copy_end
.copy_loop:
    subs    r3, #4
    ldr    r0, [r1, r3]
    str    r0, [r2, r3]
    bgt    .copy_loop
.copy_end:
    /* Call C code entry ("main" function) */
    bl  main

/**
 * @brief Default handler is an infinite loop for all unsupported events
 *
 */
.section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
	b	Infinite_Loop
	.size	Default_Handler, .-Default_Handler

/*    Macro to define default handlers. Default handler
 *    will be weak symbol and just dead loops. They can be
 *    overwritten by other handlers */
    .macro    def_default_handler    handler_name
    .align 1
    .thumb_func
    .weak    \handler_name
    .thumb_set \handler_name,Default_Handler
    .endm

    /* Default handlers for Cortex M33 internal blocks */
    def_default_handler    NMI_Handler
    def_default_handler    HardFault_Handler
    def_default_handler    MemManage_Handler
    def_default_handler    BusFault_Handler
    def_default_handler    Usage_Fault
    def_default_handler    SecureFault_Handler
    def_default_handler    SVC_Handler
    def_default_handler    Debug_Handler
    def_default_handler    PendSV_Handler
    def_default_handler    SysTick_Handler
    /* Default handlers for STM32U5 peripherals */
    def_default_handler    WWDG_Handler
    def_default_handler    PVD_Handler
    def_default_handler    RTC_Handler
    def_default_handler    RTC_S_Handler
    def_default_handler    TAMP_Handler
    def_default_handler    RAMCFG_Handler
    def_default_handler    FLASH_Handler
    def_default_handler    FLASH_S_Handler
    def_default_handler    GTZC_Handler
    def_default_handler    RCC_Handler
    def_default_handler    RCC_S_Handler
    def_default_handler    EXTI0_Handler
    def_default_handler    EXTI1_Handler
    def_default_handler    EXTI2_Handler
    def_default_handler    EXTI3_Handler
    def_default_handler    EXTI4_Handler
    def_default_handler    EXTI5_Handler
    def_default_handler    EXTI6_Handler
    def_default_handler    EXTI7_Handler
    def_default_handler    EXTI8_Handler
    def_default_handler    EXTI9_Handler
    def_default_handler    EXTI10_Handler
    def_default_handler    EXTI11_Handler
    def_default_handler    EXTI12_Handler
    def_default_handler    EXTI13_Handler
    def_default_handler    EXTI14_Handler
    def_default_handler    EXTI15_Handler
    def_default_handler    IWDG_Handler
    def_default_handler    SAES_Handler
    def_default_handler    GPDMA1_CH0_Handler
    def_default_handler    GPDMA1_CH1_Handler
    def_default_handler    GPDMA1_CH2_Handler
    def_default_handler    GPDMA1_CH3_Handler
    def_default_handler    GPDMA1_CH4_Handler
    def_default_handler    GPDMA1_CH5_Handler
    def_default_handler    GPDMA1_CH6_Handler
    def_default_handler    GPDMA1_CH7_Handler
    def_default_handler    ADC12_Handler
    def_default_handler    DAC1_Handler
    def_default_handler    FDCAN1_IT0_Handler
    def_default_handler    FDCAN1_IT1_Handler
    def_default_handler    TIM1_Handler
    def_default_handler    TIM1_UP_Handler
    def_default_handler    TIM1_TRG_Handler
    def_default_handler    TIM1_CC_Handler
    def_default_handler    TIM2_Handler
    def_default_handler    TIM3_Handler
    def_default_handler    TIM4_Handler
    def_default_handler    TIM5_Handler
    def_default_handler    TIM6_Handler
    def_default_handler    TIM7_Handler
    def_default_handler    TIM8_Handler
    def_default_handler    TIM8_UP_Handler
    def_default_handler    TIM8_TRG_Handler
    def_default_handler    TIM8_CC_Handler
    def_default_handler    I2C1_EV_Handler
    def_default_handler    I2C1_ER_Handler
    def_default_handler    I2C2_EV_Handler
    def_default_handler    I2C2_ER_Handler
    def_default_handler    SPI1_Handler
    def_default_handler    SPI2_Handler
    def_default_handler    USART1_Handler
    def_default_handler    USART2_Handler
    def_default_handler    USART3_Handler
    def_default_handler    UART4_Handler
    def_default_handler    UART5_Handler
    def_default_handler    LPUART1_Handler
    def_default_handler    LPTIM1_Handler
    def_default_handler    LPTIM2_Handler
    def_default_handler    TIM15_Handler
    def_default_handler    TIM16_Handler
    def_default_handler    TIM17_Handler
    def_default_handler    COMP_Handler
    def_default_handler    USB_Handler
    def_default_handler    CRS_Handler
    def_default_handler    FMC_Handler
    def_default_handler    OCTOSPI1_Handler
    def_default_handler    PWR_S3WU_Handler
    def_default_handler    SDMMC1_Handler
    def_default_handler    SDMMC2_Handler
    def_default_handler    GPDMA1_CH8_Handler
    def_default_handler    GPDMA1_CH9_Handler
    def_default_handler    GPDMA1_CH10_Handler
    def_default_handler    GPDMA1_CH11_Handler
    def_default_handler    GPDMA1_CH12_Handler
    def_default_handler    GPDMA1_CH13_Handler
    def_default_handler    GPDMA1_CH14_Handler
    def_default_handler    GPDMA1_CH15_Handler
    def_default_handler    I2C3_EV_Handler
    def_default_handler    I2C3_ER_Handler
    def_default_handler    SAI1_Handler
    def_default_handler    SAI2_Handler
    def_default_handler    TSC_Handler
    def_default_handler    AES_Handler
    def_default_handler    RNG_Handler
    def_default_handler    FPU_Handler
    def_default_handler    HASH_Handler
    def_default_handler    PKA_Handler
    def_default_handler    LPTIM3_Handler
    def_default_handler    SPI3_Handler
    def_default_handler    I2C4_EV_Handler
    def_default_handler    I2C4_ER_Handler
    def_default_handler    MDF1_FLT0_Handler
    def_default_handler    MDF1_FLT1_Handler
    def_default_handler    MDF1_FLT2_Handler
    def_default_handler    MDF1_FLT3_Handler
    def_default_handler    UCPD1_Handler
    def_default_handler    ICACHE_Handler
    def_default_handler    OTFDEC1_Handler
    def_default_handler    OTFDEC2_Handler
    def_default_handler    LPTIM4_Handler
    def_default_handler    DCACHE1_Handler
    def_default_handler    ADF1_FLT0_Handler
    def_default_handler    ADC4_Handler
    def_default_handler    LPDMA1_CH0_Handler
    def_default_handler    LPDMA1_CH1_Handler
    def_default_handler    LPDMA1_CH2_Handler
    def_default_handler    LPDMA1_CH3_Handler
    def_default_handler    DMA2D_Handler
    def_default_handler    DCMI_PSSI_Handler
    def_default_handler    OCTOSPI2_Handler
    def_default_handler    MDF1_FLT4_Handler
    def_default_handler    MDF1_FLT5_Handler
    def_default_handler    CORDIC_Handler
    def_default_handler    FMA_Handler
    def_default_handler    LSECSS_Handler
    def_default_handler    USART6_Handler
    def_default_handler    I2C5_EV_Handler
    def_default_handler    I2C5_ER_Handler
    def_default_handler    I2C6_EV_Handler
    def_default_handler    I2C6_ER_Handler
    def_default_handler    HSPI_Handler
    def_default_handler    GPU2D_IRQ_Handler
    def_default_handler    GPU2D_IRQSYS_Handler
    def_default_handler    GFXMMU_Handler
    def_default_handler    LCD_TFT_Handler
    def_default_handler    LCD_TFT_ERR_Handler
    def_default_handler    DSIHOST_Handler
    def_default_handler    DCACHE2_Handler
    def_default_handler    GFXTIM_Handler
    def_default_handler    JPEG_Handler
.end
