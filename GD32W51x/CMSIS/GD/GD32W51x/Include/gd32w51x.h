/*!
    \file    gd32w51x.h
    \brief   general definitions for GD32W51x
    
    \version 2021-03-25, V1.0.0, firmware for GD32W51x
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#ifndef GD32W51x_H
#define GD32W51x_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* define value of high speed crystal oscillator (HXTAL) in Hz */
#if !defined  (HXTAL_VALUE)
#define HXTAL_VALUE    ((uint32_t)40000000)
#endif /* high speed crystal oscillator value */

/* define startup timeout value of high speed crystal oscillator (HXTAL) */
#if !defined  (HXTAL_STARTUP_TIMEOUT)
#define HXTAL_STARTUP_TIMEOUT   ((uint16_t)0xFFFFU)
#endif /* high speed crystal oscillator startup timeout */

/* define value of internal 16MHz RC oscillator (IRC16M) in Hz */
#if !defined  (IRC16M_VALUE) 
#define IRC16M_VALUE  ((uint32_t)16000000)
#endif /* internal 16MHz RC oscillator value */

/* define startup timeout value of internal 16MHz RC oscillator (IRC16M) */
#if !defined  (IRC16M_STARTUP_TIMEOUT)
#define IRC16M_STARTUP_TIMEOUT   ((uint16_t)0x0500U)
#endif /* internal 16MHz RC oscillator startup timeout */

/* define value of internal 32KHz RC oscillator(IRC32K) in Hz */
#if !defined  (IRC32K_VALUE) 
#define IRC32K_VALUE  ((uint32_t)32000)
#endif /* internal 32KHz RC oscillator value */

/* define value of low speed crystal oscillator (LXTAL)in Hz */
#if !defined  (LXTAL_VALUE) 
#define LXTAL_VALUE  ((uint32_t)32768)
#endif /* low speed crystal oscillator value */

/* GD32W51x firmware library version number V1.0 */
#define __GD32W51x_STDPERIPH_VERSION_MAIN   (0x03) /*!< [31:24] main version */
#define __GD32W51x_STDPERIPH_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __GD32W51x_STDPERIPH_VERSION_SUB2   (0x00) /*!< [15:8]  sub2 version */
#define __GD32W51x_STDPERIPH_VERSION_RC     (0x00) /*!< [7:0]  release candidate */ 
#define __GD32W51x_STDPERIPH_VERSION        ((__GD32W51x_STDPERIPH_VERSION_MAIN << 24)\
                                            |(__GD32W51x_STDPERIPH_VERSION_SUB1 << 16)\
                                            |(__GD32W51x_STDPERIPH_VERSION_SUB2 << 8)\
                                            |(__GD32W51x_STDPERIPH_VERSION_RC))


/* Configuration of the Cortex-M33 Processor and Core Peripherals */
#define __CM33_REV                0x0000U   /*!< Core revision r0p1 */
#define __SAUREGION_PRESENT       1U        /*!< SAU regions present */
#define __MPU_PRESENT             1U        /*!< MPU present */
#define __VTOR_PRESENT            1U        /*!< VTOR present */
#define __NVIC_PRIO_BITS          4U        /*!< Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /*!< Set to 1 if different SysTick Config is used */
#define __FPU_PRESENT             1U        /*!< FPU present */
#define __DSP_PRESENT             1U        /*!< DSP extension present */
/* define interrupt number */
typedef enum IRQn
{
    /* Cortex-M33 processor exceptions numbers */
    Reset_IRQn                  = -15,    /*!< -15 Reset Vector, invoked on Power up and warm reset */
    NonMaskableInt_IRQn         = -14,    /*!< -14 Non maskable Interrupt, cannot be stopped or preempted */
    HardFault_IRQn              = -13,    /*!< -13 Hard Fault, all classes of Fault */
    MemoryManagement_IRQn       = -12,    /*!< -12 Memory Management, MPU mismatch, including Access Violation and No Match */
    BusFault_IRQn               = -11,    /*!< -11 Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault */
    UsageFault_IRQn             = -10,    /*!< -10 Usage Fault, i.e. Undef Instruction, Illegal State Transition */
    SecureFault_IRQn            =  -9,    /*!< -9  Secure Fault */
    SVCall_IRQn                 =  -5,    /*!< -5  System Service Call via SVC instruction */
    DebugMonitor_IRQn           =  -4,    /*!< -4  Debug Monitor */
    PendSV_IRQn                 =  -2,    /*!< -2  Pendable request for system service */
    SysTick_IRQn                =  -1,    /*!< -1  System Tick Timer */

    /* interruput numbers */              
    WWDGT_IRQn                  = 0,      /*!< window watchdog timer interrupt */
    LVD_IRQn                    = 1,      /*!< LVD through EXTI line detect interrupt */
    TAMPER_STAMP_IRQn           = 2,      /*!< tamper and timestamp through EXTI line detect */
    RTC_WKUP_IRQn               = 3,      /*!< RTC wakeup through EXTI line interrupt */
    FMC_IRQn                    = 4,      /*!< FMC interrupt */
    RCU_IRQn                    = 5,      /*!< RCU interrupt */
    EXTI0_IRQn                  = 6,      /*!< EXTI line 0 interrupts */
    EXTI1_IRQn                  = 7,      /*!< EXTI line 1 interrupts */
    EXTI2_IRQn                  = 8,      /*!< EXTI line 2 interrupts */
    EXTI3_IRQn                  = 9,      /*!< EXTI line 3 interrupts */
    EXTI4_IRQn                  = 10,     /*!< EXTI line 4 interrupts */
    DMA0_Channel0_IRQn          = 11,     /*!< DMA0 channel0 Interrupt */
    DMA0_Channel1_IRQn          = 12,     /*!< DMA0 channel1 Interrupt */
    DMA0_Channel2_IRQn          = 13,     /*!< DMA0 channel2 interrupt */
    DMA0_Channel3_IRQn          = 14,     /*!< DMA0 channel3 interrupt */
    DMA0_Channel4_IRQn          = 15,     /*!< DMA0 channel4 interrupt */
    DMA0_Channel5_IRQn          = 16,     /*!< DMA0 channel5 interrupt */
    DMA0_Channel6_IRQn          = 17,     /*!< DMA0 channel6 interrupt */
    DMA0_Channel7_IRQn          = 18,     /*!< DMA0 channel7 interrupt */
    ADC_IRQn                    = 19,     /*!< ADC interrupt */
    TAMPER_STAMP_S_IRQn         = 20,     /*!< RTC tamper and TimeStamp events security interrupt*/
    RTC_WKUP_S_IRQn             = 21,     /*!< RTC wakeup security interrupt*/
    RTC_Alarm_S_IRQn            = 22,     /*!< RTC Alarm security interrupt */
    EXTI5_9_IRQn                = 23,     /*!< EXTI[9:5] interrupts */
    TIMER0_BRK_IRQn             = 24,     /*!< TIMER0 break interrupts */
    TIMER0_UP_IRQn              = 25,     /*!< TIMER0 update interrupts */
    TIMER0_CMT_IRQn             = 26,     /*!< TIMER0 commutation interrupts */
    TIMER0_Channel_IRQn         = 27,     /*!< TIMER0 channel capture compare interrupt */
    TIMER1_IRQn                 = 28,     /*!< TIMER1 interrupt */
    TIMER2_IRQn                 = 29,     /*!< TIMER2 interrupt */
    TIMER3_IRQn                 = 30,     /*!< TIMER3 interrupts */
    I2C0_EV_IRQn                = 31,     /*!< I2C0 event interrupt */
    I2C0_ER_IRQn                = 32,     /*!< I2C0 error interrupt */
    I2C1_EV_IRQn                = 33,     /*!< I2C1 event interrupt */
    I2C1_ER_IRQn                = 34,     /*!< I2C1 error interrupt */
    SPI0_IRQn                   = 35,     /*!< SPI0 interrupt */
    SPI1_IRQn                   = 36,     /*!< SPI1 and I2S1 interrupt */
    USART0_IRQn                 = 37,     /*!< USART0 interrupt */
    USART1_IRQn                 = 38,     /*!< USART1 interrupt */
    USART2_IRQn                 = 39,     /*!< USART2 interrupt */
    EXTI10_15_IRQn              = 40,     /*!< EXTI[15:10] interrupts */
    RTC_Alarm_IRQn              = 41,     /*!< RTC alarm interrupt */
    VLVDF_IRQn                  = 42,     /*!< VLVDF interrupts */
    TIMER15_IRQn                = 44,     /*!< TIMER15 global interrupt */
    TIMER16_IRQn                = 45,     /*!< TIMER16 global interrupt */
    SDIO_IRQn                   = 49,     /*!< SDIO global interrupt */
    TIMER4_IRQn                 = 50,     /*!< TIMER4 global interrupt */
    I2C0_WKUP_IRQn              = 51,     /*!< I2C0 Wakeup interrupt*/ 
    USART0_WKUP_IRQn            = 52,     /*!< USART0 Wakeup */
    USART2_WKUP_IRQn            = 53,     /*!< USART2 Wakeup */
    TIMER5_IRQn                 = 54,     /*!< TIMER5 global interrupt */ 
    DMA1_Channel0_IRQn          = 56,     /*!< DMA1 channel0 interrupt */
    DMA1_Channel1_IRQn          = 57,     /*!< DMA1 channel1 interrupt */
    DMA1_Channel2_IRQn          = 58,     /*!< DMA1 channel2 interrupt */
    DMA1_Channel3_IRQn          = 59,     /*!< DMA1 channel3 interrupt */
    DMA1_Channel4_IRQn          = 60,     /*!< DMA1 channel4 interrupt */
    DMA1_Channel5_IRQn          = 61,     /*!< DMA1 channel5 interrupt */
    DMA1_Channel6_IRQn          = 62,     /*!< DMA1 channel6 interrupt */
    DMA1_Channel7_IRQn          = 63,     /*!< DMA1 channel7 interrupt */
    WIFI11N_WKUP_IRQn           = 66,     /*!< WIFI11N wakeup */
    USBFS_IRQn                  = 67,     /*!< USBFS interrupt */
    USBFS_WKUP_IRQn             = 76,     /*!< USBFS wakeup through EXTI line interrupt */
    CAU_IRQn                    = 79,     /*!< CAU interrupt */
    HAU_TRNG_IRQn               = 80,     /*!< HAU and TRNG interrupt */
    FPU_IRQn                    = 81,     /*!< FPU interrupt */
    WIFI11N_INT0_IRQn           = 91,     /*!< WIFI11N global interrupt0 */
    WIFI11N_INT1_IRQn           = 92,     /*!< WIFI11N global interrupt1 */
    WIFI11N_INT2_IRQn           = 93,     /*!< WIFI11N global interrupt2 */
    EFUSE_IRQn                  = 94,     /*!< EFUSE global interrupt */
    QSPI_IRQn                   = 95,     /*!< QSPI global interrupt */
    PKCAU_IRQn                  = 96,     /*!< PKCAU global interrupt */
    TSI_IRQ                     = 97,     /*!< TSI global interrupt */
    ICACHE_IRQn                 = 98,     /*!< ICACHE global interrupt */
    TZIAC_S_IRQn                = 99,     /*!< TrustZone Interrupt Controller secure interrupts */
    FMC_S_IRQn                  = 100,    /*!< FMC secure interrupt */
    QSPI_S_IRQn                = 101,     /*!< QSPI security interrupt */

#if defined (GD32W515PI) || defined (GD32W515P0)
    DCI_IRQn                    = 78,     /*!< DCI interrupt */
    HPDF_INT0_IRQn              = 89,     /*!< HPDF global Interrupt 0 */
    HPDF_INT1_IRQn              = 90,     /*!< HPDF global Interrupt 1 */
#endif /* GD32W515PI and GD32W515P0 */
} IRQn_Type;

/* includes */
#include "core_cm33.h"
#include "system_gd32w51x.h"
#include <stdint.h>

/* enum definitions */
typedef enum {DISABLE = 0, ENABLE = !DISABLE} EventStatus, ControlStatus;
typedef enum {RESET = 0, SET = !RESET} FlagStatus;
typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrStatus;

/* bit operations */
#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))
#define BIT(x)                       ((uint32_t)((uint32_t)0x00000001U<<(x)))
#define BITS(start, end)             ((0xFFFFFFFFUL << (start)) & (0xFFFFFFFFUL >> (31U - (uint32_t)(end)))) 
#define GET_BITS(regval, start, end) (((regval) & BITS((start),(end))) >> (start))

/* main flash and SRAM memory map */
#define FLASH_BASE_NS            ((uint32_t)0x08000000U)           /*!< main FLASH base address */
#define SRAM_BASE_NS             ((uint32_t)0x20000000U)           /*!< SRAM base address */
#define SRAM0_BASE_NS            ((uint32_t)0x20000000U)           /*!< SRAM0 base address */
#define SRAM1_BASE_NS            ((uint32_t)0x20010000U)           /*!< SRAM1 base address */
#define SRAM2_BASE_NS            ((uint32_t)0x20020000U)           /*!< SRAM2 base address */
#define SRAM3_BASE_NS            ((uint32_t)0x20040000U)           /*!< SRAM3 base address */

/* peripheral memory map */                                       
#define APB1_BUS_BASE_NS         ((uint32_t)0x40000000U)           /*!< apb1 base address */
#define APB2_BUS_BASE_NS         ((uint32_t)0x40010000U)           /*!< apb2 base address */
#define AHB1_BUS_BASE_NS         ((uint32_t)0x40020000U)           /*!< ahb1 base address */
#define AHB2_BUS_BASE_NS         ((uint32_t)0x4C000000U)           /*!< ahb2 base address */

/* SQPI_PSRAM memory map */                                       
#define SQPI_PSRAM               ((uint32_t)0x60000000U)           /*!< SQPI_PSRAM memory map base address */
/* QSPI_FLASH memory map */                                       
#define QSPI_FLASH               ((uint32_t)0x90000000U)           /*!< QSPI_FLASH base address */
/* advanced peripheral bus 1 memory map */
#define TIMER_BASE_NS            (APB1_BUS_BASE_NS + 0x00000000U)  /*!< TIMER base address */
#define RTC_BASE_NS              (APB1_BUS_BASE_NS + 0x00002800U)  /*!< RTC base address */
#define WWDGT_BASE_NS            (APB1_BUS_BASE_NS + 0x00002C00U)  /*!< WWDGT base address */
#define FWDGT_BASE_NS            (APB1_BUS_BASE_NS + 0x00003000U)  /*!< FWDGT base address */
#define I2S_ADD_BASE_NS          (APB1_BUS_BASE_NS + 0x00003400U)  /*!< I2S_add base address */
#define SPI_BASE_NS              (APB1_BUS_BASE_NS + 0x00003800U)  /*!< SPI base address */
#define USART_BASE_NS            (APB1_BUS_BASE_NS + 0x00004400U)  /*!< USART base address */
#define I2C_BASE_NS              (APB1_BUS_BASE_NS + 0x00005400U)  /*!< I2C base address */
#define PMU_BASE_NS              (APB1_BUS_BASE_NS + 0x00007000U)  /*!< PMU base address */

/* advanced peripheral bus 2 memory map */
#define ADC_BASE_NS              (APB2_BUS_BASE_NS + 0x00002000U)  /*!< ADC base address */
#define SDIO_BASE_NS             (APB2_BUS_BASE_NS + 0x00002C00U)  /*!< SDIO base address */
#define EXTI_BASE_NS             (APB2_BUS_BASE_NS + 0x00003C00U)  /*!< EXTI base address */
#define SYSCFG_BASE_NS           (APB2_BUS_BASE_NS + 0x00003800U)  /*!< SYSCFG base address */
#define HPDF_BASE_NS             (APB2_BUS_BASE_NS + 0x00006000U)  /*!< HPDF base address */
#define WIFI_RF_BASE_NS          (APB2_BUS_BASE_NS + 0x00007800U)  /*!< WIFI base address */

/* advanced high performance bus 1 memory map */
#define GPIO_BASE_NS             (AHB1_BUS_BASE_NS + 0x00000000U)  /*!< GPIO base address */
#define FMC_BASE_NS              (AHB1_BUS_BASE_NS + 0x00002000U)  /*!< FMC base address */
#define EFUSE_BASE_NS            (AHB1_BUS_BASE_NS + 0x00002800U)  /*!< EFUSE base address */
#define CRC_BASE_NS              (AHB1_BUS_BASE_NS + 0x00003000U)  /*!< CRC base address */
#define RCU_BASE_NS              (AHB1_BUS_BASE_NS + 0x00003800U)  /*!< RCU base address */
#define TSI_BASE_NS              (AHB1_BUS_BASE_NS + 0x00004000U)  /*!< TSI base address */
#define SQPI_REG_BASE_NS         (AHB1_BUS_BASE_NS + 0x00005400U)  /*!< SQPI base address */
#define QSPI_REG_BASE_NS         (AHB1_BUS_BASE_NS + 0x00005800U)  /*!< QSPI base address */
#define DMA_BASE_NS              (AHB1_BUS_BASE_NS + 0x00006000U)  /*!< DMA base address */
#define WIFI_BASE_NS             (AHB1_BUS_BASE_NS + 0x00010000U)  /*!< WIFI11N base address */
#define ICACHE_BASE_NS           (AHB1_BUS_BASE_NS + 0x00060000U)  /*!< ICACHE base address */
#define TZSPC_BASE_NS            (AHB1_BUS_BASE_NS + 0x00080000U)  /*!< TZSPC base address */
#define TZIAC_BASE_NS            (AHB1_BUS_BASE_NS + 0x00080400U)  /*!< TZIAC base address */
#define TZBMPC0_BASE_NS          (AHB1_BUS_BASE_NS + 0x00080800U)  /*!< TZBMPC0 base address */
#define TZBMPC1_BASE_NS          (AHB1_BUS_BASE_NS + 0x00080C00U)  /*!< TZBMPC1 base address */
#define TZBMPC2_BASE_NS          (AHB1_BUS_BASE_NS + 0x00090000U)  /*!< TZBMPC2 base address */
#define TZBMPC3_BASE_NS          (AHB1_BUS_BASE_NS + 0x00090400U)  /*!< TZBMPC3 base address */
#define USBFS_BASE_NS            (AHB1_BUS_BASE_NS + 0x08FE0000U)  /*!< USBFS base address */

/* advanced high performance bus 2 memory map */
#define DCI_BASE_NS              (AHB2_BUS_BASE_NS + 0x00050000U)  /*!< DCI base address */
#define CAU_BASE_NS              (AHB2_BUS_BASE_NS + 0x00060000U)  /*!< CAU base address */
#define HAU_BASE_NS              (AHB2_BUS_BASE_NS + 0x00060400U)  /*!< HAU base address */
#define TRNG_BASE_NS             (AHB2_BUS_BASE_NS + 0x00060800U)  /*!< TRNG base address */
#define PKCAU_BASE_NS            (AHB2_BUS_BASE_NS + 0x00061000U)  /*!< PKCAU base address */

/* main flash and SRAM memory map */
#define FLASH_BASE_S            ((uint32_t)0x0C000000U)            /*!< main FLASH base address */
#define SRAM0_BASE_S            ((uint32_t)0x30000000U)            /*!< SRAM0 base address */
#define SRAM1_BASE_S            ((uint32_t)0x30010000U)            /*!< SRAM1 bytes base address */
#define SRAM2_BASE_S            ((uint32_t)0x30020000U)            /*!< SRAM2 base address */
#define SRAM3_BASE_S            ((uint32_t)0x30040000U)            /*!< SRAM3 base address */

/* peripheral memory map */
#define APB1_BUS_BASE_S         ((uint32_t)0x50000000U)            /*!< apb1 base address */
#define APB2_BUS_BASE_S         ((uint32_t)0x50010000U)            /*!< apb2 base address */
#define AHB1_BUS_BASE_S         ((uint32_t)0x50020000U)            /*!< ahb1 base address */
#define AHB2_BUS_BASE_S         ((uint32_t)0x5C000000U)            /*!< ahb2 base address */

/* advanced peripheral bus 1 memory map */
#define TIMER_BASE_S            (APB1_BUS_BASE_S + 0x00000000U)    /*!< TIMER base address */
#define RTC_BASE_S              (APB1_BUS_BASE_S + 0x00002800U)    /*!< RTC base address */
#define WWDGT_BASE_S            (APB1_BUS_BASE_S + 0x00002C00U)    /*!< WWDGT base address */
#define FWDGT_BASE_S            (APB1_BUS_BASE_S + 0x00003000U)    /*!< FWDGT base address */
#define I2S_ADD_BASE_S          (APB1_BUS_BASE_S + 0x00003400U)    /*!< I2S base address */
#define SPI_BASE_S              (APB1_BUS_BASE_S + 0x00003800U)    /*!< SPI base address */
#define USART_BASE_S            (APB1_BUS_BASE_S + 0x00004400U)    /*!< USART base address */
#define I2C_BASE_S              (APB1_BUS_BASE_S + 0x00005400U)    /*!< I2C base address */
#define PMU_BASE_S              (APB1_BUS_BASE_S + 0x00007000U)    /*!< PMU base address */

/* advanced peripheral bus 2 memory map */
#define WIFI_RF_BASE_S          (APB2_BUS_BASE_S + 0x00007800U)    /*!< WIFI base address */
#define HPDF_BASE_S             (APB2_BUS_BASE_S + 0x00006000U)    /*!< HPDF base address */
#define SYSCFG_BASE_S           (APB2_BUS_BASE_S + 0x00003800U)    /*!< SYSCFG base address */
#define EXTI_BASE_S             (APB2_BUS_BASE_S + 0x00003C00U)    /*!< EXTI base address */
#define SDIO_BASE_S             (APB2_BUS_BASE_S + 0x00002C00U)    /*!< SDIO base address */
#define ADC_BASE_S              (APB2_BUS_BASE_S + 0x00002000U)    /*!< ADC base address */

/* advanced high performance bus 1 memory map */
#define GPIO_BASE_S             (AHB1_BUS_BASE_S + 0x00000000U)    /*!< GPIO base address */
#define FMC_BASE_S              (AHB1_BUS_BASE_S + 0x00002000U)    /*!< FMC base address */
#define EFUSE_BASE_S            (AHB1_BUS_BASE_S + 0x00002800U)    /*!< EFUSE base address */
#define CRC_BASE_S              (AHB1_BUS_BASE_S + 0x00003000U)    /*!< CRC base address */
#define RCU_BASE_S              (AHB1_BUS_BASE_S + 0x00003800U)    /*!< RCU base address */
#define TSI_BASE_S              (AHB1_BUS_BASE_S + 0x00004000U)    /*!< TSI base address */
#define SQPI_REG_BASE_S         (AHB1_BUS_BASE_S + 0x00005400U)    /*!< SQPI base address */
#define QSPI_REG_BASE_S         (AHB1_BUS_BASE_S + 0x00005800U)    /*!< QSPI base address */
#define DMA_BASE_S              (AHB1_BUS_BASE_S + 0x00006000U)    /*!< DMA base address */
#define WIFI11N_BASE_S          (AHB1_BUS_BASE_S + 0x00010000U)    /*!< WIFI11N base address */
#define ICACHE_BASE_S           (AHB1_BUS_BASE_S + 0x00060000U)    /*!< ICACHE base address */
#define TZSPC_BASE_S            (AHB1_BUS_BASE_S + 0x00080000U)    /*!< TZSPC base address */
#define TZIAC_BASE_S            (AHB1_BUS_BASE_S + 0x00080400U)    /*!< TZIAC base address */
#define TZBMPC0_BASE_S          (AHB1_BUS_BASE_S + 0x00080800U)    /*!< TZBMPC0 base address */
#define TZBMPC1_BASE_S          (AHB1_BUS_BASE_S + 0x00080C00U)    /*!< TZBMPC1 base address */
#define TZBMPC2_BASE_S          (AHB1_BUS_BASE_S + 0x00090000U)    /*!< TZBMPC2 base address */
#define TZBMPC3_BASE_S          (AHB1_BUS_BASE_S + 0x00090400U)    /*!< TZBMPC3 base address */
#define USBFS_BASE_S            (AHB1_BUS_BASE_S + 0x08FE0000U)    /*!< USBFS base address */

/* advanced high performance bus 2 memory map */
#define DCI_BASE_S              (AHB2_BUS_BASE_S + 0x00050000U)    /*!< DCI base address */
#define CAU_BASE_S              (AHB2_BUS_BASE_S + 0x00060000U)    /*!< CAU base address */
#define HAU_BASE_S              (AHB2_BUS_BASE_S + 0x00060400U)    /*!< HAU base address */
#define TRNG_BASE_S             (AHB2_BUS_BASE_S + 0x00060800U)    /*!< TRNG base address */
#define PKCAU_BASE_S            (AHB2_BUS_BASE_S + 0x00061000U)    /*!< PKCAU base address */

/* debug memory map */
#define DBG_BASE                ((uint32_t)0xE0044000U)            /*!< DBG base address */ 

#if defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U)
/* main flash and SRAM memory map */
#define FLASH_BASE            FLASH_BASE_S                         /*!< main FLASH base address */
#define SRAM0_BASE            SRAM0_BASE_S                         /*!< SRAM0 base address */
#define SRAM1_BASE            SRAM1_BASE_S                         /*!< SRAM1 base address */
#define SRAM2_BASE            SRAM2_BASE_S                         /*!< SRAM2 base address */
#define SRAM3_BASE            SRAM3_BASE_S                         /*!< SRAM3 base address */

/* peripheral memory map */
#define APB1_BUS_BASE         APB1_BUS_BASE_S                      /*!< apb1 base address */
#define APB2_BUS_BASE         APB2_BUS_BASE_S                      /*!< apb2 base address */
#define AHB1_BUS_BASE         AHB1_BUS_BASE_S                      /*!< ahb1 base address */
#define AHB2_BUS_BASE         AHB2_BUS_BASE_S                      /*!< ahb2 base address */

/* advanced peripheral bus 1 memory map */
#define TIMER_BASE            TIMER_BASE_S                         /*!< TIMER base address */
#define RTC_BASE              RTC_BASE_S                           /*!< RTC base address */
#define WWDGT_BASE            WWDGT_BASE_S                         /*!< WWDGT base address */
#define FWDGT_BASE            FWDGT_BASE_S                         /*!< FWDGT base address */
#define I2S_ADD_BASE          I2S_ADD_BASE_S                       /*!< I2S1_add base address */
#define SPI_BASE              SPI_BASE_S                           /*!< SPI base address */
#define USART_BASE            USART_BASE_S                         /*!< USART base address */
#define I2C_BASE              I2C_BASE_S                           /*!< I2C base address */
#define PMU_BASE              PMU_BASE_S                           /*!< PMU base address */

/* advanced peripheral bus 2 memory map */
#define WIFI_RF_BASE          WIFI_RF_BASE_S                       /*!< WIFI base address */
#define HPDF_BASE             HPDF_BASE_S                          /*!< HPDF base address */
#define SYSCFG_BASE           SYSCFG_BASE_S                        /*!< SYSCFG base address */
#define EXTI_BASE             EXTI_BASE_S                          /*!< EXTI base address */
#define SDIO_BASE             SDIO_BASE_S                          /*!< SDIO base address */
#define ADC_BASE              ADC_BASE_S                           /*!< ADC base address */
/* advanced high performance bus 1 memory map */
#define GPIO_BASE             GPIO_BASE_S                          /*!< GPIO base address */
#define FMC_BASE              FMC_BASE_S                           /*!< FMC base address */
#define EFUSE_BASE            EFUSE_BASE_S                         /*!< EFUSE base address */
#define CRC_BASE              CRC_BASE_S                           /*!< CRC base address */
#define RCU_BASE              RCU_BASE_S                           /*!< RCU base address */
#define TSI_BASE              TSI_BASE_S                           /*!< TSI base address */
#define SQPI_REG_BASE         SQPI_REG_BASE_S                      /*!< SQPI base address */
#define QSPI_REG_BASE         QSPI_REG_BASE_S                      /*!< QSPI base address */
#define DMA_BASE              DMA_BASE_S                           /*!< DMA base address */
#define WIFI11N_BASE          WIFI11N_BASE_S                       /*!< WIFI11N base address */
#define ICACHE_BASE           ICACHE_BASE_S                        /*!< ICACHE base address */
#define TZSPC_BASE            TZSPC_BASE_S                         /*!< TZSPC base address */
#define TZIAC_BASE            TZIAC_BASE_S                         /*!< TZIAC base address */
#define TZBMPC0_BASE          TZBMPC0_BASE_S                       /*!< TZBMPC0 base address */
#define TZBMPC1_BASE          TZBMPC1_BASE_S                       /*!< TZBMPC1 base address */
#define TZBMPC2_BASE          TZBMPC2_BASE_S                       /*!< TZBMPC2 base address */
#define TZBMPC3_BASE          TZBMPC3_BASE_S                       /*!< TZBMPC3 base address */
#define USBFS_BASE            USBFS_BASE_S                         /*!< USBFS base address */

/* advanced high performance bus 2 memory map */
#define DCI_BASE              DCI_BASE_S                           /*!< DCI base address */
#define CAU_BASE              CAU_BASE_S                           /*!< CAU base address */
#define HAU_BASE              HAU_BASE_S                           /*!< HAU base address */
#define TRNG_BASE             TRNG_BASE_S                          /*!< TRNG base address */
#define PKCAU_BASE            PKCAU_BASE_S                         /*!< PKCAU base address */

#else
/* main flash and SRAM memory map */
#define FLASH_BASE            FLASH_BASE_NS                        /*!< main FLASH base address */
#define SRAM0_BASE            SRAM0_BASE_NS                        /*!< SRAM0 base address */
#define SRAM1_BASE            SRAM1_BASE_NS                        /*!< SRAM1 base address */
#define SRAM2_BASE            SRAM2_BASE_NS                        /*!< SRAM2 base address */
#define SRAM3_BASE            SRAM3_BASE_NS                        /*!< SRAM3 base address */

/* peripheral memory map */
#define APB1_BUS_BASE         APB1_BUS_BASE_NS                     /*!< apb1 base address */
#define APB2_BUS_BASE         APB2_BUS_BASE_NS                     /*!< apb2 base address */
#define AHB1_BUS_BASE         AHB1_BUS_BASE_NS                     /*!< ahb1 base address */
#define AHB2_BUS_BASE         AHB2_BUS_BASE_NS                     /*!< ahb2 base address */

/* advanced peripheral bus 1 memory map */
#define TIMER_BASE            TIMER_BASE_NS                        /*!< TIMER base address */
#define RTC_BASE              RTC_BASE_NS                          /*!< RTC base address */
#define WWDGT_BASE            WWDGT_BASE_NS                        /*!< WWDGT base address */
#define FWDGT_BASE            FWDGT_BASE_NS                        /*!< FWDGT base address */
#define I2S_ADD_BASE          I2S_ADD_BASE_NS                      /*!< I2S1_add base address */
#define SPI_BASE              SPI_BASE_NS                          /*!< SPI base address */
#define USART_BASE            USART_BASE_NS                        /*!< USART base address */
#define I2C_BASE              I2C_BASE_NS                          /*!< I2C base address */
#define PMU_BASE              PMU_BASE_NS                          /*!< PMU base address */

/* advanced peripheral bus 2 memory map */
#define WIFI_RF_BASE          WIFI_RF_BASE_NS                      /*!< WIFI base address */
#define HPDF_BASE             HPDF_BASE_NS                         /*!< HPDF base address */
#define SYSCFG_BASE           SYSCFG_BASE_NS                       /*!< SYSCFG base address */
#define EXTI_BASE             EXTI_BASE_NS                         /*!< EXTI base address */
#define SDIO_BASE             SDIO_BASE_NS                         /*!< SDIO base address */
#define ADC_BASE              ADC_BASE_NS                          /*!< ADC base address */

/* advanced high performance bus 1 memory map */
#define GPIO_BASE             GPIO_BASE_NS                         /*!< GPIO base address */
#define FMC_BASE              FMC_BASE_NS                          /*!< FMC base address */
#define EFUSE_BASE            EFUSE_BASE_NS                        /*!< EFUSE base address */
#define CRC_BASE              CRC_BASE_NS                          /*!< CRC base address */
#define RCU_BASE              RCU_BASE_NS                          /*!< RCU base address */
#define TSI_BASE              TSI_BASE_NS                          /*!< TSI base address */
#define SQPI_REG_BASE         SQPI_REG_BASE_NS                     /*!< SQPI base address */
#define QSPI_REG_BASE         QSPI_REG_BASE_NS                     /*!< QSPI base address */
#define DMA_BASE              DMA_BASE_NS                          /*!< DMA base address */
#define WIFI11N_BASE          WIFI11N_BASE_NS                      /*!< WIFI11N base address */
#define ICACHE_BASE           ICACHE_BASE_NS                       /*!< ICACHE base address */
#define TZSPC_BASE            TZSPC_BASE_NS                        /*!< TZSPC base address */
#define TZIAC_BASE            TZIAC_BASE_NS                        /*!< TZIAC base address */
#define TZBMPC0_BASE          TZBMPC0_BASE_NS                      /*!< TZBMPC0 base address */
#define TZBMPC1_BASE          TZBMPC1_BASE_NS                      /*!< TZBMPC1 base address */
#define TZBMPC2_BASE          TZBMPC2_BASE_NS                      /*!< TZBMPC2 base address */
#define TZBMPC3_BASE          TZBMPC3_BASE_NS                      /*!< TZBMPC3 base address */
#define USBFS_BASE            USBFS_BASE_NS                        /*!< USBFS base address */

/* advanced high performance bus 2 memory map */
#define DCI_BASE              DCI_BASE_NS                          /*!< DCI base address */
#define CAU_BASE              CAU_BASE_NS                          /*!< CAU base address */
#define HAU_BASE              HAU_BASE_NS                          /*!< HAU base address */
#define TRNG_BASE             TRNG_BASE_NS                         /*!< TRNG base address */
#define PKCAU_BASE            PKCAU_BASE_NS                        /*!< PKCAU base address */

#endif /* defined (__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3U) */
/* define marco USE_STDPERIPH_DRIVER */
#if !defined  USE_STDPERIPH_DRIVER
#define USE_STDPERIPH_DRIVER
#endif /* no define USE_STDPERIPH_DRIVER*/
#ifdef USE_STDPERIPH_DRIVER
#include "gd32w51x_libopt.h"
#endif /* USE_STDPERIPH_DRIVER */

#ifdef __cplusplus
}
#endif
#endif /* GD32W51x_H */
