/*!
    \file    gd32w51x_syscfg.h
    \brief   definitions for the SYSCFG

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

#ifndef GD32W51X_SYSCFG_H
#define GD32W51X_SYSCFG_H

#include "gd32w51x.h"

#define SYSCFG         SYSCFG_BASE

/* registers definitions */
#define SYSCFG_CFG0                             REG32(SYSCFG + 0x00000000U)                       /*!< SYSCFG configuration register 0 */
#define SYSCFG_EXTISS0                          REG32(SYSCFG + 0x00000008U)                       /*!< EXTI sources selection register 0 */
#define SYSCFG_EXTISS1                          REG32(SYSCFG + 0x0000000CU)                       /*!< EXTI sources selection register 1 */
#define SYSCFG_EXTISS2                          REG32(SYSCFG + 0x00000010U)                       /*!< EXTI sources selection register 2 */
#define SYSCFG_EXTISS3                          REG32(SYSCFG + 0x00000014U)                       /*!< EXTI sources selection register 3 */
#define SYSCFG_CPSCTL                           REG32(SYSCFG + 0x00000020U)                       /*!< I/O compensation control register */
#define SYSCFG_SECCFG                           REG32(SYSCFG + 0x00000040U)                       /*!< SYSCFG secure configuration register */
#define SYSCFG_FPUINTEN                         REG32(SYSCFG + 0x00000048U)                       /*!< FPU interrupt enable register */
#define SYSCFG_CNSLOCK                          REG32(SYSCFG + 0x0000004CU)                       /*!< SYSCFG CPU non-secure lock register */
#define SYSCFG_CSLOCK                           REG32(SYSCFG + 0x00000050U)                       /*!< SYSCFG CPU secure lock register */
#define SYSCFG_CFG1                             REG32(SYSCFG + 0x00000054U)                       /*!< SYSCFG configuration register 1 */
#define SYSCFG_SCS                              REG32(SYSCFG + 0x00000058U)                       /*!< SYSCFG SRAM1 control and status register */
#define SYSCFG_SKEY                             REG32(SYSCFG + 0x0000005CU)                       /*!< SYSCFG SRAM1 key register */
#define SYSCFG_SWP0                             REG32(SYSCFG + 0x00000060U)                       /*!< SYSCFG SRAM1 write protection register 0 */
#define SYSCFG_SWP1                             REG32(SYSCFG + 0x00000064U)                       /*!< SYSCFG SRAM1 write protection register 1 */
#define SYSCFG_GSSACMD                           REG32(SYSCFG + 0x0000006CU)                       /*!< SYSCFG GSSA command register */

/* bits definitions */
/* SYSCFG_CFG0 */
#define SYSCFG_CFG0_BOOT_MODE                   BITS(0,1)                                         /*!< Boot mode */

/* SYSCFG_EXTISS0 bits definitions */
#define SYSCFG_EXTISS0_EXTI0_SS                 BITS(0,3)                                         /*!< EXTI 0 source selection */
#define SYSCFG_EXTISS0_EXTI1_SS                 BITS(4,7)                                         /*!< EXTI 1 source selection */
#define SYSCFG_EXTISS0_EXTI2_SS                 BITS(8,11)                                        /*!< EXTI 2 source selection */
#define SYSCFG_EXTISS0_EXTI3_SS                 BITS(12,15)                                       /*!< EXTI 3 source selection */

/* SYSCFG_EXTISS1 bits definitions */
#define SYSCFG_EXTISS1_EXTI4_SS                 BITS(0,3)                                         /*!< EXTI 4 source selection */
#define SYSCFG_EXTISS1_EXTI5_SS                 BITS(4,7)                                         /*!< EXTI 5 source selection */
#define SYSCFG_EXTISS1_EXTI6_SS                 BITS(8,11)                                        /*!< EXTI 6 source selection */
#define SYSCFG_EXTISS1_EXTI7_SS                 BITS(12,15)                                       /*!< EXTI 7 source selection */

/* SYSCFG_EXTISS2 bits definitions */
#define SYSCFG_EXTISS2_EXTI8_SS                 BITS(0,3)                                         /*!< EXTI 8 source selection */
#define SYSCFG_EXTISS2_EXTI9_SS                 BITS(4,7)                                         /*!< EXTI 9 source selection */
#define SYSCFG_EXTISS2_EXTI10_SS                BITS(8,11)                                        /*!< EXTI 10 source selection */
#define SYSCFG_EXTISS2_EXTI11_SS                BITS(12,15)                                       /*!< EXTI 11 source selection */

/* SYSCFG_EXTISS3 bits definitions */
#define SYSCFG_EXTISS3_EXTI12_SS                BITS(0,3)                                         /*!< EXTI 12 source selection */
#define SYSCFG_EXTISS3_EXTI13_SS                BITS(4,7)                                         /*!< EXTI 13 source selection */
#define SYSCFG_EXTISS3_EXTI14_SS                BITS(8,11)                                        /*!< EXTI 14 source selection */
#define SYSCFG_EXTISS3_EXTI15_SS                BITS(12,15)                                       /*!< EXTI 15 source selection */

/* SYSCFG_CPSCTL */
#define SYSCFG_CPSCTL_CPS_EN                    BIT(0)                                            /*!< Compensation cell power-down */
#define SYSCFG_CPSCTL_CPS_RDY                   BIT(8)                                            /*!< Compensation cell ready flag */

/* SYSCFG_SECCFG */
#define SYSCFG_SECCFG_SYSCFGSE                  BIT(0)                                            /*!< SYSCFG clock control security */
#define SYSCFG_SECCFG_CLASSBSE                  BIT(1)                                            /*!< ClassB security */
#define SYSCFG_SECCFG_SRAM1SE                   BIT(2)                                            /*!< SRAM1 security */
#define SYSCFG_SECCFG_FPUSE                     BIT(3)                                            /*!< FPU security */

/* SYSCFG_FPUINTEN */
#define SYSCFG_FPUINTEN_IOPIZ                  BIT(0)                                            /*!< invalid operation Interrupt enable */
#define SYSCFG_FPUINTEN_DZIE                   BIT(1)                                            /*!< divide-by-zero interrupt enable */
#define SYSCFG_FPUINTEN_UFIE                   BIT(2)                                            /*!< underflow interrupt enable */
#define SYSCFG_FPUINTEN_OVFIE                  BIT(3)                                            /*!< overflow interrupt enable */
#define SYSCFG_FPUINTEN_IDIE                   BIT(4)                                            /*!< input abnormal interrupt enable */
#define SYSCFG_FPUINTEN_IXIE                   BIT(5)                                            /*!< inexact interrupt enable */

/* SYSCFG_CNSLOCK */
#define SYSCFG_CNSLOCK_LOCKNSVTOR               BIT(0)                                            /*!< VTOR_NS register lock */
#define SYSCFG_CNSLOCK_LOCKNSMPU                BIT(1)                                            /*!< Non-secure MPU registers loc */

/* SYSCFG_CSLOCK */
#define SYSCFG_CSLOCK_VTSAIRLK                  BIT(0)                                            /*!< VTOR_S register and AIRCR register bits lock */
#define SYSCFG_CSLOCK_SMPULK                    BIT(1)                                            /*!< Secure MPU registers lock */
#define SYSCFG_CSLOCK_SAULK                     BIT(2)                                            /*!< SAU registers lock */

/* SYSCFG_CFG1 */
#define SYSCFG_CFG1_LOCKUP_LOCK                 BIT(0)                                            /*!< Cortex-M33 LOCKUP (hardfault) output enable bit */
#define SYSCFG_CFG1_LVD_LOCK                    BIT(2)                                            /*!< LVD lock enable bit */

/* SYSCFG_SCS */
#define SYSCFG_SCS_SRAM1ERS                     BIT(0)                                            /*!< SRAM1 erase */
#define SYSCFG_SCS_SRAM1BSY                     BIT(1)                                            /*!< SRAM1 busy by erase operation */

/* SYSCFG_SKEY */
#define SYSCFG_SKEY_KEY                         BITS(0,7)                                         /*!< SRAM1 write protection key for software erase */

/* SYSCFG_SWP0 */
#define SYSCFG_SWP0_P0WP                        BIT(0)                                            /*!< SRAM1 1 Kbyte page 0 write protection */
#define SYSCFG_SWP0_P1WP                        BIT(1)                                            /*!< SRAM1 1 Kbyte page 1 write protection */
#define SYSCFG_SWP0_P2WP                        BIT(2)                                            /*!< SRAM1 1 Kbyte page 2 write protection */
#define SYSCFG_SWP0_P3WP                        BIT(3)                                            /*!< SRAM1 1 Kbyte page 3 write protection */
#define SYSCFG_SWP0_P4WP                        BIT(4)                                            /*!< SRAM1 1 Kbyte page 4 write protection */
#define SYSCFG_SWP0_P5WP                        BIT(5)                                            /*!< SRAM1 1 Kbyte page 5 write protection */
#define SYSCFG_SWP0_P6WP                        BIT(6)                                            /*!< SRAM1 1 Kbyte page 6 write protection */
#define SYSCFG_SWP0_P7WP                        BIT(7)                                            /*!< SRAM1 1 Kbyte page 7 write protection */
#define SYSCFG_SWP0_P8WP                        BIT(8)                                            /*!< SRAM1 1 Kbyte page 8 write protection */
#define SYSCFG_SWP0_P9WP                        BIT(9)                                            /*!< SRAM1 1 Kbyte page 9 write protection */
#define SYSCFG_SWP0_P10WP                       BIT(10)                                           /*!< SRAM1 1 Kbyte page 10 write protection */
#define SYSCFG_SWP0_P11WP                       BIT(11)                                           /*!< SRAM1 1 Kbyte page 11 write protection */
#define SYSCFG_SWP0_P12WP                       BIT(12)                                           /*!< SRAM1 1 Kbyte page 12 write protection */
#define SYSCFG_SWP0_P13WP                       BIT(13)                                           /*!< SRAM1 1 Kbyte page 13 write protection */
#define SYSCFG_SWP0_P14WP                       BIT(14)                                           /*!< SRAM1 1 Kbyte page 14 write protection */
#define SYSCFG_SWP0_P15WP                       BIT(15)                                           /*!< SRAM1 1 Kbyte page 15 write protection */
#define SYSCFG_SWP0_P16WP                       BIT(16)                                           /*!< SRAM1 1 Kbyte page 16 write protection */
#define SYSCFG_SWP0_P17WP                       BIT(17)                                           /*!< SRAM1 1 Kbyte page 17 write protection */
#define SYSCFG_SWP0_P18WP                       BIT(18)                                           /*!< SRAM1 1 Kbyte page 18 write protection */
#define SYSCFG_SWP0_P19WP                       BIT(19)                                           /*!< SRAM1 1 Kbyte page 19 write protection */
#define SYSCFG_SWP0_P20WP                       BIT(20)                                           /*!< SRAM1 1 Kbyte page 20 write protection */
#define SYSCFG_SWP0_P21WP                       BIT(21)                                           /*!< SRAM1 1 Kbyte page 21 write protection */
#define SYSCFG_SWP0_P22WP                       BIT(22)                                           /*!< SRAM1 1 Kbyte page 22 write protection */
#define SYSCFG_SWP0_P23WP                       BIT(23)                                           /*!< SRAM1 1 Kbyte page 23 write protection */
#define SYSCFG_SWP0_P24WP                       BIT(24)                                           /*!< SRAM1 1 Kbyte page 24 write protection */
#define SYSCFG_SWP0_P25WP                       BIT(25)                                           /*!< SRAM1 1 Kbyte page 25 write protection */
#define SYSCFG_SWP0_P26WP                       BIT(26)                                           /*!< SRAM1 1 Kbyte page 26 write protection */
#define SYSCFG_SWP0_P27WP                       BIT(27)                                           /*!< SRAM1 1 Kbyte page 27 write protection */
#define SYSCFG_SWP0_P28WP                       BIT(28)                                           /*!< SRAM1 1 Kbyte page 28 write protection */
#define SYSCFG_SWP0_P29WP                       BIT(29)                                           /*!< SRAM1 1 Kbyte page 29 write protection */
#define SYSCFG_SWP0_P30WP                       BIT(30)                                           /*!< SRAM1 1 Kbyte page 30 write protection */
#define SYSCFG_SWP0_P31WP                       BIT(31)                                           /*!< SRAM1 1 Kbyte page 31 write protection */

/* SYSCFG_SWPR1 */
#define SYSCFG_SWP1_P32WP                       BIT(0)                                            /*!< SRAM1 1 Kbyte page 32 write protection */
#define SYSCFG_SWP1_P33WP                       BIT(1)                                            /*!< SRAM1 1 Kbyte page 33 write protection */
#define SYSCFG_SWP1_P34WP                       BIT(2)                                            /*!< SRAM1 1 Kbyte page 34 write protection */
#define SYSCFG_SWP1_P35WP                       BIT(3)                                            /*!< SRAM1 1 Kbyte page 35 write protection */
#define SYSCFG_SWP1_P36WP                       BIT(4)                                            /*!< SRAM1 1 Kbyte page 36 write protection */
#define SYSCFG_SWP1_P37WP                       BIT(5)                                            /*!< SRAM1 1 Kbyte page 37 write protection */
#define SYSCFG_SWP1_P38WP                       BIT(6)                                            /*!< SRAM1 1 Kbyte page 38 write protection */
#define SYSCFG_SWP1_P39WP                       BIT(7)                                            /*!< SRAM1 1 Kbyte page 39 write protection */
#define SYSCFG_SWP1_P40WP                       BIT(8)                                            /*!< SRAM1 1 Kbyte page 40 write protection */
#define SYSCFG_SWP1_P41WP                       BIT(9)                                            /*!< SRAM1 1 Kbyte page 41 write protection */
#define SYSCFG_SWP1_P42WP                       BIT(10)                                           /*!< SRAM1 1 Kbyte page 42 write protection */
#define SYSCFG_SWP1_P43WP                       BIT(11)                                           /*!< SRAM1 1 Kbyte page 43 write protection */
#define SYSCFG_SWP1_P44WP                       BIT(12)                                           /*!< SRAM1 1 Kbyte page 44 write protection */
#define SYSCFG_SWP1_P45WP                       BIT(13)                                           /*!< SRAM1 1 Kbyte page 45 write protection */
#define SYSCFG_SWP1_P46WP                       BIT(14)                                           /*!< SRAM1 1 Kbyte page 46 write protection */
#define SYSCFG_SWP1_P47WP                       BIT(15)                                           /*!< SRAM1 1 Kbyte page 47 write protection */
#define SYSCFG_SWP1_P48WP                       BIT(16)                                           /*!< SRAM1 1 Kbyte page 48 write protection */
#define SYSCFG_SWP1_P49WP                       BIT(17)                                           /*!< SRAM1 1 Kbyte page 49 write protection */
#define SYSCFG_SWP1_P50WP                       BIT(18)                                           /*!< SRAM1 1 Kbyte page 50 write protection */
#define SYSCFG_SWP1_P51WP                       BIT(19)                                           /*!< SRAM1 1 Kbyte page 51 write protection */
#define SYSCFG_SWP1_P52WP                       BIT(20)                                           /*!< SRAM1 1 Kbyte page 52 write protection */
#define SYSCFG_SWP1_P53WP                       BIT(21)                                           /*!< SRAM1 1 Kbyte page 53 write protection */
#define SYSCFG_SWP1_P54WP                       BIT(22)                                           /*!< SRAM1 1 Kbyte page 54 write protection */
#define SYSCFG_SWP1_P55WP                       BIT(23)                                           /*!< SRAM1 1 Kbyte page 55 write protection */
#define SYSCFG_SWP1_P56WP                       BIT(24)                                           /*!< SRAM1 1 Kbyte page 56 write protection */
#define SYSCFG_SWP1_P57WP                       BIT(25)                                           /*!< SRAM1 1 Kbyte page 57 write protection */
#define SYSCFG_SWP1_P58WP                       BIT(26)                                           /*!< SRAM1 1 Kbyte page 58 write protection */
#define SYSCFG_SWP1_P59WP                       BIT(27)                                           /*!< SRAM1 1 Kbyte page 59 write protection */
#define SYSCFG_SWP1_P60WP                       BIT(28)                                           /*!< SRAM1 1 Kbyte page 60 write protection */
#define SYSCFG_SWP1_P61WP                       BIT(29)                                           /*!< SRAM1 1 Kbyte page 61 write protection */
#define SYSCFG_SWP1_P62WP                       BIT(30)                                           /*!< SRAM1 1 Kbyte page 62 write protection */
#define SYSCFG_SWP1_P63WP                       BIT(31)                                           /*!< SRAM1 1 Kbyte page 63 write protection */

/* SYSCFG_GSSACMD */
#define SYSCFG_GSSACMD_GSSACMD                    BITS(0,7)                                       /*!< GSSA commands */

/* constants definitions */
/* EXTI source select definition */
#define EXTISS0                                 ((uint8_t)0x00U)                                  /*!< EXTI source select register 0 */
#define EXTISS1                                 ((uint8_t)0x01U)                                  /*!< EXTI source select register 1 */
#define EXTISS2                                 ((uint8_t)0x02U)                                  /*!< EXTI source select register 2 */
#define EXTISS3                                 ((uint8_t)0x03U)                                  /*!< EXTI source select register 3 */

/* EXTI source select mask bits definition */
#define EXTI_SS_MASK                            BITS(0,3)                                         /*!< EXTI source select mask */

/* EXTI source select jumping step definition */
#define EXTI_SS_JSTEP                           ((uint8_t)(0x04U))                                /*!< EXTI source select jumping step */

/* EXTI source select moving step definition */
#define EXTI_SS_MSTEP(pin)                      (EXTI_SS_JSTEP * ((pin) % EXTI_SS_JSTEP))         /*!< EXTI source select moving step */

/* EXTI source port definitions */
#define EXTI_SOURCE_GPIOA                       ((uint8_t)0x00U)                                  /*!< EXTI GPIOA configuration */
#define EXTI_SOURCE_GPIOB                       ((uint8_t)0x01U)                                  /*!< EXTI GPIOB configuration */
#define EXTI_SOURCE_GPIOC                       ((uint8_t)0x02U)                                  /*!< EXTI GPIOC configuration */

/* EXTI source pin definitions */
#define EXTI_SOURCE_PIN0                        ((uint8_t)0x00U)                                  /*!< EXTI GPIO pin0 configuration */
#define EXTI_SOURCE_PIN1                        ((uint8_t)0x01U)                                  /*!< EXTI GPIO pin1 configuration */
#define EXTI_SOURCE_PIN2                        ((uint8_t)0x02U)                                  /*!< EXTI GPIO pin2 configuration */
#define EXTI_SOURCE_PIN3                        ((uint8_t)0x03U)                                  /*!< EXTI GPIO pin3 configuration */
#define EXTI_SOURCE_PIN4                        ((uint8_t)0x04U)                                  /*!< EXTI GPIO pin4 configuration */
#define EXTI_SOURCE_PIN5                        ((uint8_t)0x05U)                                  /*!< EXTI GPIO pin5 configuration */
#define EXTI_SOURCE_PIN6                        ((uint8_t)0x06U)                                  /*!< EXTI GPIO pin6 configuration */
#define EXTI_SOURCE_PIN7                        ((uint8_t)0x07U)                                  /*!< EXTI GPIO pin7 configuration */
#define EXTI_SOURCE_PIN8                        ((uint8_t)0x08U)                                  /*!< EXTI GPIO pin8 configuration */
#define EXTI_SOURCE_PIN9                        ((uint8_t)0x09U)                                  /*!< EXTI GPIO pin9 configuration */
#define EXTI_SOURCE_PIN10                       ((uint8_t)0x0AU)                                  /*!< EXTI GPIO pin10 configuration */
#define EXTI_SOURCE_PIN11                       ((uint8_t)0x0BU)                                  /*!< EXTI GPIO pin11 configuration */
#define EXTI_SOURCE_PIN12                       ((uint8_t)0x0CU)                                  /*!< EXTI GPIO pin12 configuration */
#define EXTI_SOURCE_PIN13                       ((uint8_t)0x0DU)                                  /*!< EXTI GPIO pin13 configuration */
#define EXTI_SOURCE_PIN14                       ((uint8_t)0x0EU)                                  /*!< EXTI GPIO pin14 configuration */
#define EXTI_SOURCE_PIN15                       ((uint8_t)0x0FU)                                  /*!< EXTI GPIO pin15 configuration */

/* security definitions */
#define SYSCFG_SECURE_ACCESS                    SYSCFG_SECCFG_SYSCFGSE                            /*!< SYSCFG configuration clock in RCU registers can be written by secure access only */
#define SYSCFG_SECURE_NONSECURE_ACCESS          (~SYSCFG_SECCFG_SYSCFGSE)                         /*!< SYSCFG configuration clock in RCU registers can be written by secure and nonsecure access */

#define CLASSB_SECURE_ACCESS                    SYSCFG_SECCFG_CLASSBSE                            /*!< SYSCFG_CFG1 register can be written by secure access only */
#define CLASSB_SECURE_NONSECURE_ACCESS          (~SYSCFG_SECCFG_CLASSBSE)                         /*!< SYSCFG_CFG1 register can be written by secure and non-secure access */

#define SRAM1_SECURE_ACCESS                     SYSCFG_SECCFG_SRAM1SE                             /*!< SYSCFG_SKEY, SYSCFG_SCS and SYSCFG_SWPx registers can be written by secure only */
#define SRAM1_SECURE_NONSECURE_ACCESS           (~SYSCFG_SECCFG_SRAM1SE)                          /*!< SYSCFG_SKEY, SYSCFG_SCS and SYSCFG_SWPx registers can be written by secure and non-secure access */

#define FPU_SECURE_ACCESS                       SYSCFG_SECCFG_FPUSE                               /*!< SYSCFG_FPUINTEN register can be written by secure access only */
#define FPU_SECURE_NONSECURE_ACCESS             (~SYSCFG_SECCFG_FPUSE)                            /*!< SYSCFG_FPUINTEN register can be written by secure and non-secure access */

/* fpu interrupt definitions */
#define INVALID_OPERATION_INT                   SYSCFG_FPUINTEN_IOPIZ                             /*!< invalid operation Interrupt */
#define DEVIDE_BY_ZERO_INT                      SYSCFG_FPUINTEN_DZIE                              /*!< divide-by-zero interrupt */
#define UNDERFLOW_INT                           SYSCFG_FPUINTEN_UFIE                              /*!< underflow interrupt */
#define OVERFLOW_INT                            SYSCFG_FPUINTEN_OVFIE                             /*!< overflow interrupt */
#define INPUT_ABNORMAL_INT                      SYSCFG_FPUINTEN_IDIE                              /*!< input abnormal interrupt */
#define INEXACT_INT                             SYSCFG_FPUINTEN_IXIE                              /*!< inexact interrupt */

/* lock definitions */
#define SYSCFG_LOCK_LOCKUP                      SYSCFG_CFG1_LOCKUP_LOCK                           /*!< LOCKUP output lock*/
#define SYSCFG_LOCK_LVD                         SYSCFG_CFG1_LVD_LOCK                              /*!< LVD lock */

/* SRAM1 status definition */
#define SRAM1_BSY_FLAG                          SYSCFG_SCS_SRAM1BSY                               /*!< SRAM1 erase operation is ongoing */

/* SRAM1ERS unlock key */
#define SRAM1ERS_UNLOCK_KEY0                    ((uint8_t)0xCA)                                   /*!< unlock key0 */
#define SRAM1ERS_UNLOCK_KEY1                    ((uint8_t)0x53)                                   /*!< unlock key1 */
#define SRAM1ERS_LOCK_KEY0                      ((uint8_t)0x11)                                   /*!< lock key0 */
#define SRAM1ERS_LOCK_KEY1                      ((uint8_t)0x22)                                   /*!< lock key1 */

/* SRAM1 write protection definitions */
#define SRAM1_PAGE0_WRITE_PROTECT               SYSCFG_SWP0_P0WP                                  /*!<SRAM1 page 0 */
#define SRAM1_PAGE1_WRITE_PROTECT               SYSCFG_SWP0_P1WP                                  /*!<SRAM1 page 1 */
#define SRAM1_PAGE2_WRITE_PROTECT               SYSCFG_SWP0_P2WP                                  /*!<SRAM1 page 2 */
#define SRAM1_PAGE3_WRITE_PROTECT               SYSCFG_SWP0_P3WP                                  /*!<SRAM1 page 3 */
#define SRAM1_PAGE4_WRITE_PROTECT               SYSCFG_SWP0_P4WP                                  /*!<SRAM1 page 4 */
#define SRAM1_PAGE5_WRITE_PROTECT               SYSCFG_SWP0_P5WP                                  /*!<SRAM1 page 5 */
#define SRAM1_PAGE6_WRITE_PROTECT               SYSCFG_SWP0_P6WP                                  /*!<SRAM1 page 6 */
#define SRAM1_PAGE7_WRITE_PROTECT               SYSCFG_SWP0_P7WP                                  /*!<SRAM1 page 7 */
#define SRAM1_PAGE8_WRITE_PROTECT               SYSCFG_SWP0_P8WP                                  /*!<SRAM1 page 8 */
#define SRAM1_PAGE9_WRITE_PROTECT               SYSCFG_SWP0_P9WP                                  /*!<SRAM1 page 9 */
#define SRAM1_PAGE10_WRITE_PROTECT              SYSCFG_SWP0_P10WP                                 /*!<SRAM1 page 10 */
#define SRAM1_PAGE11_WRITE_PROTECT              SYSCFG_SWP0_P11WP                                 /*!<SRAM1 page 11 */
#define SRAM1_PAGE12_WRITE_PROTECT              SYSCFG_SWP0_P12WP                                 /*!<SRAM1 page 12 */
#define SRAM1_PAGE13_WRITE_PROTECT              SYSCFG_SWP0_P13WP                                 /*!<SRAM1 page 13 */
#define SRAM1_PAGE14_WRITE_PROTECT              SYSCFG_SWP0_P14WP                                 /*!<SRAM1 page 14 */
#define SRAM1_PAGE15_WRITE_PROTECT              SYSCFG_SWP0_P15WP                                 /*!<SRAM1 page 15 */
#define SRAM1_PAGE16_WRITE_PROTECT              SYSCFG_SWP0_P16WP                                 /*!<SRAM1 page 16 */
#define SRAM1_PAGE17_WRITE_PROTECT              SYSCFG_SWP0_P17WP                                 /*!<SRAM1 page 17 */
#define SRAM1_PAGE18_WRITE_PROTECT              SYSCFE_SWP0_P18WP                                 /*!<SRAM1 page 18 */
#define SRAM1_PAGE19_WRITE_PROTECT              SYSCFE_SWP0_P19WP                                 /*!<SRAM1 page 19 */
#define SRAM1_PAGE20_WRITE_PROTECT              SYSCFE_SWP0_P20WP                                 /*!<SRAM1 page 20 */
#define SRAM1_PAGE21_WRITE_PROTECT              SYSCFE_SWP0_P21WP                                 /*!<SRAM1 page 21 */
#define SRAM1_PAGE22_WRITE_PROTECT              SYSCFE_SWP0_P22WP                                 /*!<SRAM1 page 22 */
#define SRAM1_PAGE23_WRITE_PROTECT              SYSCFE_SWP0_P23WP                                 /*!<SRAM1 page 23 */
#define SRAM1_PAGE24_WRITE_PROTECT              SYSCFE_SWP0_P24WP                                 /*!<SRAM1 page 24 */
#define SRAM1_PAGE25_WRITE_PROTECT              SYSCFE_SWP0_P25WP                                 /*!<SRAM1 page 25 */
#define SRAM1_PAGE26_WRITE_PROTECT              SYSCFE_SWP0_P26WP                                 /*!<SRAM1 page 26 */
#define SRAM1_PAGE27_WRITE_PROTECT              SYSCFE_SWP0_P27WP                                 /*!<SRAM1 page 27 */
#define SRAM1_PAGE28_WRITE_PROTECT              SYSCFE_SWP0_P28WP                                 /*!<SRAM1 page 28 */
#define SRAM1_PAGE29_WRITE_PROTECT              SYSCFE_SWP0_P29WP                                 /*!<SRAM1 page 29 */
#define SRAM1_PAGE30_WRITE_PROTECT              SYSCFE_SWP0_P30WP                                 /*!<SRAM1 page 30 */
#define SRAM1_PAGE31_WRITE_PROTECT              SYSCFE_SWP0_P31WP                                 /*!<SRAM1 page 31 */
#define SRAM1_PAGE32_WRITE_PROTECT              SYSCFE_SWP1_P32WP                                 /*!<SRAM1 page 32 */
#define SRAM1_PAGE33_WRITE_PROTECT              SYSCFE_SWP1_P33WP                                 /*!<SRAM1 page 33 */
#define SRAM1_PAGE34_WRITE_PROTECT              SYSCFE_SWP1_P34WP                                 /*!<SRAM1 page 34 */
#define SRAM1_PAGE35_WRITE_PROTECT              SYSCFE_SWP1_P35WP                                 /*!<SRAM1 page 35 */
#define SRAM1_PAGE36_WRITE_PROTECT              SYSCFE_SWP1_P36WP                                 /*!<SRAM1 page 36 */
#define SRAM1_PAGE37_WRITE_PROTECT              SYSCFE_SWP1_P37WP                                 /*!<SRAM1 page 37 */
#define SRAM1_PAGE38_WRITE_PROTECT              SYSCFE_SWP1_P38WP                                 /*!<SRAM1 page 38 */
#define SRAM1_PAGE39_WRITE_PROTECT              SYSCFE_SWP1_P39WP                                 /*!<SRAM1 page 39 */
#define SRAM1_PAGE40_WRITE_PROTECT              SYSCFE_SWP1_P40WP                                 /*!<SRAM1 page 40 */
#define SRAM1_PAGE41_WRITE_PROTECT              SYSCFE_SWP1_P41WP                                 /*!<SRAM1 page 41 */
#define SRAM1_PAGE42_WRITE_PROTECT              SYSCFE_SWP1_P42WP                                 /*!<SRAM1 page 42 */
#define SRAM1_PAGE43_WRITE_PROTECT              SYSCFE_SWP1_P43WP                                 /*!<SRAM1 page 43 */
#define SRAM1_PAGE44_WRITE_PROTECT              SYSCFE_SWP1_P44WP                                 /*!<SRAM1 page 44 */
#define SRAM1_PAGE45_WRITE_PROTECT              SYSCFE_SWP1_P45WP                                 /*!<SRAM1 page 45 */
#define SRAM1_PAGE46_WRITE_PROTECT              SYSCFE_SWP1_P46WP                                 /*!<SRAM1 page 46 */
#define SRAM1_PAGE47_WRITE_PROTECT              SYSCFE_SWP1_P47WP                                 /*!<SRAM1 page 47 */
#define SRAM1_PAGE48_WRITE_PROTECT              SYSCFE_SWP1_P48WP                                 /*!<SRAM1 page 48 */
#define SRAM1_PAGE49_WRITE_PROTECT              SYSCFE_SWP1_P49WP                                 /*!<SRAM1 page 49 */
#define SRAM1_PAGE50_WRITE_PROTECT              SYSCFE_SWP1_P50WP                                 /*!<SRAM1 page 50 */
#define SRAM1_PAGE51_WRITE_PROTECT              SYSCFE_SWP1_P51WP                                 /*!<SRAM1 page 51 */
#define SRAM1_PAGE52_WRITE_PROTECT              SYSCFE_SWP1_P52WP                                 /*!<SRAM1 page 52 */
#define SRAM1_PAGE53_WRITE_PROTECT              SYSCFE_SWP1_P53WP                                 /*!<SRAM1 page 53 */
#define SRAM1_PAGE54_WRITE_PROTECT              SYSCFE_SWP1_P54WP                                 /*!<SRAM1 page 54 */
#define SRAM1_PAGE55_WRITE_PROTECT              SYSCFE_SWP1_P55WP                                 /*!<SRAM1 page 55 */
#define SRAM1_PAGE56_WRITE_PROTECT              SYSCFE_SWP1_P56WP                                 /*!<SRAM1 page 56 */
#define SRAM1_PAGE57_WRITE_PROTECT              SYSCFE_SWP1_P57WP                                 /*!<SRAM1 page 57 */
#define SRAM1_PAGE58_WRITE_PROTECT              SYSCFE_SWP1_P58WP                                 /*!<SRAM1 page 58 */
#define SRAM1_PAGE59_WRITE_PROTECT              SYSCFE_SWP1_P59WP                                 /*!<SRAM1 page 59 */
#define SRAM1_PAGE60_WRITE_PROTECT              SYSCFE_SWP1_P60WP                                 /*!<SRAM1 page 60 */
#define SRAM1_PAGE61_WRITE_PROTECT              SYSCFE_SWP1_P61WP                                 /*!<SRAM1 page 61 */
#define SRAM1_PAGE62_WRITE_PROTECT              SYSCFE_SWP1_P62WP                                 /*!<SRAM1 page 62 */
#define SRAM1_PAGE63_WRITE_PROTECT              SYSCFE_SWP1_P63WP                                 /*!<SRAM1 page 63 */

/* function declarations */
/* initialization functions */
/* reset the SYSCFG registers */
void syscfg_deinit(void);

/* function configuration */
/* configure the GPIO pin as EXTI Line */
void syscfg_exti_line_config(uint8_t exti_port, uint8_t exti_pin);

/* enable the compensation cell */
void compensation_pwdn_mode_enable(void);
/* disable the compensation cell */
void compensation_pwdn_mode_disable(void);

/* configure SYSCFG clock control security */
void syscfg_clock_access_security_config(uint32_t access_mode);
/* configure ClassB security */
void classb_access_security_config(uint32_t access_mode);
/* configure SRAM1 security */
void sram1_access_security_config(uint32_t access_mode);
/* configure FPU security */
void fpu_access_security_config(uint32_t access_mode);
/* disable VTOR_NS register write */
void vtor_ns_write_disable(void);
/* disable Non-secure MPU registers write */
void mpu_ns_write_disable(void);
/* disable VTOR_S register PRIS and BFHFNMINS bits in the AIRCR register write */
void vtors_aircr_write_disable(void);
/* disable secure MPU registers writes */
void mpu_s_write_disable(void);
/* disable SAU registers write */
void sau_write_disable(void);
/* connect TIMER0/15/16 break input to the selected parameter */
void syscfg_lock_config(uint32_t syscfg_lock);
/* write data to GSSA */
void gssacmd_write_data(uint32_t data);

/* SRAM1 operation functions */
/* enable sram1 erase */
ErrStatus sram1_erase(void);
/* unlock the write protection of the SRAM1ERS bit in the SYSCFG_SCS register */
void sram1_unlock(void);
/* lock the write protection of the SRAM1ERS bit in the SYSCFG_SCS register */
void sram1_lock(void);
/* SRAM1 write protect range from page0 to page31 */
void sram1_write_protect_0_31(uint32_t page);
/* SRAM1 write protect range from page32 to page63 */
void sram1_write_protect_32_63(uint32_t page);

/* flag & interrupt functions */
/* get the compensation ready flag */
FlagStatus compensation_ready_flag_get(void);
/* get SRAM1 erase busy flag */
FlagStatus sram1_bsy_flag_get(void);
/* enable floating point unit interrupt */
void fpu_interrupt_enable(uint32_t interrupt);
/* disable floating point unit interrupt */
void fpu_interrupt_disable(uint32_t interrupt);
#endif /* GD32W51X_SYSCFG_H */
