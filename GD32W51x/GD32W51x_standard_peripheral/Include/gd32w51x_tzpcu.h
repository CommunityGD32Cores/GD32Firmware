/*!
    \file    gd32w51x_tzpcu.h
    \brief   definitions for the TZPCU

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

#ifndef GD32W51X_TZPCU_H
#define GD32W51X_TZPCU_H

#include "gd32w51x.h"

/* TZPCU definitions */
#define TZSPC                                 TZSPC_BASE                        /*!< TZSPC base address */
#define TZBMPC0                               TZBMPC0_BASE                      /*!< TZBMPC0 base address */
#define TZBMPC1                               TZBMPC1_BASE                      /*!< TZBMPC1 base address */
#define TZBMPC2                               TZBMPC2_BASE                      /*!< TZBMPC2 base address */
#define TZBMPC3                               TZBMPC3_BASE                      /*!< TZBMPC3 base address */
#define TZIAC                                 TZIAC_BASE                        /*!< TZIAC base address */

/* registers definitions */
/* TZSPC registers */
#define TZPCU_TZSPC_CTL                       REG32((TZSPC) + 0x00000000U)      /*!< TZSPC control register */
#define TZPCU_TZSPC_SAM_CFG0                  REG32((TZSPC) + 0x00000010U)      /*!< TZSPC secure access mode configuration register 0 */
#define TZPCU_TZSPC_SAM_CFG1                  REG32((TZSPC) + 0x00000014U)      /*!< TZSPC secure access mode configuration register 1 */
#define TZPCU_TZSPC_SAM_CFG2                  REG32((TZSPC) + 0x00000018U)      /*!< TZSPC secure access mode configuration register 2 */
#define TZPCU_TZSPC_PAM_CFG0                  REG32((TZSPC) + 0x00000020U)      /*!< TZSPC privilege access mode configuration register 0 */
#define TZPCU_TZSPC_PAM_CFG1                  REG32((TZSPC) + 0x00000024U)      /*!< TZSPC privilege access mode configuration register 1 */
#define TZPCU_TZSPC_PAM_CFG2                  REG32((TZSPC) + 0x00000028U)      /*!< TZSPC privilege access mode configuration register 2 */
#define TZPCU_TZSPC_TZMMPC0_NSM0              REG32((TZSPC) + 0x00000030U)      /*!< TZSPC external memory 0 non-secure mark register 0 */
#define TZPCU_TZSPC_TZMMPC0_NSM1              REG32((TZSPC) + 0x00000034U)      /*!< TZSPC external memory 0 non-secure mark register 1 */
#define TZPCU_TZSPC_TZMMPC0_NSM2              REG32((TZSPC) + 0x00000038U)      /*!< TZSPC external memory 0 non-secure mark register 2 */
#define TZPCU_TZSPC_TZMMPC0_NSM3              REG32((TZSPC) + 0x0000003CU)      /*!< TZSPC external memory 0 non-secure mark register 3 */
#define TZPCU_TZSPC_TZMMPC1_NSM0              REG32((TZSPC) + 0x00000040U)      /*!< TZSPC external memory 1 non-secure mark register 0 */
#define TZPCU_TZSPC_TZMMPC1_NSM1              REG32((TZSPC) + 0x00000044U)      /*!< TZSPC external memory 1 non-secure mark register 1 */
#define TZPCU_TZSPC_DBG_CFG                   REG32((TZSPC) + 0x00000200U)      /*!< TZSPC debug configuration register */

/* TZBMPC registers */
#define TZPCU_TZBMPC_CTL(tzbmpx)              REG32((tzbmpx) + 0x00000000U)     /*!< TZBMPC control register */
#define TZPCU_TZBMPC_VEC(tzbmpx, y)           REG32((tzbmpx) + 0x00000100U\
                                                  + (0x00000004U * (y)))        /*!< TZBMPC vector register */
#define TZPCU_TZBMPC_LOCK0(tzbmpx)            REG32((tzbmpx) + 0x00000010U)     /*!< TZBMPC lock register 0 */

/* TZIAC registers */
#define TZPCU_TZIAC_INTEN0                    REG32((TZIAC) + 0x00000000U)      /*!< TZIAC enable register 0 */
#define TZPCU_TZIAC_INTEN1                    REG32((TZIAC) + 0x00000004U)      /*!< TZIAC enable register 1 */
#define TZPCU_TZIAC_INTEN2                    REG32((TZIAC) + 0x00000008U)      /*!< TZIAC enable register 2 */
#define TZPCU_TZIAC_STAT0                     REG32((TZIAC) + 0x00000010U)      /*!< TZIAC status register 0 */
#define TZPCU_TZIAC_STAT1                     REG32((TZIAC) + 0x00000014U)      /*!< TZIAC status register 1 */
#define TZPCU_TZIAC_STAT2                     REG32((TZIAC) + 0x00000018U)      /*!< TZIAC status register 2 */
#define TZPCU_TZIAC_STATC0                    REG32((TZIAC) + 0x00000020U)      /*!< TZIAC flag clear register 0 */
#define TZPCU_TZIAC_STATC1                    REG32((TZIAC) + 0x00000024U)      /*!< TZIAC flag clear register 1 */
#define TZPCU_TZIAC_STATC2                    REG32((TZIAC) + 0x00000028U)      /*!< TZIAC flag clear register 2 */

/* bits definitions */
/* TZPCU_TZSPC_CTL */
#define TZPCU_TZSPC_CTL_LK                    BIT(0)                            /*!< TZSPC items lock configuration bit */

/* TZPCU_TZSPC_SAM_CFG0 */
#define TZPCU_TZSPC_SAM_CFG0_TIMER1SAM        BIT(0)                            /*!< TIMER1 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_TIMER2SAM        BIT(1)                            /*!< TIMER2 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_TIMER3SAM        BIT(2)                            /*!< TIMER3 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_TIMER4SAM        BIT(3)                            /*!< TIMER4 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_TIMER5SAM        BIT(4)                            /*!< TIMER5 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_WWDGTSAM         BIT(6)                            /*!< WWDGT secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_FWDGTSAM         BIT(7)                            /*!< FWDGT secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_SPI1SAM          BIT(8)                            /*!< SPI1 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_USART1SAM        BIT(10)                           /*!< USART1 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_USART2SAM        BIT(11)                           /*!< USART2 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_I2C0SAM          BIT(14)                           /*!< I2C0 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_I2C1SAM          BIT(15)                           /*!< I2C1 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_USBFSSAM         BIT(26)                           /*!< USBFS secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_TIMER0SAM        BIT(30)                           /*!< TIMER0 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG0_SPI0SAM          BIT(31)                           /*!< SPI0 secure access mode configuration bit */

/* TZPCU_TZSPC_SAM_CFG1 */
#define TZPCU_TZSPC_SAM_CFG1_USART0SAM        BIT(1)                            /*!< USART0 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_TIMER15SAM       BIT(3)                            /*!< TIMER15 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_TIMER16SAM       BIT(4)                            /*!< TIMER16 secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_HPDFSAM          BIT(7)                            /*!< HPDF secure access mode configuration bit(not support on GD32W515TX series devices) */
#define TZPCU_TZSPC_SAM_CFG1_CRCSAM           BIT(8)                            /*!< CRC secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_TSISAM           BIT(9)                            /*!< TSI secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_ICACHESAM        BIT(10)                           /*!< ICACHE secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_ADCSAM           BIT(11)                           /*!< ADC secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_CAUSAM           BIT(12)                           /*!< CAU secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_HAUSAM           BIT(13)                           /*!< HAU secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_TRNGSAM          BIT(14)                           /*!< TRNG secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_PKCAUSAM         BIT(15)                           /*!< PKCAU secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG1_SDIOSAM          BIT(16)                           /*!< SDIO secure access mode configuration bit */

/* TZPCU_TZSPC_SAM_CFG2 */
#define TZPCU_TZSPC_SAM_CFG2_EFUSESAM         BIT(23)                           /*!< EFUSE secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG2_SQPI_PSRAMREGSAM BIT(26)                           /*!< SQPI PSRAMREG secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG2_QSPI_FLASHREGSAM BIT(27)                           /*!< QSPI FLASHREG secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG2_WIFI_RFSAM       BIT(28)                           /*!< WIFI RF secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG2_I2S1_ADDSAM      BIT(29)                           /*!< I2S1_ADD secure access mode configuration bit */
#define TZPCU_TZSPC_SAM_CFG2_DCISAM           BIT(30)                           /*!< DCI secure access mode configuration bit(not support on GD32W515TX series devices) */
#define TZPCU_TZSPC_SAM_CFG2_WIFISAM          BIT(31)                           /*!< WIFI secure access mode configuration bit */

/* TZPCU_TZSPC_PAM_CFG0 */
#define TZPCU_TZSPC_PAM_CFG0_TIMER1PAM        BIT(0)                            /*!< TIMER1 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_TIMER2PAM        BIT(1)                            /*!< TIMER2 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_TIMER3PAM        BIT(2)                            /*!< TIMER3 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_TIMER4PAM        BIT(3)                            /*!< TIMER4 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_TIMER5PAM        BIT(4)                            /*!< TIMER5 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_WWDGTPAM         BIT(6)                            /*!< WWDGT privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_FWDGTPAM         BIT(7)                            /*!< FWDGT privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_SPI1PAM          BIT(8)                            /*!< SPI1 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_USART1PAM        BIT(10)                           /*!< USART1 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_USART2PAM        BIT(11)                           /*!< USART2 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_I2C0PAM          BIT(14)                           /*!< I2C0 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_I2C1PAM          BIT(15)                           /*!< I2C1 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_USBFSPAM         BIT(26)                           /*!< USBFS privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_TIMER0PAM        BIT(30)                           /*!< TIMER0 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG0_SPI0PAM          BIT(31)                           /*!< SPI0 privilege access mode configuration bit */

/* TZPCU_TZSPC_PAM_CFG1 */
#define TZPCU_TZSPC_PAM_CFG1_USART0PAM        BIT(1)                            /*!< USART0 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_TIMER15PAM       BIT(3)                            /*!< TIMER15 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_TIMER16PAM       BIT(4)                            /*!< TIMER16 privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_HPDFPAM          BIT(7)                            /*!< HPDF privilege access mode configuration bit(not support on GD32W515TX series devices) */
#define TZPCU_TZSPC_PAM_CFG1_CRCPAM           BIT(8)                            /*!< CRC privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_TSIPAM           BIT(9)                            /*!< TSI privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_ICACHEPAM        BIT(10)                           /*!< ICACHE privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_ADCPAM           BIT(11)                           /*!< ADC privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_CAUPAM           BIT(12)                           /*!< CAU privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_HAUPAM           BIT(13)                           /*!< HAU privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_TRNGPAM          BIT(14)                           /*!< TRNG privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_PKCAUPAM         BIT(15)                           /*!< PKCAU privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG1_SDIOPAM          BIT(16)                           /*!< SDIO privilege access mode configuration bit */

/* TZPCU_TZSPC_PAM_CFG2 */
#define TZPCU_TZSPC_PAM_CFG2_EFUSEPAM         BIT(23)                           /*!< EFUSE privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG2_DBGPAM           BIT(25)                           /*!< DBG privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG2_SQPI_PSRAMREGPAM BIT(26)                           /*!< SQPI PSRAMREG privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG2_QSPI_FLASHREGPAM BIT(27)                           /*!< QSPI FLASHREG privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG2_WIFI_RFPAM       BIT(28)                           /*!< WIFI RF privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG2_I2S1_ADDPAM      BIT(29)                           /*!< I2S1_ADD privilege access mode configuration bit */
#define TZPCU_TZSPC_PAM_CFG2_DCIPAM           BIT(30)                           /*!< DCI privilege access mode configuration bit(not support on GD32W515TX series devices) */
#define TZPCU_TZSPC_PAM_CFG2_WIFIPAM          BIT(31)                           /*!< WIFI privilege access mode configuration bit */

/* TZPCU_TZSPC_TZMMPCx_NSM0, TZPCU_TZSPC_TZMMPCx_NSM1, TZPCU_TZSPC_TZMMPCx_NSM2 and TZPCU_TZSPC_TZMMPCx_NSM3 */
#define TZPCU_TZSPC_TZMMPC_NSM_SADD           BITS(0,13)                        /*!< the first non-secure area (multiple of 8 Kbytes) start address */
#define TZPCU_TZSPC_TZMMPC_NSM_LEN            BITS(16,30)                       /*!< length of the first non-secure area (multiple of 8 Kbytes) */

/* TZPCU_TZSPC_DBG_CFG */
#define TZPCU_TZSPC_DBG_CFG_IDEN              BIT(0)                            /*!< invasive debug enable bit */
#define TZPCU_TZSPC_DBG_CFG_NIDEN             BIT(1)                            /*!< non-invasive debug enable bit */
#define TZPCU_TZSPC_DBG_CFG_SPIDEN            BIT(2)                            /*!< secure invasive debug enable bit */
#define TZPCU_TZSPC_DBG_CFG_SPNIDEN           BIT(3)                            /*!< secure non-invasive debug enable bit */

/* TZPCU_TZBMPC_CTL */
#define TZPCU_TZBMPC_CTL_LK                   BIT(0)                            /*!< the control register of the TZBMPC sub-block lock configuration bit */
#define TZPCU_TZBMPC_CTL_SECSTATCFG           BIT(30)                           /*!< security state configuration bit. */
#define TZPCU_TZBMPC_CTL_SRWACFG              BIT(31)                           /*!< secure read/write access non-secure SRAM configuration bit. */

/* TZPCU_TZBMPC_VEC */
#define TZPCU_TZBMPC_VEC_B0                   BIT(0)                            /*!< block 0 state */
#define TZPCU_TZBMPC_VEC_B1                   BIT(1)                            /*!< block 1 state */
#define TZPCU_TZBMPC_VEC_B2                   BIT(2)                            /*!< block 2 state */
#define TZPCU_TZBMPC_VEC_B3                   BIT(3)                            /*!< block 3 state */
#define TZPCU_TZBMPC_VEC_B4                   BIT(4)                            /*!< block 4 state */
#define TZPCU_TZBMPC_VEC_B5                   BIT(5)                            /*!< block 5 state */
#define TZPCU_TZBMPC_VEC_B6                   BIT(6)                            /*!< block 6 state */
#define TZPCU_TZBMPC_VEC_B7                   BIT(7)                            /*!< block 7 state */
#define TZPCU_TZBMPC_VEC_B8                   BIT(8)                            /*!< block 8 state */
#define TZPCU_TZBMPC_VEC_B9                   BIT(9)                            /*!< block 9 state */
#define TZPCU_TZBMPC_VEC_B10                  BIT(10)                           /*!< block 10 state */
#define TZPCU_TZBMPC_VEC_B11                  BIT(11)                           /*!< block 11 state */
#define TZPCU_TZBMPC_VEC_B12                  BIT(12)                           /*!< block 12 state */
#define TZPCU_TZBMPC_VEC_B13                  BIT(13)                           /*!< block 13 state */
#define TZPCU_TZBMPC_VEC_B14                  BIT(14)                           /*!< block 14 state */
#define TZPCU_TZBMPC_VEC_B15                  BIT(15)                           /*!< block 15 state */
#define TZPCU_TZBMPC_VEC_B16                  BIT(16)                           /*!< block 16 state */
#define TZPCU_TZBMPC_VEC_B17                  BIT(17)                           /*!< block 17 state */
#define TZPCU_TZBMPC_VEC_B18                  BIT(18)                           /*!< block 18 state */
#define TZPCU_TZBMPC_VEC_B19                  BIT(19)                           /*!< block 19 state */
#define TZPCU_TZBMPC_VEC_B20                  BIT(20)                           /*!< block 20 state */
#define TZPCU_TZBMPC_VEC_B21                  BIT(21)                           /*!< block 21 state */
#define TZPCU_TZBMPC_VEC_B22                  BIT(22)                           /*!< block 22 state */
#define TZPCU_TZBMPC_VEC_B23                  BIT(23)                           /*!< block 23 state */
#define TZPCU_TZBMPC_VEC_B24                  BIT(24)                           /*!< block 24 state */
#define TZPCU_TZBMPC_VEC_B25                  BIT(25)                           /*!< block 25 state */
#define TZPCU_TZBMPC_VEC_B26                  BIT(26)                           /*!< block 26 state */
#define TZPCU_TZBMPC_VEC_B27                  BIT(27)                           /*!< block 27 state */
#define TZPCU_TZBMPC_VEC_B28                  BIT(28)                           /*!< block 28 state */
#define TZPCU_TZBMPC_VEC_B29                  BIT(29)                           /*!< block 29 state */
#define TZPCU_TZBMPC_VEC_B30                  BIT(30)                           /*!< block 30 state */
#define TZPCU_TZBMPC_VEC_B31                  BIT(31)                           /*!< block 31 state */

/* TZPCU_TZBMPC_LOCK0 */
#define TZPCU_TZBMPC_LOCK0_LKUB0              BIT(0)                            /*!< the secure access mode lock configuration bit of union-blocks 0 */
#define TZPCU_TZBMPC_LOCK0_LKUB1              BIT(1)                            /*!< the secure access mode lock configuration bit of union-blocks 1 */
#define TZPCU_TZBMPC_LOCK0_LKUB2              BIT(2)                            /*!< the secure access mode lock configuration bit of union-blocks 2 */
#define TZPCU_TZBMPC_LOCK0_LKUB3              BIT(3)                            /*!< the secure access mode lock configuration bit of union-blocks 3 */
#define TZPCU_TZBMPC_LOCK0_LKUB4              BIT(4)                            /*!< the secure access mode lock configuration bit of union-blocks 4 */
#define TZPCU_TZBMPC_LOCK0_LKUB5              BIT(5)                            /*!< the secure access mode lock configuration bit of union-blocks 5 */
#define TZPCU_TZBMPC_LOCK0_LKUB6              BIT(6)                            /*!< the secure access mode lock configuration bit of union-blocks 6 */
#define TZPCU_TZBMPC_LOCK0_LKUB7              BIT(7)                            /*!< the secure access mode lock configuration bit of union-blocks 7 */
#define TZPCU_TZBMPC_LOCK0_LKUB8              BIT(8)                            /*!< the secure access mode lock configuration bit of union-blocks 8 */
#define TZPCU_TZBMPC_LOCK0_LKUB9              BIT(9)                            /*!< the secure access mode lock configuration bit of union-blocks 9 */
#define TZPCU_TZBMPC_LOCK0_LKUB10             BIT(10)                           /*!< the secure access mode lock configuration bit of union-blocks 10 */
#define TZPCU_TZBMPC_LOCK0_LKUB11             BIT(11)                           /*!< the secure access mode lock configuration bit of union-blocks 11 */
#define TZPCU_TZBMPC_LOCK0_LKUB12             BIT(12)                           /*!< the secure access mode lock configuration bit of union-blocks 12 */
#define TZPCU_TZBMPC_LOCK0_LKUB13             BIT(13)                           /*!< the secure access mode lock configuration bit of union-blocks 13 */
#define TZPCU_TZBMPC_LOCK0_LKUB14             BIT(14)                           /*!< the secure access mode lock configuration bit of union-blocks 14 */
#define TZPCU_TZBMPC_LOCK0_LKUB15             BIT(15)                           /*!< the secure access mode lock configuration bit of union-blocks 15 */
#define TZPCU_TZBMPC_LOCK0_LKUB16             BIT(16)                           /*!< the secure access mode lock configuration bit of union-blocks 16 */
#define TZPCU_TZBMPC_LOCK0_LKUB17             BIT(17)                           /*!< the secure access mode lock configuration bit of union-blocks 17 */
#define TZPCU_TZBMPC_LOCK0_LKUB18             BIT(18)                           /*!< the secure access mode lock configuration bit of union-blocks 18 */
#define TZPCU_TZBMPC_LOCK0_LKUB19             BIT(19)                           /*!< the secure access mode lock configuration bit of union-blocks 19 */
#define TZPCU_TZBMPC_LOCK0_LKUB20             BIT(20)                           /*!< the secure access mode lock configuration bit of union-blocks 20 */
#define TZPCU_TZBMPC_LOCK0_LKUB21             BIT(21)                           /*!< the secure access mode lock configuration bit of union-blocks 21 */
#define TZPCU_TZBMPC_LOCK0_LKUB22             BIT(22)                           /*!< the secure access mode lock configuration bit of union-blocks 22 */
#define TZPCU_TZBMPC_LOCK0_LKUB23             BIT(23)                           /*!< the secure access mode lock configuration bit of union-blocks 23 */

/* TZPCU_TZIAC_INTEN0 */
#define TZPCU_TZIAC_INTEN0_TIMER1IE           BIT(0)                            /*!< TIMER1 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_TIMER2IE           BIT(1)                            /*!< TIMER2 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_TIMER3IE           BIT(2)                            /*!< TIMER3 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_TIMER4IE           BIT(3)                            /*!< TIMER4 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_TIMER5IE           BIT(4)                            /*!< TIMER5 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_WWDGTIE            BIT(6)                            /*!< WWDGT illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_FWDGTIE            BIT(7)                            /*!< FWDGT illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_SPI1IE             BIT(8)                            /*!< SPI1 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_USART1IE           BIT(10)                           /*!< USART1 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_USART2IE           BIT(11)                           /*!< USART2 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_I2C0IE             BIT(14)                           /*!< I2C0 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_I2C1IE             BIT(15)                           /*!< I2C1 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_USBFSIE            BIT(26)                           /*!< USBFS illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_TIMER0IE           BIT(30)                           /*!< TIMER0 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN0_SPI0IE             BIT(31)                           /*!< SPI0 illegal access interrupt enable bit */

/* TZPCU_TZIAC_INTEN1 */
#define TZPCU_TZIAC_INTEN1_USART0IE           BIT(1)                            /*!< USART0 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_TIMER15IE          BIT(3)                            /*!< TIMER15 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_TIMER16IE          BIT(4)                            /*!< TIMER16 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_HPDFIE             BIT(7)                            /*!< HPDF illegal access interrupt enable bit(not support on GD32W515TX series devices) */
#define TZPCU_TZIAC_INTEN1_CRCIE              BIT(8)                            /*!< CRC illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_TSIIE              BIT(9)                            /*!< TSI illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_ICACHEIE           BIT(10)                           /*!< ICACHE illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_ADCIE              BIT(11)                           /*!< ADC illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_CAUIE              BIT(12)                           /*!< CAU illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_HAUIE              BIT(13)                           /*!< HAU illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_TRNGIE             BIT(14)                           /*!< TRNG illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_PKCAUIE            BIT(15)                           /*!< PKCAU illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_SDIOIE             BIT(16)                           /*!< SDIO illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_RTCIE              BIT(19)                           /*!< RTC illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_PMUIE              BIT(20)                           /*!< PMU illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_SYSCFGIE           BIT(21)                           /*!< SYSCFG illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_DMA0IE             BIT(22)                           /*!< DMA0 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_DMA1IE             BIT(23)                           /*!< DMA1 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_RCUIE              BIT(25)                           /*!< RCU illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_FLASHIE            BIT(26)                           /*!< FLASH illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_FMCIE              BIT(27)                           /*!< FMC illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN1_EXTIIE             BIT(28)                           /*!< EXTI illegal access interrupt enable bit */

/* TZPCU_TZIAC_INTEN2 */
#define TZPCU_TZIAC_INTEN2_TZSPCIE            BIT(0)                            /*!< TZSPC illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_TZIACIE            BIT(1)                            /*!< TZIAC illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_SRAM0IE            BIT(4)                            /*!< SRAM0 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_TZBMPC0_REGIE      BIT(5)                            /*!< TZBMPC0_REG illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_SRAM1IE            BIT(6)                            /*!< SRAM1 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_TZBMPC1_REGIE      BIT(7)                            /*!< TZBMPC1_REG illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_SRAM2IE            BIT(8)                            /*!< SRAM2 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_TZBMPC2_REGIE      BIT(9)                            /*!< TZBMPC2_REG illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_SRAM3IE            BIT(10)                           /*!< SRAM3 illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_TZBMPC3_REGIE      BIT(11)                           /*!< TZBMPC3_REG illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_EFUSEIE            BIT(23)                           /*!< EFUSE illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_SQPI_PSRAMIE       BIT(24)                           /*!< SQPI_PSRAM illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_QSPI_FLASHIE       BIT(25)                           /*!< QSPI_FLASH illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_SQPI_PSRAMREGIE    BIT(26)                           /*!< SQPI PSRAMREG illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_QSPI_FLASHREGIE    BIT(27)                           /*!< QSPI FLASHREG illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_WIFI_RFIE          BIT(28)                           /*!< WIFI RF illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_I2S1_ADDIE         BIT(29)                           /*!< I2S1_ADD illegal access interrupt enable bit */
#define TZPCU_TZIAC_INTEN2_DCIIE              BIT(30)                           /*!< DCI illegal access interrupt enable bit(not support on GD32W515TX series devices) */
#define TZPCU_TZIAC_INTEN2_WIFIIE             BIT(31)                           /*!< WIFI illegal access interrupt enable bit */

/* TZPCU_TZIAC_STAT0 */
#define TZPCU_TZIAC_STAT0_TIMER1IAF           BIT(0)                            /*!< TIMER1 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_TIMER2IAF           BIT(1)                            /*!< TIMER2 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_TIMER3IAF           BIT(2)                            /*!< TIMER3 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_TIMER4IAF           BIT(3)                            /*!< TIMER4 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_TIMER5IAF           BIT(4)                            /*!< TIMER5 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_WWDGTIAF            BIT(6)                            /*!< WWDGT illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_FWDGTIAF            BIT(7)                            /*!< FWDGT illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_SPI1IAF             BIT(8)                            /*!< SPI1 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_USART1IAF           BIT(10)                           /*!< USART1 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_USART2IAF           BIT(11)                           /*!< USART2 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_I2C0IAF             BIT(14)                           /*!< I2C0 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_I2C1IAF             BIT(15)                           /*!< I2C1 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_USBFSIAF            BIT(26)                           /*!< USBFS illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_TIMER0IAF           BIT(30)                           /*!< TIMER0 illegal access event flag bit */
#define TZPCU_TZIAC_STAT0_SPI0IAF             BIT(31)                           /*!< SPI0 illegal access event flag bit */

/* TZPCU_TZIAC_STAT1 */
#define TZPCU_TZIAC_STAT1_USART0IAF           BIT(1)                            /*!< USART0 illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_TIMER15IAF          BIT(3)                            /*!< TIMER15 illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_TIMER16IAF          BIT(4)                            /*!< TIMER16 illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_HPDFIAF             BIT(7)                            /*!< HPDF illegal access event flag bit(not support on GD32W515TX series devices) */
#define TZPCU_TZIAC_STAT1_CRCIAF              BIT(8)                            /*!< CRC illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_TSIIAF              BIT(9)                            /*!< TSI illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_ICACHEIAF           BIT(10)                           /*!< ICACHE illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_ADCIAF              BIT(11)                           /*!< ADC illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_CAUIAF              BIT(12)                           /*!< CAU illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_HAUIAF              BIT(13)                           /*!< HAU illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_TRNGIAF             BIT(14)                           /*!< TRNG illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_PKCAUIAF            BIT(15)                           /*!< PKCAU illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_SDIOIAF             BIT(16)                           /*!< SDIO illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_RTCIAF              BIT(19)                           /*!< RTC illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_PMUIAF              BIT(20)                           /*!< PMU illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_SYSCFGIAF           BIT(21)                           /*!< SYSCFG illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_DMA0IAF             BIT(22)                           /*!< DMA0 illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_DMA1IAF             BIT(23)                           /*!< DMA1 illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_RCUIAF              BIT(25)                           /*!< RCU illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_FLASHIAF            BIT(26)                           /*!< FLASH illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_FMCIAF              BIT(27)                           /*!< FMC illegal access event flag bit */
#define TZPCU_TZIAC_STAT1_EXTIIAF             BIT(28)                           /*!< EXTI illegal access event flag bit */

/* TZPCU_TZIAC_STAT2 */
#define TZPCU_TZIAC_STAT2_TZSPCIAF            BIT(0)                            /*!< TZSPC illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_TZIACIAF            BIT(1)                            /*!< TZIAC illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_SRAM0IAF            BIT(4)                            /*!< SRAM0 illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_TZBMPC0_REGIAF      BIT(5)                            /*!< TZBMPC0_REG illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_SRAM1IAF            BIT(6)                            /*!< SRAM1 illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_TZBMPC1_REGIAF      BIT(7)                            /*!< TZBMPC1_REG illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_SRAM2IAF            BIT(8)                            /*!< SRAM2 illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_TZBMPC2_REGIAF      BIT(9)                            /*!< TZBMPC2_REG illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_SRAM3IAF            BIT(10)                           /*!< SRAM3 illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_TZBMPC3_REGIAF      BIT(11)                           /*!< TZBMPC3_REG illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_EFUSEIAF            BIT(23)                           /*!< EFUSE illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_SQPI_PSRAMIAF       BIT(24)                           /*!< SQPI_PSRAM illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_QSPI_FLASHIAF       BIT(25)                           /*!< QSPI_FLASH illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_SQPI_PSRAMREGIAF    BIT(26)                           /*!< SQPI PSRAMREG illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_QSPI_FLASHREGIAF    BIT(27)                           /*!< QSPI FLASHREG illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_WIFI_RFIAF          BIT(28)                           /*!< WIFI RF illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_I2S1_ADDIAF         BIT(29)                           /*!< I2S1_ADD illegal access event flag bit */
#define TZPCU_TZIAC_STAT2_DCIIAF              BIT(30)                           /*!< DCI illegal access event flag bit(not support on GD32W515TX series devices) */
#define TZPCU_TZIAC_STAT2_WIFIIAF             BIT(31)                           /*!< WIFI illegal access event flag bit */

/* TZPCU_TZIAC_STATC0 */
#define TZPCU_TZIAC_STATC0_TIMER1IAFC         BIT(0)                            /*!< TIMER1 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_TIMER2IAFC         BIT(1)                            /*!< TIMER2 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_TIMER3IAFC         BIT(2)                            /*!< TIMER3 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_TIMER4IAFC         BIT(3)                            /*!< TIMER4 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_TIMER5IAFC         BIT(4)                            /*!< TIMER5 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_WWDGTIAFC          BIT(6)                            /*!< WWDGT illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_FWDGTIAFC          BIT(7)                            /*!< FWDGT illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_SPI1IAFC           BIT(8)                            /*!< SPI1 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_USART1IAFC         BIT(10)                           /*!< USART1 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_USART2IAFC         BIT(11)                           /*!< USART2 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_I2C0IAFC           BIT(14)                           /*!< I2C0 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_I2C1IAFC           BIT(15)                           /*!< I2C1 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_USBFSIAFC          BIT(26)                           /*!< USBFS illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_TIMER0IAFC         BIT(30)                           /*!< TIMER0 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC0_SPI0IAFC           BIT(31)                           /*!< SPI0 illegal access flag clear bit */

/* TZPCU_TZIAC_STATC1 */
#define TZPCU_TZIAC_STATC1_USART0IAFC         BIT(1)                            /*!< USART0 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_TIMER15IAFC        BIT(3)                            /*!< TIMER15 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_TIMER16IAFC        BIT(4)                            /*!< TIMER16 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_HPDFIAFC           BIT(7)                            /*!< HPDF illegal access flag clear bit(not support on GD32W515TX series devices) */
#define TZPCU_TZIAC_STATC1_CRCIAFC            BIT(8)                            /*!< CRC illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_TSIIAFC            BIT(9)                            /*!< TSI illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_ICACHEIAFC         BIT(10)                           /*!< ICACHE illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_ADCIAFC            BIT(11)                           /*!< ADC illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_CAUIAFC            BIT(12)                           /*!< CAU illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_HAUIAFC            BIT(13)                           /*!< HAU illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_TRNGIAFC           BIT(14)                           /*!< TRNG illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_PKCAUIAFC          BIT(15)                           /*!< PKCAU illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_SDIOIAFC           BIT(16)                           /*!< SDIO illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_RTCIAFC            BIT(19)                           /*!< RTC illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_PMUIAFC            BIT(20)                           /*!< PMU illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_SYSCFGIAFC         BIT(21)                           /*!< SYSCFG illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_DMA0IAFC           BIT(22)                           /*!< DMA0 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_DMA1IAFC           BIT(23)                           /*!< DMA1 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_RCUIAFC            BIT(25)                           /*!< RCU illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_FLASHIAFC          BIT(26)                           /*!< FLASH illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_FMCIAFC            BIT(27)                           /*!< FMC illegal access flag clear bit */
#define TZPCU_TZIAC_STATC1_EXTIIAFC           BIT(28)                           /*!< EXTI illegal access flag clear bit */

/* TZPCU_TZIAC_STATC2 */
#define TZPCU_TZIAC_STATC2_TZSPCIAFC          BIT(0)                            /*!< TZSPC illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_TZIACIAFC          BIT(1)                            /*!< TZIAC illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_SRAM0IAFC          BIT(4)                            /*!< SRAM0 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_TZBMPC0_REGIAFC    BIT(5)                            /*!< TZBMPC0_REG illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_SRAM1IAFC          BIT(6)                            /*!< SRAM1 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_TZBMPC1_REGIAFC    BIT(7)                            /*!< TZBMPC1_REG illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_SRAM2IAFC          BIT(8)                            /*!< SRAM2 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_TZBMPC2_REGIAFC    BIT(9)                            /*!< TZBMPC2_REG illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_SRAM3IAFC          BIT(10)                           /*!< SRAM3 illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_TZBMPC3_REGIAFC    BIT(11)                           /*!< TZBMPC3_REG illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_EFUSEIAFC          BIT(23)                           /*!< EFUSE illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_SQPI_PSRAMIAFC     BIT(24)                           /*!< SQPI_PSRAM illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_QSPI_FLASHIAFC     BIT(25)                           /*!< QSPI_FLASH illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_SQPI_PSRAMREGIAFC  BIT(26)                           /*!< SQPI PSRAMREG illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_QSPI_FLASHREGIAFC  BIT(27)                           /*!< QSPI FLASHREG illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_WIFI_RFIAFC        BIT(28)                           /*!< WIFI RF illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_I2S1_ADDIAFC       BIT(29)                           /*!< I2S1_ADD illegal access flag clear bit */
#define TZPCU_TZIAC_STATC2_DCIIAFC            BIT(30)                           /*!< DCI illegal access flag clear bit(not support on GD32W515TX series devices) */
#define TZPCU_TZIAC_STATC2_WIFIIAFC           BIT(31)                           /*!< WIFI illegal access flag clear bit */

/* constants definitions */
/* peripherals position definitions */
#define TZPCU_PERIPH_TIMER1                  (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_TIMER1IE)                                      /*!< TIMER1 peripheral definitions */
#define TZPCU_PERIPH_TIMER2                  (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_TIMER2IE)                                      /*!< TIMER2 peripheral definitions */
#define TZPCU_PERIPH_TIMER3                  (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_TIMER3IE)                                      /*!< TIMER3 peripheral definitions */
#define TZPCU_PERIPH_TIMER4                  (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_TIMER4IE)                                      /*!< TIMER4 peripheral definitions */
#define TZPCU_PERIPH_TIMER5                  (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_TIMER5IE)                                      /*!< TIMER5 peripheral definitions */
#define TZPCU_PERIPH_WWDGT                   (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_WWDGTIE)                                       /*!< WWDGT peripheral definitions */
#define TZPCU_PERIPH_FWDGT                   (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_FWDGTIE)                                       /*!< FWDGT peripheral definitions */
#define TZPCU_PERIPH_SPI1                    (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_SPI1IE)                                        /*!< SPI1 peripheral definitions */
#define TZPCU_PERIPH_USART1                  (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_USART1IE)                                      /*!< USART1 peripheral definitions */
#define TZPCU_PERIPH_USART2                  (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_USART2IE)                                      /*!< USART2 peripheral definitions */
#define TZPCU_PERIPH_I2C0                    (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_I2C0IE)                                        /*!< I2C0 peripheral definitions */
#define TZPCU_PERIPH_I2C1                    (((uint32_t)0x00000000U) | TZPCU_TZIAC_INTEN0_I2C1IE)                                        /*!< I2C1 peripheral definitions */
#define TZPCU_PERIPH_USBFS                   (((uint32_t)0x00200000U) | (TZPCU_TZIAC_INTEN0_USBFSIE >> 16))                               /*!< USBFS peripheral definitions */
#define TZPCU_PERIPH_TIMER0                  (((uint32_t)0x00200000U) | (TZPCU_TZIAC_INTEN0_TIMER0IE >> 16))                              /*!< TIMER0 peripheral definitions */
#define TZPCU_PERIPH_SPI0                    (((uint32_t)0x00200000U) | (TZPCU_TZIAC_INTEN0_SPI0IE >> 16))                                /*!< SPI0 peripheral definitions */
#define TZPCU_PERIPH_USART0                  (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_USART0IE)                                      /*!< USART0 peripheral definitions */
#define TZPCU_PERIPH_TIMER15                 (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_TIMER15IE)                                     /*!< TIMER15 peripheral definitions */
#define TZPCU_PERIPH_TIMER16                 (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_TIMER16IE)                                     /*!< TIMER16 peripheral definitions */
#define TZPCU_PERIPH_HPDF                    (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_HPDFIE)                                        /*!< HPDF peripheral definitions(not support on GD32W515TX series devices) */
#define TZPCU_PERIPH_CRC                     (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_CRCIE)                                         /*!< CRC peripheral definitions */
#define TZPCU_PERIPH_TSI                     (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_TSIIE)                                         /*!< TSI peripheral definitions */
#define TZPCU_PERIPH_ICACHE                  (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_ICACHEIE)                                      /*!< ICACHE peripheral definitions */
#define TZPCU_PERIPH_ADC                     (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_ADCIE)                                         /*!< ADC peripheral definitions */
#define TZPCU_PERIPH_CAU                     (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_CAUIE)                                         /*!< CAU peripheral definitions */
#define TZPCU_PERIPH_HAU                     (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_HAUIE)                                         /*!< HAU peripheral definitions */
#define TZPCU_PERIPH_TRNG                    (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_TRNGIE)                                        /*!< TRNG peripheral definitions */
#define TZPCU_PERIPH_PKCAU                   (((uint32_t)0x10000000U) | TZPCU_TZIAC_INTEN1_PKCAUIE)                                       /*!< PKCAU peripheral definitions */
#define TZPCU_PERIPH_SDIO                    (((uint32_t)0x10200000U) | (TZPCU_TZIAC_INTEN1_SDIOIE >> 16))                                /*!< SDIO peripheral definitions */
#define TZPCU_PERIPH_RTC                     (((uint32_t)0x10200000U) | (TZPCU_TZIAC_INTEN1_RTCIE >> 16))                                 /*!< RTC peripheral definitions */
#define TZPCU_PERIPH_PMU                     (((uint32_t)0x10200000U) | (TZPCU_TZIAC_INTEN1_PMUIE >> 16))                                 /*!< PMU peripheral definitions */
#define TZPCU_PERIPH_SYSCFG                  (((uint32_t)0x10200000U) | (TZPCU_TZIAC_INTEN1_SYSCFGIE >> 16))                              /*!< SYSCFG peripheral definitions */
#define TZPCU_PERIPH_DMA0                    (((uint32_t)0x10200000U) | (TZPCU_TZIAC_INTEN1_DMA0IE >> 16))                                /*!< DMA0 peripheral definitions */
#define TZPCU_PERIPH_DMA1                    (((uint32_t)0x10200000U) | (TZPCU_TZIAC_INTEN1_DMA1IE >> 16))                                /*!< DMA1 peripheral definitions */
#define TZPCU_PERIPH_RCU                     (((uint32_t)0x10200000U) | (TZPCU_TZIAC_INTEN1_RCUIE >> 16))                                 /*!< RCU peripheral definitions */
#define TZPCU_PERIPH_FLASH                   (((uint32_t)0x10200000U) | (TZPCU_TZIAC_INTEN1_FLASHIE >> 16))                               /*!< FLASH peripheral definitions */
#define TZPCU_PERIPH_FMC                     (((uint32_t)0x10200000U) | (TZPCU_TZIAC_INTEN1_FMCIE >> 16))                                 /*!< FMC peripheral definitions */
#define TZPCU_PERIPH_EXTI                    (((uint32_t)0x10200000U) | (TZPCU_TZIAC_INTEN1_EXTIIE >> 16))                                /*!< EXTI peripheral definitions */
#define TZPCU_PERIPH_TZSPC                   (((uint32_t)0x20000000U) | TZPCU_TZIAC_INTEN2_TZSPCIE)                                       /*!< TZSPC peripheral definitions */
#define TZPCU_PERIPH_TZIAC                   (((uint32_t)0x20000000U) | TZPCU_TZIAC_INTEN2_TZIACIE)                                       /*!< TZIAC peripheral definitions */
#define TZPCU_PERIPH_SRAM0                   (((uint32_t)0x20000000U) | TZPCU_TZIAC_INTEN2_SRAM0IE)                                       /*!< SRAM0 peripheral definitions */
#define TZPCU_PERIPH_TZBMPC0_REG             (((uint32_t)0x20000000U) | TZPCU_TZIAC_INTEN2_TZBMPC0_REGIE)                                 /*!< TZBMPC0_REG peripheral definitions */
#define TZPCU_PERIPH_SRAM1                   (((uint32_t)0x20000000U) | TZPCU_TZIAC_INTEN2_SRAM1IE)                                       /*!< SRAM1 peripheral definitions */
#define TZPCU_PERIPH_TZBMPC1_REG             (((uint32_t)0x20000000U) | TZPCU_TZIAC_INTEN2_TZBMPC1_REGIE)                                 /*!< TZBMPC1_REG peripheral definitions */
#define TZPCU_PERIPH_SRAM2                   (((uint32_t)0x20000000U) | TZPCU_TZIAC_INTEN2_SRAM2IE)                                       /*!< SRAM2 peripheral definitions */
#define TZPCU_PERIPH_TZBMPC2_REG             (((uint32_t)0x20000000U) | TZPCU_TZIAC_INTEN2_TZBMPC2_REGIE)                                 /*!< TZBMPC2_REG peripheral definitions */
#define TZPCU_PERIPH_SRAM3                   (((uint32_t)0x20000000U) | TZPCU_TZIAC_INTEN2_SRAM3IE)                                       /*!< SRAM3 peripheral definitions */
#define TZPCU_PERIPH_TZBMPC3_REG             (((uint32_t)0x20000000U) | TZPCU_TZIAC_INTEN2_TZBMPC3_REGIE)                                 /*!< TZBMPC3_REG peripheral definitions */
#define TZPCU_PERIPH_EFUSE                   (((uint32_t)0x20200000U) | (TZPCU_TZIAC_INTEN2_EFUSEIE >> 16))                               /*!< EFUSE peripheral definitions */
#define TZPCU_PERIPH_SQPI_PSRAM              (((uint32_t)0x20200000U) | (TZPCU_TZIAC_INTEN2_SQPI_PSRAMIE >> 16))                          /*!< SQPI_PSRAM peripheral definitions */
#define TZPCU_PERIPH_QSPI_FLASH              (((uint32_t)0x20200000U) | (TZPCU_TZIAC_INTEN2_QSPI_FLASHIE >> 16))                          /*!< QSPI_FLASH peripheral definitions */
#define TZPCU_PERIPH_DBG                     (((uint32_t)0x20200000U) | (BIT(25) >> 16))                                                  /*!< DBG peripheral definitions */
#define TZPCU_PERIPH_SQPI_PSRAMREG           (((uint32_t)0x20200000U) | (TZPCU_TZIAC_INTEN2_SQPI_PSRAMREGIE >> 16))                       /*!< SQPI PSRAMREG peripheral definitions */
#define TZPCU_PERIPH_QSPI_FLASHREG           (((uint32_t)0x20200000U) | (TZPCU_TZIAC_INTEN2_QSPI_FLASHREGIE >> 16))                       /*!< QSPI FLASHREG peripheral definitions */
#define TZPCU_PERIPH_WIFI_RF                 (((uint32_t)0x20200000U) | (TZPCU_TZIAC_INTEN2_WIFI_RFIE >> 16))                             /*!< WIFI RF peripheral definitions */
#define TZPCU_PERIPH_I2S1_ADD                (((uint32_t)0x20200000U) | (TZPCU_TZIAC_INTEN2_I2S1_ADDIE  >> 16))                           /*!< I2S1_ADD peripheral definitions */
#define TZPCU_PERIPH_DCI                     (((uint32_t)0x20200000U) | (TZPCU_TZIAC_INTEN2_DCIIE >> 16))                                 /*!< DCI peripheral definitions(not support on GD32W515TX series devices) */
#define TZPCU_PERIPH_WIFI                    (((uint32_t)0x20200000U) | (TZPCU_TZIAC_INTEN2_WIFIIE >> 16))                                /*!< WIFI peripheral definitions */
#define TZPCU_PERIPH_ALL                     BIT(22)                                                                                      /*!< all peripheral definitions */

/* peripherals illegal access interrupt flag definitions, used for tzpcu_tziac_flag_get and tzpcu_tziac_flag_clear function */
#define TZPCU_TZIAC_FLAG_TIMER1              TZPCU_TZIAC_FLAG_TIMER1                       /*!< TIMER1 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TIMER2              TZPCU_TZIAC_FLAG_TIMER2                       /*!< TIMER2 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TIMER3              TZPCU_TZIAC_FLAG_TIMER3                       /*!< TIMER3 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TIMER4              TZPCU_TZIAC_FLAG_TIMER4                       /*!< TIMER4 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TIMER5              TZPCU_TZIAC_FLAG_TIMER5                       /*!< TIMER5 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_WWDGT               TZPCU_TZIAC_FLAG_WWDGT                        /*!< WWDGT peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_FWDGT               TZPCU_TZIAC_FLAG_FWDGT                        /*!< FWDGT peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_SPI1                TZPCU_TZIAC_FLAG_SPI1                         /*!< SPI1 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_USART1              TZPCU_TZIAC_FLAG_USART1                       /*!< USART1 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_USART2              TZPCU_TZIAC_FLAG_USART2                       /*!< USART2 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_I2C0                TZPCU_TZIAC_FLAG_I2C0                         /*!< I2C0 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_I2C1                TZPCU_TZIAC_FLAG_I2C1                         /*!< I2C1 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_USBFS               TZPCU_TZIAC_FLAG_USBFS                        /*!< USBFS peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TIMER0              TZPCU_TZIAC_FLAG_TIMER0                       /*!< TIMER0 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_SPI0                TZPCU_TZIAC_FLAG_SPI0                         /*!< SPI0 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_USART0              TZPCU_TZIAC_FLAG_USART0                       /*!< USART0 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TIMER15             TZPCU_TZIAC_FLAG_TIMER15                      /*!< TIMER15 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TIMER16             TZPCU_TZIAC_FLAG_TIMER16                      /*!< TIMER16 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_HPDF                TZPCU_TZIAC_FLAG_HPDF                         /*!< HPDF peripheral llegal access interrupt flag definitions(not support on GD32W515TX series devices) */
#define TZPCU_TZIAC_FLAG_CRC                 TZPCU_TZIAC_FLAG_CRC                          /*!< CRC peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TSI                 TZPCU_TZIAC_FLAG_TSI                          /*!< TSI peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_ICACHE              TZPCU_TZIAC_FLAG_ICACHE                       /*!< ICACHE peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_ADC                 TZPCU_TZIAC_FLAG_ADC                          /*!< ADC peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_CAU                 TZPCU_TZIAC_FLAG_CAU                          /*!< CAU peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_HAU                 TZPCU_TZIAC_FLAG_HAU                          /*!< HAU peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TRNG                TZPCU_TZIAC_FLAG_TRNG                         /*!< TRNG peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_PKCAU               TZPCU_TZIAC_FLAG_PKCAU                        /*!< PKCAU peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_SDIO                TZPCU_TZIAC_FLAG_SDIO                         /*!< SDIO peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_RTC                 TZPCU_TZIAC_FLAG_RTC                          /*!< RTC peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_PMU                 TZPCU_TZIAC_FLAG_PMU                          /*!< PMU peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_SYSCFG              TZPCU_TZIAC_FLAG_SYSCFG                       /*!< SYSCFG peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_DMA0                TZPCU_TZIAC_FLAG_DMA0                         /*!< DMA0 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_DMA1                TZPCU_TZIAC_FLAG_DMA1                         /*!< DMA1 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_RCU                 TZPCU_TZIAC_FLAG_RCU                          /*!< RCU peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_FLASH               TZPCU_TZIAC_FLAG_FLASH                        /*!< FLASH peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_FMC                 TZPCU_TZIAC_FLAG_FMC                          /*!< FMC peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_EXTI                TZPCU_TZIAC_FLAG_EXTI                         /*!< EXTI peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TZSPC               TZPCU_TZIAC_FLAG_TZSPC                        /*!< TZSPC peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TZIAC               TZPCU_TZIAC_FLAG_TZIAC                        /*!< TZIAC peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_SRAM0               TZPCU_TZIAC_FLAG_SRAM0                        /*!< SRAM0 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TZBMPC0_REG         TZPCU_TZIAC_FLAG_TZBMPC0_REG                  /*!< TZBMPC0_REG peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_SRAM1               TZPCU_TZIAC_FLAG_SRAM1                        /*!< SRAM1 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TZBMPC1_REG         TZPCU_TZIAC_FLAG_TZBMPC1_REG                  /*!< TZBMPC1_REG peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_SRAM2               TZPCU_TZIAC_FLAG_SRAM2                        /*!< SRAM2 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TZBMPC2_REG         TZPCU_TZIAC_FLAG_TZBMPC2_REG                  /*!< TZBMPC2_REG peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_SRAM3               TZPCU_TZIAC_FLAG_SRAM3                        /*!< SRAM3 peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_TZBMPC3_REG         TZPCU_TZIAC_FLAG_TZBMPC3_REG                  /*!< TZBMPC3_REG peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_EFUSE               TZPCU_TZIAC_FLAG_EFUSE                        /*!< EFUSE peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_SQPI_PSRAM          TZPCU_TZIAC_FLAG_SQPI_PSRAM                   /*!< SQPI_PSRAM peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_QSPI_FLASH          TZPCU_TZIAC_FLAG_QSPI_FLASH                   /*!< QSPI_FLASH peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_SQPI_PSRAMREG       TZPCU_TZIAC_FLAG_SQPI_PSRAMREG                /*!< SQPI PSRAMREG peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_QSPI_FLASHREG       TZPCU_TZIAC_FLAG_QSPI_FLASHREG                /*!< QSPI FLASHREG peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_WIFI_RF             TZPCU_TZIAC_FLAG_WIFI_RF                      /*!< WIFI RF peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_I2S1_ADD            TZPCU_TZIAC_FLAG_I2S1_ADD                     /*!< I2S1_ADD peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_DCI                 TZPCU_TZIAC_FLAG_DCI                          /*!< DCI peripheral llegal access interrupt flag definitions(not support on GD32W515TX series devices) */
#define TZPCU_TZIAC_FLAG_WIFI                TZPCU_TZIAC_FLAG_WIFI                         /*!< WIFI peripheral llegal access interrupt flag definitions */
#define TZPCU_TZIAC_FLAG_ALL                 BIT(22)                                       /*!< all peripheral llegal access interrupt flag definitions  */

/* peripheral attributes definitions */
#define TZPCU_SEC                            (BIT(28) | (uint32_t)0x00000001U)             /*!< peripheral secure attribute */
#define TZPCU_NSEC                           (BIT(28) | (uint32_t)0x00000000U)             /*!< peripheral non-secure attribute */
#define TZPCU_PRIV                           (BIT(29) | (uint32_t)0x00000002U)             /*!< peripheral privilege attribute */
#define TZPCU_NPRIV                          (BIT(29) | (uint32_t)0x00000000U)             /*!< peripheral non-privilege attribute */

/* QSPI flash and SQPI PSRAM configuration flag for non_secure_mark_struct member 'memory_type' */
typedef enum
{
    QSPI_FLASH_MEM = 0U,                                                /*!< QSPI flash memory */
    SQPI_PSRAM_MEM                                                      /*!< SQPI PSRAM memory */
}tzpcu_mem;

/* QSPI flash and SQPI PSRAM non secure mark region flag for non_secure_mark_struct member 'region_number' */
typedef enum
{
    NON_SECURE_MARK_REGION0            = 0U,                            /*!< non secure mark region 0 */
    NON_SECURE_MARK_REGION1,                                            /*!< non secure mark region 1 */
    NON_SECURE_MARK_REGION2,                                            /*!< non secure mark region 2(QSPI flash only) */
    NON_SECURE_MARK_REGION3                                             /*!< non secure mark region 3(QSPI flash only) */
}tzpcu_non_secure_mark_region;

/* structure of external memory non-secure mark */ 
typedef struct
{
    tzpcu_mem memory_type;                                             /*!< memory type, refer to tzpcu_mem */
    tzpcu_non_secure_mark_region region_number;                        /*!< non secure mark region number, refer to tzpcu_non_secure_mark_region */
    uint32_t start_address;                                            /*!< external memory non-secure area start address */
    uint32_t length;                                                   /*!< external memory non-secure area length */
}tzpcu_non_secure_mark_struct;

#define EXT_MEM_GRANULARITY               (0x2000U)                    /*!< granularity of QSPI flash and SQPI PSRAM is 8K */

/* debug type */
#define INVASIVE_DEBUG                    TZPCU_TZSPC_DBG_CFG_IDEN     /*!< invasive debug */
#define NON_INVASIVE_DEBUG                TZPCU_TZSPC_DBG_CFG_NIDEN    /*!< non-invasive debug */
#define SECURE_INVASIVE_DEBUG             TZPCU_TZSPC_DBG_CFG_SPIDEN   /*!< secure invasive debug */
#define SECURE_NON_INVASIVE_DEBUG         TZPCU_TZSPC_DBG_CFG_SPNIDEN  /*!< secure non-Invasive debug */

#define SECURE_ILLEGAL_ACCESS_ENABLE      ((uint32_t)0x00000000U)      /*!< secure read/write access non-secure SRAM is illegal */
#define SECURE_ILLEGAL_ACCESS_DISABLE     ((uint32_t)0x00000000U)      /*!< secure read/write access non-secure SRAM is legal */

/* default security state */
#define DEFAULT_STATE                     ((uint32_t)0x00000000U)      /*!< default state (source clock secured if a secure area exists in the TZBMPC and vice-versa) */
#define INVERT_STATE                      ((uint32_t)0x00000001U)      /*!< invert the state, source clock remains secure even if no secure block is set in the TZBMPC */

/* block secure access mode */
#define BLOCK_SECURE_ACCESS_MODE_SEC      ((uint32_t)0x00000000U)     /*!< block secure access mode is secure */
#define BLOCK_SECURE_ACCESS_MODE_NSEC     ((uint32_t)0x00000001U)     /*!< block secure access mode is non-secure */

/* union block lock configure */
#define UNION_BLOCK_LOCK                  ((uint32_t)0x00000001U)     /*!< lock union block */
#define UNION_BLOCK_UNLOCK                ((uint32_t)0x00000000U)     /*!< unlock union block */

/* illegal access event flag */
#define NO_ILLEGAL_ACCESS_PENDING         ((uint32_t)0x00000000U)     /*!< no illegal access event pending */
#define ILLEGAL_ACCESS_PENDING            ((uint32_t)0x00000001U)     /*!< illegal access event pending */

/* function declarations */
/* TZSPC function */
/* configure peripherals secure attributes */
void tzpcu_tzspc_peripheral_attributes_config(uint32_t periph, uint32_t attributes);
/* get peripherals secure attributes */
uint32_t tzpcu_tzspc_peripheral_attributes_get(uint32_t periph);

#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
/* initialize the TZPCU non-secure mark struct with the default values */
void tzpcu_non_secure_mark_struct_para_init(tzpcu_non_secure_mark_struct* init_struct);
/* configure external memory non-secure mark */
ErrStatus tzpcu_tzspc_emnsm_config(tzpcu_non_secure_mark_struct *p_non_secure_mark);
/* lock TZSPC items */
void tzpcu_tzspc_items_lock(void);
/* configure debug type */
void tzpcu_tzspc_dbg_config(uint32_t dbg_type, ControlStatus config_value);
#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */

/* TZBMPC function */
/* lock the control register of the TZBMPC sub-block */
void tzpcu_tzbmpc_lock(uint32_t tzbmpx);
/* configure default security state */
void tzpcu_tzbmpc_security_state_config(uint32_t tzbmpx, uint32_t sec_state);
/* configure secure access state */
void tzpcu_tzbmpc_secure_access_config(uint32_t tzbmpx, uint32_t sec_illaccess_state);
/* configure block secure access mode */
void tzpcu_tzbmpc_block_secure_access_mode_config(uint32_t tzbmpx, uint32_t block_pos_num, uint32_t access_mode);
/* lock configure union block secure access mode */
void tzpcu_tzbmpc_union_block_lock(uint32_t tzbmpx, uint32_t union_block_postion_num);

/* TZIAC function */
/* enable illegal access interrupt */
void tzpcu_tziac_interrupt_enable(uint32_t periph);
/* disable illegal access interrupt */
void tzpcu_tziac_interrupt_disable(uint32_t periph);
/* get illegal access interrupt flag */
uint32_t tzpcu_tziac_flag_get(uint32_t periph_flag);
/* clear illegal access interrupt flag */
void tzpcu_tziac_flag_clear(uint32_t periph_flag);

#endif /* GD32W51X_TZPCU_H */
