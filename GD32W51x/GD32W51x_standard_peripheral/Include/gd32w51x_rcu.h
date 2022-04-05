/*!
    \file    gd32w51x_rcu.h
    \brief   definitions for the RCU
    
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

#ifndef GD32W51X_RCU_H
#define GD32W51X_RCU_H

#include "gd32w51x.h"

/* RCU definitions */
#define RCU                             RCU_BASE

/* registers definitions */
#define RCU_CTL                         REG32(RCU + 0x00000000U)        /*!< control register */
#define RCU_PLL                         REG32(RCU + 0x00000004U)        /*!< PLL register */
#define RCU_CFG0                        REG32(RCU + 0x00000008U)        /*!< clock configuration register 0 */
#define RCU_INT                         REG32(RCU + 0x0000000CU)        /*!< clock interrupt register */
#define RCU_AHB1RST                     REG32(RCU + 0x00000010U)        /*!< AHB1 reset register */
#define RCU_AHB2RST                     REG32(RCU + 0x00000014U)        /*!< AHB2 reset register */
#define RCU_AHB3RST                     REG32(RCU + 0x00000018U)        /*!< AHB3 reset register */
#define RCU_APB1RST                     REG32(RCU + 0x00000020U)        /*!< APB1 reset register */
#define RCU_APB2RST                     REG32(RCU + 0x00000024U)        /*!< APB2 reset register */
#define RCU_AHB1EN                      REG32(RCU + 0x00000030U)        /*!< AHB1 enable register */
#define RCU_AHB2EN                      REG32(RCU + 0x00000034U)        /*!< AHB2 enable register */
#define RCU_AHB3EN                      REG32(RCU + 0x00000038U)        /*!< AHB3 enable register */
#define RCU_APB1EN                      REG32(RCU + 0x00000040U)        /*!< APB1 enable register */
#define RCU_APB2EN                      REG32(RCU + 0x00000044U)        /*!< APB2 enable register */
#define RCU_AHB1SPEN                    REG32(RCU + 0x00000050U)        /*!< AHB1 sleep mode enable register */
#define RCU_AHB2SPEN                    REG32(RCU + 0x00000054U)        /*!< AHB2 sleep mode enable register */
#define RCU_AHB3SPEN                    REG32(RCU + 0x00000058U)        /*!< AHB3 sleep mode enable register */ 
#define RCU_APB1SPEN                    REG32(RCU + 0x00000060U)        /*!< APB1 sleep mode enable register */
#define RCU_APB2SPEN                    REG32(RCU + 0x00000064U)        /*!< APB2 sleep mode enable register */
#define RCU_BDCTL                       REG32(RCU + 0x00000070U)        /*!< backup domain control register */
#define RCU_RSTSCK                      REG32(RCU + 0x00000074U)        /*!< reset source / clock register */
#define RCU_PLLSSCTL                    REG32(RCU + 0x00000080U)        /*!< PLL clock spread spectrum control register */
#define RCU_PLLCFG                      REG32(RCU + 0x00000084U)        /*!< PLL clock configuration register */ 
#define RCU_CFG1                        REG32(RCU + 0x0000008CU)        /*!< clock configuration register 1 */
#define RCU_ADDCTL                      REG32(RCU + 0x00000090U)        /*!< additional clock control register */
#define RCU_SECP_CFG                    REG32(RCU + 0x000000C0U)        /*!< secure configuration register */
#define RCU_SECP_STAT                   REG32(RCU + 0x000000C4U)        /*!< secure status register */
#define RCU_AHB1SECP_STAT               REG32(RCU + 0x000000C8U)        /*!< AHB1 secure status register */
#define RCU_AHB2SECP_STAT               REG32(RCU + 0x000000CCU)        /*!< AHB2 secure status register */
#define RCU_AHB3SECP_STAT               REG32(RCU + 0x000000D0U)        /*!< AHB3 secure status register */
#define RCU_APB1SECP_STAT               REG32(RCU + 0x000000D4U)        /*!< APB1 secure status register */
#define RCU_APB2SECP_STAT               REG32(RCU + 0x000000D8U)        /*!< APB2 secure status register */
#define RCU_VKEY                        REG32(RCU + 0x00000100U)        /*!< voltage key register */
#define RCU_DSV                         REG32(RCU + 0x00000134U)        /*!< deep-sleep mode voltage register */

/* bits definitions */
/* RCU_CTL */
#define RCU_CTL_IRC16MEN                BIT(0)                    /*!< internal high speed oscillator enable */
#define RCU_CTL_IRC16MSTB               BIT(1)                    /*!< IRC16M high speed internal oscillator stabilization flag */
#define RCU_CTL_RCUPRIP                 BIT(2)                    /*!< privilege protection */
#define RCU_CTL_IRC16MADJ               BITS(3,7)                 /*!< high speed internal oscillator clock trim adjust value */
#define RCU_CTL_IRC16MCALIB             BITS(8,15)                /*!< high speed internal oscillator calibration value register */
#define RCU_CTL_HXTALEN                 BIT(16)                   /*!< external high speed oscillator enable */
#define RCU_CTL_HXTALSTB                BIT(17)                   /*!< external crystal oscillator clock stabilization flag */
#define RCU_CTL_HXTALBPS                BIT(18)                   /*!< external crystal oscillator clock bypass mode enable */
#define RCU_CTL_CKMEN                   BIT(19)                   /*!< HXTAL clock monitor enable */
#define RCU_CTL_PLLDIGPU                BIT(20)                   /*!< PLLDIG power up */
#define RCU_CTL_PLLDIGEN                BIT(21)                   /*!< PLLDIG enable */
#define RCU_CTL_RFCKMEN                 BIT(22)                   /*!< HXTAL clock monitor enable */
#define RCU_CTL_PLLDIGSTB               BIT(23)                   /*!< PLLDIG clock stabilization flag */
#define RCU_CTL_PLLEN                   BIT(24)                   /*!< PLL enable */
#define RCU_CTL_PLLSTB                  BIT(25)                   /*!< PLL clock stabilization flag */
#define RCU_CTL_PLLI2SEN                BIT(26)                   /*!< PLLI2S enable */
#define RCU_CTL_PLLI2SSTB               BIT(27)                   /*!< PLLI2S clock stabilization flag */
#define RCU_CTL_HXTALPU                 BIT(28)                   /*!< high speed crystal oscillator (HXTAL) power up */ 
#define RCU_CTL_HXTALENI2S              BIT(29)                   /*!< high speed crystal oscillator enable for PLLI2S */  
#define RCU_CTL_HXTALENPLL              BIT(30)                   /*!< high speed crystal oscillator enable for system PLLsys */ 
#define RCU_CTL_HXTALREADY              BIT(31)                   /*!< high speed crystal oscillator ready set by software */ 

/* RCU_PLL */
#define RCU_PLL_PLLPSC                  BITS(0,5)                 /*!< The PLL VCO source clock prescaler */
#define RCU_PLL_PLLN                    BITS(6,14)                /*!< The PLL VCO clock multi factor */
#define RCU_PLL_PLLSEL                  BIT(15)                   /*!< PLL Clock Source Selection */
#define RCU_PLL_PLLP                    BITS(16,17)               /*!< The PLLP output frequency division factor from PLL VCO clock */

/* RCU_CFG0 */
#define RCU_CFG0_SCS                    BITS(0,1)                 /*!< system clock switch */
#define RCU_CFG0_SCSS                   BITS(2,3)                 /*!< system clock switch status */
#define RCU_CFG0_AHBPSC                 BITS(4,7)                 /*!< AHB prescaler selection */
#define RCU_CFG0_APB1PSC                BITS(10,12)               /*!< APB1 prescaler selection */
#define RCU_CFG0_APB2PSC                BITS(13,15)               /*!< APB2 prescaler selection */
#define RCU_CFG0_RTCDIV                 BITS(16,20)               /*!< RTC clock divider factor */
#define RCU_CFG0_CKOUT0SEL              BITS(21,22)               /*!< CKOUT0 Clock Source Selection */
#define RCU_CFG0_CKOUT0DIV              BITS(24,26)               /*!< The CK_OUT0 divider which the CK_OUT0 frequency can be reduced */
#define RCU_CFG0_CKOUT1DIV              BITS(27,29)               /*!< The CK_OUT1 divider which the CK_OUT1 frequency can be reduced */
#define RCU_CFG0_CKOUT1SEL              BITS(30,31)               /*!< CKOUT1 Clock Source Selection */

/* RCU_INT */
#define RCU_INT_IRC32KSTBIF             BIT(0)                    /*!< IRC32K stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF              BIT(1)                    /*!< LXTAL stabilization interrupt flag */
#define RCU_INT_IRC16MSTBIF             BIT(2)                    /*!< IRC16M stabilization interrupt flag */
#define RCU_INT_HXTALSTBIF              BIT(3)                    /*!< HXTAL stabilization interrupt flag */
#define RCU_INT_PLLSTBIF                BIT(4)                    /*!< PLL stabilization interrupt flag */
#define RCU_INT_PLLI2SSTBIF             BIT(5)                    /*!< PLLI2S stabilization interrupt flag */
#define RCU_INT_PLLDIGSTBIF             BIT(6)                    /*!< PLLDIG stabilization interrupt flag */
#define RCU_INT_CKMIF                   BIT(7)                    /*!< HXTAL clock stuck interrupt flag */
#define RCU_INT_IRC32KSTBIE             BIT(8)                    /*!< IRC32K stabilization interrupt enable */
#define RCU_INT_LXTALSTBIE              BIT(9)                    /*!< LXTAL stabilization interrupt enable */
#define RCU_INT_IRC16MSTBIE             BIT(10)                   /*!< IRC16M stabilization interrupt enable */
#define RCU_INT_HXTALSTBIE              BIT(11)                   /*!< HXTAL stabilization interrupt enable */
#define RCU_INT_PLLSTBIE                BIT(12)                   /*!< PLL stabilization interrupt enable */
#define RCU_INT_PLLI2SSTBIE             BIT(13)                   /*!< PLLI2S Stabilization Interrupt Enable */
#define RCU_INT_PLLDIGSTBIE             BIT(14)                   /*!< PLLDIG Stabilization Interrupt Enable */
#define RCU_INT_IRC32KSTBIC             BIT(16)                   /*!< IRC32K Stabilization Interrupt Clear */
#define RCU_INT_LXTALSTBIC              BIT(17)                   /*!< LXTAL Stabilization Interrupt Clear */
#define RCU_INT_IRC16MSTBIC             BIT(18)                   /*!< IRC16M Stabilization Interrupt Clear */
#define RCU_INT_HXTALSTBIC              BIT(19)                   /*!< HXTAL Stabilization Interrupt Clear */
#define RCU_INT_PLLSTBIC                BIT(20)                   /*!< PLL stabilization Interrupt Clear */
#define RCU_INT_PLLI2SSTBIC             BIT(21)                   /*!< PLLI2S stabilization Interrupt Clear */
#define RCU_INT_PLLDIGSTBIC             BIT(22)                   /*!< PLLDIG stabilization Interrupt Clear */
#define RCU_INT_CKMIC                   BIT(23)                   /*!< HXTAL Clock Stuck Interrupt Clear */

/* RCU_AHB1RST */
#define RCU_AHB1RST_PARST               BIT(0)                    /*!< GPIO port A reset */
#define RCU_AHB1RST_PBRST               BIT(1)                    /*!< GPIO port B reset */
#define RCU_AHB1RST_PCRST               BIT(2)                    /*!< GPIO port C reset */
#define RCU_AHB1RST_TZPCURST            BIT(7)                    /*!< TZPCU reset */
#define RCU_AHB1RST_TSIRST              BIT(8)                    /*!< TSI reset */
#define RCU_AHB1RST_CRCRST              BIT(12)                   /*!< CRC reset */ 
#define RCU_AHB1RST_WIFIRST             BIT(13)                   /*!< WIFI reset */ 
#define RCU_AHB1RST_DMA0RST             BIT(21)                   /*!< DMA0 reset */
#define RCU_AHB1RST_DMA1RST             BIT(22)                   /*!< DMA1 reset */
#define RCU_AHB1RST_USBFSRST            BIT(29)                   /*!< USBFS reset */

/* RCU_AHB2RST */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_AHB2RST_DCIRST              BIT(0)                    /*!< DCI reset */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_AHB2RST_PKCAURST            BIT(3)                    /*!< PKCAU reset */
#define RCU_AHB2RST_CAURST              BIT(4)                    /*!< CAU reset */
#define RCU_AHB2RST_HAURST              BIT(5)                    /*!< HAU reset */
#define RCU_AHB2RST_TRNGRST             BIT(6)                    /*!< TRNG reset */

                                    
/* RCU_AHB3RST */
#define RCU_AHB3RST_SQPIRST             BIT(0)                    /*!< SQPI reset */
#define RCU_AHB3RST_QSPIRST             BIT(1)                    /*!< QSPI reset */

/* RCU_APB1RST */
#define RCU_APB1RST_TIMER1RST           BIT(0)                    /*!< TIMER1 reset */
#define RCU_APB1RST_TIMER2RST           BIT(1)                    /*!< TIMER2 reset */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_APB1RST_TIMER3RST           BIT(2)                    /*!< TIMER3 reset */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_APB1RST_TIMER4RST           BIT(3)                    /*!< TIMER4 reset */
#define RCU_APB1RST_TIMER5RST           BIT(4)                    /*!< TIMER5 reset */
#define RCU_APB1RST_WWDGTRST            BIT(11)                   /*!< WWDGT reset */
#define RCU_APB1RST_SPI1RST             BIT(14)                   /*!< SPI1 reset */
#define RCU_APB1RST_USART1RST           BIT(17)                   /*!< USART1 reset */
#define RCU_APB1RST_USART0RST           BIT(18)                   /*!< USART0 reset */
#define RCU_APB1RST_I2C0RST             BIT(21)                   /*!< I2C0 reset */
#define RCU_APB1RST_I2C1RST             BIT(22)                   /*!< I2C1 reset */
#define RCU_APB1RST_PMURST              BIT(28)                   /*!< PMU reset */

/* RCU_APB2RST */
#define RCU_APB2RST_TIMER0RST           BIT(0)                    /*!< TIMER0 reset */
#define RCU_APB2RST_USART2RST           BIT(4)                    /*!< USART2 reset */
#define RCU_APB2RST_ADCRST              BIT(8)                    /*!< ADC reset */
#define RCU_APB2RST_SDIORST             BIT(11)                   /*!< SDIO reset */
#define RCU_APB2RST_SPI0RST             BIT(12)                   /*!< SPI0 reset */
#define RCU_APB2RST_SYSCFGRST           BIT(14)                   /*!< SYSCFG reset */
#define RCU_APB2RST_TIMER15RST          BIT(17)                   /*!< TIMER15 reset */
#define RCU_APB2RST_TIMER16RST          BIT(18)                   /*!< TIMER16 reset */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_APB2RST_HPDFRST             BIT(30)                   /*!< HPDF reset */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_APB2RST_RFRST               BIT(31)                   /*!< RF reset */

/* RCU_AHB1EN */
#define RCU_AHB1EN_PAEN                 BIT(0)                    /*!< GPIO port A clock enable */
#define RCU_AHB1EN_PBEN                 BIT(1)                    /*!< GPIO port B clock enable */
#define RCU_AHB1EN_PCEN                 BIT(2)                    /*!< GPIO port C clock enable */
#define RCU_AHB1EN_TZPCUEN              BIT(7)                    /*!< TZPCU clock enable */
#define RCU_AHB1EN_TSIEN                BIT(8)                    /*!< TSI clock enable */
#define RCU_AHB1EN_CRCEN                BIT(12)                   /*!< CRC clock enable */
#define RCU_AHB1EN_WIFIEN               BIT(13)                   /*!< WIFI module clock enable */
#define RCU_AHB1EN_WIFIRUNEN            BIT(14)                   /*!< WIFIRUNEN clock enable */
#define RCU_AHB1EN_SRAM0EN              BIT(16)                   /*!< SRAM0 clock enable */
#define RCU_AHB1EN_SRAM1EN              BIT(17)                   /*!< SRAM1 clock enable */
#define RCU_AHB1EN_SRAM2EN              BIT(18)                   /*!< SRAM2 clock enable */
#define RCU_AHB1EN_SRAM3EN              BIT(19)                   /*!< SRAM3 clock enable */
#define RCU_AHB1EN_DMA0EN               BIT(21)                   /*!< DMA0 clock enable */
#define RCU_AHB1EN_DMA1EN               BIT(22)                   /*!< DMA1 clock enable */
#define RCU_AHB1EN_USBFSEN              BIT(29)                   /*!< USBFS clock enable */

/* RCU_AHB2EN */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_AHB2EN_DCIEN                BIT(0)                    /*!< DCI clock enable */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_AHB2EN_PKCAUEN              BIT(3)                    /*!< PKCAU clock enable */
#define RCU_AHB2EN_CAUEN                BIT(4)                    /*!< CAU clock enable */
#define RCU_AHB2EN_HAUEN                BIT(5)                    /*!< HAU clock enable */
#define RCU_AHB2EN_TRNGEN               BIT(6)                    /*!< TRNG clock enable */

/* RCU_AHB3EN */
#define RCU_AHB3EN_SQPIEN               BIT(0)                    /*!< SQPI clock enable */
#define RCU_AHB3EN_QSPIEN               BIT(1)                    /*!< QSPI clock enable */

/* RCU_APB1EN */
#define RCU_APB1EN_TIMER1EN             BIT(0)                    /*!< TIMER1 clock enable */
#define RCU_APB1EN_TIMER2EN             BIT(1)                    /*!< TIMER2 clock enable */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_APB1EN_TIMER3EN             BIT(2)                    /*!< TIMER3 clock enable */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_APB1EN_TIMER4EN             BIT(3)                    /*!< TIMER4 clock enable */
#define RCU_APB1EN_TIMER5EN             BIT(4)                    /*!< TIMER5 clock enable */
#define RCU_APB1EN_WWDGTEN              BIT(11)                   /*!< WWDGT clock enable */
#define RCU_APB1EN_SPI1EN               BIT(14)                   /*!< SPI1 clock enable */
#define RCU_APB1EN_USART1EN             BIT(17)                   /*!< USART1 clock enable */
#define RCU_APB1EN_USART0EN             BIT(18)                   /*!< USART0 clock enable */
#define RCU_APB1EN_I2C0EN               BIT(21)                   /*!< I2C0 clock enable */
#define RCU_APB1EN_I2C1EN               BIT(22)                   /*!< I2C1 clock enable */
#define RCU_APB1EN_PMUEN                BIT(28)                   /*!< PMU clock enable */

/* RCU_APB2EN */
#define RCU_APB2EN_TIMER0EN             BIT(0)                    /*!< TIMER0 clock enable */
#define RCU_APB2EN_USART2EN             BIT(4)                    /*!< USART2 clock enable */
#define RCU_APB2EN_ADCEN                BIT(8)                    /*!< ADC clock enable */
#define RCU_APB2EN_SDIOEN               BIT(11)                   /*!< SDIO clock enable */
#define RCU_APB2EN_SPI0EN               BIT(12)                   /*!< SPI0 clock enable */
#define RCU_APB2EN_SYSCFGEN             BIT(14)                   /*!< SYSCFG clock enable */
#define RCU_APB2EN_TIMER15EN            BIT(17)                   /*!< TIMER15 clock enable */
#define RCU_APB2EN_TIMER16EN            BIT(18)                   /*!< TIMER16 clock enable */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_APB2EN_HPDFEN               BIT(30)                   /*!< HPDF clock enable */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_APB2EN_RFEN                 BIT(31)                   /*!< RF clock enable */

/* RCU_AHB1SPEN */
#define RCU_AHB1SPEN_PASPEN             BIT(0)                    /*!< GPIO port A clock enable when sleep mode */
#define RCU_AHB1SPEN_PBSPEN             BIT(1)                    /*!< GPIO port B clock enable when sleep mode */
#define RCU_AHB1SPEN_PCSPEN             BIT(2)                    /*!< GPIO port C clock enable when sleep mode */
#define RCU_AHB1SPEN_TZPCUSPEN          BIT(7)                    /*!< TZPCU clock enable when sleep mode */
#define RCU_AHB1SPEN_TSISPEN            BIT(8)                    /*!< TSI clock enable when sleep mode */
#define RCU_AHB1SPEN_CRCSPEN            BIT(12)                   /*!< CRC clock enable when sleep mode */
#define RCU_AHB1SPEN_WIFISPEN           BIT(13)                   /*!< WIFI clock enable when sleep mode */
#define RCU_AHB1SPEN_WIFIRUNSPEN        BIT(14)                   /*!< WIFIRUN clock enable when sleep mode */
#define RCU_AHB1SPEN_FMCSPEN            BIT(15)                   /*!< FMC clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM0SPEN          BIT(16)                   /*!< SRAM0 clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM1SPEN          BIT(17)                   /*!< SRAM1 clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM2SPEN          BIT(18)                   /*!< SRAM2 clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM3SPEN          BIT(19)                   /*!< SRAM3 clock enable when sleep mode */
#define RCU_AHB1SPEN_DMA0SPEN           BIT(21)                   /*!< DMA0 clock when sleep mode enable */
#define RCU_AHB1SPEN_DMA1SPEN           BIT(22)                   /*!< DMA1 clock when sleep mode enable */
#define RCU_AHB1SPEN_USBFSSPEN          BIT(29)                   /*!< USBFS clock enable when sleep mode */

/* RCU_AHB2SPEN */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_AHB2SPEN_DCISPEN            BIT(0)                    /*!< DCI clock enable when sleep mode */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_AHB2SPEN_PKCAUSPEN          BIT(3)                    /*!< PKCAU clock enable when sleep mode */
#define RCU_AHB2SPEN_CAUSPEN            BIT(4)                    /*!< CAU clock enable when sleep mode */
#define RCU_AHB2SPEN_HAUSPEN            BIT(5)                    /*!< HAU clock enable when sleep mode */
#define RCU_AHB2SPEN_TRNGSPEN           BIT(6)                    /*!< TRNG clock enable when sleep mode */

/* RCU_AHB3SPEN */
#define RCU_AHB3SPEN_SQPISPEN           BIT(0)                    /*!< SQPI clock enable when sleep mode */
#define RCU_AHB3SPEN_QSPISPEN           BIT(1)                    /*!< QSPI clock enable when sleep mode */

/* RCU_APB1SPEN */
#define RCU_APB1SPEN_TIMER1SPEN         BIT(0)                    /*!< TIMER1 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER2SPEN         BIT(1)                    /*!< TIMER2 clock enable when sleep mode */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_APB1SPEN_TIMER3SPEN         BIT(2)                    /*!< TIMER3 clock enable when sleep mode */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_APB1SPEN_TIMER4SPEN         BIT(3)                    /*!< TIMER4 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER5SPEN         BIT(4)                    /*!< TIMER5 clock enable when sleep mode */
#define RCU_APB1SPEN_WWDGTSPEN          BIT(11)                   /*!< WWDGT clock enable when sleep mode */
#define RCU_APB1SPEN_SPI1SPEN           BIT(14)                   /*!< SPI1 clock enable when sleep mode */
#define RCU_APB1SPEN_USART1SPEN         BIT(17)                   /*!< USART1 clock enable when sleep mode*/
#define RCU_APB1SPEN_USART0SPEN         BIT(18)                   /*!< USART0 clock enable when sleep mode*/
#define RCU_APB1SPEN_I2C0SPEN           BIT(21)                   /*!< I2C0 clock enable when sleep mode */
#define RCU_APB1SPEN_I2C1SPEN           BIT(22)                   /*!< I2C1 clock enable when sleep mode*/
#define RCU_APB1SPEN_PMUSPEN            BIT(28)                   /*!< PMU clock enable when sleep mode */

/* RCU_APB2SPEN */
#define RCU_APB2SPEN_TIMER0SPEN         BIT(0)                    /*!< TIMER0 clock enable when sleep mode */
#define RCU_APB2SPEN_USART2SPEN         BIT(4)                    /*!< USART2 clock enable when sleep mode */
#define RCU_APB2SPEN_ADCSPEN            BIT(8)                    /*!< ADC clock enable when sleep mode */
#define RCU_APB2SPEN_SDIOSPEN           BIT(11)                   /*!< SDIO clock enable when sleep mode */
#define RCU_APB2SPEN_SPI0SPEN           BIT(12)                   /*!< SPI0 clock enable when sleep mode */
#define RCU_APB2SPEN_SYSCFGSPEN         BIT(14)                   /*!< SYSCFG clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER15SPEN        BIT(17)                   /*!< TIMER15 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER16SPEN        BIT(18)                   /*!< TIMER16 clock enable when sleep mode */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_APB2SPEN_HPDFSPEN           BIT(30)                   /*!< HPDF clock enable when sleep mode */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_APB2SPEN_RFSPEN             BIT(31)                   /*!< RF clock enable when sleep mode */

/* RCU_BDCTL */
#define RCU_BDCTL_LXTALEN               BIT(0)                    /*!< LXTAL enable */
#define RCU_BDCTL_LXTALSTB              BIT(1)                    /*!< low speed crystal oscillator stabilization flag */
#define RCU_BDCTL_LXTALBPS              BIT(2)                    /*!< LXTAL bypass mode enable */
#define RCU_BDCTL_LXTALDRI              BITS(3,4)                 /*!< LXTAL drive capability */
#define RCU_BDCTL_RTCSRC                BITS(8,9)                 /*!< RTC clock entry selection */
#define RCU_BDCTL_RTCEN                 BIT(15)                   /*!< RTC clock enable */
#define RCU_BDCTL_BKPRST                BIT(16)                   /*!< backup domain reset */

/* RCU_RSTSCK */
#define RCU_RSTSCK_IRC32KEN             BIT(0)                    /*!< IRC32K enable */
#define RCU_RSTSCK_IRC32KSTB            BIT(1)                    /*!< IRC32K stabilization flag */
#define RCU_RSTSCK_RSTFC                BIT(24)                   /*!< reset flag clear */
#define RCU_RSTSCK_OBLRSTF              BIT(25)                   /*!< option byte loader reset flag */
#define RCU_RSTSCK_EPRSTF               BIT(26)                   /*!< external pin reset flag */
#define RCU_RSTSCK_PORRSTF              BIT(27)                   /*!< power reset flag */
#define RCU_RSTSCK_SWRSTF               BIT(28)                   /*!< software reset flag */
#define RCU_RSTSCK_FWDGTRSTF            BIT(29)                   /*!< free watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF            BIT(30)                   /*!< window watchdog timer reset flag */
#define RCU_RSTSCK_LPRSTF               BIT(31)                   /*!< low-power reset flag */

/* RCU_PLLSSCTL */
#define RCU_PLLSSCTL_MODCNT             BITS(0,12)                /*!< these bits configure PLL spread spectrum modulation 
                                                                       profile amplitude and frequency. the following criteria 
                                                                       must be met: MODSTEP*MODCNT=215-1 */
#define RCU_PLLSSCTL_MODSTEP            BITS(13,27)               /*!< these bits configure PLL spread spectrum modulation 
                                                                       profile amplitude and frequency. the following criteria 
                                                                       must be met: MODSTEP*MODCNT=215-1 */
#define RCU_PLLSSCTL_SS_TYPE            BIT(30)                   /*!< PLL spread spectrum modulation type select */
#define RCU_PLLSSCTL_SSCGON             BIT(31)                   /*!< PLL spread spectrum modulation enable */

/* RCU_PLLCFG */
#define RCU_PLLCFG_PLLI2SDIV            BITS(0,5)                 /*!< PLLI2S clock divider factor */
#define RCU_PLLCFG_PLLI2SN              BITS(8,14)                /*!< the PLLI2S VCO clock multiplication factor */
#define RCU_PLLCFG_PLLI2SPSC            BITS(16,18)               /*!< the PLLI2S VCO source clock pre-scale */
#define RCU_PLLCFG_PLLDIGOSEL           BITS(24,25)               /*!< PLLDIG output frequency select */
#define RCU_PLLCFG_PLLDIGFSYSDIV        BITS(26,31)               /*!< PLLDIG clock divider factor for system clock */

/* RCU_CFG1 */
#define RCU_CFG1_IRC16MDIV              BITS(0,8)                 /*!< IRC16M clock divider factor for system clock */
#define RCU_CFG1_BGVBIT                 BITS(9,11)                /*!< bandGap Power adjust */
#define RCU_CFG1_RFPLLCALEN             BIT(14)                   /*!< RF PLL Calculation enable */
#define RCU_CFG1_RFPLLLOCK              BIT(15)                   /*!< RF PLL LOCK */
#define RCU_CFG1_RFPLLPU                BIT(16)                   /*!< RFPLL power on enable */
#define RCU_CFG1_LDOANAPU               BIT(17)                   /*!< LDO analog power on enable for RF filter */
#define RCU_CFG1_LDOCLKPU               BIT(18)                   /*!< LDO clock power on enable for RF/ADC/DAC */
#define RCU_CFG1_BGPU                   BIT(19)                   /*!< bandGap power on enable */
#define RCU_CFG1_LDO_CLK_LQB            BIT(20)                   /*!< trust zone security by RF */
#define RCU_CFG1_LDO_ANA_LQB            BIT(21)                   /*!< trust zone security by RF */
#define RCU_CFG1_TIMERSEL               BIT(24)                   /*!< TIMER clock selection */
#define RCU_CFG1_I2C0SEL                BITS(26,27)               /*!< I2C0 Clock Source Selection */
#define RCU_CFG1_USART2SEL              BITS(28,29)               /*!< USART2 Clock Source Selection */
#define RCU_CFG1_USART0SEL              BITS(30,31)               /*!< USART0 Clock Source Selection */

/* RCU_ADDCTL */
#define RCU_ADDCTL_USBFSSEL             BIT(0)                    /*!< USBFS clock selection */
#define RCU_ADDCTL_USBFSDIV             BITS(1,5)                 /*!< USBFS clock divider factor */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_ADDCTL_HPDFSEL              BIT(11)                   /*!< HPDF clock source selection */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_ADDCTL_I2SSEL               BITS(12,13)               /*!< I2S clock source selection */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_ADDCTL_HPDFAUDIOSEL         BITS(14,15)               /*!< HPDF AUDIO clock source selection */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_ADDCTL_SDIOSEL_0            BIT(16)                   /*!< SDIO clock selection */
#define RCU_ADDCTL_SDIODIV              BITS(17,21)               /*!< SDIO clock divider factor */
#define RCU_ADDCTL_SDIOSEL_1            BIT(22)                   /*!< SDIO clock selection */
#define RCU_ADDCTL_PLLFI2SDIV           BITS(24,29)               /*!< the PLL divider factor for I2S clock */

/* RCU_SECP_CFG */
#define RCU_SECP_CFG_IRC16MSECP            BIT(0)                 /*!< IRC16M clock configuration and status bits security */
#define RCU_SECP_CFG_HXTALSECP             BIT(1)                 /*!< HXTAL clock configuration and status bits security */
#define RCU_SECP_CFG_IRC32KSECP            BIT(2)                 /*!< IRC32K clock configuration and status bits security */
#define RCU_SECP_CFG_LXTALSECP             BIT(3)                 /*!< LXTAL clock configuration and status bits security */
#define RCU_SECP_CFG_SYSCLKSECP            BIT(4)                 /*!< SYSCLK clock selection, STOPWUCK bit, clock output on MCO configuration */
#define RCU_SECP_CFG_PRESCSECP             BIT(5)                 /*!< AHBx/APBx prescaler configuration bits security */
#define RCU_SECP_CFG_PLLSECP               BIT(6)                 /*!< main PLL configuration and status bits security */
#define RCU_SECP_CFG_PLLDIGSECP            BIT(7)                 /*!< PLLDIG configuration and status bits security */
#define RCU_SECP_CFG_PLLI2SSECP            BIT(8)                 /*!< PLLI2S configuration and status bits security */
#define RCU_SECP_CFG_RMVRSTFSECP           BIT(9)                 /*!< remove reset flag security */
#define RCU_SECP_CFG_BKPSECP               BIT(10)                /*!< BKP security */

/* RCU_SECP_STAT */
#define RCU_SECP_STAT_IRC16MSECF          BIT(0)                  /*!< IRC16M clock configuration and status bits security flag */
#define RCU_SECP_STAT_HXTALSECF           BIT(1)                  /*!< HXTAL clock configuration and status bits security flag */
#define RCU_SECP_STAT_IRC32KSECF          BIT(2)                  /*!< IRC32K clock configuration and status bits security flag */
#define RCU_SECP_STAT_LXTALSECF           BIT(3)                  /*!< LXTAL clock configuration and status bits security flag */
#define RCU_SECP_STAT_SYSCLKSECF          BIT(4)                  /*!< SYSCLK clock selection, STOPWUCK bit, clock output on MCO configuration */
#define RCU_SECP_STAT_PRESCSECF           BIT(5)                  /*!< AHBx/APBx prescaler configuration bits security flag */
#define RCU_SECP_STAT_PLLSECF             BIT(6)                  /*!< main PLL configuration and status bits security flag */
#define RCU_SECP_STAT_PLLDIGSECF          BIT(7)                  /*!< PLLDIG configuration and status bits security flag */
#define RCU_SECP_STAT_PLLI2SSECF          BIT(8)                  /*!< PLLI2S configuration and status bits security flag */
#define RCU_SECP_STAT_RMVRSTFSECF         BIT(9)                  /*!< remove reset flag security flag */
#define RCU_SECP_STAT_BKPSECF             BIT(10)                 /*!< BKP security flag */

/* RCU_AHB1SECP_STAT */
#define RCU_AHB1SECP_STAT_PASECF          BIT(0)                  /*!< GPIO port A security flag */
#define RCU_AHB1SECP_STAT_PBSECF          BIT(1)                  /*!< GPIO port B security flag */
#define RCU_AHB1SECP_STAT_PCSECF          BIT(2)                  /*!< GPIO port C security flag */
#define RCU_AHB1SECP_STAT_CRCSECF         BIT(12)                 /*!< CRC security flag */
#define RCU_AHB1SECP_STAT_WIFISECF        BIT(13)                 /*!< WIFI security flag */
#define RCU_AHB1SECP_STAT_FLITFSECF       BIT(15)                 /*!< FLITF security flag */
#define RCU_AHB1SECP_STAT_SRAM0SECF       BIT(16)                 /*!< SRAM0 security flag */
#define RCU_AHB1SECP_STAT_SRAM1SECF       BIT(17)                 /*!< SRAM1 security flag */
#define RCU_AHB1SECP_STAT_SRAM2SECF       BIT(18)                 /*!< SRAM2 security flag */
#define RCU_AHB1SECP_STAT_SRAM3SECF       BIT(19)                 /*!< SRAM3 security flag */
#define RCU_AHB1SECP_STAT_DMA0SECF        BIT(21)                 /*!< DMA0 security flag */
#define RCU_AHB1SECP_STAT_DMA1SECF        BIT(22)                 /*!< DMA1 security flag */
#define RCU_AHB1SECP_STAT_USBFSSECF       BIT(29)                 /*!< USBFS security flag */

/* RCU_AHB2SECP_STAT */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_AHB2SECP_STAT_DCISECF         BIT(0)                  /*!< DCI security flag */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_AHB2SECP_STAT_PKCAUSECF       BIT(3)                  /*!< PKCAU security flag */
#define RCU_AHB2SECP_STAT_CAUSECF         BIT(4)                  /*!< CAU security flag */
#define RCU_AHB2SECP_STAT_HAUSECF         BIT(5)                  /*!< HAU security flag */
#define RCU_AHB2SECP_STAT_TRNGSECF        BIT(6)                  /*!< TRNG security flag */

/* RCU_AHB3SECP_STAT */
#define RCU_AHB3SECP_STAT_SQPISECF        BIT(0)                  /*!< SQPI security flag */
#define RCU_AHB3SECP_STAT_QSPISECF        BIT(1)                  /*!< QSPI security flag */

/* RCU_APB1SECP_STAT */
#define RCU_APB1SECP_STAT_TIMER1SECF      BIT(0)                  /*!< TIMER1 security flag */
#define RCU_APB1SECP_STAT_TIMER2SECF      BIT(1)                  /*!< TIMER2 security flag */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_APB1SECP_STAT_TIMER3SECF      BIT(2)                  /*!< TIMER3 security flag */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_APB1SECP_STAT_TIMER4SECF      BIT(3)                  /*!< TIMER4 security flag */
#define RCU_APB1SECP_STAT_TIMER5SECF      BIT(4)                  /*!< TIMER5 security flag */
#define RCU_APB1SECP_STAT_WWDGTSECF       BIT(11)                 /*!< WWDGT security flag */
#define RCU_APB1SECP_STAT_SPI1SECF        BIT(14)                 /*!< SPI1 security flag */
#define RCU_APB1SECP_STAT_USART1SECF      BIT(17)                 /*!< USART1 security flag */
#define RCU_APB1SECP_STAT_USART2SECF      BIT(18)                 /*!< USART2 security flag */
#define RCU_APB1SECP_STAT_I2C0SECF        BIT(21)                 /*!< I2C0 security flag */
#define RCU_APB1SECP_STAT_I2C1SECF        BIT(22)                 /*!< I2C1 security flag */
#define RCU_APB1SECP_STAT_PMUSECF         BIT(28)                 /*!< PMU security flag */

/* RCU_APB2SECP_STAT */
#define RCU_APB2SECP_STAT_TIMER0SECF      BIT(0)                  /*!< TIMER0 security flag */
#define RCU_APB2SECP_STAT_USART0SECF      BIT(4)                  /*!< USART0 security flag */
#define RCU_APB2SECP_STAT_ADCSECF         BIT(8)                  /*!< ADC security flag */
#define RCU_APB2SECP_STAT_SDIOSECF        BIT(11)                 /*!< SDIO security flag */
#define RCU_APB2SECP_STAT_SPI0SECF        BIT(12)                 /*!< SPI0 security flag */
#define RCU_APB2SECP_STAT_SYSCFGSECF      BIT(14)                 /*!< SYSCFG security flag */
#define RCU_APB2SECP_STAT_TIMER15SECF     BIT(17)                 /*!< TIMER15 security flag */
#define RCU_APB2SECP_STAT_TIMER16SECF     BIT(18)                 /*!< TIMER16 security flag */
#if (defined(GD32W515PI) || defined(GD32W515P0))
#define RCU_APB2SECP_STAT_HPDFSECF        BIT(30)                 /*!< HPDF security flag */
#endif /* GD32W515PI and GD32W515P0 */
#define RCU_APB2SECP_STAT_RFSECF          BIT(31)                 /*!< RF security flag */

/* RCU_VKEY */
#define RCU_VKEY_KEY                    BITS(0,31)                /*!< RCU_DSV key register */

/* RCU_DSV */
#define RCU_DSV_DSLPVS                  BITS(0,1)                 /*!< deep-sleep mode voltage select */

/* constants definitions */
/* define the peripheral clock enable bit position and its register index offset */
#define RCU_REGIDX_BIT(regidx, bitpos)      (((uint32_t)(regidx) << 6U) | (uint32_t)(bitpos))
#define RCU_REG_VAL(periph)                 (REG32(RCU + ((uint32_t)(periph) >> 6U)))
#define RCU_BIT_POS(val)                    ((uint32_t)(val) & 0x1FU)
/* define the voltage key unlock value */
#define RCU_VKEY_UNLOCK                     ((uint32_t)0x1A2B3C4DU)

/* register offset */
/* peripherals enable */
#define AHB1EN_REG_OFFSET               0x30U                     /*!< AHB1 enable register offset */
#define AHB2EN_REG_OFFSET               0x34U                     /*!< AHB2 enable register offset */
#define AHB3EN_REG_OFFSET               0x38U                     /*!< AHB3 enable register offset */
#define APB1EN_REG_OFFSET               0x40U                     /*!< APB1 enable register offset */
#define APB2EN_REG_OFFSET               0x44U                     /*!< APB2 enable register offset */
#define AHB1SPEN_REG_OFFSET             0x50U                     /*!< AHB1 sleep mode enable register offset */
#define AHB2SPEN_REG_OFFSET             0x54U                     /*!< AHB2 sleep mode enable register offset */
#define AHB3SPEN_REG_OFFSET             0x58U                     /*!< AHB3 sleep mode enable register offset */
#define APB1SPEN_REG_OFFSET             0x60U                     /*!< APB1 sleep mode enable register offset */
#define APB2SPEN_REG_OFFSET             0x64U                     /*!< APB2 sleep mode enable register offset */

/* peripherals reset */
#define AHB1RST_REG_OFFSET              0x10U                     /*!< AHB1 reset register offset */
#define AHB2RST_REG_OFFSET              0x14U                     /*!< AHB2 reset register offset */
#define AHB3RST_REG_OFFSET              0x18U                     /*!< AHB3 reset register offset */
#define APB1RST_REG_OFFSET              0x20U                     /*!< APB1 reset register offset */
#define APB2RST_REG_OFFSET              0x24U                     /*!< APB2 reset register offset */
#define RSTSCK_REG_OFFSET               0x74U                     /*!< reset source/clock register offset */

/* clock control */
#define CTL_REG_OFFSET                  0x00U                     /*!< control register offset */
#define BDCTL_REG_OFFSET                0x70U                     /*!< backup domain control register offset */
#define ADDCTL_REG_OFFSET               0x90U                     /*!< additional clock control register offset */

/* clock stabilization and stuck interrupt */
#define INT_REG_OFFSET                  0x0CU                     /*!< clock interrupt register offset */

/* configuration register */
#define PLL_REG_OFFSET                  0x04U                     /*!< PLL register offset */
#define CFG0_REG_OFFSET                 0x08U                     /*!< clock configuration register 0 offset */
#define PLLSSCTL_REG_OFFSET             0x80U                     /*!< PLL clock spread spectrum control register offset */
#define PLLCFG_REG_OFFSET               0x84U                     /*!< PLLI2S register offset */
#define CFG1_REG_OFFSET                 0x8CU                     /*!< clock configuration register 1 offset */

/* secure configuration and status register */
#define SECPCFG_REG_OFFSET              0xC0U                     /*!< secure protection configuration register offset */
#define SECPSTAT_REG_OFFSET             0xC4U                     /*!< secure protection status register offset */
#define AHB1SECPSTAT_REG_OFFSET         0xC8U                     /*!< AHB1 secure protection status register offset */
#define AHB2SECPSTAT_REG_OFFSET         0xCCU                     /*!< AHB2 secure protection status register offset */
#define AHB3SECPSTAT_REG_OFFSET         0xD0U                     /*!< AHB3 secure protection status register offset */
#define APB1SECPSTAT_REG_OFFSET         0xD4U                     /*!< APB1 secure protection status register offset */
#define APB2SECPSTAT_REG_OFFSET         0xD8U                     /*!< APB2 secure protection status register offset */

/* peripherals reset */
typedef enum
{
    /* AHB1 peripherals */
    RCU_GPIOARST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 0U),              /*!< GPIOA clock reset */
    RCU_GPIOBRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 1U),              /*!< GPIOB clock reset */
    RCU_GPIOCRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 2U),              /*!< GPIOC clock reset */
    RCU_TZPCURST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 7U),              /*!< TZPCU clock reset */
    RCU_TSIRST       = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 8U),              /*!< TSI clock reset */
    RCU_CRCRST       = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 12U),             /*!< CRC clock reset */
    RCU_WIFIRST      = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 13U),             /*!< WIFI clock reset */
    RCU_DMA0RST      = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 21U),             /*!< DMA0 clock reset */
    RCU_DMA1RST      = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 22U),             /*!< DMA1 clock reset */ 
    RCU_USBFSRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 29U),             /*!< USBFS clock reset */
    /* AHB2 peripherals */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_DCIRST       = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 0U),              /*!< DCI clock reset */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_PKCAURST     = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 3U),              /*!< PKCAU clock reset */
    RCU_CAURST       = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 4U),              /*!< CAU clock reset */
    RCU_HAURST       = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 5U),              /*!< HAU clock reset */
    RCU_TRNGRST      = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 6U),              /*!< TRNG clock reset */
    /* AHB3 peripherals */
    RCU_SQPIRST      = RCU_REGIDX_BIT(AHB3RST_REG_OFFSET, 0U),              /*!< SQPI clock reset */
    RCU_QSPIRST      = RCU_REGIDX_BIT(AHB3RST_REG_OFFSET, 1U),              /*!< SQPI clock reset */
    /* APB1 peripherals */
    RCU_TIMER1RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 0U),              /*!< TIMER1 clock reset */
    RCU_TIMER2RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 1U),              /*!< TIMER2 clock reset */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_TIMER3RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 2U),              /*!< TIMER3 clock reset */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_TIMER4RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 3U),              /*!< TIMER4 clock reset */
    RCU_TIMER5RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 4U),              /*!< TIMER5 clock reset */
    RCU_WWDGTRST     = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 11U),             /*!< WWDGT clock reset */
    RCU_SPI1RST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 14U),             /*!< SPI1 clock reset */
    RCU_USART1RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 17U),             /*!< USART1 clock reset */
    RCU_USART0RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 18U),             /*!< USART0 clock reset */
    RCU_I2C0RST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 21U),             /*!< I2C0 clock reset */
    RCU_I2C1RST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 22U),             /*!< I2C1 clock reset */
    RCU_PMURST       = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 28U),             /*!< PMU clock reset */
    /* APB2 peripherals */
    RCU_TIMER0RST    = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 0U),              /*!< TIMER0 clock reset */
    RCU_USART2RST    = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 4U),              /*!< USART2 clock reset */
    RCU_ADCRST       = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 8U),              /*!< ADC clock reset */
    RCU_SDIORST      = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 11U),             /*!< SDIO clock reset */
    RCU_SPI0RST      = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 12U),             /*!< SPI0 clock reset */
    RCU_SYSCFGRST    = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 14U),             /*!< SYSCFG clock reset */
    RCU_TIMER15RST   = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 17U),             /*!< TIMER15 clock reset */
    RCU_TIMER16RST   = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 18U),             /*!< TIMER16 clock reset */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_HPDFRST      = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 30U),             /*!< HPDF clock reset */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_RFRST        = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 31U)              /*!< RF clock reset */
}rcu_periph_reset_enum;

/* peripheral clock enable */
typedef enum
{
    /* AHB1 peripherals */
    RCU_GPIOA     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 0U),                  /*!< GPIOA clock */
    RCU_GPIOB     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 1U),                  /*!< GPIOB clock */
    RCU_GPIOC     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 2U),                  /*!< GPIOC clock */
    RCU_TZPCU     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 7U),                  /*!< TZPCU clock */
    RCU_TSI       = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 8U),                  /*!< TSI clock */
    RCU_CRC       = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 12U),                 /*!< CRC clock */
    RCU_WIFI      = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 13U),                 /*!< WIFI clock */
    RCU_WIFIRUN   = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 14U),                 /*!< WIFIRUN clock */
    RCU_SRAM0     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 16U),                 /*!< SRAM0 clock */
    RCU_SRAM1     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 17U),                 /*!< SRAM1 clock */
    RCU_SRAM2     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 18U),                 /*!< SRAM2 clock */
    RCU_SRAM3     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 19U),                 /*!< SRAM3 clock */	
    RCU_DMA0      = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 21U),                 /*!< DMA0 clock */
    RCU_DMA1      = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 22U),                 /*!< DMA1 clock */
    RCU_USBFS     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 29U),                 /*!< USBFS clock */
    /* AHB2 peripherals */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_DCI       = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 0U),                  /*!< DCI clock */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_PKCAU     = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 3U),                  /*!< PKCAU clock */
    RCU_CAU       = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 4U),                  /*!< CAU clock */
    RCU_HAU       = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 5U),                  /*!< HAU clock */
    RCU_TRNG      = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 6U),                  /*!< TRNG clock */
    /* AHB3 peripherals */
    RCU_SQPI      = RCU_REGIDX_BIT(AHB3EN_REG_OFFSET, 0U),                  /*!< SQPI clock */
    RCU_QSPI      = RCU_REGIDX_BIT(AHB3EN_REG_OFFSET, 1U),                  /*!< QSPI clock */
    /* APB1 peripherals */
    RCU_TIMER1    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 0U),                  /*!< TIMER1 clock */
    RCU_TIMER2    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 1U),                  /*!< TIMER2 clock */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_TIMER3    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 2U),                  /*!< TIMER3 clock */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_TIMER4    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 3U),                  /*!< TIMER4 clock */
    RCU_TIMER5    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 4U),                  /*!< TIMER5 clock */
    RCU_WWDGT     = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 11U),                 /*!< WWDGT clock */
    RCU_SPI1      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 14U),                 /*!< SPI1 clock */
    RCU_USART1    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 17U),                 /*!< USART1 clock */
    RCU_USART0    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 18U),                 /*!< USART0 clock */
    RCU_I2C0      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 21U),                 /*!< I2C0 clock */
    RCU_I2C1      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 22U),                 /*!< I2C1 clock */
    RCU_PMU       = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 28U),                 /*!< PMU clock */
    RCU_RTC       = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 15U),                  /*!< RTC clock */
    /* APB2 peripherals */
    RCU_TIMER0    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 0U),                  /*!< TIMER0 clock */
    RCU_USART2    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 4U),                  /*!< USART0 clock */
    RCU_ADC       = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 8U),                  /*!< ADC clock */
    RCU_SDIO      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 11U),                 /*!< SDIO clock */
    RCU_SPI0      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 12U),                 /*!< SPI0 clock */
    RCU_SYSCFG    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 14U),                 /*!< SYSCFG clock */
    RCU_TIMER15   = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 17U),                 /*!< TIMER15 clock */
    RCU_TIMER16   = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 18U),                 /*!< TIMER16 clock */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_HPDF      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 30U),                 /*!< HPDF clock */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_RF        = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 31U)                  /*!< RF clock */
}rcu_periph_enum;

/* peripheral clock enable when sleep mode*/
typedef enum
{
    /* AHB1 peripherals */
    RCU_GPIOA_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 0U),            /*!< GPIOA clock when sleep mode */
    RCU_GPIOB_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 1U),            /*!< GPIOB clock when sleep mode */
    RCU_GPIOC_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 2U),            /*!< GPIOC clock when sleep mode */
    RCU_TZPCU_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 7U),            /*!< TZPCU clock when sleep mode */
    RCU_TSI_SLP       = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 8U),            /*!< TSI clock when sleep mode */
    RCU_CRC_SLP       = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 12U),           /*!< CRC clock when sleep mode */
    RCU_WIFI_SLP      = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 13U),           /*!< WIFI clock when sleep mode */
    RCU_WIFIRUN_SLP   = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 14U),           /*!< WIFIRUN clock when sleep mode */
    RCU_FMC_SLP       = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 15U),           /*!< FMC clock when sleep mode*/
    RCU_SRAM0_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 16U),           /*!< SRAM0 clock when sleep mode */
    RCU_SRAM1_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 17U),           /*!< SRAM1 clock when sleep mode */
    RCU_SRAM2_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 18U),           /*!< SRAM2 clock when sleep mode*/
    RCU_SRAM3_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 19U),           /*!< SRAM3 clock when sleep mode */
    RCU_DMA0_SLP      = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 21U),           /*!< DMA0 clock when sleep mode */
    RCU_DMA1_SLP      = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 22U),           /*!< DMA1 clock when sleep mode */
    RCU_USBFS_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 29U),           /*!< USBFS clock when sleep mode */
    /* AHB2 peripherals */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_DCI_SLP       = RCU_REGIDX_BIT(AHB2SPEN_REG_OFFSET, 0U),            /*!< DCI clock when sleep mode */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_PKCAU_SLP     = RCU_REGIDX_BIT(AHB2SPEN_REG_OFFSET, 3U),            /*!< PKCAU clock when sleep mode*/
    RCU_CAU_SLP       = RCU_REGIDX_BIT(AHB2SPEN_REG_OFFSET, 4U),            /*!< CAU clock when sleep mode */
    RCU_HAU_SLP       = RCU_REGIDX_BIT(AHB2SPEN_REG_OFFSET, 5U),            /*!< HAU clock when sleep mode */
    RCU_TRNG_SLP      = RCU_REGIDX_BIT(AHB2SPEN_REG_OFFSET, 6U),            /*!< TRNG clock when sleep mode */
    /* AHB3 peripherals */
    RCU_SQPI_SLP      = RCU_REGIDX_BIT(AHB3SPEN_REG_OFFSET, 0U),            /*!< SQPI clock when sleep mode */
    RCU_QSPI_SLP      = RCU_REGIDX_BIT(AHB3SPEN_REG_OFFSET, 1U),            /*!< QSPI clock when sleep mode */
    /* APB1 peripherals */
    RCU_TIMER1_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 0U),            /*!< TIMER1 clock when sleep mode */
    RCU_TIMER2_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 1U),            /*!< TIMER2 clock when sleep mode */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_TIMER3_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 2U),            /*!< TIMER3 clock when sleep mode */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_TIMER4_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 3U),            /*!< TIMER4 clock when sleep mode*/
    RCU_TIMER5_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 4U),            /*!< TIMER5 clock when sleep mode */ 
    RCU_WWDGT_SLP     = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 11U),           /*!< WWDGT clock when sleep mode */
    RCU_SPI1_SLP      = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 14U),           /*!< SPI1 clock when sleep mode */
    RCU_USART1_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 17U),           /*!< USART1 clock when sleep mode */
    RCU_USART0_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 18U),           /*!< USART2 clock when sleep mode*/
    RCU_I2C0_SLP      = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 21U),           /*!< I2C0 clock when sleep mode */
    RCU_I2C1_SLP      = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 22U),           /*!< I2C1 clock when sleep mode */
    RCU_PMU_SLP       = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 28U),           /*!< PMU clock when sleep mode */
    /* APB2 peripherals */
    RCU_TIMER0_SLP    = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 0U),            /*!< TIMER0 clock when sleep mode */
    RCU_USART2_SLP    = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 4U),            /*!< USART0 clock when sleep mode */
    RCU_ADC_SLP       = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 8U),            /*!< ADC clock when sleep mode */
    RCU_SDIO_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 11U),           /*!< SDIO clock when sleep mode */
    RCU_SPI0_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 12U),           /*!< SPI0 clock when sleep mode */
    RCU_SYSCFG_SLP    = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 14U),           /*!< SYSCFG clock when sleep mode */
    RCU_TIMER15_SLP   = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 17U),           /*!< TIMER15 clock when sleep mode*/
    RCU_TIMER16_SLP   = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 18U),           /*!< TIMER16 clock when sleep mode */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_HPDF_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 30U),           /*!< HPDF clock when sleep mode */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_RF_SLP        = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 31U)            /*!< RF clock when sleep mode */
}rcu_periph_sleep_enum;

/* clock stabilization and peripheral reset flags */
typedef enum
{
    /* clock stabilization flags */
    RCU_FLAG_IRC16MSTB     = RCU_REGIDX_BIT(CTL_REG_OFFSET, 1U),            /*!< IRC16M stabilization flags */
    RCU_FLAG_HXTALSTB      = RCU_REGIDX_BIT(CTL_REG_OFFSET, 17U),           /*!< HXTAL stabilization flags */
    RCU_FLAG_PLLDIGSTB     = RCU_REGIDX_BIT(CTL_REG_OFFSET, 23U),           /*!< PLLDIG stabilization flags */
    RCU_FLAG_PLLSTB        = RCU_REGIDX_BIT(CTL_REG_OFFSET, 25U),           /*!< PLL stabilization flags */
    RCU_FLAG_PLLI2SSTB     = RCU_REGIDX_BIT(CTL_REG_OFFSET, 27U),           /*!< PLLI2S stabilization flags */
    RCU_FLAG_LXTALSTB      = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 1U),          /*!< LXTAL stabilization flags */
    RCU_FLAG_IRC32KSTB     = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 1U),         /*!< IRC32K stabilization flags */
    /* reset source flags */
    RCU_FLAG_OBLRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 25U),        /*!< option byte loader reset flag */
    RCU_FLAG_EPRST         = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 26U),        /*!< external PIN reset flags */
    RCU_FLAG_PORRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 27U),        /*!< power reset flags */
    RCU_FLAG_SWRST         = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 28U),        /*!< software reset flags */
    RCU_FLAG_FWDGTRST      = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 29U),        /*!< FWDGT reset flags */
    RCU_FLAG_WWDGTRST      = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 30U),        /*!< WWDGT reset flags */
    RCU_FLAG_LPRST         = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 31U)         /*!< low-power reset flags */
}rcu_flag_enum;

/* clock stabilization and ckm interrupt flags */
typedef enum
{
    RCU_INT_FLAG_IRC32KSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 0U),            /*!< IRC32K stabilization interrupt flag */
    RCU_INT_FLAG_LXTALSTB  = RCU_REGIDX_BIT(INT_REG_OFFSET, 1U),            /*!< LXTAL stabilization interrupt flag */
    RCU_INT_FLAG_IRC16MSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 2U),            /*!< IRC16M stabilization interrupt flag */
    RCU_INT_FLAG_HXTALSTB  = RCU_REGIDX_BIT(INT_REG_OFFSET, 3U),            /*!< HXTAL stabilization interrupt flag */
    RCU_INT_FLAG_PLLSTB    = RCU_REGIDX_BIT(INT_REG_OFFSET, 4U),            /*!< PLL stabilization interrupt flag */
    RCU_INT_FLAG_PLLI2SSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 5U),            /*!< PLLI2S stabilization interrupt flag */
    RCU_INT_FLAG_PLLDIGSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 6U),            /*!< PLLDIG stabilization interrupt flag */
    RCU_INT_FLAG_CKM       = RCU_REGIDX_BIT(INT_REG_OFFSET, 7U)             /*!< HXTAL clock stuck interrupt flag */
}rcu_int_flag_enum;

/* clock stabilization and stuck interrupt flags clear */
typedef enum
{
    RCU_INT_FLAG_IRC32KSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 16U),       /*!< IRC32K stabilization interrupt flags clear */
    RCU_INT_FLAG_LXTALSTB_CLR  = RCU_REGIDX_BIT(INT_REG_OFFSET, 17U),       /*!< LXTAL stabilization interrupt flags clear */
    RCU_INT_FLAG_IRC16MSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 18U),       /*!< IRC16M stabilization interrupt flags clear */
    RCU_INT_FLAG_HXTALSTB_CLR  = RCU_REGIDX_BIT(INT_REG_OFFSET, 19U),       /*!< HXTAL stabilization interrupt flags clear */
    RCU_INT_FLAG_PLLSTB_CLR    = RCU_REGIDX_BIT(INT_REG_OFFSET, 20U),       /*!< PLL stabilization interrupt flags clear */
    RCU_INT_FLAG_PLLI2SSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 21U),       /*!< PLLI2S stabilization interrupt flags clear */
    RCU_INT_FLAG_PLLDIGSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 22U),       /*!< PLLDIG stabilization interrupt flags clear */
    RCU_INT_FLAG_CKM_CLR       = RCU_REGIDX_BIT(INT_REG_OFFSET, 23U)        /*!< CKM interrupt flags clear */
}rcu_int_flag_clear_enum;

/* clock stabilization interrupt enable or disable */
typedef enum
{
    RCU_INT_IRC32KSTB       = RCU_REGIDX_BIT(INT_REG_OFFSET, 8U),           /*!< IRC32K stabilization interrupt */
    RCU_INT_LXTALSTB        = RCU_REGIDX_BIT(INT_REG_OFFSET, 9U),           /*!< LXTAL stabilization interrupt */
    RCU_INT_IRC16MSTB       = RCU_REGIDX_BIT(INT_REG_OFFSET, 10U),          /*!< IRC16M stabilization interrupt */
    RCU_INT_HXTALSTB        = RCU_REGIDX_BIT(INT_REG_OFFSET, 11U),          /*!< HXTAL stabilization interrupt */
    RCU_INT_PLLSTB          = RCU_REGIDX_BIT(INT_REG_OFFSET, 12U),          /*!< PLL stabilization interrupt */
    RCU_INT_PLLI2SSTB       = RCU_REGIDX_BIT(INT_REG_OFFSET, 13U),          /*!< PLLI2S stabilization interrupt */
    RCU_INT_PLLDIGSTB       = RCU_REGIDX_BIT(INT_REG_OFFSET, 14U)           /*!< PLLDIG stabilization interrupt */
}rcu_int_enum;

/* oscillator types */
typedef enum
{
    RCU_HXTAL      = RCU_REGIDX_BIT(CTL_REG_OFFSET, 16U),                   /*!< HXTAL */
    RCU_LXTAL      = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 0U),                  /*!< LXTAL */
    RCU_IRC16M     = RCU_REGIDX_BIT(CTL_REG_OFFSET, 0U),                    /*!< IRC16M */
    RCU_IRC32K     = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 0U),                 /*!< IRC32K */
    RCU_PLLDIG_CK  = RCU_REGIDX_BIT(CTL_REG_OFFSET, 21U),                   /*!< PLLDIG */
    RCU_PLL_CK     = RCU_REGIDX_BIT(CTL_REG_OFFSET, 24U),                   /*!< PLL */
    RCU_PLLI2S_CK  = RCU_REGIDX_BIT(CTL_REG_OFFSET, 26U)                    /*!< PLLI2S */
}rcu_osci_type_enum;

/* rcu clock frequency */
typedef enum
{
    CK_SYS      = 0,                                                        /*!< system clock */
    CK_AHB,                                                                 /*!< AHB clock */
    CK_APB1,                                                                /*!< APB1 clock */
    CK_APB2,                                                                /*!< APB2 clock */
    CK_USART0,                                                              /*!< USART0 clock */
    CK_USART2,                                                              /*!< USART2 clock */
    CK_I2C0                                                                 /*!< I2C0 clock */
}rcu_clock_freq_enum;

/* clock security attribution enable or disable */
typedef enum
{
    RCU_SEC_IRC16MSEC       = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 0U),        /*!< IRC16M clock configuration and status bits security */
    RCU_SEC_HXTALSEC        = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 1U),        /*!< HXTAL clock configuration and status bits security */
    RCU_SEC_IRC32KSEC       = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 2U),        /*!< IRC32K clock configuration and status bits security */
    RCU_SEC_LXTALSEC        = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 3U),        /*!< LXTAL clock configuration and status bits security */
    RCU_SEC_SYSCLKSEC       = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 4U),        /*!< SYSCLK clock selection, STOPWUCK bit, clock output on MCO configuration */
    RCU_SEC_PRESCSEC        = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 5U),        /*!< AHBx/APBx prescaler configuration bits security */
    RCU_SEC_PLLSEC          = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 6U),        /*!< main PLL configuration and status bits security */
    RCU_SEC_PLLDIGSEC       = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 7U),        /*!< PLLDIG configuration and status bits security */
    RCU_SEC_PLLI2SSEC       = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 8U),        /*!< PLLI2S configuration and status bits security */
    RCU_SEC_RMVRSTFSEC      = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 9U),        /*!< remove reset flag security */
    RCU_SEC_BKPSEC          = RCU_REGIDX_BIT(SECPCFG_REG_OFFSET, 10U)        /*!< BKP security */
}rcu_sec_enum;

/* clock security status flag */
typedef enum
{
    RCU_SEC_FLAG_IRC16MSEC       = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 0U),  /*!< IRC16M clock configuration and status bits security */
    RCU_SEC_FLAG_HXTALSEC        = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 1U),  /*!< HXTAL clock configuration and status bits security */
    RCU_SEC_FLAG_IRC32KSEC       = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 2U),  /*!< IRC32K clock configuration and status bits security */
    RCU_SEC_FLAG_LXTALSEC        = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 3U),  /*!< LXTAL clock configuration and status bits security */
    RCU_SEC_FLAG_SYSCLKSEC       = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 4U),  /*!< SYSCLK clock selection, STOPWUCK bit, clock output on MCO configuration */
    RCU_SEC_FLAG_PRESCSEC        = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 5U),  /*!< AHBx/APBx prescaler configuration bits security */
    RCU_SEC_FLAG_PLLSEC          = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 6U),  /*!< main PLL configuration and status bits security */
    RCU_SEC_FLAG_PLLDIGSEC       = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 7U),  /*!< PLLDIG configuration and status bits security */
    RCU_SEC_FLAG_PLLI2SSEC       = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 8U),  /*!< PLLI2S configuration and status bits security */
    RCU_SEC_FLAG_RMVRSTFSEC      = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 9U),  /*!< remove reset flag security */
    RCU_SEC_FLAG_BKPSEC          = RCU_REGIDX_BIT(SECPSTAT_REG_OFFSET, 10U), /*!< BKP security */
    /* AHB1 peripherals */
    RCU_SEC_FLAG_GPIOA     = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 0U),                  /*!< GPIOA clock */
    RCU_SEC_FLAG_GPIOB     = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 1U),                  /*!< GPIOB clock */
    RCU_SEC_FLAG_GPIOC     = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 2U),                  /*!< GPIOC clock */
    RCU_SEC_FLAG_CRC       = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 12U),                 /*!< CRC clock */
    RCU_SEC_FLAG_WIFI      = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 13U),                 /*!< WIFI clock */
    RCU_SEC_FLAG_FMC       = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 15U),                 /*!< FMC clock */
    RCU_SEC_FLAG_SRAM0     = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 16U),                 /*!< SRAM0 clock */
    RCU_SEC_FLAG_SRAM1     = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 17U),                 /*!< SRAM1 clock */
    RCU_SEC_FLAG_SRAM2     = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 18U),                 /*!< SRAM2 clock */
    RCU_SEC_FLAG_SRAM3     = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 19U),                 /*!< SRAM3 clock */
    RCU_SEC_FLAG_DMA0      = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 21U),                 /*!< DMA0 clock */
    RCU_SEC_FLAG_DMA1      = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 22U),                 /*!< DMA1 clock */
    RCU_SEC_FLAG_USBFS     = RCU_REGIDX_BIT(AHB1SECPSTAT_REG_OFFSET, 29U),                 /*!< USBFS clock */
    /* AHB2 peripherals */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_SEC_FLAG_DCI       = RCU_REGIDX_BIT(AHB2SECPSTAT_REG_OFFSET, 0U),                  /*!< DCI clock */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_SEC_FLAG_PKCAU       = RCU_REGIDX_BIT(AHB2SECPSTAT_REG_OFFSET, 3U),                /*!< PKCAU clock */
    RCU_SEC_FLAG_CAU      = RCU_REGIDX_BIT(AHB2SECPSTAT_REG_OFFSET, 4U),                   /*!< CAU clock */
    RCU_SEC_FLAG_HAU      = RCU_REGIDX_BIT(AHB2SECPSTAT_REG_OFFSET, 5U),                   /*!< HAU clock */
    RCU_SEC_FLAG_TRNG      = RCU_REGIDX_BIT(AHB2SECPSTAT_REG_OFFSET, 6U),                  /*!< TRNG clock */
    /* AHB3 peripherals */
    RCU_SEC_FLAG_SQPI      = RCU_REGIDX_BIT(AHB3SECPSTAT_REG_OFFSET, 0U),                  /*!< SQPI clock */
    RCU_SEC_FLAG_QSPI      = RCU_REGIDX_BIT(AHB3SECPSTAT_REG_OFFSET, 1U),                  /*!< QSPI clock */
    /* APB1 peripherals */
    RCU_SEC_FLAG_TIMER1    = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 0U),                  /*!< TIMER1 clock */
    RCU_SEC_FLAG_TIMER2    = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 1U),                  /*!< TIMER2 clock */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_SEC_FLAG_TIMER3    = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 2U),                  /*!< TIMER3 clock */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_SEC_FLAG_TIMER4    = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 3U),                  /*!< TIMER4 clock */
    RCU_SEC_FLAG_TIMER5    = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 4U),                  /*!< TIMER5 clock */
    RCU_SEC_FLAG_WWDGT     = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 11U),                 /*!< WWDGT clock */
    RCU_SEC_FLAG_SPI1      = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 14U),                 /*!< SPI1 clock */
    RCU_SEC_FLAG_USART1    = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 17U),                 /*!< USART1 clock */
    RCU_SEC_FLAG_USART0    = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 18U),                 /*!< USART0 clock */
    RCU_SEC_FLAG_I2C0      = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 21U),                 /*!< I2C0 clock */
    RCU_SEC_FLAG_I2C1      = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 22U),                 /*!< I2C1 clock */
    RCU_SEC_FLAG_PMU       = RCU_REGIDX_BIT(APB1SECPSTAT_REG_OFFSET, 28U),                 /*!< PMU clock */
    /* APB2 peripherals */
    RCU_SEC_FLAG_TIMER0    = RCU_REGIDX_BIT(APB2SECPSTAT_REG_OFFSET, 0U),                  /*!< TIMER0 clock */
    RCU_SEC_FLAG_USART2    = RCU_REGIDX_BIT(APB2SECPSTAT_REG_OFFSET, 4U),                  /*!< USART2 clock */
    RCU_SEC_FLAG_ADC       = RCU_REGIDX_BIT(APB2SECPSTAT_REG_OFFSET, 8U),                  /*!< ADC clock */
    RCU_SEC_FLAG_SDIO      = RCU_REGIDX_BIT(APB2SECPSTAT_REG_OFFSET, 11U),                 /*!< SDIO clock */
    RCU_SEC_FLAG_SPI0      = RCU_REGIDX_BIT(APB2SECPSTAT_REG_OFFSET, 12U),                 /*!< SPI0 clock */
    RCU_SEC_FLAG_SYSCFG    = RCU_REGIDX_BIT(APB2SECPSTAT_REG_OFFSET, 14U),                 /*!< SYSCFG clock */
    RCU_SEC_FLAG_TIMER15   = RCU_REGIDX_BIT(APB2SECPSTAT_REG_OFFSET, 17U),                 /*!< TIMER15 clock */
    RCU_SEC_FLAG_TIMER16   = RCU_REGIDX_BIT(APB2SECPSTAT_REG_OFFSET, 18U),                 /*!< TIMER16 clock */
    #if (defined(GD32W515PI) || defined(GD32W515P0))
    RCU_SEC_FLAG_HPDF      = RCU_REGIDX_BIT(APB2SECPSTAT_REG_OFFSET, 30U),                 /*!< HPDF clock */
    #endif /* GD32W515PI and GD32W515P0 */
    RCU_SEC_FLAG_RF        = RCU_REGIDX_BIT(APB2SECPSTAT_REG_OFFSET, 31U)                  /*!< RF clock */
}rcu_sec_flag_enum;

/* RCU_CFG0 register bit define */
/* system clock source select */
#define CFG0_SCS(regval)                (BITS(0,1) & ((uint32_t)(regval) << 0U))
#define RCU_CKSYSSRC_IRC16M             CFG0_SCS(0)                        /*!< system clock source select IRC16M */
#define RCU_CKSYSSRC_HXTAL              CFG0_SCS(1)                        /*!< system clock source select HXTAL */
#define RCU_CKSYSSRC_PLLP               CFG0_SCS(2)                        /*!< system clock source select PLLP */
#define RCU_CKSYSSRC_PLLDIG             CFG0_SCS(3)                        /*!< system clock source select PLLDIG */

/* system clock source select status */
#define CFG0_SCSS(regval)               (BITS(2,3) & ((uint32_t)(regval) << 2U))
#define RCU_SCSS_IRC16M                 CFG0_SCSS(0)                       /*!< system clock source select IRC16M */
#define RCU_SCSS_HXTAL                  CFG0_SCSS(1)                       /*!< system clock source select HXTAL */
#define RCU_SCSS_PLLP                   CFG0_SCSS(2)                       /*!< system clock source select PLLP */
#define RCU_SCSS_PLLDIG                 CFG0_SCSS(3)                       /*!< system clock source select PLLDIG */

/* AHB prescaler selection */
#define CFG0_AHBPSC(regval)             (BITS(4,7) & ((uint32_t)(regval) << 4U))
#define RCU_AHB_CKSYS_DIV1              CFG0_AHBPSC(0)                     /*!< AHB prescaler select CK_SYS */
#define RCU_AHB_CKSYS_DIV2              CFG0_AHBPSC(8)                     /*!< AHB prescaler select CK_SYS/2 */
#define RCU_AHB_CKSYS_DIV4              CFG0_AHBPSC(9)                     /*!< AHB prescaler select CK_SYS/4 */
#define RCU_AHB_CKSYS_DIV8              CFG0_AHBPSC(10)                    /*!< AHB prescaler select CK_SYS/8 */
#define RCU_AHB_CKSYS_DIV16             CFG0_AHBPSC(11)                    /*!< AHB prescaler select CK_SYS/16 */
#define RCU_AHB_CKSYS_DIV64             CFG0_AHBPSC(12)                    /*!< AHB prescaler select CK_SYS/64 */
#define RCU_AHB_CKSYS_DIV128            CFG0_AHBPSC(13)                    /*!< AHB prescaler select CK_SYS/128 */
#define RCU_AHB_CKSYS_DIV256            CFG0_AHBPSC(14)                    /*!< AHB prescaler select CK_SYS/256 */
#define RCU_AHB_CKSYS_DIV512            CFG0_AHBPSC(15)                    /*!< AHB prescaler select CK_SYS/512 */

/* APB1 prescaler selection */
#define CFG0_APB1PSC(regval)            (BITS(10,12) & ((uint32_t)(regval) << 10U))
#define RCU_APB1_CKAHB_DIV4             CFG0_APB1PSC(5)                    /*!< APB1 prescaler select CK_AHB/4 */
#define RCU_APB1_CKAHB_DIV8             CFG0_APB1PSC(6)                    /*!< APB1 prescaler select CK_AHB/8 */
#define RCU_APB1_CKAHB_DIV16            CFG0_APB1PSC(7)                    /*!< APB1 prescaler select CK_AHB/16 */

/* APB2 prescaler selection */
#define CFG0_APB2PSC(regval)            (BITS(13,15) & ((uint32_t)(regval) << 13U))
#define RCU_APB2_CKAHB_DIV1             CFG0_APB2PSC(0)                    /*!< APB2 prescaler select CK_AHB/2 */
#define RCU_APB2_CKAHB_DIV2             CFG0_APB2PSC(4)                    /*!< APB2 prescaler select CK_AHB/2 */
#define RCU_APB2_CKAHB_DIV4             CFG0_APB2PSC(5)                    /*!< APB2 prescaler select CK_AHB/4 */
#define RCU_APB2_CKAHB_DIV8             CFG0_APB2PSC(6)                    /*!< APB2 prescaler select CK_AHB/8 */
#define RCU_APB2_CKAHB_DIV16            CFG0_APB2PSC(7)                    /*!< APB2 prescaler select CK_AHB/16 */

/* RTC clock divider factor from HXTAL clock */
#define CFG0_RTCDIV(regval)             (BITS(16,20) & ((uint32_t)(regval) << 16U))
#define RCU_RTC_HXTAL_DIV1              CFG0_RTCDIV(0)                     /*!< RTCDIV clock select CK_HXTAL/1 */
#define RCU_RTC_HXTAL_DIV2              CFG0_RTCDIV(1)                     /*!< RTCDIV clock select CK_HXTAL/2 */
#define RCU_RTC_HXTAL_DIV3              CFG0_RTCDIV(2)                     /*!< RTCDIV clock select CK_HXTAL/3 */
#define RCU_RTC_HXTAL_DIV4              CFG0_RTCDIV(3)                     /*!< RTCDIV clock select CK_HXTAL/4 */
#define RCU_RTC_HXTAL_DIV5              CFG0_RTCDIV(4)                     /*!< RTCDIV clock select CK_HXTAL/5 */
#define RCU_RTC_HXTAL_DIV6              CFG0_RTCDIV(5)                     /*!< RTCDIV clock select CK_HXTAL/6 */
#define RCU_RTC_HXTAL_DIV7              CFG0_RTCDIV(6)                     /*!< RTCDIV clock select CK_HXTAL/7 */
#define RCU_RTC_HXTAL_DIV8              CFG0_RTCDIV(7)                     /*!< RTCDIV clock select CK_HXTAL/8 */
#define RCU_RTC_HXTAL_DIV9              CFG0_RTCDIV(8)                     /*!< RTCDIV clock select CK_HXTAL/9 */
#define RCU_RTC_HXTAL_DIV10             CFG0_RTCDIV(9)                     /*!< RTCDIV clock select CK_HXTAL/10 */
#define RCU_RTC_HXTAL_DIV11             CFG0_RTCDIV(10)                    /*!< RTCDIV clock select CK_HXTAL/11 */
#define RCU_RTC_HXTAL_DIV12             CFG0_RTCDIV(11)                    /*!< RTCDIV clock select CK_HXTAL/12 */
#define RCU_RTC_HXTAL_DIV13             CFG0_RTCDIV(12)                    /*!< RTCDIV clock select CK_HXTAL/13 */
#define RCU_RTC_HXTAL_DIV14             CFG0_RTCDIV(13)                    /*!< RTCDIV clock select CK_HXTAL/14 */
#define RCU_RTC_HXTAL_DIV15             CFG0_RTCDIV(14)                    /*!< RTCDIV clock select CK_HXTAL/15 */
#define RCU_RTC_HXTAL_DIV16             CFG0_RTCDIV(15)                    /*!< RTCDIV clock select CK_HXTAL/16 */
#define RCU_RTC_HXTAL_DIV17             CFG0_RTCDIV(16)                    /*!< RTCDIV clock select CK_HXTAL/17 */
#define RCU_RTC_HXTAL_DIV18             CFG0_RTCDIV(17)                    /*!< RTCDIV clock select CK_HXTAL/18 */
#define RCU_RTC_HXTAL_DIV19             CFG0_RTCDIV(18)                    /*!< RTCDIV clock select CK_HXTAL/19 */
#define RCU_RTC_HXTAL_DIV20             CFG0_RTCDIV(19)                    /*!< RTCDIV clock select CK_HXTAL/20 */
#define RCU_RTC_HXTAL_DIV21             CFG0_RTCDIV(20)                    /*!< RTCDIV clock select CK_HXTAL/21 */
#define RCU_RTC_HXTAL_DIV22             CFG0_RTCDIV(21)                    /*!< RTCDIV clock select CK_HXTAL/22 */
#define RCU_RTC_HXTAL_DIV23             CFG0_RTCDIV(22)                    /*!< RTCDIV clock select CK_HXTAL/23 */
#define RCU_RTC_HXTAL_DIV24             CFG0_RTCDIV(23)                    /*!< RTCDIV clock select CK_HXTAL/24 */
#define RCU_RTC_HXTAL_DIV25             CFG0_RTCDIV(24)                    /*!< RTCDIV clock select CK_HXTAL/25 */
#define RCU_RTC_HXTAL_DIV26             CFG0_RTCDIV(25)                    /*!< RTCDIV clock select CK_HXTAL/26 */
#define RCU_RTC_HXTAL_DIV27             CFG0_RTCDIV(26)                    /*!< RTCDIV clock select CK_HXTAL/27 */
#define RCU_RTC_HXTAL_DIV28             CFG0_RTCDIV(27)                    /*!< RTCDIV clock select CK_HXTAL/28 */
#define RCU_RTC_HXTAL_DIV29             CFG0_RTCDIV(28)                    /*!< RTCDIV clock select CK_HXTAL/29 */
#define RCU_RTC_HXTAL_DIV30             CFG0_RTCDIV(29)                    /*!< RTCDIV clock select CK_HXTAL/30 */
#define RCU_RTC_HXTAL_DIV31             CFG0_RTCDIV(30)                    /*!< RTCDIV clock select CK_HXTAL/31 */
#define RCU_RTC_HXTAL_DIV32             CFG0_RTCDIV(31)                    /*!< RTCDIV clock select CK_HXTAL/32 */

/* CKOUT0 Clock source selection */
#define CFG0_CKOUT0SEL(regval)          (BITS(21,22) & ((uint32_t)(regval) << 21U))
#define RCU_CKOUT0SRC_IRC16M            CFG0_CKOUT0SEL(0)                  /*!< internal 16M RC oscillator clock selected */
#define RCU_CKOUT0SRC_LXTAL             CFG0_CKOUT0SEL(1)                  /*!< low speed crystal oscillator clock (LXTAL) selected */
#define RCU_CKOUT0SRC_HXTAL             CFG0_CKOUT0SEL(2)                  /*!< high speed crystal oscillator clock (HXTAL) selected */
#define RCU_CKOUT0SRC_PLLP              CFG0_CKOUT0SEL(3)                  /*!< CK_PLLP clock selected */

/* The CK_OUT0 divider */
#define CFG0_CKOUT0DIV(regval)          (BITS(24,26) & ((uint32_t)(regval) << 24U))
#define RCU_CKOUT0_DIV1                 CFG0_CKOUT0DIV(0)                  /*!< CK_OUT0 is divided by 1 */
#define RCU_CKOUT0_DIV2                 CFG0_CKOUT0DIV(4)                  /*!< CK_OUT0 is divided by 2 */
#define RCU_CKOUT0_DIV3                 CFG0_CKOUT0DIV(5)                  /*!< CK_OUT0 is divided by 3 */
#define RCU_CKOUT0_DIV4                 CFG0_CKOUT0DIV(6)                  /*!< CK_OUT0 is divided by 4 */
#define RCU_CKOUT0_DIV5                 CFG0_CKOUT0DIV(7)                  /*!< CK_OUT0 is divided by 5 */

/* CKOUT1 Clock source selection */
#define CFG0_CKOUT1SEL(regval)          (BITS(30,31) & ((uint32_t)(regval) << 30U))
#define RCU_CKOUT1SRC_CKSYS             CFG0_CKOUT1SEL(0)                  /*!< system clock selected */
#define RCU_CKOUT1SRC_PLLI2S            CFG0_CKOUT1SEL(1)                  /*!< CK_PLLI2S clock selected */
#define RCU_CKOUT1SRC_HXTAL             CFG0_CKOUT1SEL(2)                  /*!< high speed crystal oscillator clock (HXTAL) selected */
#define RCU_CKOUT1SRC_PLLDIG            CFG0_CKOUT1SEL(3)                  /*!< CK_PLLDIG clock selected */

/* The CK_OUT1 divider */
#define CFG0_CKOUT1DIV(regval)          (BITS(27,29) & ((uint32_t)(regval) << 27U))
#define RCU_CKOUT1_DIV1                 CFG0_CKOUT1DIV(0)                  /*!< CK_OUT1 is divided by 1 */
#define RCU_CKOUT1_DIV2                 CFG0_CKOUT1DIV(4)                  /*!< CK_OUT1 is divided by 2 */
#define RCU_CKOUT1_DIV3                 CFG0_CKOUT1DIV(5)                  /*!< CK_OUT1 is divided by 3 */
#define RCU_CKOUT1_DIV4                 CFG0_CKOUT1DIV(6)                  /*!< CK_OUT1 is divided by 4 */
#define RCU_CKOUT1_DIV5                 CFG0_CKOUT1DIV(7)                  /*!< CK_OUT1 is divided by 5 */

/* I2C0 Clock source selection */
#define CFG1_I2C0SEL(regval)            (BITS(26,27) & ((uint32_t)(regval) << 26U))
#define RCU_I2C0SRC_CKAPB1              CFG1_I2C0SEL(0)                    /*!< CKAPB1 clock selected as I2C0 source clock */
#define RCU_I2C0SRC_CKSYS               CFG1_I2C0SEL(1)                    /*!< System clock selected as I2C0 source clock */
#define RCU_I2C0SRC_IRC16M              CFG1_I2C0SEL(2)                    /*!< CK_IRC16M clock selected as I2C0 source clock */

/* USART2 Clock source selection */
#define CFG1_USART2SEL(regval)            (BITS(28,29) & ((uint32_t)(regval) << 28U))
#define RCU_USART2SRC_CKAPB2              CFG1_USART2SEL(0)                 /*!< CKAPB2 clock selected as USART2 source clock */
#define RCU_USART2SRC_CKSYS               CFG1_USART2SEL(1)                /*!< SYSCLK selected as USART2 source clock */
#define RCU_USART2SRC_LXTAL               CFG1_USART2SEL(2)                /*!< CK_LXTAL selected as USART2 source clock */
#define RCU_USART2SRC_IRC16M              CFG1_USART2SEL(3)                /*!< CK_IRC16M selected as USART2 source clock */

/* USART0 Clock source selection */
#define CFG1_USART0SEL(regval)            (BITS(30,31) & ((uint32_t)(regval) << 30U))
#define RCU_USART0SRC_CKAPB1              CFG1_USART0SEL(0)                /*!< CKAPB1 clock selected as USART0 source clock */
#define RCU_USART0SRC_CKSYS               CFG1_USART0SEL(1)                /*!< SYSCLK selected as USART0 source clock */
#define RCU_USART0SRC_LXTAL               CFG1_USART0SEL(2)                /*!< CK_LXTAL selected as USART0 source clock */
#define RCU_USART0SRC_IRC16M              CFG1_USART0SEL(3)                /*!< CK_IRC16M selected as USART0 source clock */

/* USBFS Clock source selection */
#define RCU_USBFSSRC_PLL               ((uint32_t)0x00000000U)             /*!< PLL output clock selected as USBFS source clock */
#define RCU_USBFSSRC_PLLDIG             RCU_ADDCTL_USBFSSEL                /*!< PLLDIG selected as USBFS source clock */

/* RCU_ADDCTL register bit define */
/* the divider factor from USBFS clock */
#define ADDCTL_USBFSDIV(regval)        (BITS(1,5) & ((uint32_t)(regval) << 1U))
#define RCU_USBFS_DIV1                  ADDCTL_USBFSDIV(0)                          /*!< USBFSDIV input source clock divided by 1 */
#define RCU_USBFS_DIV2                  ADDCTL_USBFSDIV(1)                          /*!< USBFSDIV input source clock divided by 2 */
#define RCU_USBFS_DIV3                  ADDCTL_USBFSDIV(2)                          /*!< USBFSDIV input source clock divided by 3 */
#define RCU_USBFS_DIV4                  ADDCTL_USBFSDIV(3)                          /*!< USBFSDIV input source clock divided by 4 */
#define RCU_USBFS_DIV5                  ADDCTL_USBFSDIV(4)                          /*!< USBFSDIV input source clock divided by 5 */
#define RCU_USBFS_DIV6                  ADDCTL_USBFSDIV(5)                          /*!< USBFSDIV input source clock divided by 6 */
#define RCU_USBFS_DIV7                  ADDCTL_USBFSDIV(6)                          /*!< USBFSDIV input source clock divided by 7 */
#define RCU_USBFS_DIV8                  ADDCTL_USBFSDIV(7)                          /*!< USBFSDIV input source clock divided by 8 */
#define RCU_USBFS_DIV9                  ADDCTL_USBFSDIV(8)                          /*!< USBFSDIV input source clock divided by 9 */
#define RCU_USBFS_DIV10                 ADDCTL_USBFSDIV(9)                          /*!< USBFSDIV input source clock divided by 10 */
#define RCU_USBFS_DIV11                 ADDCTL_USBFSDIV(10)                         /*!< USBFSDIV input source clock divided by 11 */
#define RCU_USBFS_DIV12                 ADDCTL_USBFSDIV(11)                         /*!< USBFSDIV input source clock divided by 12 */
#define RCU_USBFS_DIV13                 ADDCTL_USBFSDIV(12)                         /*!< USBFSDIV input source clock divided by 13 */
#define RCU_USBFS_DIV14                 ADDCTL_USBFSDIV(13)                         /*!< USBFSDIV input source clock divided by 14 */
#define RCU_USBFS_DIV15                 ADDCTL_USBFSDIV(14)                         /*!< USBFSDIV input source clock divided by 15 */
#define RCU_USBFS_DIV16                 ADDCTL_USBFSDIV(15)                         /*!< USBFSDIV input source clock divided by 16 */
#define RCU_USBFS_DIV17                 ADDCTL_USBFSDIV(16)                         /*!< USBFSDIV input source clock divided by 17 */
#define RCU_USBFS_DIV18                 ADDCTL_USBFSDIV(17)                         /*!< USBFSDIV input source clock divided by 18 */
#define RCU_USBFS_DIV19                 ADDCTL_USBFSDIV(18)                         /*!< USBFSDIV input source clock divided by 19 */
#define RCU_USBFS_DIV20                 ADDCTL_USBFSDIV(19)                         /*!< USBFSDIV input source clock divided by 20 */
#define RCU_USBFS_DIV21                 ADDCTL_USBFSDIV(20)                         /*!< USBFSDIV input source clock divided by 21 */
#define RCU_USBFS_DIV22                 ADDCTL_USBFSDIV(21)                         /*!< USBFSDIV input source clock divided by 22 */
#define RCU_USBFS_DIV23                 ADDCTL_USBFSDIV(22)                         /*!< USBFSDIV input source clock divided by 23 */
#define RCU_USBFS_DIV24                 ADDCTL_USBFSDIV(23)                         /*!< USBFSDIV input source clock divided by 24 */
#define RCU_USBFS_DIV25                 ADDCTL_USBFSDIV(24)                         /*!< USBFSDIV input source clock divided by 25 */
#define RCU_USBFS_DIV26                 ADDCTL_USBFSDIV(25)                         /*!< USBFSDIV input source clock divided by 26 */
#define RCU_USBFS_DIV27                 ADDCTL_USBFSDIV(26)                         /*!< USBFSDIV input source clock divided by 27 */
#define RCU_USBFS_DIV28                 ADDCTL_USBFSDIV(27)                         /*!< USBFSDIV input source clock divided by 28 */
#define RCU_USBFS_DIV29                 ADDCTL_USBFSDIV(28)                         /*!< USBFSDIV input source clock divided by 29 */
#define RCU_USBFS_DIV30                 ADDCTL_USBFSDIV(29)                         /*!< USBFSDIV input source clock divided by 30 */
#define RCU_USBFS_DIV31                 ADDCTL_USBFSDIV(30)                         /*!< USBFSDIV input source clock divided by 31 */
#define RCU_USBFS_DIV32                 ADDCTL_USBFSDIV(31)                         /*!< USBFSDIV input source clock divided by 32 */

#if (defined(GD32W515PI) || defined(GD32W515P0))
/* HPDF Clock source selection */
#define RCU_HPDFSRC_PCLK2               ((uint32_t)0x00000000U)                       /*!< PCLK2 clock selected as HPDF source clock */
#define RCU_HPDFSRC_CKSYS               RCU_ADDCTL_HPDFSEL                            /*!< system Clock selected as HPDF source clock */
#endif /* GD32W515PI and GD32W515P0 */

/* I2S Clock source selection */
#define ADDCTL_I2SSEL(regval)           (BITS(12,13) & ((uint32_t)(regval) << 12U))
#define RCU_I2SSRC_PLLI2S               ADDCTL_I2SSEL(0)                              /*!< PLLI2S output clock selected as I2S source clock */
#define RCU_I2SSRC_I2S_CKIN             ADDCTL_I2SSEL(1)                              /*!< external I2S_CKIN pin selected as I2S source clock */
#define RCU_I2SSRC_I2S_PLLDIV           ADDCTL_I2SSEL(2)                              /*!< PLL division selected as I2S source clock */

#if (defined(GD32W515PI) || defined(GD32W515P0))
/* HPDFAUDIO Clock source selection */
#define ADDCTL_HPDFAUDIOSEL(regval)     (BITS(14,15) & ((uint32_t)(regval) << 14U))
#define RCU_HPDFAUDIOSRC_PLLI2S          ADDCTL_HPDFAUDIOSEL(0)                       /*!< PLLI2S output clock selected as I2S source clock */
#define RCU_HPDFAUDIOSRC_I2S_CKIN        ADDCTL_HPDFAUDIOSEL(1)                       /*!< external I2S_CKIN pin selected as I2S source clock */
#define RCU_HPDFAUDIOSRC_PLL             ADDCTL_HPDFAUDIOSEL(2)                       /*!< PLL selected as I2S source clock */
#define RCU_HPDFAUDIOSRC_IRC16M          ADDCTL_HPDFAUDIOSEL(3)                       /*!< IRC16M selected as I2S source clock */
#endif /* GD32W515PI and GD32W515P0 */

/* SDIO Clock source selection */
#define RCU_SDIOSRC_MASK                 0X00410000U    
#define RCU_SDIOSRC_PLL                  0X00000000U                                  /*!< select PLL clock */
#define RCU_SDIOSRC_PLLDIG               RCU_ADDCTL_SDIOSEL_0                         /*!< select PLLDIG clock */
#define RCU_SDIOSRC_IRC16M               RCU_ADDCTL_SDIOSEL_1                         /*!< select IRC16M */
#define RCU_SDIOSRC_HXTAL                RCU_ADDCTL_SDIOSEL_0 | RCU_ADDCTL_SDIOSEL_1  /*!< select HXTAL */

/* SDIODIV division factor */
#define ADDCTL_SDIODIV(regval)            (BITS(17,21) & ((uint32_t)(regval) << 17U))
#define RCU_SDIODIV_DIV1                  ADDCTL_SDIODIV(0)                         /*!< SDIODIV input source clock divided by 1 */
#define RCU_SDIODIV_DIV2                  ADDCTL_SDIODIV(1)                         /*!< SDIODIV input source clock divided by 2 */
#define RCU_SDIODIV_DIV3                  ADDCTL_SDIODIV(2)                         /*!< SDIODIV input source clock divided by 3 */
#define RCU_SDIODIV_DIV4                  ADDCTL_SDIODIV(3)                         /*!< SDIODIV input source clock divided by 4 */
#define RCU_SDIODIV_DIV5                  ADDCTL_SDIODIV(4)                         /*!< SDIODIV input source clock divided by 5 */
#define RCU_SDIODIV_DIV6                  ADDCTL_SDIODIV(5)                         /*!< SDIODIV input source clock divided by 6 */
#define RCU_SDIODIV_DIV7                  ADDCTL_SDIODIV(6)                         /*!< SDIODIV input source clock divided by 7 */
#define RCU_SDIODIV_DIV8                  ADDCTL_SDIODIV(7)                         /*!< SDIODIV input source clock divided by 8 */
#define RCU_SDIODIV_DIV9                  ADDCTL_SDIODIV(8)                         /*!< SDIODIV input source clock divided by 9 */
#define RCU_SDIODIV_DIV10                 ADDCTL_SDIODIV(9)                         /*!< SDIODIV input source clock divided by 10 */
#define RCU_SDIODIV_DIV11                 ADDCTL_SDIODIV(10)                        /*!< SDIODIV input source clock divided by 11 */
#define RCU_SDIODIV_DIV12                 ADDCTL_SDIODIV(11)                        /*!< SDIODIV input source clock divided by 12 */
#define RCU_SDIODIV_DIV13                 ADDCTL_SDIODIV(12)                        /*!< SDIODIV input source clock divided by 13 */
#define RCU_SDIODIV_DIV14                 ADDCTL_SDIODIV(13)                        /*!< SDIODIV input source clock divided by 14 */
#define RCU_SDIODIV_DIV15                 ADDCTL_SDIODIV(14)                        /*!< SDIODIV input source clock divided by 15 */
#define RCU_SDIODIV_DIV16                 ADDCTL_SDIODIV(15)                        /*!< SDIODIV input source clock divided by 16 */
#define RCU_SDIODIV_DIV17                 ADDCTL_SDIODIV(16)                        /*!< SDIODIV input source clock divided by 17 */
#define RCU_SDIODIV_DIV18                 ADDCTL_SDIODIV(17)                        /*!< SDIODIV input source clock divided by 18 */
#define RCU_SDIODIV_DIV19                 ADDCTL_SDIODIV(18)                        /*!< SDIODIV input source clock divided by 19 */
#define RCU_SDIODIV_DIV20                 ADDCTL_SDIODIV(19)                        /*!< SDIODIV input source clock divided by 20 */
#define RCU_SDIODIV_DIV21                 ADDCTL_SDIODIV(20)                        /*!< SDIODIV input source clock divided by 21 */
#define RCU_SDIODIV_DIV22                 ADDCTL_SDIODIV(21)                        /*!< SDIODIV input source clock divided by 22 */
#define RCU_SDIODIV_DIV23                 ADDCTL_SDIODIV(22)                        /*!< SDIODIV input source clock divided by 23 */
#define RCU_SDIODIV_DIV24                 ADDCTL_SDIODIV(23)                        /*!< SDIODIV input source clock divided by 24 */
#define RCU_SDIODIV_DIV25                 ADDCTL_SDIODIV(24)                        /*!< SDIODIV input source clock divided by 25 */
#define RCU_SDIODIV_DIV26                 ADDCTL_SDIODIV(25)                        /*!< SDIODIV input source clock divided by 26 */
#define RCU_SDIODIV_DIV27                 ADDCTL_SDIODIV(26)                        /*!< SDIODIV input source clock divided by 27 */
#define RCU_SDIODIV_DIV28                 ADDCTL_SDIODIV(27)                        /*!< SDIODIV input source clock divided by 28 */
#define RCU_SDIODIV_DIV29                 ADDCTL_SDIODIV(28)                        /*!< SDIODIV input source clock divided by 29 */
#define RCU_SDIODIV_DIV30                 ADDCTL_SDIODIV(29)                        /*!< SDIODIV input source clock divided by 30 */
#define RCU_SDIODIV_DIV31                 ADDCTL_SDIODIV(30)                        /*!< SDIODIV input source clock divided by 31 */
#define RCU_SDIODIV_DIV32                 ADDCTL_SDIODIV(31)                        /*!< SDIODIV input source clock divided by 32 */

/* RCU_ADDCTL register bit define */
/* PLLFI2SDIV division factor */
#define ADDCTL_PLLFI2SDIV(regval)         (BITS(24,29) & ((uint32_t)(regval) << 24U))
#define RCU_PLLFI2SDIV_DIV1                ADDCTL_PLLFI2SDIV(0)                     /*!< PLL clock divided by 1 for I2S clock */
#define RCU_PLLFI2SDIV_DIV2                ADDCTL_PLLFI2SDIV(1)                     /*!< PLL clock divided by 2 for I2S clock */
#define RCU_PLLFI2SDIV_DIV3                ADDCTL_PLLFI2SDIV(2)                     /*!< PLL clock divided by 3 for I2S clock */
#define RCU_PLLFI2SDIV_DIV4                ADDCTL_PLLFI2SDIV(3)                     /*!< PLL clock divided by 4 for I2S clock */
#define RCU_PLLFI2SDIV_DIV5                ADDCTL_PLLFI2SDIV(4)                     /*!< PLL clock divided by 5 for I2S clock */
#define RCU_PLLFI2SDIV_DIV6                ADDCTL_PLLFI2SDIV(5)                     /*!< PLL clock divided by 6 for I2S clock */
#define RCU_PLLFI2SDIV_DIV7                ADDCTL_PLLFI2SDIV(6)                     /*!< PLL clock divided by 7 for I2S clock */
#define RCU_PLLFI2SDIV_DIV8                ADDCTL_PLLFI2SDIV(7)                     /*!< PLL clock divided by 8 for I2S clock */
#define RCU_PLLFI2SDIV_DIV9                ADDCTL_PLLFI2SDIV(8)                     /*!< PLL clock divided by 9 for I2S clock */
#define RCU_PLLFI2SDIV_DIV10               ADDCTL_PLLFI2SDIV(9)                     /*!< PLL clock divided by 10 for I2S clock */
#define RCU_PLLFI2SDIV_DIV11               ADDCTL_PLLFI2SDIV(10)                    /*!< PLL clock divided by 11 for I2S clock */
#define RCU_PLLFI2SDIV_DIV12               ADDCTL_PLLFI2SDIV(11)                    /*!< PLL clock divided by 12 for I2S clock */
#define RCU_PLLFI2SDIV_DIV13               ADDCTL_PLLFI2SDIV(12)                    /*!< PLL clock divided by 13 for I2S clock */
#define RCU_PLLFI2SDIV_DIV14               ADDCTL_PLLFI2SDIV(13)                    /*!< PLL clock divided by 14 for I2S clock */
#define RCU_PLLFI2SDIV_DIV15               ADDCTL_PLLFI2SDIV(14)                    /*!< PLL clock divided by 15 for I2S clock */
#define RCU_PLLFI2SDIV_DIV16               ADDCTL_PLLFI2SDIV(15)                    /*!< PLL clock divided by 16 for I2S clock */
#define RCU_PLLFI2SDIV_DIV17               ADDCTL_PLLFI2SDIV(16)                    /*!< PLL clock divided by 17 for I2S clock */
#define RCU_PLLFI2SDIV_DIV18               ADDCTL_PLLFI2SDIV(17)                    /*!< PLL clock divided by 18 for I2S clock */
#define RCU_PLLFI2SDIV_DIV19               ADDCTL_PLLFI2SDIV(18)                    /*!< PLL clock divided by 19 for I2S clock */
#define RCU_PLLFI2SDIV_DIV20               ADDCTL_PLLFI2SDIV(19)                    /*!< PLL clock divided by 20 for I2S clock */
#define RCU_PLLFI2SDIV_DIV21               ADDCTL_PLLFI2SDIV(20)                    /*!< PLL clock divided by 21 for I2S clock */
#define RCU_PLLFI2SDIV_DIV22               ADDCTL_PLLFI2SDIV(21)                    /*!< PLL clock divided by 22 for I2S clock */
#define RCU_PLLFI2SDIV_DIV23               ADDCTL_PLLFI2SDIV(22)                    /*!< PLL clock divided by 23 for I2S clock */
#define RCU_PLLFI2SDIV_DIV24               ADDCTL_PLLFI2SDIV(23)                    /*!< PLL clock divided by 24 for I2S clock */
#define RCU_PLLFI2SDIV_DIV25               ADDCTL_PLLFI2SDIV(24)                    /*!< PLL clock divided by 25 for I2S clock */
#define RCU_PLLFI2SDIV_DIV26               ADDCTL_PLLFI2SDIV(25)                    /*!< PLL clock divided by 26 for I2S clock */
#define RCU_PLLFI2SDIV_DIV27               ADDCTL_PLLFI2SDIV(26)                    /*!< PLL clock divided by 27 for I2S clock */
#define RCU_PLLFI2SDIV_DIV28               ADDCTL_PLLFI2SDIV(27)                    /*!< PLL clock divided by 28 for I2S clock */
#define RCU_PLLFI2SDIV_DIV29               ADDCTL_PLLFI2SDIV(28)                    /*!< PLL clock divided by 29 for I2S clock */
#define RCU_PLLFI2SDIV_DIV30               ADDCTL_PLLFI2SDIV(29)                    /*!< PLL clock divided by 30 for I2S clock */
#define RCU_PLLFI2SDIV_DIV31               ADDCTL_PLLFI2SDIV(30)                    /*!< PLL clock divided by 31 for I2S clock */
#define RCU_PLLFI2SDIV_DIV32               ADDCTL_PLLFI2SDIV(31)                    /*!< PLL clock divided by 32 for I2S clock */
#define RCU_PLLFI2SDIV_DIV33               ADDCTL_PLLFI2SDIV(32)                    /*!< PLL clock divided by 33 for I2S clock */
#define RCU_PLLFI2SDIV_DIV34               ADDCTL_PLLFI2SDIV(33)                    /*!< PLL clock divided by 34 for I2S clock */
#define RCU_PLLFI2SDIV_DIV35               ADDCTL_PLLFI2SDIV(34)                    /*!< PLL clock divided by 35 for I2S clock */
#define RCU_PLLFI2SDIV_DIV36               ADDCTL_PLLFI2SDIV(35)                    /*!< PLL clock divided by 36 for I2S clock */
#define RCU_PLLFI2SDIV_DIV37               ADDCTL_PLLFI2SDIV(36)                    /*!< PLL clock divided by 37 for I2S clock */
#define RCU_PLLFI2SDIV_DIV38               ADDCTL_PLLFI2SDIV(37)                    /*!< PLL clock divided by 38 for I2S clock */
#define RCU_PLLFI2SDIV_DIV39               ADDCTL_PLLFI2SDIV(38)                    /*!< PLL clock divided by 39 for I2S clock */
#define RCU_PLLFI2SDIV_DIV40               ADDCTL_PLLFI2SDIV(39)                    /*!< PLL clock divided by 40 for I2S clock */
#define RCU_PLLFI2SDIV_DIV41               ADDCTL_PLLFI2SDIV(40)                    /*!< PLL clock divided by 41 for I2S clock */
#define RCU_PLLFI2SDIV_DIV42               ADDCTL_PLLFI2SDIV(41)                    /*!< PLL clock divided by 42 for I2S clock */
#define RCU_PLLFI2SDIV_DIV43               ADDCTL_PLLFI2SDIV(42)                    /*!< PLL clock divided by 43 for I2S clock */
#define RCU_PLLFI2SDIV_DIV44               ADDCTL_PLLFI2SDIV(43)                    /*!< PLL clock divided by 44 for I2S clock */
#define RCU_PLLFI2SDIV_DIV45               ADDCTL_PLLFI2SDIV(44)                    /*!< PLL clock divided by 45 for I2S clock */
#define RCU_PLLFI2SDIV_DIV46               ADDCTL_PLLFI2SDIV(45)                    /*!< PLL clock divided by 46 for I2S clock */
#define RCU_PLLFI2SDIV_DIV47               ADDCTL_PLLFI2SDIV(46)                    /*!< PLL clock divided by 47 for I2S clock */
#define RCU_PLLFI2SDIV_DIV48               ADDCTL_PLLFI2SDIV(47)                    /*!< PLL clock divided by 48 for I2S clock */
#define RCU_PLLFI2SDIV_DIV49               ADDCTL_PLLFI2SDIV(48)                    /*!< PLL clock divided by 49 for I2S clock */
#define RCU_PLLFI2SDIV_DIV50               ADDCTL_PLLFI2SDIV(49)                    /*!< PLL clock divided by 50 for I2S clock */
#define RCU_PLLFI2SDIV_DIV51               ADDCTL_PLLFI2SDIV(50)                    /*!< PLL clock divided by 51 for I2S clock */
#define RCU_PLLFI2SDIV_DIV52               ADDCTL_PLLFI2SDIV(51)                    /*!< PLL clock divided by 52 for I2S clock */
#define RCU_PLLFI2SDIV_DIV53               ADDCTL_PLLFI2SDIV(52)                    /*!< PLL clock divided by 53 for I2S clock */
#define RCU_PLLFI2SDIV_DIV54               ADDCTL_PLLFI2SDIV(53)                    /*!< PLL clock divided by 54 for I2S clock */
#define RCU_PLLFI2SDIV_DIV55               ADDCTL_PLLFI2SDIV(54)                    /*!< PLL clock divided by 55 for I2S clock */
#define RCU_PLLFI2SDIV_DIV56               ADDCTL_PLLFI2SDIV(55)                    /*!< PLL clock divided by 56 for I2S clock */
#define RCU_PLLFI2SDIV_DIV57               ADDCTL_PLLFI2SDIV(56)                    /*!< PLL clock divided by 57 for I2S clock */
#define RCU_PLLFI2SDIV_DIV58               ADDCTL_PLLFI2SDIV(57)                    /*!< PLL clock divided by 58 for I2S clock */
#define RCU_PLLFI2SDIV_DIV59               ADDCTL_PLLFI2SDIV(58)                    /*!< PLL clock divided by 59 for I2S clock */
#define RCU_PLLFI2SDIV_DIV60               ADDCTL_PLLFI2SDIV(59)                    /*!< PLL clock divided by 60 for I2S clock */
#define RCU_PLLFI2SDIV_DIV61               ADDCTL_PLLFI2SDIV(60)                    /*!< PLL clock divided by 61 for I2S clock */
#define RCU_PLLFI2SDIV_DIV62               ADDCTL_PLLFI2SDIV(61)                    /*!< PLL clock divided by 62 for I2S clock */
#define RCU_PLLFI2SDIV_DIV63               ADDCTL_PLLFI2SDIV(62)                    /*!< PLL clock divided by 63 for I2S clock */
#define RCU_PLLFI2SDIV_DIV64               ADDCTL_PLLFI2SDIV(63)                    /*!< PLL clock divided by 64 for I2S clock */

/* TIMER clock selection */
#define RCU_TIMER_PSC_MUL2              ~RCU_CFG1_TIMERSEL                 /*!< if APB1PSC/APB2PSC in RCU_CFG0 register is 0b0xx(CK_APBx = CK_AHB) 
                                                                                or 0b100(CK_APBx = CK_AHB/2), the TIMER clock is equal to CK_AHB(CK_TIMERx = CK_AHB).
                                                                                or else, the TIMER clock is twice the corresponding APB clock (TIMER in APB1 domain: CK_TIMERx = 2 x CK_APB1; 
                                                                                TIMER in APB2 domain: CK_TIMERx = 2 x CK_APB2) */
#define RCU_TIMER_PSC_MUL4              RCU_CFG1_TIMERSEL                  /*!< if APB1PSC/APB2PSC in RCU_CFG0 register is 0b0xx(CK_APBx = CK_AHB), 
                                                                                0b100(CK_APBx = CK_AHB/2), or 0b101(CK_APBx = CK_AHB/4), the TIMER clock is equal to CK_AHB(CK_TIMERx = CK_AHB). 
                                                                                or else, the TIMER clock is four timers the corresponding APB clock (TIMER in APB1 domain: CK_TIMERx = 4 x CK_APB1;  
                                                                                TIMER in APB2 domain: CK_TIMERx = 4 x CK_APB2) */

/* RCU_PLLSSCTL register bit define */
/* PLL spread spectrum modulation type select */
#define RCU_SS_TYPE_CENTER              ((uint32_t)0x00000000U)            /*!< center type is selected */
#define RCU_SS_TYPE_DOWN                RCU_PLLSSCTL_SS_TYPE               /*!< down type is selected */

/* RCU_PLL register bit define */
/* The PLL VCO source clock prescaler */
#define RCU_PLLPSC_DIV_MIN              ((uint32_t)2U)                     /*!< PLLPSC_DIV min value */
#define RCU_PLLPSC_DIV_MAX              ((uint32_t)63U)                    /*!< PLLPSC_DIV max value */

/* The PLL VCO clock multi factor */
#define RCU_PLLN_MUL_MIN                ((uint32_t)64U)                    /*!< PLLN_MUL min value */
#define RCU_PLLN_MUL_MAX                ((uint32_t)511U)                   /*!< PLLN_MUL max value */
#define RCU_SS_MODULATION_CENTER_INC    ((uint32_t)7U)                     /*!< minimum factor of PLLN in center mode */
#define RCU_SS_MODULATION_DOWN_INC      ((uint32_t)11U)                    /*!< minimum factor of PLLN in down mode */

/* The PLLP output frequency division factor from PLL VCO clock */
#define RCU_PLLP_DIV_MIN                ((uint32_t)2U)                     /*!< PLLP_DIV min value */
#define RCU_PLLP_DIV_MAX                ((uint32_t)8U)                     /*!< PLLP_DIV max value */
                                         
/* PLL Clock Source Selection  */
#define RCU_PLLSRC_IRC16M               ((uint32_t)0x00000000U)            /*!< IRC16M clock selected as source clock of PLL, PLLDIG, PLLI2S */
#define RCU_PLLSRC_HXTAL                RCU_PLL_PLLSEL                     /*!< HXTAL clock selected as source clock of PLL, PLLDIG, PLLI2S */


#define CHECK_PLL_PSC_VALID(val)        (((val) >= RCU_PLLPSC_DIV_MIN)&&((val) <= RCU_PLLPSC_DIV_MAX))
#define CHECK_PLL_N_VALID(val, inc)     (((val) >= (RCU_PLLN_MUL_MIN + (inc)))&&((val) <= RCU_PLLN_MUL_MAX))
#define CHECK_PLL_P_VALID(val)          (((val) == 2U) || ((val) == 4U) || ((val) == 6U) || ((val) == 8U))

/* RCU_BDCTL register bit define */
/* LXTAL drive capability */
#define BDCTL_LXTALDRI(regval)         (BITS(3,4) & ((uint32_t)(regval) << 3U))
#define RCU_LXTALDRI_LOWER_DRIVE        ((uint32_t)0x00000000)             /*!< LXTAL drive capability is selected lower */
#define RCU_LXTALDRI_HIGH_DRIVE         BDCTL_LXTALDRI(1)                  /*!< LXTAL drive capability is selected high */
#define RCU_LXTALDRI_HIGHER_DRIVE       BDCTL_LXTALDRI(2)                  /*!< LXTAL drive capability is selected higher */
#define RCU_LXTALDRI_HIGHEST_DRIVE      BDCTL_LXTALDRI(3)                  /*!< LXTAL drive capability is selected highest */

/* RTC clock entry selection */
#define BDCTL_RTCSRC(regval)            (BITS(8,9) & ((uint32_t)(regval) << 8U))
#define RCU_RTCSRC_NONE                 BDCTL_RTCSRC(0)                    /*!< no clock selected */
#define RCU_RTCSRC_LXTAL                BDCTL_RTCSRC(1)                    /*!< RTC source clock select LXTAL  */
#define RCU_RTCSRC_IRC32K               BDCTL_RTCSRC(2)                    /*!< RTC source clock select IRC32K */
#define RCU_RTCSRC_HXTAL_DIV_RTCDIV     BDCTL_RTCSRC(3)                    /*!< RTC source clock select HXTAL/RTCDIV */

/* RCU_PLLCFG register bit define */
/* the divider factor from PLLI2S clock */
#define PLLCFG_PLLI2SDIV(regval)        (BITS(0,5) & ((uint32_t)(regval) << 0U))
#define RCU_PLLI2S_DIV32                PLLCFG_PLLI2SDIV(0)                         /*!< CK_PLLI2SDIV clock select CK_PLLI2S/32 */
#define RCU_PLLI2S_NONE_1               PLLCFG_PLLI2SDIV(1)                         /*!< reserved */
#define RCU_PLLI2S_NONE_2               PLLCFG_PLLI2SDIV(2)                         /*!< reserved */
#define RCU_PLLI2S_DIV1_5               PLLCFG_PLLI2SDIV(3)                         /*!< CK_PLLI2SDIV clock select CK_PLLI2S/1.5 */
#define RCU_PLLI2S_DIV2                 PLLCFG_PLLI2SDIV(4)                         /*!< CK_PLLI2SDIV clock select CK_PLLI2S/2 */
#define RCU_PLLI2S_DIV2_5               PLLCFG_PLLI2SDIV(5)                         /*!< CK_PLLI2SDIV clock select CK_PLLI2S/2.5 */
#define RCU_PLLI2S_DIV3                 PLLCFG_PLLI2SDIV(6)                         /*!< CK_PLLI2SDIV clock select CK_PLLI2S/3 */
#define RCU_PLLI2S_DIV3_5               PLLCFG_PLLI2SDIV(7)                         /*!< CK_PLLI2SDIV clock select CK_PLLI2S/3.5 */
#define RCU_PLLI2S_DIV4                 PLLCFG_PLLI2SDIV(8)                         /*!< CK_PLLI2SDIV clock select CK_PLLI2S/4 */
#define RCU_PLLI2S_DIV4_5               PLLCFG_PLLI2SDIV(9)                         /*!< CK_PLLI2SDIV clock select CK_PLLI2S/4.5 */
#define RCU_PLLI2S_DIV5                 PLLCFG_PLLI2SDIV(10)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/5 */
#define RCU_PLLI2S_DIV5_5               PLLCFG_PLLI2SDIV(11)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/5.5 */
#define RCU_PLLI2S_DIV6                 PLLCFG_PLLI2SDIV(12)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/6 */
#define RCU_PLLI2S_DIV6_5               PLLCFG_PLLI2SDIV(13)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/6.5 */
#define RCU_PLLI2S_DIV7                 PLLCFG_PLLI2SDIV(14)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/7 */
#define RCU_PLLI2S_DIV7_5               PLLCFG_PLLI2SDIV(15)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/7.5 */
#define RCU_PLLI2S_DIV8                 PLLCFG_PLLI2SDIV(16)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/8 */
#define RCU_PLLI2S_DIV8_5               PLLCFG_PLLI2SDIV(17)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/8.5 */
#define RCU_PLLI2S_DIV9                 PLLCFG_PLLI2SDIV(18)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/9 */
#define RCU_PLLI2S_DIV9_5               PLLCFG_PLLI2SDIV(19)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/9.5 */
#define RCU_PLLI2S_DIV10                PLLCFG_PLLI2SDIV(20)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/10 */
#define RCU_PLLI2S_DIV10_5              PLLCFG_PLLI2SDIV(21)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/10.5 */
#define RCU_PLLI2S_DIV11                PLLCFG_PLLI2SDIV(22)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/11 */
#define RCU_PLLI2S_DIV11_5              PLLCFG_PLLI2SDIV(23)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/11.5 */
#define RCU_PLLI2S_DIV12                PLLCFG_PLLI2SDIV(24)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/12 */
#define RCU_PLLI2S_DIV12_5              PLLCFG_PLLI2SDIV(25)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/12.5 */
#define RCU_PLLI2S_DIV13                PLLCFG_PLLI2SDIV(26)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/13 */
#define RCU_PLLI2S_DIV13_5              PLLCFG_PLLI2SDIV(27)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/13.5 */
#define RCU_PLLI2S_DIV14                PLLCFG_PLLI2SDIV(28)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/14 */
#define RCU_PLLI2S_DIV14_5              PLLCFG_PLLI2SDIV(29)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/14.5 */
#define RCU_PLLI2S_DIV15                PLLCFG_PLLI2SDIV(30)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/15 */
#define RCU_PLLI2S_DIV15_5              PLLCFG_PLLI2SDIV(31)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/15.5 */
#define RCU_PLLI2S_DIV16                PLLCFG_PLLI2SDIV(32)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/16 */
#define RCU_PLLI2S_DIV16_5              PLLCFG_PLLI2SDIV(33)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/16.5 */
#define RCU_PLLI2S_DIV17                PLLCFG_PLLI2SDIV(34)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/17 */
#define RCU_PLLI2S_DIV17_5              PLLCFG_PLLI2SDIV(35)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/17.5 */
#define RCU_PLLI2S_DIV18                PLLCFG_PLLI2SDIV(36)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/18 */
#define RCU_PLLI2S_DIV18_5              PLLCFG_PLLI2SDIV(37)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/18.5 */
#define RCU_PLLI2S_DIV19                PLLCFG_PLLI2SDIV(38)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/19 */
#define RCU_PLLI2S_DIV19_5              PLLCFG_PLLI2SDIV(39)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/19.5 */
#define RCU_PLLI2S_DIV20                PLLCFG_PLLI2SDIV(40)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/20 */
#define RCU_PLLI2S_DIV20_5              PLLCFG_PLLI2SDIV(41)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/20.5 */
#define RCU_PLLI2S_DIV21                PLLCFG_PLLI2SDIV(42)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/21 */
#define RCU_PLLI2S_DIV21_5              PLLCFG_PLLI2SDIV(43)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/21.5 */
#define RCU_PLLI2S_DIV22                PLLCFG_PLLI2SDIV(44)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/22 */
#define RCU_PLLI2S_DIV22_5              PLLCFG_PLLI2SDIV(45)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/22.5 */
#define RCU_PLLI2S_DIV23                PLLCFG_PLLI2SDIV(46)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/23 */
#define RCU_PLLI2S_DIV23_5              PLLCFG_PLLI2SDIV(47)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/23.5 */
#define RCU_PLLI2S_DIV24                PLLCFG_PLLI2SDIV(48)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/24 */
#define RCU_PLLI2S_DIV24_5              PLLCFG_PLLI2SDIV(49)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/24.5 */
#define RCU_PLLI2S_DIV25                PLLCFG_PLLI2SDIV(50)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/25 */
#define RCU_PLLI2S_DIV25_5              PLLCFG_PLLI2SDIV(51)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/25.5 */
#define RCU_PLLI2S_DIV26                PLLCFG_PLLI2SDIV(52)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/26 */
#define RCU_PLLI2S_DIV26_5              PLLCFG_PLLI2SDIV(53)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/26.5 */
#define RCU_PLLI2S_DIV27                PLLCFG_PLLI2SDIV(54)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/27 */
#define RCU_PLLI2S_DIV27_5              PLLCFG_PLLI2SDIV(55)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/27.5 */
#define RCU_PLLI2S_DIV28                PLLCFG_PLLI2SDIV(56)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/28 */
#define RCU_PLLI2S_DIV28_5              PLLCFG_PLLI2SDIV(57)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/28.5 */
#define RCU_PLLI2S_DIV29                PLLCFG_PLLI2SDIV(58)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/29 */
#define RCU_PLLI2S_DIV29_5              PLLCFG_PLLI2SDIV(59)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/29.5 */
#define RCU_PLLI2S_DIV30                PLLCFG_PLLI2SDIV(60)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/30 */
#define RCU_PLLI2S_DIV30_5              PLLCFG_PLLI2SDIV(61)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/30.5 */
#define RCU_PLLI2S_DIV31                PLLCFG_PLLI2SDIV(62)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/31 */
#define RCU_PLLI2S_DIV31_5              PLLCFG_PLLI2SDIV(63)                        /*!< CK_PLLI2SDIV clock select CK_PLLI2S/31.5 */

/* RCU_PLLI2S register bit define */
/* The PLLI2S VCO clock multi factor */
#define RCU_PLLI2SN_MUL_MIN             8U
#define RCU_PLLI2SN_MUL_MAX             127U

#define CHECK_PLLI2S_N_VALID(val)       (((val) >= RCU_PLLI2SN_MUL_MIN)&&((val) <= RCU_PLLI2SN_MUL_MAX))

/* PLLI2S VCO source clock pre-scale */
#define PLLCFG_PLLI2SPSC(regval)         (BITS(16,18) & ((uint32_t)(regval) << 16U))
#define RCU_PLLI2SSRC_DIV1                PLLCFG_PLLI2SPSC(0)                       /*!< PLLI2S vco input source clock divided by 1 */
#define RCU_PLLI2SSRC_DIV2                PLLCFG_PLLI2SPSC(1)                       /*!< PLLI2S vco input source clock divided by 2 */
#define RCU_PLLI2SSRC_DIV3                PLLCFG_PLLI2SPSC(2)                       /*!< PLLI2S vco input source clock divided by 3 */
#define RCU_PLLI2SSRC_DIV4                PLLCFG_PLLI2SPSC(3)                       /*!< PLLI2S vco input source clock divided by 4 */
#define RCU_PLLI2SSRC_DIV5                PLLCFG_PLLI2SPSC(4)                       /*!< PLLI2S vco input source clock divided by 5 */
#define RCU_PLLI2SSRC_DIV6                PLLCFG_PLLI2SPSC(5)                       /*!< PLLI2S vco input source clock divided by 6 */
#define RCU_PLLI2SSRC_DIV7                PLLCFG_PLLI2SPSC(6)                       /*!< PLLI2S vco input source clock divided by 7 */
#define RCU_PLLI2SSRC_DIV8                PLLCFG_PLLI2SPSC(7)                       /*!< PLLI2S vco input source clock divided by 8 */

/* PLLDIG Clock Selection  */
#define PLLDIG_OUTPUTSEL(regval)         (BITS(24,25) & ((uint32_t)(regval) << 24U))
#define RCU_PLLDIG_192M                   PLLDIG_OUTPUTSEL(0)                       /*!< selected 192Mhz as PLLDIG output frequency */
#define RCU_PLLDIG_240M                   PLLDIG_OUTPUTSEL(1)                       /*!< selected 240Mhz as PLLDIG output frequency */
#define RCU_PLLDIG_320M                   PLLDIG_OUTPUTSEL(2)                       /*!< selected 320Mhz as PLLDIG output frequency */
#define RCU_PLLDIG_480M                   PLLDIG_OUTPUTSEL(3)                       /*!< selected 480Mhz as PLLDIG output frequency */

/* PLLDIG clock divider factor for system clock */
#define PLLCFG_PLLDIGFSYSDIV(regval)         (BITS(26,31) & ((uint32_t)(regval) << 26U))
#define RCU_PLLDIGFSYS_DIV1               PLLCFG_PLLDIGFSYSDIV(0)                   /*!< PLLDIG clock divided by 1 for system clock */
#define RCU_PLLDIGFSYS_DIV2               PLLCFG_PLLDIGFSYSDIV(1)                   /*!< PLLDIG clock divided by 2 for system clock */
#define RCU_PLLDIGFSYS_DIV3               PLLCFG_PLLDIGFSYSDIV(2)                   /*!< PLLDIG clock divided by 3 for system clock */
#define RCU_PLLDIGFSYS_DIV4               PLLCFG_PLLDIGFSYSDIV(3)                   /*!< PLLDIG clock divided by 4 for system clock */
#define RCU_PLLDIGFSYS_DIV5               PLLCFG_PLLDIGFSYSDIV(4)                   /*!< PLLDIG clock divided by 5 for system clock */
#define RCU_PLLDIGFSYS_DIV6               PLLCFG_PLLDIGFSYSDIV(5)                   /*!< PLLDIG clock divided by 6 for system clock */
#define RCU_PLLDIGFSYS_DIV7               PLLCFG_PLLDIGFSYSDIV(6)                   /*!< PLLDIG clock divided by 7 for system clock */
#define RCU_PLLDIGFSYS_DIV8               PLLCFG_PLLDIGFSYSDIV(7)                   /*!< PLLDIG clock divided by 8 for system clock */
#define RCU_PLLDIGFSYS_DIV9               PLLCFG_PLLDIGFSYSDIV(8)                   /*!< PLLDIG clock divided by 9 for system clock */
#define RCU_PLLDIGFSYS_DIV10              PLLCFG_PLLDIGFSYSDIV(9)                   /*!< PLLDIG clock divided by 10 for system clock */
#define RCU_PLLDIGFSYS_DIV11              PLLCFG_PLLDIGFSYSDIV(10)                  /*!< PLLDIG clock divided by 11 for system clock */
#define RCU_PLLDIGFSYS_DIV12              PLLCFG_PLLDIGFSYSDIV(11)                  /*!< PLLDIG clock divided by 12 for system clock */
#define RCU_PLLDIGFSYS_DIV13              PLLCFG_PLLDIGFSYSDIV(12)                  /*!< PLLDIG clock divided by 13 for system clock */
#define RCU_PLLDIGFSYS_DIV14              PLLCFG_PLLDIGFSYSDIV(13)                  /*!< PLLDIG clock divided by 14 for system clock */
#define RCU_PLLDIGFSYS_DIV15              PLLCFG_PLLDIGFSYSDIV(14)                  /*!< PLLDIG clock divided by 15 for system clock */
#define RCU_PLLDIGFSYS_DIV16              PLLCFG_PLLDIGFSYSDIV(15)                  /*!< PLLDIG clock divided by 16 for system clock */
#define RCU_PLLDIGFSYS_DIV17              PLLCFG_PLLDIGFSYSDIV(16)                  /*!< PLLDIG clock divided by 17 for system clock */
#define RCU_PLLDIGFSYS_DIV18              PLLCFG_PLLDIGFSYSDIV(17)                  /*!< PLLDIG clock divided by 18 for system clock */
#define RCU_PLLDIGFSYS_DIV19              PLLCFG_PLLDIGFSYSDIV(18)                  /*!< PLLDIG clock divided by 19 for system clock */
#define RCU_PLLDIGFSYS_DIV20              PLLCFG_PLLDIGFSYSDIV(19)                  /*!< PLLDIG clock divided by 20 for system clock */
#define RCU_PLLDIGFSYS_DIV21              PLLCFG_PLLDIGFSYSDIV(20)                  /*!< PLLDIG clock divided by 21 for system clock */
#define RCU_PLLDIGFSYS_DIV22              PLLCFG_PLLDIGFSYSDIV(21)                  /*!< PLLDIG clock divided by 22 for system clock */
#define RCU_PLLDIGFSYS_DIV23              PLLCFG_PLLDIGFSYSDIV(22)                  /*!< PLLDIG clock divided by 23 for system clock */
#define RCU_PLLDIGFSYS_DIV24              PLLCFG_PLLDIGFSYSDIV(23)                  /*!< PLLDIG clock divided by 24 for system clock */
#define RCU_PLLDIGFSYS_DIV25              PLLCFG_PLLDIGFSYSDIV(24)                  /*!< PLLDIG clock divided by 25 for system clock */
#define RCU_PLLDIGFSYS_DIV26              PLLCFG_PLLDIGFSYSDIV(25)                  /*!< PLLDIG clock divided by 26 for system clock */
#define RCU_PLLDIGFSYS_DIV27              PLLCFG_PLLDIGFSYSDIV(26)                  /*!< PLLDIG clock divided by 27 for system clock */
#define RCU_PLLDIGFSYS_DIV28              PLLCFG_PLLDIGFSYSDIV(27)                  /*!< PLLDIG clock divided by 28 for system clock */
#define RCU_PLLDIGFSYS_DIV29              PLLCFG_PLLDIGFSYSDIV(28)                  /*!< PLLDIG clock divided by 29 for system clock */
#define RCU_PLLDIGFSYS_DIV30              PLLCFG_PLLDIGFSYSDIV(29)                  /*!< PLLDIG clock divided by 30 for system clock */
#define RCU_PLLDIGFSYS_DIV31              PLLCFG_PLLDIGFSYSDIV(30)                  /*!< PLLDIG clock divided by 31 for system clock */
#define RCU_PLLDIGFSYS_DIV32              PLLCFG_PLLDIGFSYSDIV(31)                  /*!< PLLDIG clock divided by 32 for system clock */
#define RCU_PLLDIGFSYS_DIV33              PLLCFG_PLLDIGFSYSDIV(32)                  /*!< PLLDIG clock divided by 33 for system clock */
#define RCU_PLLDIGFSYS_DIV34              PLLCFG_PLLDIGFSYSDIV(33)                  /*!< PLLDIG clock divided by 34 for system clock */
#define RCU_PLLDIGFSYS_DIV35              PLLCFG_PLLDIGFSYSDIV(34)                  /*!< PLLDIG clock divided by 35 for system clock */
#define RCU_PLLDIGFSYS_DIV36              PLLCFG_PLLDIGFSYSDIV(35)                  /*!< PLLDIG clock divided by 36 for system clock */
#define RCU_PLLDIGFSYS_DIV37              PLLCFG_PLLDIGFSYSDIV(36)                  /*!< PLLDIG clock divided by 37 for system clock */
#define RCU_PLLDIGFSYS_DIV38              PLLCFG_PLLDIGFSYSDIV(37)                  /*!< PLLDIG clock divided by 38 for system clock */
#define RCU_PLLDIGFSYS_DIV39              PLLCFG_PLLDIGFSYSDIV(38)                  /*!< PLLDIG clock divided by 39 for system clock */
#define RCU_PLLDIGFSYS_DIV40              PLLCFG_PLLDIGFSYSDIV(39)                  /*!< PLLDIG clock divided by 40 for system clock */
#define RCU_PLLDIGFSYS_DIV41              PLLCFG_PLLDIGFSYSDIV(40)                  /*!< PLLDIG clock divided by 41 for system clock */
#define RCU_PLLDIGFSYS_DIV42              PLLCFG_PLLDIGFSYSDIV(41)                  /*!< PLLDIG clock divided by 42 for system clock */
#define RCU_PLLDIGFSYS_DIV43              PLLCFG_PLLDIGFSYSDIV(42)                  /*!< PLLDIG clock divided by 43 for system clock */
#define RCU_PLLDIGFSYS_DIV44              PLLCFG_PLLDIGFSYSDIV(43)                  /*!< PLLDIG clock divided by 44 for system clock */
#define RCU_PLLDIGFSYS_DIV45              PLLCFG_PLLDIGFSYSDIV(44)                  /*!< PLLDIG clock divided by 45 for system clock */
#define RCU_PLLDIGFSYS_DIV46              PLLCFG_PLLDIGFSYSDIV(45)                  /*!< PLLDIG clock divided by 46 for system clock */
#define RCU_PLLDIGFSYS_DIV47              PLLCFG_PLLDIGFSYSDIV(46)                  /*!< PLLDIG clock divided by 47 for system clock */
#define RCU_PLLDIGFSYS_DIV48              PLLCFG_PLLDIGFSYSDIV(47)                  /*!< PLLDIG clock divided by 48 for system clock */
#define RCU_PLLDIGFSYS_DIV49              PLLCFG_PLLDIGFSYSDIV(48)                  /*!< PLLDIG clock divided by 49 for system clock */
#define RCU_PLLDIGFSYS_DIV50              PLLCFG_PLLDIGFSYSDIV(49)                  /*!< PLLDIG clock divided by 50 for system clock */
#define RCU_PLLDIGFSYS_DIV51              PLLCFG_PLLDIGFSYSDIV(50)                  /*!< PLLDIG clock divided by 51 for system clock */
#define RCU_PLLDIGFSYS_DIV52              PLLCFG_PLLDIGFSYSDIV(51)                  /*!< PLLDIG clock divided by 52 for system clock */
#define RCU_PLLDIGFSYS_DIV53              PLLCFG_PLLDIGFSYSDIV(52)                  /*!< PLLDIG clock divided by 53 for system clock */
#define RCU_PLLDIGFSYS_DIV54              PLLCFG_PLLDIGFSYSDIV(53)                  /*!< PLLDIG clock divided by 54 for system clock */
#define RCU_PLLDIGFSYS_DIV55              PLLCFG_PLLDIGFSYSDIV(54)                  /*!< PLLDIG clock divided by 55 for system clock */
#define RCU_PLLDIGFSYS_DIV56              PLLCFG_PLLDIGFSYSDIV(55)                  /*!< PLLDIG clock divided by 56 for system clock */
#define RCU_PLLDIGFSYS_DIV57              PLLCFG_PLLDIGFSYSDIV(56)                  /*!< PLLDIG clock divided by 57 for system clock */
#define RCU_PLLDIGFSYS_DIV58              PLLCFG_PLLDIGFSYSDIV(57)                  /*!< PLLDIG clock divided by 58 for system clock */
#define RCU_PLLDIGFSYS_DIV59              PLLCFG_PLLDIGFSYSDIV(58)                  /*!< PLLDIG clock divided by 59 for system clock */
#define RCU_PLLDIGFSYS_DIV60              PLLCFG_PLLDIGFSYSDIV(59)                  /*!< PLLDIG clock divided by 60 for system clock */
#define RCU_PLLDIGFSYS_DIV61              PLLCFG_PLLDIGFSYSDIV(60)                  /*!< PLLDIG clock divided by 61 for system clock */
#define RCU_PLLDIGFSYS_DIV62              PLLCFG_PLLDIGFSYSDIV(61)                  /*!< PLLDIG clock divided by 62 for system clock */
#define RCU_PLLDIGFSYS_DIV63              PLLCFG_PLLDIGFSYSDIV(62)                  /*!< PLLDIG clock divided by 63 for system clock */
#define RCU_PLLDIGFSYS_DIV64              PLLCFG_PLLDIGFSYSDIV(63)                  /*!< PLLDIG clock divided by 64 for system clock */

/* PLLDIG Clock Selection  */
#define PLLDIG_OUTPUTSEL(regval)         (BITS(24,25) & ((uint32_t)(regval) << 24U))
#define RCU_PLLDIG_192M                   PLLDIG_OUTPUTSEL(0)                       /*!< selected 192Mhz as PLLDIG output frequency */
#define RCU_PLLDIG_240M                   PLLDIG_OUTPUTSEL(1)                       /*!< selected 240Mhz as PLLDIG output frequency */
#define RCU_PLLDIG_320M                   PLLDIG_OUTPUTSEL(2)                       /*!< selected 320Mhz as PLLDIG output frequency */
#define RCU_PLLDIG_480M                   PLLDIG_OUTPUTSEL(3)                       /*!< selected 480Mhz as PLLDIG output frequency */

/* IRC16M clock divider factor for system clock */
#define CFG1_IRC16MDIV(regval)           (BITS(0,8) & ((uint32_t)(regval) << 0U))
#define RCU_IRC16M_DIV1                   CFG1_IRC16MDIV(0)                         /*!< IRC16M clock divided by 1 for system clock */
#define RCU_IRC16M_DIV2                   CFG1_IRC16MDIV(1)                         /*!< IRC16M clock divided by 2 for system clock */
#define RCU_IRC16M_DIV3                   CFG1_IRC16MDIV(2)                         /*!< IRC16M clock divided by 3 for system clock */
#define RCU_IRC16M_DIV4                   CFG1_IRC16MDIV(3)                         /*!< IRC16M clock divided by 4 for system clock */
#define RCU_IRC16M_DIV5                   CFG1_IRC16MDIV(4)                         /*!< IRC16M clock divided by 5 for system clock */
#define RCU_IRC16M_DIV6                   CFG1_IRC16MDIV(5)                         /*!< IRC16M clock divided by 6 for system clock */
#define RCU_IRC16M_DIV7                   CFG1_IRC16MDIV(6)                         /*!< IRC16M clock divided by 7 for system clock */
#define RCU_IRC16M_DIV8                   CFG1_IRC16MDIV(7)                         /*!< IRC16M clock divided by 8 for system clock */
#define RCU_IRC16M_DIV9                   CFG1_IRC16MDIV(8)                         /*!< IRC16M clock divided by 9 for system clock */
#define RCU_IRC16M_DIV10                  CFG1_IRC16MDIV(9)                         /*!< IRC16M clock divided by 10 for system clock */
#define RCU_IRC16M_DIV11                  CFG1_IRC16MDIV(10)                        /*!< IRC16M clock divided by 11 for system clock */
#define RCU_IRC16M_DIV12                  CFG1_IRC16MDIV(11)                        /*!< IRC16M clock divided by 12 for system clock */
#define RCU_IRC16M_DIV13                  CFG1_IRC16MDIV(12)                        /*!< IRC16M clock divided by 13 for system clock */
#define RCU_IRC16M_DIV14                  CFG1_IRC16MDIV(13)                        /*!< IRC16M clock divided by 14 for system clock */
#define RCU_IRC16M_DIV15                  CFG1_IRC16MDIV(14)                        /*!< IRC16M clock divided by 15 for system clock */
#define RCU_IRC16M_DIV16                  CFG1_IRC16MDIV(15)                        /*!< IRC16M clock divided by 16 for system clock */
#define RCU_IRC16M_DIV17                  CFG1_IRC16MDIV(16)                        /*!< IRC16M clock divided by 17 for system clock */
#define RCU_IRC16M_DIV18                  CFG1_IRC16MDIV(17)                        /*!< IRC16M clock divided by 18 for system clock */
#define RCU_IRC16M_DIV19                  CFG1_IRC16MDIV(18)                        /*!< IRC16M clock divided by 19 for system clock */
#define RCU_IRC16M_DIV20                  CFG1_IRC16MDIV(19)                        /*!< IRC16M clock divided by 20 for system clock */
#define RCU_IRC16M_DIV21                  CFG1_IRC16MDIV(20)                        /*!< IRC16M clock divided by 21 for system clock */
#define RCU_IRC16M_DIV22                  CFG1_IRC16MDIV(21)                        /*!< IRC16M clock divided by 22 for system clock */
#define RCU_IRC16M_DIV23                  CFG1_IRC16MDIV(22)                        /*!< IRC16M clock divided by 23 for system clock */
#define RCU_IRC16M_DIV24                  CFG1_IRC16MDIV(23)                        /*!< IRC16M clock divided by 24 for system clock */
#define RCU_IRC16M_DIV25                  CFG1_IRC16MDIV(24)                        /*!< IRC16M clock divided by 25 for system clock */
#define RCU_IRC16M_DIV26                  CFG1_IRC16MDIV(25)                        /*!< IRC16M clock divided by 26 for system clock */
#define RCU_IRC16M_DIV27                  CFG1_IRC16MDIV(26)                        /*!< IRC16M clock divided by 27 for system clock */
#define RCU_IRC16M_DIV28                  CFG1_IRC16MDIV(27)                        /*!< IRC16M clock divided by 28 for system clock */
#define RCU_IRC16M_DIV29                  CFG1_IRC16MDIV(28)                        /*!< IRC16M clock divided by 29 for system clock */
#define RCU_IRC16M_DIV30                  CFG1_IRC16MDIV(29)                        /*!< IRC16M clock divided by 30 for system clock */
#define RCU_IRC16M_DIV31                  CFG1_IRC16MDIV(30)                        /*!< IRC16M clock divided by 31 for system clock */
#define RCU_IRC16M_DIV32                  CFG1_IRC16MDIV(31)                        /*!< IRC16M clock divided by 32 for system clock */
#define RCU_IRC16M_DIV33                  CFG1_IRC16MDIV(32)                        /*!< IRC16M clock divided by 33 for system clock */
#define RCU_IRC16M_DIV34                  CFG1_IRC16MDIV(33)                        /*!< IRC16M clock divided by 34 for system clock */
#define RCU_IRC16M_DIV35                  CFG1_IRC16MDIV(34)                        /*!< IRC16M clock divided by 35 for system clock */
#define RCU_IRC16M_DIV36                  CFG1_IRC16MDIV(35)                        /*!< IRC16M clock divided by 36 for system clock */
#define RCU_IRC16M_DIV37                  CFG1_IRC16MDIV(36)                        /*!< IRC16M clock divided by 37 for system clock */
#define RCU_IRC16M_DIV38                  CFG1_IRC16MDIV(37)                        /*!< IRC16M clock divided by 38 for system clock */
#define RCU_IRC16M_DIV39                  CFG1_IRC16MDIV(38)                        /*!< IRC16M clock divided by 39 for system clock */
#define RCU_IRC16M_DIV40                  CFG1_IRC16MDIV(39)                        /*!< IRC16M clock divided by 40 for system clock */
#define RCU_IRC16M_DIV41                  CFG1_IRC16MDIV(40)                        /*!< IRC16M clock divided by 41 for system clock */
#define RCU_IRC16M_DIV42                  CFG1_IRC16MDIV(41)                        /*!< IRC16M clock divided by 42 for system clock */
#define RCU_IRC16M_DIV43                  CFG1_IRC16MDIV(42)                        /*!< IRC16M clock divided by 43 for system clock */
#define RCU_IRC16M_DIV44                  CFG1_IRC16MDIV(43)                        /*!< IRC16M clock divided by 44 for system clock */
#define RCU_IRC16M_DIV45                  CFG1_IRC16MDIV(44)                        /*!< IRC16M clock divided by 45 for system clock */
#define RCU_IRC16M_DIV46                  CFG1_IRC16MDIV(45)                        /*!< IRC16M clock divided by 46 for system clock */
#define RCU_IRC16M_DIV47                  CFG1_IRC16MDIV(46)                        /*!< IRC16M clock divided by 47 for system clock */
#define RCU_IRC16M_DIV48                  CFG1_IRC16MDIV(47)                        /*!< IRC16M clock divided by 48 for system clock */
#define RCU_IRC16M_DIV49                  CFG1_IRC16MDIV(48)                        /*!< IRC16M clock divided by 49 for system clock */
#define RCU_IRC16M_DIV50                  CFG1_IRC16MDIV(49)                        /*!< IRC16M clock divided by 50 for system clock */
#define RCU_IRC16M_DIV51                  CFG1_IRC16MDIV(50)                        /*!< IRC16M clock divided by 51 for system clock */
#define RCU_IRC16M_DIV52                  CFG1_IRC16MDIV(51)                        /*!< IRC16M clock divided by 52 for system clock */
#define RCU_IRC16M_DIV53                  CFG1_IRC16MDIV(52)                        /*!< IRC16M clock divided by 53 for system clock */
#define RCU_IRC16M_DIV54                  CFG1_IRC16MDIV(53)                        /*!< IRC16M clock divided by 54 for system clock */
#define RCU_IRC16M_DIV55                  CFG1_IRC16MDIV(54)                        /*!< IRC16M clock divided by 55 for system clock */
#define RCU_IRC16M_DIV56                  CFG1_IRC16MDIV(55)                        /*!< IRC16M clock divided by 56 for system clock */
#define RCU_IRC16M_DIV57                  CFG1_IRC16MDIV(56)                        /*!< IRC16M clock divided by 57 for system clock */
#define RCU_IRC16M_DIV58                  CFG1_IRC16MDIV(57)                        /*!< IRC16M clock divided by 58 for system clock */
#define RCU_IRC16M_DIV59                  CFG1_IRC16MDIV(58)                        /*!< IRC16M clock divided by 59 for system clock */
#define RCU_IRC16M_DIV60                  CFG1_IRC16MDIV(59)                        /*!< IRC16M clock divided by 60 for system clock */
#define RCU_IRC16M_DIV61                  CFG1_IRC16MDIV(60)                        /*!< IRC16M clock divided by 61 for system clock */
#define RCU_IRC16M_DIV62                  CFG1_IRC16MDIV(61)                        /*!< IRC16M clock divided by 62 for system clock */
#define RCU_IRC16M_DIV63                  CFG1_IRC16MDIV(62)                        /*!< IRC16M clock divided by 63 for system clock */
#define RCU_IRC16M_DIV64                  CFG1_IRC16MDIV(63)                        /*!< IRC16M clock divided by 64 for system clock */
#define RCU_IRC16M_DIV65                  CFG1_IRC16MDIV(64)                        /*!< IRC16M clock divided by 65 for system clock */
#define RCU_IRC16M_DIV66                  CFG1_IRC16MDIV(65)                        /*!< IRC16M clock divided by 66 for system clock */
#define RCU_IRC16M_DIV67                  CFG1_IRC16MDIV(66)                        /*!< IRC16M clock divided by 67 for system clock */
#define RCU_IRC16M_DIV68                  CFG1_IRC16MDIV(67)                        /*!< IRC16M clock divided by 68 for system clock */
#define RCU_IRC16M_DIV69                  CFG1_IRC16MDIV(68)                        /*!< IRC16M clock divided by 69 for system clock */
#define RCU_IRC16M_DIV70                  CFG1_IRC16MDIV(69)                        /*!< IRC16M clock divided by 70 for system clock */
#define RCU_IRC16M_DIV71                  CFG1_IRC16MDIV(70)                        /*!< IRC16M clock divided by 71 for system clock */
#define RCU_IRC16M_DIV72                  CFG1_IRC16MDIV(71)                        /*!< IRC16M clock divided by 72 for system clock */
#define RCU_IRC16M_DIV73                  CFG1_IRC16MDIV(72)                        /*!< IRC16M clock divided by 73 for system clock */
#define RCU_IRC16M_DIV74                  CFG1_IRC16MDIV(73)                        /*!< IRC16M clock divided by 74 for system clock */
#define RCU_IRC16M_DIV75                  CFG1_IRC16MDIV(74)                        /*!< IRC16M clock divided by 75 for system clock */
#define RCU_IRC16M_DIV76                  CFG1_IRC16MDIV(75)                        /*!< IRC16M clock divided by 76 for system clock */
#define RCU_IRC16M_DIV77                  CFG1_IRC16MDIV(76)                        /*!< IRC16M clock divided by 77 for system clock */
#define RCU_IRC16M_DIV78                  CFG1_IRC16MDIV(77)                        /*!< IRC16M clock divided by 78 for system clock */
#define RCU_IRC16M_DIV79                  CFG1_IRC16MDIV(78)                        /*!< IRC16M clock divided by 79 for system clock */
#define RCU_IRC16M_DIV80                  CFG1_IRC16MDIV(79)                        /*!< IRC16M clock divided by 80 for system clock */
#define RCU_IRC16M_DIV81                  CFG1_IRC16MDIV(80)                        /*!< IRC16M clock divided by 81 for system clock */
#define RCU_IRC16M_DIV82                  CFG1_IRC16MDIV(81)                        /*!< IRC16M clock divided by 82 for system clock */
#define RCU_IRC16M_DIV83                  CFG1_IRC16MDIV(82)                        /*!< IRC16M clock divided by 83 for system clock */
#define RCU_IRC16M_DIV84                  CFG1_IRC16MDIV(83)                        /*!< IRC16M clock divided by 84 for system clock */
#define RCU_IRC16M_DIV85                  CFG1_IRC16MDIV(84)                        /*!< IRC16M clock divided by 85 for system clock */
#define RCU_IRC16M_DIV86                  CFG1_IRC16MDIV(85)                        /*!< IRC16M clock divided by 86 for system clock */
#define RCU_IRC16M_DIV87                  CFG1_IRC16MDIV(86)                        /*!< IRC16M clock divided by 87 for system clock */
#define RCU_IRC16M_DIV88                  CFG1_IRC16MDIV(87)                        /*!< IRC16M clock divided by 88 for system clock */
#define RCU_IRC16M_DIV89                  CFG1_IRC16MDIV(88)                        /*!< IRC16M clock divided by 89 for system clock */
#define RCU_IRC16M_DIV90                  CFG1_IRC16MDIV(89)                        /*!< IRC16M clock divided by 90 for system clock */
#define RCU_IRC16M_DIV91                  CFG1_IRC16MDIV(90)                        /*!< IRC16M clock divided by 91 for system clock */
#define RCU_IRC16M_DIV92                  CFG1_IRC16MDIV(91)                        /*!< IRC16M clock divided by 92 for system clock */
#define RCU_IRC16M_DIV93                  CFG1_IRC16MDIV(92)                        /*!< IRC16M clock divided by 93 for system clock */
#define RCU_IRC16M_DIV94                  CFG1_IRC16MDIV(93)                        /*!< IRC16M clock divided by 94 for system clock */
#define RCU_IRC16M_DIV95                  CFG1_IRC16MDIV(94)                        /*!< IRC16M clock divided by 95 for system clock */
#define RCU_IRC16M_DIV96                  CFG1_IRC16MDIV(95)                        /*!< IRC16M clock divided by 96 for system clock */
#define RCU_IRC16M_DIV97                  CFG1_IRC16MDIV(96)                        /*!< IRC16M clock divided by 97 for system clock */
#define RCU_IRC16M_DIV98                  CFG1_IRC16MDIV(97)                        /*!< IRC16M clock divided by 98 for system clock */
#define RCU_IRC16M_DIV99                  CFG1_IRC16MDIV(98)                        /*!< IRC16M clock divided by 99 for system clock */
#define RCU_IRC16M_DIV100                 CFG1_IRC16MDIV(99)                        /*!< IRC16M clock divided by 100 for system clock */
#define RCU_IRC16M_DIV101                 CFG1_IRC16MDIV(100)                       /*!< IRC16M clock divided by 101 for system clock */
#define RCU_IRC16M_DIV102                 CFG1_IRC16MDIV(101)                       /*!< IRC16M clock divided by 102 for system clock */
#define RCU_IRC16M_DIV103                 CFG1_IRC16MDIV(102)                       /*!< IRC16M clock divided by 103 for system clock */
#define RCU_IRC16M_DIV104                 CFG1_IRC16MDIV(103)                       /*!< IRC16M clock divided by 104 for system clock */
#define RCU_IRC16M_DIV105                 CFG1_IRC16MDIV(104)                       /*!< IRC16M clock divided by 105 for system clock */
#define RCU_IRC16M_DIV106                 CFG1_IRC16MDIV(105)                       /*!< IRC16M clock divided by 106 for system clock */
#define RCU_IRC16M_DIV107                 CFG1_IRC16MDIV(106)                       /*!< IRC16M clock divided by 107 for system clock */
#define RCU_IRC16M_DIV108                 CFG1_IRC16MDIV(107)                       /*!< IRC16M clock divided by 108 for system clock */
#define RCU_IRC16M_DIV109                 CFG1_IRC16MDIV(108)                       /*!< IRC16M clock divided by 109 for system clock */
#define RCU_IRC16M_DIV110                 CFG1_IRC16MDIV(109)                       /*!< IRC16M clock divided by 110 for system clock */
#define RCU_IRC16M_DIV111                 CFG1_IRC16MDIV(110)                       /*!< IRC16M clock divided by 111 for system clock */
#define RCU_IRC16M_DIV112                 CFG1_IRC16MDIV(111)                       /*!< IRC16M clock divided by 112 for system clock */
#define RCU_IRC16M_DIV113                 CFG1_IRC16MDIV(112)                       /*!< IRC16M clock divided by 113 for system clock */
#define RCU_IRC16M_DIV114                 CFG1_IRC16MDIV(113)                       /*!< IRC16M clock divided by 114 for system clock */
#define RCU_IRC16M_DIV115                 CFG1_IRC16MDIV(114)                       /*!< IRC16M clock divided by 115 for system clock */
#define RCU_IRC16M_DIV116                 CFG1_IRC16MDIV(115)                       /*!< IRC16M clock divided by 116 for system clock */
#define RCU_IRC16M_DIV117                 CFG1_IRC16MDIV(116)                       /*!< IRC16M clock divided by 117 for system clock */
#define RCU_IRC16M_DIV118                 CFG1_IRC16MDIV(117)                       /*!< IRC16M clock divided by 118 for system clock */
#define RCU_IRC16M_DIV119                 CFG1_IRC16MDIV(118)                       /*!< IRC16M clock divided by 119 for system clock */
#define RCU_IRC16M_DIV120                 CFG1_IRC16MDIV(119)                       /*!< IRC16M clock divided by 120 for system clock */
#define RCU_IRC16M_DIV121                 CFG1_IRC16MDIV(120)                       /*!< IRC16M clock divided by 121 for system clock */
#define RCU_IRC16M_DIV122                 CFG1_IRC16MDIV(121)                       /*!< IRC16M clock divided by 122 for system clock */
#define RCU_IRC16M_DIV123                 CFG1_IRC16MDIV(122)                       /*!< IRC16M clock divided by 123 for system clock */
#define RCU_IRC16M_DIV124                 CFG1_IRC16MDIV(123)                       /*!< IRC16M clock divided by 124 for system clock */
#define RCU_IRC16M_DIV125                 CFG1_IRC16MDIV(124)                       /*!< IRC16M clock divided by 125 for system clock */
#define RCU_IRC16M_DIV126                 CFG1_IRC16MDIV(125)                       /*!< IRC16M clock divided by 126 for system clock */
#define RCU_IRC16M_DIV127                 CFG1_IRC16MDIV(126)                       /*!< IRC16M clock divided by 127 for system clock */
#define RCU_IRC16M_DIV128                 CFG1_IRC16MDIV(127)                       /*!< IRC16M clock divided by 128 for system clock */
#define RCU_IRC16M_DIV129                 CFG1_IRC16MDIV(128)                       /*!< IRC16M clock divided by 129 for system clock */
#define RCU_IRC16M_DIV130                 CFG1_IRC16MDIV(129)                       /*!< IRC16M clock divided by 130 for system clock */
#define RCU_IRC16M_DIV131                 CFG1_IRC16MDIV(130)                       /*!< IRC16M clock divided by 131 for system clock */
#define RCU_IRC16M_DIV132                 CFG1_IRC16MDIV(131)                       /*!< IRC16M clock divided by 132 for system clock */
#define RCU_IRC16M_DIV133                 CFG1_IRC16MDIV(132)                       /*!< IRC16M clock divided by 133 for system clock */
#define RCU_IRC16M_DIV134                 CFG1_IRC16MDIV(133)                       /*!< IRC16M clock divided by 134 for system clock */
#define RCU_IRC16M_DIV135                 CFG1_IRC16MDIV(134)                       /*!< IRC16M clock divided by 135 for system clock */
#define RCU_IRC16M_DIV136                 CFG1_IRC16MDIV(135)                       /*!< IRC16M clock divided by 136 for system clock */
#define RCU_IRC16M_DIV137                 CFG1_IRC16MDIV(136)                       /*!< IRC16M clock divided by 137 for system clock */
#define RCU_IRC16M_DIV138                 CFG1_IRC16MDIV(137)                       /*!< IRC16M clock divided by 138 for system clock */
#define RCU_IRC16M_DIV139                 CFG1_IRC16MDIV(138)                       /*!< IRC16M clock divided by 139 for system clock */
#define RCU_IRC16M_DIV140                 CFG1_IRC16MDIV(139)                       /*!< IRC16M clock divided by 140 for system clock */
#define RCU_IRC16M_DIV141                 CFG1_IRC16MDIV(140)                       /*!< IRC16M clock divided by 141 for system clock */
#define RCU_IRC16M_DIV142                 CFG1_IRC16MDIV(141)                       /*!< IRC16M clock divided by 142 for system clock */
#define RCU_IRC16M_DIV143                 CFG1_IRC16MDIV(142)                       /*!< IRC16M clock divided by 143 for system clock */
#define RCU_IRC16M_DIV144                 CFG1_IRC16MDIV(143)                       /*!< IRC16M clock divided by 144 for system clock */
#define RCU_IRC16M_DIV145                 CFG1_IRC16MDIV(144)                       /*!< IRC16M clock divided by 145 for system clock */
#define RCU_IRC16M_DIV146                 CFG1_IRC16MDIV(145)                       /*!< IRC16M clock divided by 146 for system clock */
#define RCU_IRC16M_DIV147                 CFG1_IRC16MDIV(146)                       /*!< IRC16M clock divided by 147 for system clock */
#define RCU_IRC16M_DIV148                 CFG1_IRC16MDIV(147)                       /*!< IRC16M clock divided by 148 for system clock */
#define RCU_IRC16M_DIV149                 CFG1_IRC16MDIV(148)                       /*!< IRC16M clock divided by 149 for system clock */
#define RCU_IRC16M_DIV150                 CFG1_IRC16MDIV(149)                       /*!< IRC16M clock divided by 150 for system clock */
#define RCU_IRC16M_DIV151                 CFG1_IRC16MDIV(150)                       /*!< IRC16M clock divided by 151 for system clock */
#define RCU_IRC16M_DIV152                 CFG1_IRC16MDIV(151)                       /*!< IRC16M clock divided by 152 for system clock */
#define RCU_IRC16M_DIV153                 CFG1_IRC16MDIV(152)                       /*!< IRC16M clock divided by 153 for system clock */
#define RCU_IRC16M_DIV154                 CFG1_IRC16MDIV(153)                       /*!< IRC16M clock divided by 154 for system clock */
#define RCU_IRC16M_DIV155                 CFG1_IRC16MDIV(154)                       /*!< IRC16M clock divided by 155 for system clock */
#define RCU_IRC16M_DIV156                 CFG1_IRC16MDIV(155)                       /*!< IRC16M clock divided by 156 for system clock */
#define RCU_IRC16M_DIV157                 CFG1_IRC16MDIV(156)                       /*!< IRC16M clock divided by 157 for system clock */
#define RCU_IRC16M_DIV158                 CFG1_IRC16MDIV(157)                       /*!< IRC16M clock divided by 158 for system clock */
#define RCU_IRC16M_DIV159                 CFG1_IRC16MDIV(158)                       /*!< IRC16M clock divided by 159 for system clock */
#define RCU_IRC16M_DIV160                 CFG1_IRC16MDIV(159)                       /*!< IRC16M clock divided by 160 for system clock */
#define RCU_IRC16M_DIV161                 CFG1_IRC16MDIV(160)                       /*!< IRC16M clock divided by 161 for system clock */
#define RCU_IRC16M_DIV162                 CFG1_IRC16MDIV(161)                       /*!< IRC16M clock divided by 162 for system clock */
#define RCU_IRC16M_DIV163                 CFG1_IRC16MDIV(162)                       /*!< IRC16M clock divided by 163 for system clock */
#define RCU_IRC16M_DIV164                 CFG1_IRC16MDIV(163)                       /*!< IRC16M clock divided by 164 for system clock */
#define RCU_IRC16M_DIV165                 CFG1_IRC16MDIV(164)                       /*!< IRC16M clock divided by 165 for system clock */
#define RCU_IRC16M_DIV166                 CFG1_IRC16MDIV(165)                       /*!< IRC16M clock divided by 166 for system clock */
#define RCU_IRC16M_DIV167                 CFG1_IRC16MDIV(166)                       /*!< IRC16M clock divided by 167 for system clock */
#define RCU_IRC16M_DIV168                 CFG1_IRC16MDIV(167)                       /*!< IRC16M clock divided by 168 for system clock */
#define RCU_IRC16M_DIV169                 CFG1_IRC16MDIV(168)                       /*!< IRC16M clock divided by 169 for system clock */
#define RCU_IRC16M_DIV170                 CFG1_IRC16MDIV(169)                       /*!< IRC16M clock divided by 170 for system clock */
#define RCU_IRC16M_DIV171                 CFG1_IRC16MDIV(170)                       /*!< IRC16M clock divided by 171 for system clock */
#define RCU_IRC16M_DIV172                 CFG1_IRC16MDIV(171)                       /*!< IRC16M clock divided by 172 for system clock */
#define RCU_IRC16M_DIV173                 CFG1_IRC16MDIV(172)                       /*!< IRC16M clock divided by 173 for system clock */
#define RCU_IRC16M_DIV174                 CFG1_IRC16MDIV(173)                       /*!< IRC16M clock divided by 174 for system clock */
#define RCU_IRC16M_DIV175                 CFG1_IRC16MDIV(174)                       /*!< IRC16M clock divided by 175 for system clock */
#define RCU_IRC16M_DIV176                 CFG1_IRC16MDIV(175)                       /*!< IRC16M clock divided by 176 for system clock */
#define RCU_IRC16M_DIV177                 CFG1_IRC16MDIV(176)                       /*!< IRC16M clock divided by 177 for system clock */
#define RCU_IRC16M_DIV178                 CFG1_IRC16MDIV(177)                       /*!< IRC16M clock divided by 178 for system clock */
#define RCU_IRC16M_DIV179                 CFG1_IRC16MDIV(178)                       /*!< IRC16M clock divided by 179 for system clock */
#define RCU_IRC16M_DIV180                 CFG1_IRC16MDIV(179)                       /*!< IRC16M clock divided by 180 for system clock */
#define RCU_IRC16M_DIV181                 CFG1_IRC16MDIV(180)                       /*!< IRC16M clock divided by 181 for system clock */
#define RCU_IRC16M_DIV182                 CFG1_IRC16MDIV(181)                       /*!< IRC16M clock divided by 182 for system clock */
#define RCU_IRC16M_DIV183                 CFG1_IRC16MDIV(182)                       /*!< IRC16M clock divided by 183 for system clock */
#define RCU_IRC16M_DIV184                 CFG1_IRC16MDIV(183)                       /*!< IRC16M clock divided by 184 for system clock */
#define RCU_IRC16M_DIV185                 CFG1_IRC16MDIV(184)                       /*!< IRC16M clock divided by 185 for system clock */
#define RCU_IRC16M_DIV186                 CFG1_IRC16MDIV(185)                       /*!< IRC16M clock divided by 186 for system clock */
#define RCU_IRC16M_DIV187                 CFG1_IRC16MDIV(186)                       /*!< IRC16M clock divided by 187 for system clock */
#define RCU_IRC16M_DIV188                 CFG1_IRC16MDIV(187)                       /*!< IRC16M clock divided by 188 for system clock */
#define RCU_IRC16M_DIV189                 CFG1_IRC16MDIV(188)                       /*!< IRC16M clock divided by 189 for system clock */
#define RCU_IRC16M_DIV190                 CFG1_IRC16MDIV(189)                       /*!< IRC16M clock divided by 190 for system clock */
#define RCU_IRC16M_DIV191                 CFG1_IRC16MDIV(190)                       /*!< IRC16M clock divided by 191 for system clock */
#define RCU_IRC16M_DIV192                 CFG1_IRC16MDIV(191)                       /*!< IRC16M clock divided by 192 for system clock */
#define RCU_IRC16M_DIV193                 CFG1_IRC16MDIV(192)                       /*!< IRC16M clock divided by 193 for system clock */
#define RCU_IRC16M_DIV194                 CFG1_IRC16MDIV(193)                       /*!< IRC16M clock divided by 194 for system clock */
#define RCU_IRC16M_DIV195                 CFG1_IRC16MDIV(194)                       /*!< IRC16M clock divided by 195 for system clock */
#define RCU_IRC16M_DIV196                 CFG1_IRC16MDIV(195)                       /*!< IRC16M clock divided by 196 for system clock */
#define RCU_IRC16M_DIV197                 CFG1_IRC16MDIV(196)                       /*!< IRC16M clock divided by 197 for system clock */
#define RCU_IRC16M_DIV198                 CFG1_IRC16MDIV(197)                       /*!< IRC16M clock divided by 198 for system clock */
#define RCU_IRC16M_DIV199                 CFG1_IRC16MDIV(198)                       /*!< IRC16M clock divided by 199 for system clock */
#define RCU_IRC16M_DIV200                 CFG1_IRC16MDIV(199)                       /*!< IRC16M clock divided by 200 for system clock */
#define RCU_IRC16M_DIV201                 CFG1_IRC16MDIV(200)                       /*!< IRC16M clock divided by 201 for system clock */
#define RCU_IRC16M_DIV202                 CFG1_IRC16MDIV(201)                       /*!< IRC16M clock divided by 202 for system clock */
#define RCU_IRC16M_DIV203                 CFG1_IRC16MDIV(202)                       /*!< IRC16M clock divided by 203 for system clock */
#define RCU_IRC16M_DIV204                 CFG1_IRC16MDIV(203)                       /*!< IRC16M clock divided by 204 for system clock */
#define RCU_IRC16M_DIV205                 CFG1_IRC16MDIV(204)                       /*!< IRC16M clock divided by 205 for system clock */
#define RCU_IRC16M_DIV206                 CFG1_IRC16MDIV(205)                       /*!< IRC16M clock divided by 206 for system clock */
#define RCU_IRC16M_DIV207                 CFG1_IRC16MDIV(206)                       /*!< IRC16M clock divided by 207 for system clock */
#define RCU_IRC16M_DIV208                 CFG1_IRC16MDIV(207)                       /*!< IRC16M clock divided by 208 for system clock */
#define RCU_IRC16M_DIV209                 CFG1_IRC16MDIV(208)                       /*!< IRC16M clock divided by 209 for system clock */
#define RCU_IRC16M_DIV210                 CFG1_IRC16MDIV(209)                       /*!< IRC16M clock divided by 210 for system clock */
#define RCU_IRC16M_DIV211                 CFG1_IRC16MDIV(210)                       /*!< IRC16M clock divided by 211 for system clock */
#define RCU_IRC16M_DIV212                 CFG1_IRC16MDIV(211)                       /*!< IRC16M clock divided by 212 for system clock */
#define RCU_IRC16M_DIV213                 CFG1_IRC16MDIV(212)                       /*!< IRC16M clock divided by 213 for system clock */
#define RCU_IRC16M_DIV214                 CFG1_IRC16MDIV(213)                       /*!< IRC16M clock divided by 214 for system clock */
#define RCU_IRC16M_DIV215                 CFG1_IRC16MDIV(214)                       /*!< IRC16M clock divided by 215 for system clock */
#define RCU_IRC16M_DIV216                 CFG1_IRC16MDIV(215)                       /*!< IRC16M clock divided by 216 for system clock */
#define RCU_IRC16M_DIV217                 CFG1_IRC16MDIV(216)                       /*!< IRC16M clock divided by 217 for system clock */
#define RCU_IRC16M_DIV218                 CFG1_IRC16MDIV(217)                       /*!< IRC16M clock divided by 218 for system clock */
#define RCU_IRC16M_DIV219                 CFG1_IRC16MDIV(218)                       /*!< IRC16M clock divided by 219 for system clock */
#define RCU_IRC16M_DIV220                 CFG1_IRC16MDIV(219)                       /*!< IRC16M clock divided by 220 for system clock */
#define RCU_IRC16M_DIV221                 CFG1_IRC16MDIV(220)                       /*!< IRC16M clock divided by 221 for system clock */
#define RCU_IRC16M_DIV222                 CFG1_IRC16MDIV(221)                       /*!< IRC16M clock divided by 222 for system clock */
#define RCU_IRC16M_DIV223                 CFG1_IRC16MDIV(222)                       /*!< IRC16M clock divided by 223 for system clock */
#define RCU_IRC16M_DIV224                 CFG1_IRC16MDIV(223)                       /*!< IRC16M clock divided by 224 for system clock */
#define RCU_IRC16M_DIV225                 CFG1_IRC16MDIV(224)                       /*!< IRC16M clock divided by 225 for system clock */
#define RCU_IRC16M_DIV226                 CFG1_IRC16MDIV(225)                       /*!< IRC16M clock divided by 226 for system clock */
#define RCU_IRC16M_DIV227                 CFG1_IRC16MDIV(226)                       /*!< IRC16M clock divided by 227 for system clock */
#define RCU_IRC16M_DIV228                 CFG1_IRC16MDIV(227)                       /*!< IRC16M clock divided by 228 for system clock */
#define RCU_IRC16M_DIV229                 CFG1_IRC16MDIV(228)                       /*!< IRC16M clock divided by 229 for system clock */
#define RCU_IRC16M_DIV230                 CFG1_IRC16MDIV(229)                       /*!< IRC16M clock divided by 230 for system clock */
#define RCU_IRC16M_DIV231                 CFG1_IRC16MDIV(230)                       /*!< IRC16M clock divided by 231 for system clock */
#define RCU_IRC16M_DIV232                 CFG1_IRC16MDIV(231)                       /*!< IRC16M clock divided by 232 for system clock */
#define RCU_IRC16M_DIV233                 CFG1_IRC16MDIV(232)                       /*!< IRC16M clock divided by 233 for system clock */
#define RCU_IRC16M_DIV234                 CFG1_IRC16MDIV(233)                       /*!< IRC16M clock divided by 234 for system clock */
#define RCU_IRC16M_DIV235                 CFG1_IRC16MDIV(234)                       /*!< IRC16M clock divided by 235 for system clock */
#define RCU_IRC16M_DIV236                 CFG1_IRC16MDIV(235)                       /*!< IRC16M clock divided by 236 for system clock */
#define RCU_IRC16M_DIV237                 CFG1_IRC16MDIV(236)                       /*!< IRC16M clock divided by 237 for system clock */
#define RCU_IRC16M_DIV238                 CFG1_IRC16MDIV(237)                       /*!< IRC16M clock divided by 238 for system clock */
#define RCU_IRC16M_DIV239                 CFG1_IRC16MDIV(238)                       /*!< IRC16M clock divided by 239 for system clock */
#define RCU_IRC16M_DIV240                 CFG1_IRC16MDIV(239)                       /*!< IRC16M clock divided by 240 for system clock */
#define RCU_IRC16M_DIV241                 CFG1_IRC16MDIV(240)                       /*!< IRC16M clock divided by 241 for system clock */
#define RCU_IRC16M_DIV242                 CFG1_IRC16MDIV(241)                       /*!< IRC16M clock divided by 242 for system clock */
#define RCU_IRC16M_DIV243                 CFG1_IRC16MDIV(242)                       /*!< IRC16M clock divided by 243 for system clock */
#define RCU_IRC16M_DIV244                 CFG1_IRC16MDIV(243)                       /*!< IRC16M clock divided by 244 for system clock */
#define RCU_IRC16M_DIV245                 CFG1_IRC16MDIV(244)                       /*!< IRC16M clock divided by 245 for system clock */
#define RCU_IRC16M_DIV246                 CFG1_IRC16MDIV(245)                       /*!< IRC16M clock divided by 246 for system clock */
#define RCU_IRC16M_DIV247                 CFG1_IRC16MDIV(246)                       /*!< IRC16M clock divided by 247 for system clock */
#define RCU_IRC16M_DIV248                 CFG1_IRC16MDIV(247)                       /*!< IRC16M clock divided by 248 for system clock */
#define RCU_IRC16M_DIV249                 CFG1_IRC16MDIV(248)                       /*!< IRC16M clock divided by 249 for system clock */
#define RCU_IRC16M_DIV250                 CFG1_IRC16MDIV(249)                       /*!< IRC16M clock divided by 250 for system clock */
#define RCU_IRC16M_DIV251                 CFG1_IRC16MDIV(250)                       /*!< IRC16M clock divided by 251 for system clock */
#define RCU_IRC16M_DIV252                 CFG1_IRC16MDIV(251)                       /*!< IRC16M clock divided by 252 for system clock */
#define RCU_IRC16M_DIV253                 CFG1_IRC16MDIV(252)                       /*!< IRC16M clock divided by 253 for system clock */
#define RCU_IRC16M_DIV254                 CFG1_IRC16MDIV(253)                       /*!< IRC16M clock divided by 254 for system clock */
#define RCU_IRC16M_DIV255                 CFG1_IRC16MDIV(254)                       /*!< IRC16M clock divided by 255 for system clock */
#define RCU_IRC16M_DIV256                 CFG1_IRC16MDIV(255)                       /*!< IRC16M clock divided by 256 for system clock */
#define RCU_IRC16M_DIV257                 CFG1_IRC16MDIV(256)                       /*!< IRC16M clock divided by 257 for system clock */
#define RCU_IRC16M_DIV258                 CFG1_IRC16MDIV(257)                       /*!< IRC16M clock divided by 258 for system clock */
#define RCU_IRC16M_DIV259                 CFG1_IRC16MDIV(258)                       /*!< IRC16M clock divided by 259 for system clock */
#define RCU_IRC16M_DIV260                 CFG1_IRC16MDIV(259)                       /*!< IRC16M clock divided by 260 for system clock */
#define RCU_IRC16M_DIV261                 CFG1_IRC16MDIV(260)                       /*!< IRC16M clock divided by 261 for system clock */
#define RCU_IRC16M_DIV262                 CFG1_IRC16MDIV(261)                       /*!< IRC16M clock divided by 262 for system clock */
#define RCU_IRC16M_DIV263                 CFG1_IRC16MDIV(262)                       /*!< IRC16M clock divided by 263 for system clock */
#define RCU_IRC16M_DIV264                 CFG1_IRC16MDIV(263)                       /*!< IRC16M clock divided by 264 for system clock */
#define RCU_IRC16M_DIV265                 CFG1_IRC16MDIV(264)                       /*!< IRC16M clock divided by 265 for system clock */
#define RCU_IRC16M_DIV266                 CFG1_IRC16MDIV(265)                       /*!< IRC16M clock divided by 266 for system clock */
#define RCU_IRC16M_DIV267                 CFG1_IRC16MDIV(266)                       /*!< IRC16M clock divided by 267 for system clock */
#define RCU_IRC16M_DIV268                 CFG1_IRC16MDIV(267)                       /*!< IRC16M clock divided by 268 for system clock */
#define RCU_IRC16M_DIV269                 CFG1_IRC16MDIV(268)                       /*!< IRC16M clock divided by 269 for system clock */
#define RCU_IRC16M_DIV270                 CFG1_IRC16MDIV(269)                       /*!< IRC16M clock divided by 270 for system clock */
#define RCU_IRC16M_DIV271                 CFG1_IRC16MDIV(270)                       /*!< IRC16M clock divided by 271 for system clock */
#define RCU_IRC16M_DIV272                 CFG1_IRC16MDIV(271)                       /*!< IRC16M clock divided by 272 for system clock */
#define RCU_IRC16M_DIV273                 CFG1_IRC16MDIV(272)                       /*!< IRC16M clock divided by 273 for system clock */
#define RCU_IRC16M_DIV274                 CFG1_IRC16MDIV(273)                       /*!< IRC16M clock divided by 274 for system clock */
#define RCU_IRC16M_DIV275                 CFG1_IRC16MDIV(274)                       /*!< IRC16M clock divided by 275 for system clock */
#define RCU_IRC16M_DIV276                 CFG1_IRC16MDIV(275)                       /*!< IRC16M clock divided by 276 for system clock */
#define RCU_IRC16M_DIV277                 CFG1_IRC16MDIV(276)                       /*!< IRC16M clock divided by 277 for system clock */
#define RCU_IRC16M_DIV278                 CFG1_IRC16MDIV(277)                       /*!< IRC16M clock divided by 278 for system clock */
#define RCU_IRC16M_DIV279                 CFG1_IRC16MDIV(278)                       /*!< IRC16M clock divided by 279 for system clock */
#define RCU_IRC16M_DIV280                 CFG1_IRC16MDIV(279)                       /*!< IRC16M clock divided by 280 for system clock */
#define RCU_IRC16M_DIV281                 CFG1_IRC16MDIV(280)                       /*!< IRC16M clock divided by 281 for system clock */
#define RCU_IRC16M_DIV282                 CFG1_IRC16MDIV(281)                       /*!< IRC16M clock divided by 282 for system clock */
#define RCU_IRC16M_DIV283                 CFG1_IRC16MDIV(282)                       /*!< IRC16M clock divided by 283 for system clock */
#define RCU_IRC16M_DIV284                 CFG1_IRC16MDIV(283)                       /*!< IRC16M clock divided by 284 for system clock */
#define RCU_IRC16M_DIV285                 CFG1_IRC16MDIV(284)                       /*!< IRC16M clock divided by 285 for system clock */
#define RCU_IRC16M_DIV286                 CFG1_IRC16MDIV(285)                       /*!< IRC16M clock divided by 286 for system clock */
#define RCU_IRC16M_DIV287                 CFG1_IRC16MDIV(286)                       /*!< IRC16M clock divided by 287 for system clock */
#define RCU_IRC16M_DIV288                 CFG1_IRC16MDIV(287)                       /*!< IRC16M clock divided by 288 for system clock */
#define RCU_IRC16M_DIV289                 CFG1_IRC16MDIV(288)                       /*!< IRC16M clock divided by 289 for system clock */
#define RCU_IRC16M_DIV290                 CFG1_IRC16MDIV(289)                       /*!< IRC16M clock divided by 290 for system clock */
#define RCU_IRC16M_DIV291                 CFG1_IRC16MDIV(290)                       /*!< IRC16M clock divided by 291 for system clock */
#define RCU_IRC16M_DIV292                 CFG1_IRC16MDIV(291)                       /*!< IRC16M clock divided by 292 for system clock */
#define RCU_IRC16M_DIV293                 CFG1_IRC16MDIV(292)                       /*!< IRC16M clock divided by 293 for system clock */
#define RCU_IRC16M_DIV294                 CFG1_IRC16MDIV(293)                       /*!< IRC16M clock divided by 294 for system clock */
#define RCU_IRC16M_DIV295                 CFG1_IRC16MDIV(294)                       /*!< IRC16M clock divided by 295 for system clock */
#define RCU_IRC16M_DIV296                 CFG1_IRC16MDIV(295)                       /*!< IRC16M clock divided by 296 for system clock */
#define RCU_IRC16M_DIV297                 CFG1_IRC16MDIV(296)                       /*!< IRC16M clock divided by 297 for system clock */
#define RCU_IRC16M_DIV298                 CFG1_IRC16MDIV(297)                       /*!< IRC16M clock divided by 298 for system clock */
#define RCU_IRC16M_DIV299                 CFG1_IRC16MDIV(298)                       /*!< IRC16M clock divided by 299 for system clock */
#define RCU_IRC16M_DIV300                 CFG1_IRC16MDIV(299)                       /*!< IRC16M clock divided by 300 for system clock */
#define RCU_IRC16M_DIV301                 CFG1_IRC16MDIV(300)                       /*!< IRC16M clock divided by 301 for system clock */
#define RCU_IRC16M_DIV302                 CFG1_IRC16MDIV(301)                       /*!< IRC16M clock divided by 302 for system clock */
#define RCU_IRC16M_DIV303                 CFG1_IRC16MDIV(302)                       /*!< IRC16M clock divided by 303 for system clock */
#define RCU_IRC16M_DIV304                 CFG1_IRC16MDIV(303)                       /*!< IRC16M clock divided by 304 for system clock */
#define RCU_IRC16M_DIV305                 CFG1_IRC16MDIV(304)                       /*!< IRC16M clock divided by 305 for system clock */
#define RCU_IRC16M_DIV306                 CFG1_IRC16MDIV(305)                       /*!< IRC16M clock divided by 306 for system clock */
#define RCU_IRC16M_DIV307                 CFG1_IRC16MDIV(306)                       /*!< IRC16M clock divided by 307 for system clock */
#define RCU_IRC16M_DIV308                 CFG1_IRC16MDIV(307)                       /*!< IRC16M clock divided by 308 for system clock */
#define RCU_IRC16M_DIV309                 CFG1_IRC16MDIV(308)                       /*!< IRC16M clock divided by 309 for system clock */
#define RCU_IRC16M_DIV310                 CFG1_IRC16MDIV(309)                       /*!< IRC16M clock divided by 310 for system clock */
#define RCU_IRC16M_DIV311                 CFG1_IRC16MDIV(310)                       /*!< IRC16M clock divided by 311 for system clock */
#define RCU_IRC16M_DIV312                 CFG1_IRC16MDIV(311)                       /*!< IRC16M clock divided by 312 for system clock */
#define RCU_IRC16M_DIV313                 CFG1_IRC16MDIV(312)                       /*!< IRC16M clock divided by 313 for system clock */
#define RCU_IRC16M_DIV314                 CFG1_IRC16MDIV(313)                       /*!< IRC16M clock divided by 314 for system clock */
#define RCU_IRC16M_DIV315                 CFG1_IRC16MDIV(314)                       /*!< IRC16M clock divided by 315 for system clock */
#define RCU_IRC16M_DIV316                 CFG1_IRC16MDIV(315)                       /*!< IRC16M clock divided by 316 for system clock */
#define RCU_IRC16M_DIV317                 CFG1_IRC16MDIV(316)                       /*!< IRC16M clock divided by 317 for system clock */
#define RCU_IRC16M_DIV318                 CFG1_IRC16MDIV(317)                       /*!< IRC16M clock divided by 318 for system clock */
#define RCU_IRC16M_DIV319                 CFG1_IRC16MDIV(318)                       /*!< IRC16M clock divided by 319 for system clock */
#define RCU_IRC16M_DIV320                 CFG1_IRC16MDIV(319)                       /*!< IRC16M clock divided by 320 for system clock */
#define RCU_IRC16M_DIV321                 CFG1_IRC16MDIV(320)                       /*!< IRC16M clock divided by 321 for system clock */
#define RCU_IRC16M_DIV322                 CFG1_IRC16MDIV(321)                       /*!< IRC16M clock divided by 322 for system clock */
#define RCU_IRC16M_DIV323                 CFG1_IRC16MDIV(322)                       /*!< IRC16M clock divided by 323 for system clock */
#define RCU_IRC16M_DIV324                 CFG1_IRC16MDIV(323)                       /*!< IRC16M clock divided by 324 for system clock */
#define RCU_IRC16M_DIV325                 CFG1_IRC16MDIV(324)                       /*!< IRC16M clock divided by 325 for system clock */
#define RCU_IRC16M_DIV326                 CFG1_IRC16MDIV(325)                       /*!< IRC16M clock divided by 326 for system clock */
#define RCU_IRC16M_DIV327                 CFG1_IRC16MDIV(326)                       /*!< IRC16M clock divided by 327 for system clock */
#define RCU_IRC16M_DIV328                 CFG1_IRC16MDIV(327)                       /*!< IRC16M clock divided by 328 for system clock */
#define RCU_IRC16M_DIV329                 CFG1_IRC16MDIV(328)                       /*!< IRC16M clock divided by 329 for system clock */
#define RCU_IRC16M_DIV330                 CFG1_IRC16MDIV(329)                       /*!< IRC16M clock divided by 330 for system clock */
#define RCU_IRC16M_DIV331                 CFG1_IRC16MDIV(330)                       /*!< IRC16M clock divided by 331 for system clock */
#define RCU_IRC16M_DIV332                 CFG1_IRC16MDIV(331)                       /*!< IRC16M clock divided by 332 for system clock */
#define RCU_IRC16M_DIV333                 CFG1_IRC16MDIV(332)                       /*!< IRC16M clock divided by 333 for system clock */
#define RCU_IRC16M_DIV334                 CFG1_IRC16MDIV(333)                       /*!< IRC16M clock divided by 334 for system clock */
#define RCU_IRC16M_DIV335                 CFG1_IRC16MDIV(334)                       /*!< IRC16M clock divided by 335 for system clock */
#define RCU_IRC16M_DIV336                 CFG1_IRC16MDIV(335)                       /*!< IRC16M clock divided by 336 for system clock */
#define RCU_IRC16M_DIV337                 CFG1_IRC16MDIV(336)                       /*!< IRC16M clock divided by 337 for system clock */
#define RCU_IRC16M_DIV338                 CFG1_IRC16MDIV(337)                       /*!< IRC16M clock divided by 338 for system clock */
#define RCU_IRC16M_DIV339                 CFG1_IRC16MDIV(338)                       /*!< IRC16M clock divided by 339 for system clock */
#define RCU_IRC16M_DIV340                 CFG1_IRC16MDIV(339)                       /*!< IRC16M clock divided by 340 for system clock */
#define RCU_IRC16M_DIV341                 CFG1_IRC16MDIV(340)                       /*!< IRC16M clock divided by 341 for system clock */
#define RCU_IRC16M_DIV342                 CFG1_IRC16MDIV(341)                       /*!< IRC16M clock divided by 342 for system clock */
#define RCU_IRC16M_DIV343                 CFG1_IRC16MDIV(342)                       /*!< IRC16M clock divided by 343 for system clock */
#define RCU_IRC16M_DIV344                 CFG1_IRC16MDIV(343)                       /*!< IRC16M clock divided by 344 for system clock */
#define RCU_IRC16M_DIV345                 CFG1_IRC16MDIV(344)                       /*!< IRC16M clock divided by 345 for system clock */
#define RCU_IRC16M_DIV346                 CFG1_IRC16MDIV(345)                       /*!< IRC16M clock divided by 346 for system clock */
#define RCU_IRC16M_DIV347                 CFG1_IRC16MDIV(346)                       /*!< IRC16M clock divided by 347 for system clock */
#define RCU_IRC16M_DIV348                 CFG1_IRC16MDIV(347)                       /*!< IRC16M clock divided by 348 for system clock */
#define RCU_IRC16M_DIV349                 CFG1_IRC16MDIV(348)                       /*!< IRC16M clock divided by 349 for system clock */
#define RCU_IRC16M_DIV350                 CFG1_IRC16MDIV(349)                       /*!< IRC16M clock divided by 350 for system clock */
#define RCU_IRC16M_DIV351                 CFG1_IRC16MDIV(350)                       /*!< IRC16M clock divided by 351 for system clock */
#define RCU_IRC16M_DIV352                 CFG1_IRC16MDIV(351)                       /*!< IRC16M clock divided by 352 for system clock */
#define RCU_IRC16M_DIV353                 CFG1_IRC16MDIV(352)                       /*!< IRC16M clock divided by 353 for system clock */
#define RCU_IRC16M_DIV354                 CFG1_IRC16MDIV(353)                       /*!< IRC16M clock divided by 354 for system clock */
#define RCU_IRC16M_DIV355                 CFG1_IRC16MDIV(354)                       /*!< IRC16M clock divided by 355 for system clock */
#define RCU_IRC16M_DIV356                 CFG1_IRC16MDIV(355)                       /*!< IRC16M clock divided by 356 for system clock */
#define RCU_IRC16M_DIV357                 CFG1_IRC16MDIV(356)                       /*!< IRC16M clock divided by 357 for system clock */
#define RCU_IRC16M_DIV358                 CFG1_IRC16MDIV(357)                       /*!< IRC16M clock divided by 358 for system clock */
#define RCU_IRC16M_DIV359                 CFG1_IRC16MDIV(358)                       /*!< IRC16M clock divided by 359 for system clock */
#define RCU_IRC16M_DIV360                 CFG1_IRC16MDIV(359)                       /*!< IRC16M clock divided by 360 for system clock */
#define RCU_IRC16M_DIV361                 CFG1_IRC16MDIV(360)                       /*!< IRC16M clock divided by 361 for system clock */
#define RCU_IRC16M_DIV362                 CFG1_IRC16MDIV(361)                       /*!< IRC16M clock divided by 362 for system clock */
#define RCU_IRC16M_DIV363                 CFG1_IRC16MDIV(362)                       /*!< IRC16M clock divided by 363 for system clock */
#define RCU_IRC16M_DIV364                 CFG1_IRC16MDIV(363)                       /*!< IRC16M clock divided by 364 for system clock */
#define RCU_IRC16M_DIV365                 CFG1_IRC16MDIV(364)                       /*!< IRC16M clock divided by 365 for system clock */
#define RCU_IRC16M_DIV366                 CFG1_IRC16MDIV(365)                       /*!< IRC16M clock divided by 366 for system clock */
#define RCU_IRC16M_DIV367                 CFG1_IRC16MDIV(366)                       /*!< IRC16M clock divided by 367 for system clock */
#define RCU_IRC16M_DIV368                 CFG1_IRC16MDIV(367)                       /*!< IRC16M clock divided by 368 for system clock */
#define RCU_IRC16M_DIV369                 CFG1_IRC16MDIV(368)                       /*!< IRC16M clock divided by 369 for system clock */
#define RCU_IRC16M_DIV370                 CFG1_IRC16MDIV(369)                       /*!< IRC16M clock divided by 370 for system clock */
#define RCU_IRC16M_DIV371                 CFG1_IRC16MDIV(370)                       /*!< IRC16M clock divided by 371 for system clock */
#define RCU_IRC16M_DIV372                 CFG1_IRC16MDIV(371)                       /*!< IRC16M clock divided by 372 for system clock */
#define RCU_IRC16M_DIV373                 CFG1_IRC16MDIV(372)                       /*!< IRC16M clock divided by 373 for system clock */
#define RCU_IRC16M_DIV374                 CFG1_IRC16MDIV(373)                       /*!< IRC16M clock divided by 374 for system clock */
#define RCU_IRC16M_DIV375                 CFG1_IRC16MDIV(374)                       /*!< IRC16M clock divided by 375 for system clock */
#define RCU_IRC16M_DIV376                 CFG1_IRC16MDIV(375)                       /*!< IRC16M clock divided by 376 for system clock */
#define RCU_IRC16M_DIV377                 CFG1_IRC16MDIV(376)                       /*!< IRC16M clock divided by 377 for system clock */
#define RCU_IRC16M_DIV378                 CFG1_IRC16MDIV(377)                       /*!< IRC16M clock divided by 378 for system clock */
#define RCU_IRC16M_DIV379                 CFG1_IRC16MDIV(378)                       /*!< IRC16M clock divided by 379 for system clock */
#define RCU_IRC16M_DIV380                 CFG1_IRC16MDIV(379)                       /*!< IRC16M clock divided by 380 for system clock */
#define RCU_IRC16M_DIV381                 CFG1_IRC16MDIV(380)                       /*!< IRC16M clock divided by 381 for system clock */
#define RCU_IRC16M_DIV382                 CFG1_IRC16MDIV(381)                       /*!< IRC16M clock divided by 382 for system clock */
#define RCU_IRC16M_DIV383                 CFG1_IRC16MDIV(382)                       /*!< IRC16M clock divided by 383 for system clock */
#define RCU_IRC16M_DIV384                 CFG1_IRC16MDIV(383)                       /*!< IRC16M clock divided by 384 for system clock */
#define RCU_IRC16M_DIV385                 CFG1_IRC16MDIV(384)                       /*!< IRC16M clock divided by 385 for system clock */
#define RCU_IRC16M_DIV386                 CFG1_IRC16MDIV(385)                       /*!< IRC16M clock divided by 386 for system clock */
#define RCU_IRC16M_DIV387                 CFG1_IRC16MDIV(386)                       /*!< IRC16M clock divided by 387 for system clock */
#define RCU_IRC16M_DIV388                 CFG1_IRC16MDIV(387)                       /*!< IRC16M clock divided by 388 for system clock */
#define RCU_IRC16M_DIV389                 CFG1_IRC16MDIV(388)                       /*!< IRC16M clock divided by 389 for system clock */
#define RCU_IRC16M_DIV390                 CFG1_IRC16MDIV(389)                       /*!< IRC16M clock divided by 390 for system clock */
#define RCU_IRC16M_DIV391                 CFG1_IRC16MDIV(390)                       /*!< IRC16M clock divided by 391 for system clock */
#define RCU_IRC16M_DIV392                 CFG1_IRC16MDIV(391)                       /*!< IRC16M clock divided by 392 for system clock */
#define RCU_IRC16M_DIV393                 CFG1_IRC16MDIV(392)                       /*!< IRC16M clock divided by 393 for system clock */
#define RCU_IRC16M_DIV394                 CFG1_IRC16MDIV(393)                       /*!< IRC16M clock divided by 394 for system clock */
#define RCU_IRC16M_DIV395                 CFG1_IRC16MDIV(394)                       /*!< IRC16M clock divided by 395 for system clock */
#define RCU_IRC16M_DIV396                 CFG1_IRC16MDIV(395)                       /*!< IRC16M clock divided by 396 for system clock */
#define RCU_IRC16M_DIV397                 CFG1_IRC16MDIV(396)                       /*!< IRC16M clock divided by 397 for system clock */
#define RCU_IRC16M_DIV398                 CFG1_IRC16MDIV(397)                       /*!< IRC16M clock divided by 398 for system clock */
#define RCU_IRC16M_DIV399                 CFG1_IRC16MDIV(398)                       /*!< IRC16M clock divided by 399 for system clock */
#define RCU_IRC16M_DIV400                 CFG1_IRC16MDIV(399)                       /*!< IRC16M clock divided by 400 for system clock */
#define RCU_IRC16M_DIV401                 CFG1_IRC16MDIV(400)                       /*!< IRC16M clock divided by 401 for system clock */
#define RCU_IRC16M_DIV402                 CFG1_IRC16MDIV(401)                       /*!< IRC16M clock divided by 402 for system clock */
#define RCU_IRC16M_DIV403                 CFG1_IRC16MDIV(402)                       /*!< IRC16M clock divided by 403 for system clock */
#define RCU_IRC16M_DIV404                 CFG1_IRC16MDIV(403)                       /*!< IRC16M clock divided by 404 for system clock */
#define RCU_IRC16M_DIV405                 CFG1_IRC16MDIV(404)                       /*!< IRC16M clock divided by 405 for system clock */
#define RCU_IRC16M_DIV406                 CFG1_IRC16MDIV(405)                       /*!< IRC16M clock divided by 406 for system clock */
#define RCU_IRC16M_DIV407                 CFG1_IRC16MDIV(406)                       /*!< IRC16M clock divided by 407 for system clock */
#define RCU_IRC16M_DIV408                 CFG1_IRC16MDIV(407)                       /*!< IRC16M clock divided by 408 for system clock */
#define RCU_IRC16M_DIV409                 CFG1_IRC16MDIV(408)                       /*!< IRC16M clock divided by 409 for system clock */
#define RCU_IRC16M_DIV410                 CFG1_IRC16MDIV(409)                       /*!< IRC16M clock divided by 410 for system clock */
#define RCU_IRC16M_DIV411                 CFG1_IRC16MDIV(410)                       /*!< IRC16M clock divided by 411 for system clock */
#define RCU_IRC16M_DIV412                 CFG1_IRC16MDIV(411)                       /*!< IRC16M clock divided by 412 for system clock */
#define RCU_IRC16M_DIV413                 CFG1_IRC16MDIV(412)                       /*!< IRC16M clock divided by 413 for system clock */
#define RCU_IRC16M_DIV414                 CFG1_IRC16MDIV(413)                       /*!< IRC16M clock divided by 414 for system clock */
#define RCU_IRC16M_DIV415                 CFG1_IRC16MDIV(414)                       /*!< IRC16M clock divided by 415 for system clock */
#define RCU_IRC16M_DIV416                 CFG1_IRC16MDIV(415)                       /*!< IRC16M clock divided by 416 for system clock */
#define RCU_IRC16M_DIV417                 CFG1_IRC16MDIV(416)                       /*!< IRC16M clock divided by 417 for system clock */
#define RCU_IRC16M_DIV418                 CFG1_IRC16MDIV(417)                       /*!< IRC16M clock divided by 418 for system clock */
#define RCU_IRC16M_DIV419                 CFG1_IRC16MDIV(418)                       /*!< IRC16M clock divided by 419 for system clock */
#define RCU_IRC16M_DIV420                 CFG1_IRC16MDIV(419)                       /*!< IRC16M clock divided by 420 for system clock */
#define RCU_IRC16M_DIV421                 CFG1_IRC16MDIV(420)                       /*!< IRC16M clock divided by 421 for system clock */
#define RCU_IRC16M_DIV422                 CFG1_IRC16MDIV(421)                       /*!< IRC16M clock divided by 422 for system clock */
#define RCU_IRC16M_DIV423                 CFG1_IRC16MDIV(422)                       /*!< IRC16M clock divided by 423 for system clock */
#define RCU_IRC16M_DIV424                 CFG1_IRC16MDIV(423)                       /*!< IRC16M clock divided by 424 for system clock */
#define RCU_IRC16M_DIV425                 CFG1_IRC16MDIV(424)                       /*!< IRC16M clock divided by 425 for system clock */
#define RCU_IRC16M_DIV426                 CFG1_IRC16MDIV(425)                       /*!< IRC16M clock divided by 426 for system clock */
#define RCU_IRC16M_DIV427                 CFG1_IRC16MDIV(426)                       /*!< IRC16M clock divided by 427 for system clock */
#define RCU_IRC16M_DIV428                 CFG1_IRC16MDIV(427)                       /*!< IRC16M clock divided by 428 for system clock */
#define RCU_IRC16M_DIV429                 CFG1_IRC16MDIV(428)                       /*!< IRC16M clock divided by 429 for system clock */
#define RCU_IRC16M_DIV430                 CFG1_IRC16MDIV(429)                       /*!< IRC16M clock divided by 430 for system clock */
#define RCU_IRC16M_DIV431                 CFG1_IRC16MDIV(430)                       /*!< IRC16M clock divided by 431 for system clock */
#define RCU_IRC16M_DIV432                 CFG1_IRC16MDIV(431)                       /*!< IRC16M clock divided by 432 for system clock */
#define RCU_IRC16M_DIV433                 CFG1_IRC16MDIV(432)                       /*!< IRC16M clock divided by 433 for system clock */
#define RCU_IRC16M_DIV434                 CFG1_IRC16MDIV(433)                       /*!< IRC16M clock divided by 434 for system clock */
#define RCU_IRC16M_DIV435                 CFG1_IRC16MDIV(434)                       /*!< IRC16M clock divided by 435 for system clock */
#define RCU_IRC16M_DIV436                 CFG1_IRC16MDIV(435)                       /*!< IRC16M clock divided by 436 for system clock */
#define RCU_IRC16M_DIV437                 CFG1_IRC16MDIV(436)                       /*!< IRC16M clock divided by 437 for system clock */
#define RCU_IRC16M_DIV438                 CFG1_IRC16MDIV(437)                       /*!< IRC16M clock divided by 438 for system clock */
#define RCU_IRC16M_DIV439                 CFG1_IRC16MDIV(438)                       /*!< IRC16M clock divided by 439 for system clock */
#define RCU_IRC16M_DIV440                 CFG1_IRC16MDIV(439)                       /*!< IRC16M clock divided by 440 for system clock */
#define RCU_IRC16M_DIV441                 CFG1_IRC16MDIV(440)                       /*!< IRC16M clock divided by 441 for system clock */
#define RCU_IRC16M_DIV442                 CFG1_IRC16MDIV(441)                       /*!< IRC16M clock divided by 442 for system clock */
#define RCU_IRC16M_DIV443                 CFG1_IRC16MDIV(442)                       /*!< IRC16M clock divided by 443 for system clock */
#define RCU_IRC16M_DIV444                 CFG1_IRC16MDIV(443)                       /*!< IRC16M clock divided by 444 for system clock */
#define RCU_IRC16M_DIV445                 CFG1_IRC16MDIV(444)                       /*!< IRC16M clock divided by 445 for system clock */
#define RCU_IRC16M_DIV446                 CFG1_IRC16MDIV(445)                       /*!< IRC16M clock divided by 446 for system clock */
#define RCU_IRC16M_DIV447                 CFG1_IRC16MDIV(446)                       /*!< IRC16M clock divided by 447 for system clock */
#define RCU_IRC16M_DIV448                 CFG1_IRC16MDIV(447)                       /*!< IRC16M clock divided by 448 for system clock */
#define RCU_IRC16M_DIV449                 CFG1_IRC16MDIV(448)                       /*!< IRC16M clock divided by 449 for system clock */
#define RCU_IRC16M_DIV450                 CFG1_IRC16MDIV(449)                       /*!< IRC16M clock divided by 450 for system clock */
#define RCU_IRC16M_DIV451                 CFG1_IRC16MDIV(450)                       /*!< IRC16M clock divided by 451 for system clock */
#define RCU_IRC16M_DIV452                 CFG1_IRC16MDIV(451)                       /*!< IRC16M clock divided by 452 for system clock */
#define RCU_IRC16M_DIV453                 CFG1_IRC16MDIV(452)                       /*!< IRC16M clock divided by 453 for system clock */
#define RCU_IRC16M_DIV454                 CFG1_IRC16MDIV(453)                       /*!< IRC16M clock divided by 454 for system clock */
#define RCU_IRC16M_DIV455                 CFG1_IRC16MDIV(454)                       /*!< IRC16M clock divided by 455 for system clock */
#define RCU_IRC16M_DIV456                 CFG1_IRC16MDIV(455)                       /*!< IRC16M clock divided by 456 for system clock */
#define RCU_IRC16M_DIV457                 CFG1_IRC16MDIV(456)                       /*!< IRC16M clock divided by 457 for system clock */
#define RCU_IRC16M_DIV458                 CFG1_IRC16MDIV(457)                       /*!< IRC16M clock divided by 458 for system clock */
#define RCU_IRC16M_DIV459                 CFG1_IRC16MDIV(458)                       /*!< IRC16M clock divided by 459 for system clock */
#define RCU_IRC16M_DIV460                 CFG1_IRC16MDIV(459)                       /*!< IRC16M clock divided by 460 for system clock */
#define RCU_IRC16M_DIV461                 CFG1_IRC16MDIV(460)                       /*!< IRC16M clock divided by 461 for system clock */
#define RCU_IRC16M_DIV462                 CFG1_IRC16MDIV(461)                       /*!< IRC16M clock divided by 462 for system clock */
#define RCU_IRC16M_DIV463                 CFG1_IRC16MDIV(462)                       /*!< IRC16M clock divided by 463 for system clock */
#define RCU_IRC16M_DIV464                 CFG1_IRC16MDIV(463)                       /*!< IRC16M clock divided by 464 for system clock */
#define RCU_IRC16M_DIV465                 CFG1_IRC16MDIV(464)                       /*!< IRC16M clock divided by 465 for system clock */
#define RCU_IRC16M_DIV466                 CFG1_IRC16MDIV(465)                       /*!< IRC16M clock divided by 466 for system clock */
#define RCU_IRC16M_DIV467                 CFG1_IRC16MDIV(466)                       /*!< IRC16M clock divided by 467 for system clock */
#define RCU_IRC16M_DIV468                 CFG1_IRC16MDIV(467)                       /*!< IRC16M clock divided by 468 for system clock */
#define RCU_IRC16M_DIV469                 CFG1_IRC16MDIV(468)                       /*!< IRC16M clock divided by 469 for system clock */
#define RCU_IRC16M_DIV470                 CFG1_IRC16MDIV(469)                       /*!< IRC16M clock divided by 470 for system clock */
#define RCU_IRC16M_DIV471                 CFG1_IRC16MDIV(470)                       /*!< IRC16M clock divided by 471 for system clock */
#define RCU_IRC16M_DIV472                 CFG1_IRC16MDIV(471)                       /*!< IRC16M clock divided by 472 for system clock */
#define RCU_IRC16M_DIV473                 CFG1_IRC16MDIV(472)                       /*!< IRC16M clock divided by 473 for system clock */
#define RCU_IRC16M_DIV474                 CFG1_IRC16MDIV(473)                       /*!< IRC16M clock divided by 474 for system clock */
#define RCU_IRC16M_DIV475                 CFG1_IRC16MDIV(474)                       /*!< IRC16M clock divided by 475 for system clock */
#define RCU_IRC16M_DIV476                 CFG1_IRC16MDIV(475)                       /*!< IRC16M clock divided by 476 for system clock */
#define RCU_IRC16M_DIV477                 CFG1_IRC16MDIV(476)                       /*!< IRC16M clock divided by 477 for system clock */
#define RCU_IRC16M_DIV478                 CFG1_IRC16MDIV(477)                       /*!< IRC16M clock divided by 478 for system clock */
#define RCU_IRC16M_DIV479                 CFG1_IRC16MDIV(478)                       /*!< IRC16M clock divided by 479 for system clock */
#define RCU_IRC16M_DIV480                 CFG1_IRC16MDIV(479)                       /*!< IRC16M clock divided by 480 for system clock */
#define RCU_IRC16M_DIV481                 CFG1_IRC16MDIV(480)                       /*!< IRC16M clock divided by 481 for system clock */
#define RCU_IRC16M_DIV482                 CFG1_IRC16MDIV(481)                       /*!< IRC16M clock divided by 482 for system clock */
#define RCU_IRC16M_DIV483                 CFG1_IRC16MDIV(482)                       /*!< IRC16M clock divided by 483 for system clock */
#define RCU_IRC16M_DIV484                 CFG1_IRC16MDIV(483)                       /*!< IRC16M clock divided by 484 for system clock */
#define RCU_IRC16M_DIV485                 CFG1_IRC16MDIV(484)                       /*!< IRC16M clock divided by 485 for system clock */
#define RCU_IRC16M_DIV486                 CFG1_IRC16MDIV(485)                       /*!< IRC16M clock divided by 486 for system clock */
#define RCU_IRC16M_DIV487                 CFG1_IRC16MDIV(486)                       /*!< IRC16M clock divided by 487 for system clock */
#define RCU_IRC16M_DIV488                 CFG1_IRC16MDIV(487)                       /*!< IRC16M clock divided by 488 for system clock */
#define RCU_IRC16M_DIV489                 CFG1_IRC16MDIV(488)                       /*!< IRC16M clock divided by 489 for system clock */
#define RCU_IRC16M_DIV490                 CFG1_IRC16MDIV(489)                       /*!< IRC16M clock divided by 490 for system clock */
#define RCU_IRC16M_DIV491                 CFG1_IRC16MDIV(490)                       /*!< IRC16M clock divided by 491 for system clock */
#define RCU_IRC16M_DIV492                 CFG1_IRC16MDIV(491)                       /*!< IRC16M clock divided by 492 for system clock */
#define RCU_IRC16M_DIV493                 CFG1_IRC16MDIV(492)                       /*!< IRC16M clock divided by 493 for system clock */
#define RCU_IRC16M_DIV494                 CFG1_IRC16MDIV(493)                       /*!< IRC16M clock divided by 494 for system clock */
#define RCU_IRC16M_DIV495                 CFG1_IRC16MDIV(494)                       /*!< IRC16M clock divided by 495 for system clock */
#define RCU_IRC16M_DIV496                 CFG1_IRC16MDIV(495)                       /*!< IRC16M clock divided by 496 for system clock */
#define RCU_IRC16M_DIV497                 CFG1_IRC16MDIV(496)                       /*!< IRC16M clock divided by 497 for system clock */
#define RCU_IRC16M_DIV498                 CFG1_IRC16MDIV(497)                       /*!< IRC16M clock divided by 498 for system clock */
#define RCU_IRC16M_DIV499                 CFG1_IRC16MDIV(498)                       /*!< IRC16M clock divided by 499 for system clock */
#define RCU_IRC16M_DIV500                 CFG1_IRC16MDIV(499)                       /*!< IRC16M clock divided by 500 for system clock */
#define RCU_IRC16M_DIV501                 CFG1_IRC16MDIV(500)                       /*!< IRC16M clock divided by 501 for system clock */
#define RCU_IRC16M_DIV502                 CFG1_IRC16MDIV(501)                       /*!< IRC16M clock divided by 502 for system clock */
#define RCU_IRC16M_DIV503                 CFG1_IRC16MDIV(502)                       /*!< IRC16M clock divided by 503 for system clock */
#define RCU_IRC16M_DIV504                 CFG1_IRC16MDIV(503)                       /*!< IRC16M clock divided by 504 for system clock */
#define RCU_IRC16M_DIV505                 CFG1_IRC16MDIV(504)                       /*!< IRC16M clock divided by 505 for system clock */
#define RCU_IRC16M_DIV506                 CFG1_IRC16MDIV(505)                       /*!< IRC16M clock divided by 506 for system clock */
#define RCU_IRC16M_DIV507                 CFG1_IRC16MDIV(506)                       /*!< IRC16M clock divided by 507 for system clock */
#define RCU_IRC16M_DIV508                 CFG1_IRC16MDIV(507)                       /*!< IRC16M clock divided by 508 for system clock */
#define RCU_IRC16M_DIV509                 CFG1_IRC16MDIV(508)                       /*!< IRC16M clock divided by 509 for system clock */
#define RCU_IRC16M_DIV510                 CFG1_IRC16MDIV(509)                       /*!< IRC16M clock divided by 510 for system clock */
#define RCU_IRC16M_DIV511                 CFG1_IRC16MDIV(510)                       /*!< IRC16M clock divided by 511 for system clock */
#define RCU_IRC16M_DIV512                 CFG1_IRC16MDIV(511)                       /*!< IRC16M clock divided by 512 for system clock */

/* Deep-sleep mode voltage */
#define DSV_DSLPVS(regval)               (BITS(0,2) & ((uint32_t)(regval) << 0U))
#define RCU_DEEPSLEEP_V_1_1               DSV_DSLPVS(0)                             /*!< core voltage is 1.1V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_1_0               DSV_DSLPVS(1)                             /*!< core voltage is 1.0V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_0_9               DSV_DSLPVS(2)                             /*!< core voltage is 0.9V in deep-sleep mode */
#define RCU_DEEPSLEEP_V_0_8               DSV_DSLPVS(3)                             /*!< core voltage is 0.8V in deep-sleep mode */

/* function declarations */
/* deinitialize the RCU */
void rcu_deinit(void);
/* enable the peripherals clock */
void rcu_periph_clock_enable(rcu_periph_enum periph);
/* disable the peripherals clock */
void rcu_periph_clock_disable(rcu_periph_enum periph);
/* enable the peripherals clock when sleep mode */
void rcu_periph_clock_sleep_enable(rcu_periph_sleep_enum periph);
/* disable the peripherals clock when sleep mode */
void rcu_periph_clock_sleep_disable(rcu_periph_sleep_enum periph);
/* reset the peripherals */
void rcu_periph_reset_enable(rcu_periph_reset_enum periph_reset);
/* disable reset the peripheral */
void rcu_periph_reset_disable(rcu_periph_reset_enum periph_reset);
/* reset the BKP */
void rcu_bkp_reset_enable(void);
/* disable the BKP reset */
void rcu_bkp_reset_disable(void);
/* enable HXTAL for PLLI2S */
void rcu_hxtal_plli2s_enable(void);
/* disable HXTAL for PLLI2S */
void rcu_hxtal_plli2s_disable(void);
/* enable HXTAL for system CK_PLLP */
void rcu_hxtal_pllp_enable(void);
/* disable HXTAL for system CK_PLLP */
void rcu_hxtal_pllp_disable(void);
/* enable CK_PLLDIG */
void rcu_plldig_enable(void);
/* disable CK_PLLDIG */
void rcu_plldig_disable(void);
/* power on the HXTAL */
void rcu_hxtal_poweron(void);
/* power down the HXTAL */
void rcu_hxtal_powerdown(void);
/* power on the PLLDIG */
void rcu_plldig_poweron(void);
/*  power down the PLLDIG */
void rcu_plldig_powerdown(void);
/* enable the RF PLL calculation */
void rcu_rfpll_cal_enable(void);
/* disable the RF PLL calculation */
void rcu_rfpll_cal_disable(void);
/* power on the RF PLL */
void rcu_rfpll_poweron(void);
/* power down the RF PLL */
void rcu_rfpll_powerdown(void);
/* power on LDO analog */
void rcu_ldoana_poweron(void);
/* power down LDO analog */
void rcu_ldoana_powerdown(void);
/* power on the LDO clock */
void rcu_ldoclk_poweron(void);
/* power down the LDO clock */
void rcu_ldoclk_powerdown(void);
/* power on the BandGap */
void rcu_bandgap_poweron(void);
/* power down the BandGap */
void rcu_bandgap_powerdown(void);

/* configure the system clock source */
void rcu_system_clock_source_config(uint32_t ck_sys);
/* get the system clock source */
uint32_t rcu_system_clock_source_get(void);
/* configure the AHB prescaler selection */
void rcu_ahb_clock_config(uint32_t ck_ahb);
/* configure the APB1 prescaler selection */
void rcu_apb1_clock_config(uint32_t ck_apb1);
/* configure the APB2 prescaler selection */
void rcu_apb2_clock_config(uint32_t ck_apb2);
/* configure the CK_OUT0 clock source and divider */
void rcu_ckout0_config(uint32_t ckout0_src, uint32_t ckout0_div);
/* configure the CK_OUT1 clock source and divider */
void rcu_ckout1_config(uint32_t ckout1_src, uint32_t ckout1_div);
/* configure the PLL clock source selection and PLL multiply factor */
ErrStatus rcu_pll_config(uint32_t pll_src, uint32_t pll_psc, uint32_t pll_n, uint32_t pll_p);
/* configure the PLLI2S clock */
ErrStatus rcu_plli2s_config(uint32_t plli2s_n, uint32_t plli2s_psc,uint32_t plli2s_div);
/* configure the PLLDIG clock */
void rcu_plldig_config(uint32_t plldig_clk);
/* configure PLLDIG clock divider factor for system clock */
void rcu_plldigfsys_div_config(uint32_t plldigfsys_div);
/* configure the RTC clock source selection */
void rcu_rtc_clock_config(uint32_t rtc_clock_source);
/* configure the frequency division of RTC clock when HXTAL was selected as its clock source */
void rcu_rtc_div_config(uint32_t rtc_div);
/* configure the I2S clock source selection */
void rcu_i2s_clock_config(uint32_t i2s_clock_source);
/* configure the PLL divider factor for I2S clock */
void rcu_pllfi2s_clock_div_config(uint32_t pllfi2s_div);
#if (defined(GD32W515PI) || defined(GD32W515P0))
/* configure the HPDF clock source selection */
void rcu_hpdf_clock_config(uint32_t hpdf_clock_source);
/* configure the HPDF AUDIO clock source selection */
void rcu_hpdf_audio_clock_config(uint32_t hpdfaudio_clock_source);
#endif /* GD32W515PI and GD32W515P0 */
/* configure the SDIO clock source selection */
void rcu_sdio_clock_config(uint32_t sdio_clock_source);
/* configure the frequency division of the sdio source clock */
void rcu_sdio_div_config(uint32_t sdio_div);
/* configure the usb clock source selection */
void rcu_usbfs_clock_config(uint32_t usbfs_clock_source);
/* configure the frequency division of the usbfs source clock */
void rcu_usbfs_div_config(uint32_t usbfs_div);
/* configure the I2C0 clock source selection */
void rcu_i2c0_clock_config(uint32_t i2c0_clock_source);
/* configure the USART0 clock source selection */
void rcu_usart0_clock_config(uint32_t usart0_clock_source);
/* configure the USART2 clock source selection */
void rcu_usart2_clock_config(uint32_t usart2_clock_source);
/* configure IRC16M clock divider factor for system clock */
void rcu_irc16m_div_config(uint32_t irc16m_div);
/* configure the TIMER clock prescaler selection */
void rcu_timer_clock_prescaler_config(uint32_t timer_clock_prescaler);       

/* get the clock stabilization and periphral reset flags */
FlagStatus rcu_flag_get(rcu_flag_enum flag);
/* clear the reset flag */
void rcu_all_reset_flag_clear(void);
/* get the clock stabilization interrupt and ckm flags */
FlagStatus rcu_interrupt_flag_get(rcu_int_flag_enum int_flag);
/* clear the interrupt flags */
void rcu_interrupt_flag_clear(rcu_int_flag_clear_enum int_flag);
/* enable the stabilization interrupt */
void rcu_interrupt_enable(rcu_int_enum interrupt);
/* disable the stabilization interrupt */
void rcu_interrupt_disable(rcu_int_enum interrupt);

/* configure the LXTAL drive capability */
void rcu_lxtal_drive_capability_config(uint32_t lxtal_dricap);
/* wait for oscillator stabilization flags is SET or oscillator startup is timeout */
ErrStatus rcu_osci_stab_wait(rcu_osci_type_enum osci);
/* turn on the oscillator */
void rcu_osci_on(rcu_osci_type_enum osci);
/* turn off the oscillator */
void rcu_osci_off(rcu_osci_type_enum osci);
/* enable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it */
void rcu_osci_bypass_mode_enable(rcu_osci_type_enum osci);
/* disable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it */
void rcu_osci_bypass_mode_disable(rcu_osci_type_enum osci);
/* enable the HXTAL clock monitor */
void rcu_hxtal_clock_monitor_enable(void);
/* disable the HXTAL clock monitor */
void rcu_hxtal_clock_monitor_disable(void);
/* enable the RF HXTAL clock monitor */
void rcu_rf_hxtal_clock_monitor_enable(void);
/* disable the RF HXTAL clock monitor */
void rcu_rf_hxtal_clock_monitor_disable(void);

/* set the IRC16M adjust value */
void rcu_irc16m_adjust_value_set(uint32_t irc16m_adjval);
/* configure the spread spectrum modulation for the main PLL clock */
void rcu_spread_spectrum_config(uint32_t spread_spectrum_type, uint32_t modstep, uint32_t modcnt);
/* enable the spread spectrum modulation for the main PLL clock */
void rcu_spread_spectrum_enable(void);
/* disable the spread spectrum modulation for the main PLL clock */
void rcu_spread_spectrum_disable(void);          
/* unlock the voltage key */
void rcu_voltage_key_unlock(void);
/* set the deep sleep mode voltage */
void rcu_deepsleep_voltage_set(uint32_t dsvol);

/* enable the security attribution */
void rcu_security_enable(rcu_sec_enum security);
/* disable the security attribution */
void rcu_security_disable(rcu_sec_enum security);
/* enable the privileged access */
void rcu_privilege_enable(void);
/* disable the privileged access */
void rcu_privilege_disable(void);
/* get the secure status flag */
FlagStatus rcu_security_flag_get(rcu_sec_flag_enum sec_flag);
/* get the system clock, bus and peripheral clock frequency */
uint32_t rcu_clock_freq_get(rcu_clock_freq_enum clock);
#endif /* GD32W51X_RCU_H */
