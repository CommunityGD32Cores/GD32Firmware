/*!
    \file    gd32w51x_rcu.c
    \brief   RCU driver

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

#include "gd32w51x_rcu.h"

/* define clock source */
#define SEL_IRC16M                  0x00U                            /* IRC16M is selected as CK_SYS */
#define SEL_HXTAL                   0x01U                            /* HXTAL is selected as CK_SYS */
#define SEL_PLLP                    0x02U                            /* PLLP is selected as CK_SYS */
#define SEL_PLLDIG                  0x03U                            /* PLLDIG is selected as CK_SYS */

/* define startup timeout count */
#define OSC_STARTUP_TIMEOUT         ((uint32_t)0x000fffffU)
#define LXTAL_STARTUP_TIMEOUT       ((uint32_t)0x0fffffffU)

/* RCU IRC16M adjust value mask and offset*/
#define RCU_IRC16M_ADJUST_MASK      ((uint32_t)0x0000001FU)
#define RCU_IRC16M_ADJUST_OFFSET    ((uint32_t)0x00000003U)

/*!
    \brief      deinitialize the RCU
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_deinit(void)
{
    /* enable IRC16M */
    RCU_CTL |= RCU_CTL_IRC16MEN;
    rcu_osci_stab_wait(RCU_IRC16M);
    RCU_CFG0 &= ~RCU_CFG0_SCS;
    /* reset CTL register */
    RCU_CTL &= ~(RCU_CTL_HXTALEN | RCU_CTL_CKMEN | RCU_CTL_PLLEN | RCU_CTL_PLLI2SEN | RCU_CTL_PLLDIGEN |
                 RCU_CTL_HXTALPU | RCU_CTL_HXTALENI2S | RCU_CTL_PLLDIGPU | RCU_CTL_HXTALENPLL | RCU_CTL_RFCKMEN);
    /* reset CFG0 register */
    RCU_CFG0 &= ~(RCU_CFG0_SCS | RCU_CFG0_AHBPSC | RCU_CFG0_APB1PSC | RCU_CFG0_APB2PSC |
                  RCU_CFG0_RTCDIV | RCU_CFG0_CKOUT0SEL  | RCU_CFG0_CKOUT0DIV |
                  RCU_CFG0_CKOUT1DIV | RCU_CFG0_CKOUT1SEL);
    /* reset CFG1 register */
    RCU_CFG1 = 0x00000000U;
    /* reset ADDCTL register */
    RCU_ADDCTL = 0x00000000U;
    /* reset PLL register */
    RCU_PLL = 0x00003010U;
    /* reset PLLCFG register */
    RCU_PLLCFG = 0x03000000U;
    /* reset INT register */
    RCU_INT = 0x00000000U;
}

/*!
    \brief      enable the peripherals clock
    \param[in]  periph: RCU peripherals, refer to rcu_periph_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_GPIOx (x=A,B,C): GPIO ports clock
      \arg        RCU_TZPCU:TZPCU clock
      \arg        RCU_TSI: TSI clock
      \arg        RCU_CRC: CRC clock
      \arg        RCU_WIFI: WIFI clock
      \arg        RCU_WIFIRUN: WIFIRUN clock
      \arg        RCU_SRAMx (x=0,1,2,3):SRAM clock
      \arg        RCU_DMAx (x=0,1): DMA clock
      \arg        RCU_USBFS: USBFS clock
      \arg        RCU_DCI (not available for GD32W515TX series): DCI clock
      \arg        RCU_PKCAU: PKCAU clock
      \arg        RCU_CAU: CAU clock
      \arg        RCU_HAU: HAU clock
      \arg        RCU_TRNG: TRNG clock
      \arg        RCU_SQPI: SQPI clock
      \arg        RCU_QSPI: QSPI clock
      \arg        RCU_TIMERx (x=0,1,2,3,4,5,15,16,TIMER3 are not available for GD32W515TX series): TIMER clock
      \arg        RCU_WWDGT: WWDGT clock
      \arg        RCU_SPIx (x=0,1): SPI clock
      \arg        RCU_USARTx (x=0,1,2): USART clock
      \arg        RCU_I2Cx (x=0,1): I2C clock
      \arg        RCU_PMU: PMU clock
      \arg        RCU_RTC: RTC clock
      \arg        RCU_ADC : ADC clock
      \arg        RCU_SDIO: SDIO clock
      \arg        RCU_SYSCFG: SYSCFG clock
      \arg        RCU_HPDF (not available for GD32W515TX series): HPDF clock
      \arg        RCU_RF: RF clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_enable(rcu_periph_enum periph)
{
    RCU_REG_VAL(periph) |= BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      disable the peripherals clock
    \param[in]  periph: RCU peripherals, refer to rcu_periph_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_GPIOx (x=A,B,C): GPIO ports clock
      \arg        RCU_TZPCU:TZPCU clock
      \arg        RCU_TSI: TSI clock
      \arg        RCU_CRC: CRC clock
      \arg        RCU_WIFI: WIFI clock
      \arg        RCU_WIFIRUN: WIFIRUN clock
      \arg        RCU_SRAMx: (x=0,1,2,3):SRAM clock
      \arg        RCU_DMAx (x=0,1): DMA clock
      \arg        RCU_USBFS: USBFS clock
      \arg        RCU_DCI (not available for GD32W515TX series): DCI clock
      \arg        RCU_PKCAU: PKCAU clock
      \arg        RCU_CAU: CAU clock
      \arg        RCU_HAU: HAU clock
      \arg        RCU_TRNG: TRNG clock
      \arg        RCU_SQPI: SQPI clock
      \arg        RCU_QSPI: QSPI clock
      \arg        RCU_TIMERx (x=0,1,2,3,4,5,15,16,TIMER3 are not available for GD32W515TX series): TIMER clock
      \arg        RCU_WWDGT: WWDGT clock
      \arg        RCU_SPIx (x=0,1): SPI clock
      \arg        RCU_USARTx (x=0,1,2): USART clock
      \arg        RCU_I2Cx (x=0,1): I2C clock
      \arg        RCU_PMU: PMU clock
      \arg        RCU_RTC: RTC clock
      \arg        RCU_ADC: ADC clock
      \arg        RCU_SDIO: SDIO clock
      \arg        RCU_SYSCFG: SYSCFG clock
      \arg        RCU_HPDF (not available for GD32W515TX series): HPDF clock
      \arg        RCU_RF: RF clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_disable(rcu_periph_enum periph)
{
    RCU_REG_VAL(periph) &= ~BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      enable the peripherals clock when sleep mode
    \param[in]  periph: RCU peripherals, refer to rcu_periph_sleep_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_GPIOx_SLP (x=A,B,C): GPIO ports clock
      \arg        RCU_TZPCU_SLP: TZPCU clock
      \arg        RCU_TSI_SLP: TSI clock
      \arg        RCU_CRC_SLP: CRC clock
      \arg        RCU_WIFI_SLP: WIFI clock
      \arg        RCU_WIFIRUN_SLP: WIFIRUN clock
      \arg        RCU_FMC_SLP: FMC clock
      \arg        RCU_SRAMx_SLP:(x=0,1,2,3): SRAM clock
      \arg        RCU_DMAx_SLP (x=0,1): DMA clock
      \arg        RCU_USBFS_SLP: USBFS clock
      \arg        RCU_DCI_SLP (not available for GD32W515TX series): DCI clock
      \arg        RCU_PKCAU_SLP: PKCAU clock
      \arg        RCU_CAU_SLP: CAU clock
      \arg        RCU_HAU_SLP: HAU clock
      \arg        RCU_TRNG_SLP: TRNG clock
      \arg        RCU_SQPI_SLP: SQPI clock
      \arg        RCU_QSPI_SLP: QSPI clock
      \arg        RCU_TIMERx_SLP (x=0,1,2,3,4,5,15,16,TIMER3 are not available for GD32W515TX series): TIMER clock
      \arg        RCU_WWDGT_SLP: WWDGT clock
      \arg        RCU_SPIx_SLP (x=0,1): SPI clock
      \arg        RCU_USARTx_SLP (x=0,1,2): USART clock
      \arg        RCU_I2Cx_SLP (x=0,1): I2C clock
      \arg        RCU_PMU_SLP: PMU clock
      \arg        RCU_RTC_SLP: RTC clock
      \arg        RCU_ADC_SLP: ADC clock
      \arg        RCU_SDIO_SLP: SDIO clock
      \arg        RCU_SYSCFG_SLP: SYSCFG clock
      \arg        RCU_HPDF_SLP:(not available for GD32W515TX series): HPDF clock
      \arg        RCU_RF_SLP: RF clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_sleep_enable(rcu_periph_sleep_enum periph)
{
    RCU_REG_VAL(periph) |= BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      disable the peripherals clock when sleep mode
    \param[in]  periph: RCU peripherals, refer to rcu_periph_sleep_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_GPIOx_SLP (x=A,B,C): GPIO ports clock
      \arg        RCU_TZPCU_SLP: TZPCU clock
      \arg        RCU_TSI_SLP: TSI clock
      \arg        RCU_CRC_SLP: CRC clock
      \arg        RCU_WIFI_SLP: WIFI clock
      \arg        RCU_WIFIRUN_SLP: WIFIRUN clock
      \arg        RCU_FMC_SLP: WIFI clock
      \arg        RCU_SRAMx_SLP:(x=0,1,2,3): SRAM clock
      \arg        RCU_DMAx_SLP (x=0,1): DMA clock
      \arg        RCU_USBFS_SLP: USBFS clock
      \arg        RCU_DCI_SLP (not available for GD32W515TX series): DCI clock
      \arg        RCU_PKCAU_SLP: PKCAU clock
      \arg        RCU_CAU_SLP: CAU clock
      \arg        RCU_HAU_SLP: HAU clock
      \arg        RCU_TRNG_SLP: TRNG clock
      \arg        RCU_SQPI_SLP: SQPI clock
      \arg        RCU_QSPI_SLP: QSPI clock
      \arg        RCU_TIMERx_SLP (x=0,1,2,3,4,5,15,16,TIMER3 are not available for GD32W515TX series): TIMER clock
      \arg        RCU_WWDGT_SLP: WWDGT clock
      \arg        RCU_SPIx_SLP (x=0,1): SPI clock
      \arg        RCU_USARTx_SLP (x=0,1,2): USART clock
      \arg        RCU_I2Cx_SLP (x=0,1): I2C clock
      \arg        RCU_PMU_SLP: PMU clock
      \arg        RCU_RTC_SLP: RTC clock
      \arg        RCU_ADC_SLP: ADC clock
      \arg        RCU_SDIO_SLP: SDIO clock
      \arg        RCU_SYSCFG_SLP: SYSCFG clock
      \arg        RCU_HPDF_SLP:(not available for GD32W515TX series):HPDF clock
      \arg        RCU_RF_SLP: RF clock
    \param[out] none
    \retval     none
*/
void rcu_periph_clock_sleep_disable(rcu_periph_sleep_enum periph)
{
    RCU_REG_VAL(periph) &= ~BIT(RCU_BIT_POS(periph));
}

/*!
    \brief      reset the peripherals
    \param[in]  periph_reset: RCU peripherals reset, refer to rcu_periph_reset_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_GPIOxRST (x=A,B,C): reset GPIO ports
      \arg        RCU_TZPCURST: reset TZPCU
      \arg        RCU_TSIRST: reset TSI
      \arg        RCU_CRCRST: reset CRC
      \arg        RCU_WIFIRST: reset WIFI
      \arg        RCU_DMAxRST (x=0,1): reset DMA
      \arg        RCU_USBFSRST: reset USBFS
      \arg        RCU_DCIRST (not available for GD32W515TX series): reset DCI
      \arg        RCU_PKCAURST: reset PKCAU
      \arg        RCU_CAURST: reset CAU
      \arg        RCU_HAURST: reset HAU
      \arg        RCU_TRNGRST: reset TRNG
      \arg        RCU_SQPIRST: reset SQPI
      \arg        RCU_QSPIRST: reset QSPI
      \arg        RCU_TIMERxRST (x=0,1,2,3,4,5,15,16,TIMER3 are not available for GD32W515TX series): reset TIMER
      \arg        RCU_WWDGTRST: reset WWDGT
      \arg        RCU_SPIxRST (x=0,1): reset SPI
      \arg        RCU_USARTxRST (x=0,1,2): reset USART
      \arg        RCU_I2CxRST (x=0,1): reset I2C
      \arg        RCU_PMURST: reset PMU
      \arg        RCU_ADCRST : reset ADC
      \arg        RCU_SDIORST: reset SDIO
      \arg        RCU_SYSCFGRST: reset SYSCFG
      \arg        RCU_HPDFRST (not available for GD32W515TX series): reset HPDF
      \arg        RCU_RFRST: reset RF
    \param[out] none
    \retval     none
*/
void rcu_periph_reset_enable(rcu_periph_reset_enum periph_reset)
{
    RCU_REG_VAL(periph_reset) |= BIT(RCU_BIT_POS(periph_reset));
}

/*!
    \brief      disable reset the peripheral
    \param[in]  periph_reset: RCU peripherals reset, refer to rcu_periph_reset_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_GPIOxRST (x=A,B,C): reset GPIO ports
      \arg        RCU_TZPCURST: reset TZPCU
      \arg        RCU_TSIRST: reset TSI
      \arg        RCU_CRCRST: reset CRC
      \arg        RCU_WIFIRST: reset WIFI
      \arg        RCU_DMAxRST (x=0,1): reset DMA
      \arg        RCU_USBFSRST: reset USBFS
      \arg        RCU_DCIRST (not available for GD32W515TX series): reset DCI
      \arg        RCU_PKCAURST: reset PKCAU
      \arg        RCU_CAURST: reset CAU
      \arg        RCU_HAURST: reset HAU
      \arg        RCU_TRNGRST: reset TRNG
      \arg        RCU_SQPIRST: reset SQPI
      \arg        RCU_QSPIRST: reset QSPI
      \arg        RCU_TIMERxRST (x=0,1,2,3,4,5,15,16,TIMER3 are not available for GD32W515TX series): reset TIMER
      \arg        RCU_WWDGTRST: reset WWDGT
      \arg        RCU_SPIxRST (x=0,1): reset SPI
      \arg        RCU_USARTxRST (x=0,1,2): reset USART
      \arg        RCU_I2CxRST (x=0,1): reset I2C
      \arg        RCU_PMURST: reset PMU
      \arg        RCU_ADCRST : reset ADC
      \arg        RCU_SDIORST: reset SDIO
      \arg        RCU_SYSCFGRST: reset SYSCFG
      \arg        RCU_HPDFRST (not available for GD32W515TX series): reset HPDF
      \arg        RCU_RFRST: reset RF
    \param[out] none
    \retval     none
*/
void rcu_periph_reset_disable(rcu_periph_reset_enum periph_reset)
{
    RCU_REG_VAL(periph_reset) &= ~BIT(RCU_BIT_POS(periph_reset));
}

/*!
    \brief      reset the BKP
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_bkp_reset_enable(void)
{
    RCU_BDCTL |= RCU_BDCTL_BKPRST;
}

/*!
    \brief      disable the BKP reset
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_bkp_reset_disable(void)
{
    RCU_BDCTL &= ~RCU_BDCTL_BKPRST;
}

/*!
    \brief      enable HXTAL for PLLI2S
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_plli2s_enable(void)
{
    RCU_CTL |= RCU_CTL_HXTALENI2S;
}

/*!
    \brief      disable HXTAL for PLLI2S
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_plli2s_disable(void)
{
    RCU_CTL &= ~RCU_CTL_HXTALENI2S;
}

/*!
    \brief      enable HXTAL for system CK_PLLP
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_pllp_enable(void)
{
    RCU_CTL |= RCU_CTL_HXTALENPLL;
}

/*!
    \brief      disable HXTAL for system CK_PLLP
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_pllp_disable(void)
{
    RCU_CTL &= ~RCU_CTL_HXTALENPLL;
}

/*!
    \brief      enable CK_PLLDIG
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_plldig_enable(void)
{
    RCU_CTL |= RCU_CTL_PLLDIGEN;
}

/*!
    \brief      disable CK_PLLDIG
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_plldig_disable(void)
{
    RCU_CTL &= ~RCU_CTL_PLLDIGEN;
}

/*!
    \brief      power on the HXTAL
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_poweron(void)
{
    RCU_CTL |= RCU_CTL_HXTALPU;
}

/*!
    \brief      power down the HXTAL 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_powerdown(void)
{
    RCU_CTL &= ~RCU_CTL_HXTALPU;
}

/*!
    \brief      power on the PLLDIG 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_plldig_poweron(void)
{
    RCU_CTL |= RCU_CTL_PLLDIGPU;
}

/*!
    \brief      power down the PLLDIG
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_plldig_powerdown(void)
{
    RCU_CTL &= ~RCU_CTL_PLLDIGPU;
}

/*!
    \brief      enable the RF PLL calculation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_rfpll_cal_enable(void)
{
    RCU_CFG1 |= RCU_CFG1_RFPLLCALEN;
}

/*!
    \brief      disable the RF PLL calculation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_rfpll_cal_disable(void)
{
    RCU_CFG1 &= ~RCU_CFG1_RFPLLCALEN;
}

/*!
    \brief      power on the RF PLL 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_rfpll_poweron(void)
{
    RCU_CFG1 |= RCU_CFG1_RFPLLPU;
}

/*!
    \brief      power down the RF PLL 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_rfpll_powerdown(void)
{
    RCU_CFG1 &= ~RCU_CFG1_RFPLLPU;
}

/*!
    \brief      power on LDO analog 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_ldoana_poweron(void)
{
    RCU_CFG1 |= RCU_CFG1_LDOANAPU;
}

/*!
    \brief      power down LDO analog
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_ldoana_powerdown(void)
{
    RCU_CFG1 &= ~RCU_CFG1_LDOANAPU;
}

/*!
    \brief      power on the LDO clock 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_ldoclk_poweron(void)
{
    RCU_CFG1 |= RCU_CFG1_LDOCLKPU;
}

/*!
    \brief      power down the LDO clock 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_ldoclk_powerdown(void)
{
    RCU_CFG1 &= ~RCU_CFG1_LDOCLKPU;
}

/*!
    \brief      power on the BandGap 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_bandgap_poweron(void)
{
    RCU_CFG1 |= RCU_CFG1_BGPU;
}

/*!
    \brief      power down the BandGap 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_bandgap_powerdown(void)
{
    RCU_CFG1 &= ~RCU_CFG1_BGPU;
}

/*!
    \brief      configure the system clock source
    \param[in]  ck_sys: system clock source select
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKSYSSRC_IRC16M: select CK_IRC16M as the CK_SYS source
      \arg        RCU_CKSYSSRC_HXTAL: select CK_HXTAL as the CK_SYS source
      \arg        RCU_CKSYSSRC_PLLP: select CK_PLLP as the CK_SYS source
      \arg        RCU_CKSYSSRC_PLLDIG: select CK_PLLDIG as the CK_SYS source
    \param[out] none
    \retval     none
*/
void rcu_system_clock_source_config(uint32_t ck_sys)
{
    uint32_t reg;
    
    reg = RCU_CFG0;
    /* reset the SCS bits and set according to ck_sys */
    reg &= ~RCU_CFG0_SCS;
    RCU_CFG0 = (reg | ck_sys);
}

/*!
    \brief      get the system clock source
    \param[in]  none
    \param[out] none
    \retval     which clock is selected as CK_SYS source
      \arg        RCU_SCSS_IRC16M: CK_IRC16M is selected as the CK_SYS source
      \arg        RCU_SCSS_HXTAL: CK_HXTAL is selected as the CK_SYS source
      \arg        RCU_SCSS_PLLP: CK_PLLP is selected as the CK_SYS source
      \arg        RCU_CKSYSSRC_PLLDIG: CK_PLLDIG is selected as the CK_SYS source
*/
uint32_t rcu_system_clock_source_get(void)
{
    return (RCU_CFG0 & RCU_CFG0_SCSS);
}

/*!
    \brief      configure the AHB clock prescaler selection
    \param[in]  ck_ahb: AHB clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_AHB_CKSYS_DIVx, x=1, 2, 4, 8, 16, 64, 128, 256, 512
    \param[out] none
    \retval     none
*/
void rcu_ahb_clock_config(uint32_t ck_ahb)
{
    uint32_t reg;
    
    reg = RCU_CFG0;
    /* reset the AHBPSC bits and set according to ck_ahb */
    reg &= ~RCU_CFG0_AHBPSC;
    RCU_CFG0 = (reg | ck_ahb);
}

/*!
    \brief      configure the APB1 clock prescaler selection
    \param[in]  ck_apb1: APB1 clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_APB1_CKAHB_DIV4: select CK_AHB/4 as CK_APB1
      \arg        RCU_APB1_CKAHB_DIV8: select CK_AHB/8 as CK_APB1
      \arg        RCU_APB1_CKAHB_DIV16: select CK_AHB/16 as CK_APB1
    \param[out] none
    \retval     none
*/
void rcu_apb1_clock_config(uint32_t ck_apb1)
{
    uint32_t reg;
    
    reg = RCU_CFG0;
    /* reset the APB1PSC and set according to ck_apb1 */
    reg &= ~RCU_CFG0_APB1PSC;
    RCU_CFG0 = (reg | ck_apb1);
}

/*!
    \brief      configure the APB2 clock prescaler selection
    \param[in]  ck_apb2: APB2 clock prescaler selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_APB2_CKAHB_DIV1: select CK_AHB as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV2: select CK_AHB/2 as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV4: select CK_AHB/4 as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV8: select CK_AHB/8 as CK_APB2
      \arg        RCU_APB2_CKAHB_DIV16: select CK_AHB/16 as CK_APB2
    \param[out] none
    \retval     none
*/
void rcu_apb2_clock_config(uint32_t ck_apb2)
{
    uint32_t reg;
    
    reg = RCU_CFG0;
    /* reset the APB2PSC and set according to ck_apb2 */
    reg &= ~RCU_CFG0_APB2PSC;
    RCU_CFG0 = (reg | ck_apb2);
}

/*!
    \brief      configure the CK_OUT0 clock source and divider
    \param[in]  ckout0_src: CK_OUT0 clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKOUT0SRC_IRC16M: IRC16M selected
      \arg        RCU_CKOUT0SRC_LXTAL: LXTAL selected
      \arg        RCU_CKOUT0SRC_HXTAL: HXTAL selected
      \arg        RCU_CKOUT0SRC_PLLP: PLLP selected
    \param[in]  ckout0_div: CK_OUT0 divider 
      \arg        RCU_CKOUT0_DIVx(x=1,2,3,4,5): CK_OUT0 is divided by x
    \param[out] none
    \retval     none
*/
void rcu_ckout0_config(uint32_t ckout0_src, uint32_t ckout0_div)
{
    uint32_t reg;
    
    reg = RCU_CFG0;
    /* reset the CKOUT0SRC, CKOUT0DIV and set according to ckout0_src and ckout0_div */
    reg &= ~(RCU_CFG0_CKOUT0SEL | RCU_CFG0_CKOUT0DIV );
    RCU_CFG0 = (reg | ckout0_src | ckout0_div);
}

/*!
    \brief      configure the CK_OUT1 clock source and divider
    \param[in]  ckout1_src: CK_OUT1 clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_CKOUT1SRC_CKSYS: system clock selected
      \arg        RCU_CKOUT1SRC_PLLI2S: PLLI2S selected
      \arg        RCU_CKOUT1SRC_HXTAL: HXTAL selected
      \arg        RCU_CKOUT1SRC_PLLDIG: PLLDIG selected           
    \param[in]  ckout1_div: CK_OUT1 divider 
      \arg        RCU_CKOUT1_DIVx(x=1,2,3,4,5): CK_OUT1 is divided by x
    \param[out] none
    \retval     none
*/
void rcu_ckout1_config(uint32_t ckout1_src, uint32_t ckout1_div)
{
    uint32_t reg;
    
    reg = RCU_CFG0;
    /* reset the CKOUT1SRC, CKOUT1DIV and set according to ckout1_src and ckout1_div */
    reg &= ~(RCU_CFG0_CKOUT1SEL | RCU_CFG0_CKOUT1DIV);
    RCU_CFG0 = (reg | ckout1_src | ckout1_div);
}

/*!
    \brief      configure the main PLL clock 
    \param[in]  pll_src: PLL clock source selection
      \arg        RCU_PLLSRC_IRC16M: select IRC16M as PLL source clock
      \arg        RCU_PLLSRC_HXTAL: select HXTAL as PLL source clock
    \param[in]  pll_psc: the PLL VCO source clock prescaler
      \arg         this parameter should be selected between 2 and 63
    \param[in]  pll_n: the PLL VCO clock multi factor
      \arg        this parameter should be selected between 64 and 511
    \param[in]  pll_p: the PLLP output frequency division factor from PLL VCO clock
      \arg        this parameter should be selected 2,4,6,8
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus rcu_pll_config(uint32_t pll_src, uint32_t pll_psc, uint32_t pll_n, uint32_t pll_p)
{
    uint32_t ss_modulation_inc;
    uint32_t ss_modulation_reg;
    
    ss_modulation_inc = 0U;
    ss_modulation_reg = RCU_PLLSSCTL;

    /* calculate the minimum factor of PLLN */
    if((ss_modulation_reg & RCU_PLLSSCTL_SSCGON) == RCU_PLLSSCTL_SSCGON){
        if((ss_modulation_reg & RCU_SS_TYPE_DOWN) == RCU_SS_TYPE_DOWN){
            ss_modulation_inc += RCU_SS_MODULATION_DOWN_INC;
        }else{
            ss_modulation_inc += RCU_SS_MODULATION_CENTER_INC;
        }
    }
    
    /* check the function parameter */
    if(CHECK_PLL_PSC_VALID(pll_psc) && CHECK_PLL_N_VALID(pll_n,ss_modulation_inc) && 
       CHECK_PLL_P_VALID(pll_p)){
         RCU_PLL = pll_psc | (pll_n << 6) | (((pll_p >> 1) - 1U) << 16) |
                   (pll_src);
    }else{
        /* return status */
        return ERROR;
    }
    
    /* return status */
    return SUCCESS;
}

/*!
    \brief      configure the PLLI2S clock
    \param[in]  plli2s_n: PLLI2S clock VCO clock multiplication factor
      \arg        this parameter should be selected between 8 and 127
    \param[in]  plli2s_psc: PLLI2S VCO source clock pre-scale
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLLI2SSRC_DIVx(x=1,2,...,8): PLLI2S vco input source clock divided by x
    \param[in]  plli2s_div: PLLI2SDIV clock divider factor
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLLI2S_DIVx(x=1_5,2,2_5,3,...,32): PLLI2SDIV input source clock divided by x
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus rcu_plli2s_config(uint32_t plli2s_n, uint32_t plli2s_psc,uint32_t plli2s_div)
{
    /* check the function parameter */
    if(CHECK_PLLI2S_N_VALID(plli2s_n) ){
        if(RCU_PLLI2S_DIV32 == plli2s_div){
            RCU_PLLCFG &= ~(RCU_PLLCFG_PLLI2SDIV|RCU_PLLCFG_PLLI2SN|RCU_PLLCFG_PLLI2SPSC);
            RCU_PLLCFG |= (plli2s_n << 8U) | plli2s_psc;
        }
        else{
            RCU_PLLCFG &= ~(RCU_PLLCFG_PLLI2SDIV|RCU_PLLCFG_PLLI2SN|RCU_PLLCFG_PLLI2SPSC);
            RCU_PLLCFG |= (plli2s_n << 8U) | plli2s_psc| plli2s_div;
        }
    }else{
        /* return status */
        return ERROR;
    }
    
    /* return status */
    return SUCCESS;  
}

/*!
    \brief      configure the PLLDIG output clock frequency
    \param[in]  plldig_clk: the PLLDIG VCO clock this parameter sselection
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLLDIG_192M: selected 192Mhz as PLLDIG output frequency
      \arg        RCU_PLLDIG_240M: selected 240Mhz as PLLDIG output frequency
      \arg        RCU_PLLDIG_320M: selected 320Mhz as PLLDIG output frequency
      \arg        RCU_PLLDIG_480M: selected 480Mhz as PLLDIG output frequency
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
void rcu_plldig_config(uint32_t plldig_clk)
{
       RCU_PLLCFG &= ~(RCU_PLLCFG_PLLDIGOSEL);

       RCU_PLLCFG |= plldig_clk;
}

/*!
    \brief      configure PLLDIG clock divider factor for system clock
    \param[in]  plldigfsys_div: PLLDIG clock divider factor for system clock
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLLDIGFSYS_DIVx(x=1,2,3,...,64): PLLDIG clock divided by x for system clock
    \param[out] none
    \retval     none
*/
void rcu_plldigfsys_div_config(uint32_t plldigfsys_div)
{
    uint32_t reg;

    reg = RCU_PLLCFG;
    /* reset the PLLDIGFSYSDIV bits and set according to plldigfsys_div */
    reg &= ~RCU_PLLCFG_PLLDIGFSYSDIV;
    RCU_PLLCFG = (reg | plldigfsys_div);
}

/*!
    \brief      configure the RTC clock source selection
    \param[in]  rtc_clock_source: RTC clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_RTCSRC_NONE: no clock selected
      \arg        RCU_RTCSRC_LXTAL: CK_LXTAL selected as RTC source clock
      \arg        RCU_RTCSRC_IRC32K: CK_IRC32K selected as RTC source clock
      \arg        RCU_RTCSRC_HXTAL_DIV_RTCDIV: CK_HXTAL/RTCDIV selected as RTC source clock
    \param[out] none
    \retval     none
*/
void rcu_rtc_clock_config(uint32_t rtc_clock_source)
{
    uint32_t reg;
    
    reg = RCU_BDCTL; 
    /* reset the RTCSRC bits and set according to rtc_clock_source */
    reg &= ~RCU_BDCTL_RTCSRC;
    RCU_BDCTL = (reg | rtc_clock_source);
}

/*!
    \brief      configure the frequency division of RTC clock when HXTAL was selected as its clock source 
    \param[in]  rtc_div: RTC clock frequency division
                only one parameter can be selected which is shown as below:
      \arg        RCU_RTC_HXTAL_NONE: no clock for RTC
      \arg        RCU_RTC_HXTAL_DIVx: RTCDIV clock select CK_HXTAL/x, x = 2....31
    \param[out] none
    \retval     none
*/
void rcu_rtc_div_config(uint32_t rtc_div)
{
    uint32_t reg;    
    reg = RCU_CFG0; 
    /* reset the RTCDIV bits and set according to rtc_div value */
    reg &= ~RCU_CFG0_RTCDIV;
    RCU_CFG0 = (reg | rtc_div);
}

/*!
    \brief      configure the I2S clock source selection
    \param[in]  i2s_clock_source: I2S clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_I2SSRC_PLLI2S: CK_PLLI2S selected as I2S source clock
      \arg        RCU_I2SSRC_I2S_CKIN: external i2s_ckin pin selected as I2S source clock
      \arg        RCU_I2SSRC_I2S_PLLDIV: PLL division selected as I2S source clock
    \param[out] none
    \retval     none
*/
void rcu_i2s_clock_config(uint32_t i2s_clock_source)
{
    uint32_t reg;
    
    reg = RCU_ADDCTL; 
    /* reset the I2SSEL bit and set according to i2s_clock_source */
    reg &= ~RCU_ADDCTL_I2SSEL;
    RCU_ADDCTL = (reg | i2s_clock_source);
}

/*!
    \brief      configure the PLL divider factor for I2S clock 
    \param[in]  pllfi2s_div: PLL divider factor for I2S clock
                only one parameter can be selected which is shown as below:
      \arg        RCU_PLLFI2SDIV_DIVx(x=1,2,...,64): PLL clock divided by x for I2S clock
    \param[out] none
    \retval     none
*/
void rcu_pllfi2s_clock_div_config(uint32_t pllfi2s_div)
{
    uint32_t reg;

    reg = RCU_ADDCTL;
    /* reset the PLLFI2SDIV bits and set according to pllfi2s_div */
    reg &= ~RCU_ADDCTL_PLLFI2SDIV;
    RCU_ADDCTL = (reg | pllfi2s_div);
}

#if (defined(GD32W515PI) || defined(GD32W515P0))
/*!
    \brief      configure the hpdf clock source selection
    \param[in]  hpdf_clock_source: hpdf clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_HPDFSRC_PCLK2: PCLK2 clock selected as HPDF source clock
      \arg        RCU_HPDFSRC_CKSYS: System clock selected as HPDF source clock
    \param[out] none
    \retval     none
*/
void rcu_hpdf_clock_config(uint32_t hpdf_clock_source)
{
    uint32_t reg;
    
    reg = RCU_ADDCTL; 
    /* reset the HPDFSEL bit and set according to hpdf_clock_source */
    reg &= ~RCU_ADDCTL_HPDFSEL;
    RCU_ADDCTL = (reg | hpdf_clock_source);
}

/*!
    \brief      configure the HPDF AUDIO clock source selection
    \param[in]  hpdfaudio_clock_source: HPDF AUDIO clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_HPDFAUDIOSRC_PLLI2S: PLLI2S output clock selected as HPDF AUDIO source clock
      \arg        RCU_HPDFAUDIOSRC_I2S_CKIN: external I2S_CKIN PIN selected as HPDF AUDIO source clock
      \arg        RCU_HPDFAUDIOSRC_PLL: PLL division selected as HPDF AUDIO source clock
      \arg        RCU_HPDFAUDIOSRC_IRC16M: IRC16M selected as HPDF AUDIO source clock
    \param[out] none
    \retval     none
*/
void rcu_hpdf_audio_clock_config(uint32_t hpdfaudio_clock_source)
{
    uint32_t reg;
    
    reg = RCU_ADDCTL; 
    /* reset the HPDFAUDIOSEL bit and set according to hpdfaudio_clock_source */
    reg &= ~RCU_ADDCTL_HPDFAUDIOSEL;
    RCU_ADDCTL = (reg | hpdfaudio_clock_source);
}
#endif /* GD32W515PI and GD32W515P0 */

/*!
    \brief      configure the SDIO clock source selection
    \param[in]  sdio_clock_source: SDIO clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_SDIOSRC_PLL: PLL output clock selected as SDIO source clock
      \arg        RCU_SDIOSRC_PLLDIG: PLLDIG selected as SDIO source clock
      \arg        RCU_SDIOSRC_IRC16M: IRC16M selected as SDIO source clock
      \arg        RCU_SDIOSRC_HXTAL: HXTAL selected as SDIO source clock
    \param[out] none
    \retval     none
*/
void rcu_sdio_clock_config(uint32_t sdio_clock_source)
{
    uint32_t reg;
    
    reg = RCU_ADDCTL; 
    /* reset the SDIOSEL bit and set according to sdio_clock_source */
    reg &= ~(RCU_SDIOSRC_MASK);
    RCU_ADDCTL = (reg | sdio_clock_source);
}

/*!
    \brief      configure the frequency division of the sdio source clock 
    \param[in]  sdio_div: SDIO clock frequency division
                only one parameter can be selected which is shown as below:
      \arg        RCU_SDIODIV_DIVx: SDIODIV input source clock divided by x, x = 1,...,32
    \param[out] none
    \retval     none
*/
void rcu_sdio_div_config(uint32_t sdio_div)
{
    uint32_t reg;    
    reg = RCU_ADDCTL; 
    /* reset the RTCDIV bits and set according to rtc_div value */
    reg &= ~RCU_ADDCTL_SDIODIV;
    RCU_ADDCTL = (reg | sdio_div);
}

/*!
    \brief      configure the USBFS clock source selection
    \param[in]  usbfs_clock_source: USBFS clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_USBFSSRC_PLL: PLL clock selected as USB source clock
      \arg        RCU_USBFSSRC_PLLDIG: PLLDIG clock selected as USB source clock
    \param[out] none
    \retval     none
*/
void rcu_usbfs_clock_config(uint32_t usbfs_clock_source)
{
    uint32_t reg;
    
    reg = RCU_ADDCTL; 
    /* reset the USBFSSEL bit and set according to usbfs_clock_source */
    reg &= ~RCU_ADDCTL_USBFSSEL;
    RCU_ADDCTL = (reg | usbfs_clock_source);
}

/*!
    \brief      configure the frequency division of the usbfs source clock 
    \param[in]  usbfs_div: USBFS clock frequency division
                only one parameter can be selected which is shown as below:
      \arg        RCU_USBFS_DIVx: USBFSDIV input source clock divided by x, x = 1....32
    \param[out] none
    \retval     none
*/
void rcu_usbfs_div_config(uint32_t usbfs_div)
{
    uint32_t reg;    
    reg = RCU_ADDCTL; 
    /* reset the RTCDIV bits and set according to rtc_div value */
    reg &= ~RCU_ADDCTL_USBFSDIV;
    RCU_ADDCTL = (reg | usbfs_div);
}

/*!
    \brief      configure the I2C0 clock source selection
    \param[in]  i2c0_clock_source: I2C0 clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_I2C0SRC_CKAPB1: CK_APB1 selected as I2C0 source clock
      \arg        RCU_I2C0SRC_CKSYS: CK_SYS selected as I2C0 source clock
      \arg        RCU_I2C0SRC_IRC16M: CK_IRC16M selected as I2C0 source clock
    \param[out] none
    \retval     none
*/
void rcu_i2c0_clock_config(uint32_t i2c0_clock_source)
{
    uint32_t reg;
    
    reg = RCU_CFG1; 
    /* reset the I2C0SEL bit and set according to i2c0_clock_source */
    reg &= ~RCU_CFG1_I2C0SEL;
    RCU_CFG1 = (reg | i2c0_clock_source);
}

/*!
    \brief      configure the USART0 clock source selection
    \param[in]  usart0_clock_source: USART0 clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_USART0SRC_CKAPB1: CK_APB1 selected as USART0 source clock
      \arg        RCU_USART0SRC_CKSYS: CK_SYS selected as USART0 source clock
      \arg        RCU_USART0SRC_LXTAL: CK_LXTAL selected as USART0 source clock
      \arg        RCU_USART0SRC_IRC16M: CK_IRC16M selected as USART0 source clock
    \param[out] none
    \retval     none
*/
void rcu_usart0_clock_config(uint32_t usart0_clock_source)
{
    uint32_t reg;
    
    reg = RCU_CFG1; 
    /* reset the USART0SEL bit and set according to usart0_clock_source */
    reg &= ~RCU_CFG1_USART0SEL;
    RCU_CFG1 = (reg | usart0_clock_source);
}

/*!
    \brief      configure the USART2 clock source selection
    \param[in]  usart2_clock_source: USART2 clock source selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_USART2SRC_CKAPB1: CK_APB1 selected as USART2 source clock
      \arg        RCU_USART2SRC_CKSYS: CK_SYS selected as USART2 source clock
      \arg        RCU_USART2SRC_LXTAL: CK_LXTAL selected as USART2 source clock
      \arg        RCU_USART2SRC_IRC16M: CK_IRC16M selected as USART2 source clock
    \param[out] none
    \retval     none
*/
void rcu_usart2_clock_config(uint32_t usart2_clock_source)
{
    uint32_t reg;
    
    reg = RCU_CFG1; 
    /* reset the USART2SEL bit and set according to usart2_clock_source */
    reg &= ~RCU_CFG1_USART2SEL;
    RCU_CFG1 = (reg | usart2_clock_source);
}

/*!
    \brief      configure IRC16M clock divider factor for system clock
    \param[in]  irc16m_div: IRC16M clock divider factor for system clock
                only one parameter can be selected which is shown as below:
      \arg        RCU_IRC16M_DIVx(x=1,2,3,...,512): IRC16M clock divided by x for system clock
    \param[out] none
    \retval     none
*/
void rcu_irc16m_div_config(uint32_t irc16m_div)
{
    uint32_t reg;
    reg = RCU_CFG1;
/* reset the IRC16MDIV bits and set according to irc16m_div */
    reg &= ~RCU_CFG1_IRC16MDIV;
    RCU_CFG1 = (reg | irc16m_div);
}

/*!
    \brief      configure the TIMER clock prescaler selection
    \param[in]  timer_clock_prescaler: TIMER clock selection
                only one parameter can be selected which is shown as below:
      \arg        RCU_TIMER_PSC_MUL2: if APB1PSC/APB2PSC in RCU_CFG0 register is 0b0xx(CK_APBx = CK_AHB) 
                                      or 0b100(CK_APBx = CK_AHB/2), the TIMER clock is equal to CK_AHB(CK_TIMERx = CK_AHB).
                                      or else, the TIMER clock is twice the corresponding APB clock (TIMER in APB1 domain: CK_TIMERx = 2 x CK_APB1; 
                                      TIMER in APB2 domain: CK_TIMERx = 2 x CK_APB2)
      \arg        RCU_TIMER_PSC_MUL4: if APB1PSC/APB2PSC in RCU_CFG0 register is 0b0xx(CK_APBx = CK_AHB), 
                                      0b100(CK_APBx = CK_AHB/2), or 0b101(CK_APBx = CK_AHB/4), the TIMER clock is equal to CK_AHB(CK_TIMERx = CK_AHB). 
                                      or else, the TIMER clock is four timers the corresponding APB clock (TIMER in APB1 domain: CK_TIMERx = 4 x CK_APB1;  
                                      TIMER in APB2 domain: CK_TIMERx = 4 x CK_APB2)
    \param[out] none
    \retval     none
*/
void rcu_timer_clock_prescaler_config(uint32_t timer_clock_prescaler)
{
    /* configure the TIMERSEL bit and select the TIMER clock prescaler */
    if(timer_clock_prescaler == RCU_TIMER_PSC_MUL2){
        RCU_CFG1 &= timer_clock_prescaler;
    }else{
        RCU_CFG1 |= timer_clock_prescaler;
    }
}

/*!
    \brief      get the clock stabilization and periphral reset flags
    \param[in]  flag: the clock stabilization and periphral reset flags, refer to rcu_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_FLAG_IRC16MSTB: IRC16M stabilization flag
      \arg        RCU_FLAG_HXTALSTB: HXTAL stabilization flag
      \arg        RCU_FLAG_PLLSTB: PLL stabilization flag
      \arg        RCU_FLAG_PLLI2SSTB: PLLI2S stabilization flag
      \arg        RCU_FLAG_PLLDIGSTB: PLLDIG stabilization flag
      \arg        RCU_FLAG_LXTALSTB: LXTAL stabilization flag
      \arg        RCU_FLAG_IRC32KSTB: IRC32K stabilization flag
      \arg        RCU_FLAG_OBLRST: option byte loader reset flags
      \arg        RCU_FLAG_EPRST: external PIN reset flag
      \arg        RCU_FLAG_PORRST: power reset flag
      \arg        RCU_FLAG_SWRST: software reset flag
      \arg        RCU_FLAG_FWDGTRST: free watchdog timer reset flag
      \arg        RCU_FLAG_WWDGTRST: window watchdog timer reset flag
      \arg        RCU_FLAG_LPRST: low-power reset flag
    \param[out] none
    \retval     none
*/
FlagStatus rcu_flag_get(rcu_flag_enum flag)
{
    /* get the rcu flag */
    if(RESET != (RCU_REG_VAL(flag) & BIT(RCU_BIT_POS(flag)))){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear all the reset flag
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_all_reset_flag_clear(void)
{
    RCU_RSTSCK |= RCU_RSTSCK_RSTFC;
}

/*!
    \brief      get the clock stabilization interrupt and ckm flags
    \param[in]  int_flag: interrupt and ckm flags, refer to rcu_int_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_FLAG_IRC32KSTB: IRC32K stabilization interrupt flag
      \arg        RCU_INT_FLAG_LXTALSTB: LXTAL stabilization interrupt flag
      \arg        RCU_INT_FLAG_IRC16MSTB: IRC16M stabilization interrupt flag
      \arg        RCU_INT_FLAG_HXTALSTB: HXTAL stabilization interrupt flag
      \arg        RCU_INT_FLAG_PLLSTB: PLL stabilization interrupt flag
      \arg        RCU_INT_FLAG_PLLI2SSTB: PLLI2S stabilization interrupt flag
      \arg        RCU_INT_FLAG_PLLDIGSTB: PLLDIG stabilization interrupt flag
      \arg        RCU_INT_FLAG_CKM: HXTAL clock stuck interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rcu_interrupt_flag_get(rcu_int_flag_enum int_flag)
{
    /* get the rcu interrupt flag */
    if(RESET != (RCU_REG_VAL(int_flag) & BIT(RCU_BIT_POS(int_flag)))){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear the interrupt flags
    \param[in]  int_flag: clock stabilization and stuck interrupt flags clear, refer to rcu_int_flag_clear_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_FLAG_IRC32KSTB_CLR: IRC32K stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_LXTALSTB_CLR: LXTAL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_IRC16MSTB_CLR: IRC16M stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_HXTALSTB_CLR: HXTAL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_PLLSTB_CLR: PLL stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_PLLI2SSTB_CLR: PLLI2S stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_PLLDIGSTB_CLR: PLLDIG stabilization interrupt flag clear
      \arg        RCU_INT_FLAG_CKM_CLR: clock stuck interrupt flag clear
    \param[out] none
    \retval     none
*/
void rcu_interrupt_flag_clear(rcu_int_flag_clear_enum int_flag)
{
    RCU_REG_VAL(int_flag) |= BIT(RCU_BIT_POS(int_flag));
}

/*!
    \brief      enable the stabilization interrupt
    \param[in]  interrupt: clock stabilization interrupt, refer to rcu_int_enum
                Only one parameter can be selected which is shown as below:
      \arg        RCU_INT_IRC32KSTB: IRC32K stabilization interrupt enable
      \arg        RCU_INT_LXTALSTB: LXTAL stabilization interrupt enable
      \arg        RCU_INT_IRC16MSTB: IRC16M stabilization interrupt enable
      \arg        RCU_INT_HXTALSTB: HXTAL stabilization interrupt enable
      \arg        RCU_INT_PLLSTB: PLL stabilization interrupt enable
      \arg        RCU_INT_PLLI2SSTB: PLLI2S stabilization interrupt enable
      \arg        RCU_INT_PLLDIGSTB: PLLDIG stabilization interrupt enable
    \param[out] none
    \retval     none
*/
void rcu_interrupt_enable(rcu_int_enum interrupt)
{
    RCU_REG_VAL(interrupt) |= BIT(RCU_BIT_POS(interrupt));
}


/*!
    \brief      disable the stabilization interrupt
    \param[in]  interrupt: clock stabilization interrupt, refer to rcu_int_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_INT_IRC32KSTB: IRC32K stabilization interrupt disable
      \arg        RCU_INT_LXTALSTB: LXTAL stabilization interrupt disable
      \arg        RCU_INT_IRC16MSTB: IRC16M stabilization interrupt disable
      \arg        RCU_INT_HXTALSTB: HXTAL stabilization interrupt disable
      \arg        RCU_INT_PLLSTB: PLL stabilization interrupt disable
      \arg        RCU_INT_PLLI2SSTB: PLLI2S stabilization interrupt disable
      \arg        RCU_INT_PLLDIGSTB: PLLDIG stabilization interrupt disable
    \param[out] none
    \retval     none
*/
void rcu_interrupt_disable(rcu_int_enum interrupt)
{
    RCU_REG_VAL(interrupt) &= ~BIT(RCU_BIT_POS(interrupt));
}

/*!
    \brief      configure the LXTAL drive capability
    \param[in]  lxtal_dricap: drive capability of LXTAL
                only one parameter can be selected which is shown as below:
      \arg        RCU_LXTALDRI_LOWER_DRIVE: lower driving capability
      \arg        RCU_LXTALDRI_HIGH_DRIVE: high driving capability
      \arg        RCU_LXTALDRI_HIGHER_DRIVE: higher driving capability
      \arg        RCU_LXTALDRI_HIGHEST_DRIVE: highest driving capability
    \param[out] none
    \retval     none
*/
void rcu_lxtal_drive_capability_config(uint32_t lxtal_dricap)
{
    uint32_t reg;
    
    reg = RCU_BDCTL;
    
    /* reset the LXTALDRI bits and set according to lxtal_dricap */
    reg &= ~RCU_BDCTL_LXTALDRI;
    RCU_BDCTL = (reg | lxtal_dricap);
}

/*!
    \brief      wait for oscillator stabilization flags is SET or oscillator startup is timeout
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
      \arg        RCU_IRC16M: IRC16M
      \arg        RCU_IRC48M: IRC48M
      \arg        RCU_IRC32K: IRC32K
      \arg        RCU_PLL_CK: PLL
      \arg        RCU_PLLI2S_CK: PLLI2S
      \arg        RCU_PLLDIG_CK: PLLDIG
    \param[out] none
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus rcu_osci_stab_wait(rcu_osci_type_enum osci)
{
    uint32_t stb_cnt = 0U;
    ErrStatus reval = ERROR;
    FlagStatus osci_stat = RESET;
    
    switch(osci){
    /* wait HXTAL stable */
    case RCU_HXTAL:
        while((RESET == osci_stat) && (HXTAL_STARTUP_TIMEOUT != stb_cnt)){
            osci_stat = rcu_flag_get(RCU_FLAG_HXTALSTB);
            stb_cnt++;
        }
        
        /* check whether flag is set */
        if(RESET != rcu_flag_get(RCU_FLAG_HXTALSTB)){
            reval = SUCCESS;
        }
        break;
    /* wait LXTAL stable */
    case RCU_LXTAL:
        while((RESET == osci_stat) && (LXTAL_STARTUP_TIMEOUT != stb_cnt)){
            osci_stat = rcu_flag_get(RCU_FLAG_LXTALSTB);
            stb_cnt++;
        }
        
        /* check whether flag is set */
        if(RESET != rcu_flag_get(RCU_FLAG_LXTALSTB)){
            reval = SUCCESS;
        }
        break;
    /* wait IRC16M stable */    
    case RCU_IRC16M:
        while((RESET == osci_stat) && (IRC16M_STARTUP_TIMEOUT != stb_cnt)){
            osci_stat = rcu_flag_get(RCU_FLAG_IRC16MSTB);
            stb_cnt++;
        }
        
        /* check whether flag is set */
        if(RESET != rcu_flag_get(RCU_FLAG_IRC16MSTB)){
            reval = SUCCESS;
        }
        break;
    /* wait IRC32K stable */
    case RCU_IRC32K:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)){
            osci_stat = rcu_flag_get(RCU_FLAG_IRC32KSTB);
            stb_cnt++;
        }
        
        /* check whether flag is set */
        if(RESET != rcu_flag_get(RCU_FLAG_IRC32KSTB)){
            reval = SUCCESS;
        }
        break;
    /* wait PLL stable */    
    case RCU_PLL_CK:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)){
            osci_stat = rcu_flag_get(RCU_FLAG_PLLSTB);
            stb_cnt++;
        }
        
        /* check whether flag is set */
        if(RESET != rcu_flag_get(RCU_FLAG_PLLSTB)){
            reval = SUCCESS;
        }
        break;
    /* wait PLLI2S stable */
    case RCU_PLLI2S_CK:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)){
            osci_stat = rcu_flag_get(RCU_FLAG_PLLI2SSTB);
            stb_cnt++;
        }
        
        /* check whether flag is set */
        if(RESET != rcu_flag_get(RCU_FLAG_PLLI2SSTB)){
            reval = SUCCESS;
        }
        break;
    /* wait PLLDIG stable */    
    case RCU_PLLDIG_CK:
        while((RESET == osci_stat) && (OSC_STARTUP_TIMEOUT != stb_cnt)){
            osci_stat = rcu_flag_get(RCU_FLAG_PLLDIGSTB);
            stb_cnt++;
        }
        
        /* check whether flag is set */
        if(RESET != rcu_flag_get(RCU_FLAG_PLLDIGSTB)){
            reval = SUCCESS;
        }
        break;
    
    default:
        break;
    }
    
    /* return value */
    return reval;
}

/*!
    \brief      turn on the oscillator
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
      \arg        RCU_IRC16M: IRC16M
      \arg        RCU_IRC48M: IRC48M
      \arg        RCU_IRC32K: IRC32K
      \arg        RCU_PLLDIG_CK: PLLDIG
      \arg        RCU_PLL_CK: PLL
      \arg        RCU_PLLI2S_CK: PLLI2S
    \param[out] none
    \retval     none
*/
void rcu_osci_on(rcu_osci_type_enum osci)
{
    RCU_REG_VAL(osci) |= BIT(RCU_BIT_POS(osci));
}

/*!
    \brief      turn off the oscillator
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: HXTAL
      \arg        RCU_LXTAL: LXTAL
      \arg        RCU_IRC16M: IRC16M
      \arg        RCU_IRC48M: IRC48M
      \arg        RCU_IRC32K: IRC32K
      \arg        RCU_PLLDIG_CK: PLLDIG
      \arg        RCU_PLL_CK: PLL
      \arg        RCU_PLLI2S_CK: PLLI2S
    \param[out] none
    \retval     none
*/
void rcu_osci_off(rcu_osci_type_enum osci)
{
    RCU_REG_VAL(osci) &= ~BIT(RCU_BIT_POS(osci));
}

/*!
    \brief      enable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: high speed crystal oscillator(HXTAL)
      \arg        RCU_LXTAL: low speed crystal oscillator(LXTAL)
    \param[out] none
    \retval     none
*/
void rcu_osci_bypass_mode_enable(rcu_osci_type_enum osci)
{
    uint32_t reg;

    switch(osci){
    /* enable HXTAL to bypass mode */    
    case RCU_HXTAL:
        RCU_CTL &= ~RCU_CTL_HXTALEN;
        RCU_CTL &= ~RCU_CTL_HXTALBPS;
        break;
    /* enable LXTAL to bypass mode */
    case RCU_LXTAL:
        reg = RCU_BDCTL;
        RCU_BDCTL &= ~RCU_BDCTL_LXTALEN;
        RCU_BDCTL = (reg | RCU_BDCTL_LXTALBPS);
        break;
    case RCU_IRC16M:
    case RCU_IRC32K:
    case RCU_PLL_CK:
    case RCU_PLLI2S_CK:
    case RCU_PLLDIG_CK:    
        break;
    default:
        break;
    }
}

/*!
    \brief      disable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it
    \param[in]  osci: oscillator types, refer to rcu_osci_type_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_HXTAL: high speed crystal oscillator(HXTAL)
      \arg        RCU_LXTAL: low speed crystal oscillator(LXTAL)
    \param[out] none
    \retval     none
*/
void rcu_osci_bypass_mode_disable(rcu_osci_type_enum osci)
{
    uint32_t reg;
    
    switch(osci){
    /* disable HXTAL to bypass mode */    
    case RCU_HXTAL:
        reg = RCU_CTL;
        RCU_CTL &= ~RCU_CTL_HXTALEN;
        RCU_CTL = (reg & RCU_CTL_HXTALBPS);
        break;
    /* disable LXTAL to bypass mode */
    case RCU_LXTAL:
        reg = RCU_BDCTL;
        RCU_BDCTL &= ~RCU_BDCTL_LXTALEN;
        RCU_BDCTL = (reg & ~RCU_BDCTL_LXTALBPS);
        break;
    case RCU_IRC16M:
    case RCU_IRC32K:
    case RCU_PLL_CK:
    case RCU_PLLI2S_CK:
    case RCU_PLLDIG_CK:    
        break;
    default:
        break;
    }
}

/*!
    \brief      enable the HXTAL clock monitor
    \param[in]  none
    \param[out] none
    \retval     none
*/

void rcu_hxtal_clock_monitor_enable(void)
{
    RCU_CTL |= RCU_CTL_CKMEN;
}

/*!
    \brief      disable the HXTAL clock monitor
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_hxtal_clock_monitor_disable(void)
{
    RCU_CTL &= ~RCU_CTL_CKMEN;
}

/*!
    \brief      enable the RF HXTAL clock monitor 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_rf_hxtal_clock_monitor_enable(void)
{
    RCU_CTL |= RCU_CTL_RFCKMEN;
}

/*!
    \brief      disable the RF HXTAL clock monitor 
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_rf_hxtal_clock_monitor_disable(void)
{
    RCU_CTL &= ~RCU_CTL_RFCKMEN;
}

/*!
    \brief      set the IRC16M adjust value
    \param[in]  irc16m_adjval: IRC16M adjust value, must be between 0 and 0x1F
      \arg        0x00 - 0x1F
    \param[out] none
    \retval     none
*/
void rcu_irc16m_adjust_value_set(uint32_t irc16m_adjval)
{
    uint32_t reg;
    
    reg = RCU_CTL;
    /* reset the IRC16MADJ bits and set according to irc16m_adjval */
    reg &= ~RCU_CTL_IRC16MADJ;
    RCU_CTL = (reg | ((irc16m_adjval & RCU_IRC16M_ADJUST_MASK) << RCU_IRC16M_ADJUST_OFFSET));
}

/*!
    \brief      unlock the voltage key
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_voltage_key_unlock(void)
{
    RCU_VKEY = RCU_VKEY_UNLOCK;
}

/*!
    \brief      deep-sleep mode voltage select
    \param[in]  dsvol: deep sleep mode voltage
                only one parameter can be selected which is shown as below:
      \arg        RCU_DEEPSLEEP_V_1_1: the core voltage is 1.1V
      \arg        RCU_DEEPSLEEP_V_1_0: the core voltage is 1.0V
      \arg        RCU_DEEPSLEEP_V_0_9: the core voltage is 0.9V
      \arg        RCU_DEEPSLEEP_V_0_8: the core voltage is 0.8V
    \param[out] none
    \retval     none
*/
void rcu_deepsleep_voltage_set(uint32_t dsvol)
{    
    dsvol &= RCU_DSV_DSLPVS;
    RCU_DSV = dsvol;
}

/*!
    \brief      configure the spread spectrum modulation for the main PLL clock
    \param[in]  spread_spectrum_type: PLL spread spectrum modulation type select
      \arg        RCU_SS_TYPE_CENTER: center spread type is selected
      \arg        RCU_SS_TYPE_DOWN: down spread type is selected
    \param[in]  modstep: configure PLL spread spectrum modulation profile amplitude and frequency
      \arg        This parameter should be selected between 0 and 7FFF.The following criteria must be met: MODSTEP*MODCNT <=2^15-1
    \param[in]  modcnt: configure PLL spread spectrum modulation profile amplitude and frequency
      \arg        This parameter should be selected between 0 and 1FFF.The following criteria must be met: MODSTEP*MODCNT <=2^15-1
    \param[out] none
    \retval     none
*/
void rcu_spread_spectrum_config(uint32_t spread_spectrum_type, uint32_t modstep, uint32_t modcnt)
{
    uint32_t reg;
    
    reg = RCU_PLLSSCTL;
    /* reset the RCU_PLLSSCTL register bits */
    reg &= ~(RCU_PLLSSCTL_MODCNT | RCU_PLLSSCTL_MODSTEP | RCU_PLLSSCTL_SS_TYPE);
    RCU_PLLSSCTL = (reg | spread_spectrum_type | modstep << 13 | modcnt);
}

/*!
    \brief      enable the PLL spread spectrum modulation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_spread_spectrum_enable(void)
{
    RCU_PLLSSCTL |= RCU_PLLSSCTL_SSCGON;
}

/*!
    \brief      disable the PLL spread spectrum modulation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_spread_spectrum_disable(void)
{
    RCU_PLLSSCTL &= ~RCU_PLLSSCTL_SSCGON;
}

/*!
    \brief      enable the security attribution
    \param[in]  security: clock security attribution, refer to rcu_sec_enum
                Only one parameter can be selected which is shown as below:
      \arg        RCU_SEC_IRC16MSEC: IRC16M clock configuration and status bits security
      \arg        RCU_SEC_HXTALSEC: HXTAL clock configuration and status bits security
      \arg        RCU_SEC_IRC32KSEC: IRC32K clock configuration and status bits security
      \arg        RCU_SEC_LXTALSEC: LXTAL clock configuration and status bits security
      \arg        RCU_SEC_SYSCLKSEC: SYSCLK clock selection, STOPWUCK bit, clock output on MCO configuration
      \arg        RCU_SEC_PRESCSEC: AHBx/APBx prescaler configuration bits security
      \arg        RCU_SEC_PLLSEC: main PLL configuration and status bits security
      \arg        RCU_SEC_PLLDIGSEC: PLLDIG configuration and status bits security
      \arg        RCU_SEC_PLLI2SSEC: PLLI2S configuration and status bits security
      \arg        RCU_SEC_RMVRSTFSEC: remove reset flag security
      \arg        RCU_SEC_BKPSEC: BKP security
    \param[out] none
    \retval     none
*/
void rcu_security_enable(rcu_sec_enum security)
{
    RCU_REG_VAL(security) |= BIT(RCU_BIT_POS(security));
}
/*!
    \brief      disable the security attribution
    \param[in]  security: clock security attribution, refer to rcu_sec_enum
                Only one parameter can be selected which is shown as below:
      \arg        RCU_SEC_IRC16MSEC: IRC16M clock configuration and status bits security
      \arg        RCU_SEC_HXTALSEC: HXTAL clock configuration and status bits security
      \arg        RCU_SEC_IRC32KSEC: IRC32K clock configuration and status bits security
      \arg        RCU_SEC_LXTALSEC: LXTAL clock configuration and status bits security
      \arg        RCU_SEC_SYSCLKSEC: SYSCLK clock selection, STOPWUCK bit, clock output on MCO configuration
      \arg        RCU_SEC_PRESCSEC: AHBx/APBx prescaler configuration bits security
      \arg        RCU_SEC_PLLSEC: main PLL configuration and status bits security
      \arg        RCU_SEC_PLLDIGSEC: PLLDIG configuration and status bits security
      \arg        RCU_SEC_PLLI2SSEC: PLLI2S configuration and status bits security
      \arg        RCU_SEC_RMVRSTFSEC: remove reset flag security
      \arg        RCU_SEC_BKPSEC: BKP security
    \param[out] none
    \retval     none
*/
void rcu_security_disable(rcu_sec_enum security)
{
    RCU_REG_VAL(security) &= ~BIT(RCU_BIT_POS(security));
}

/*!
    \brief      enable the privileged access
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_privilege_enable(void)
{
    RCU_CTL |= RCU_CTL_RCUPRIP;
}
/*!
    \brief      disable the privileged access
    \param[in]  none
    \param[out] none
    \retval     none
*/
void rcu_privilege_disable(void)
{
    RCU_CTL &= ~RCU_CTL_RCUPRIP;
}

/*!
    \brief      get the peripherals clock secure flag
    \param[in]  sec_flag: secure flag, refer to rcu_sec_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        RCU_SEC_FLAG_IRC16MSEC: IRC16M secure flag 
      \arg        RCU_SEC_FLAG_HXTALSEC: HXTAL secure flag 
      \arg        RCU_SEC_FLAG_IRC32KSEC: LXTAL secure flag
      \arg        RCU_SEC_FLAG_LXTALSEC: LXTAL clock secure flag
      \arg        RCU_SEC_FLAG_SYSCLKSEC: SYSCLK secure flag
      \arg        RCU_SEC_FLAG_PRESCSEC: PRESC secure flag
      \arg        RCU_SEC_FLAG_PLLSEC: PLL secure flag 
      \arg        RCU_SEF_PLLDIGSEC: PLLDIG secure flag
      \arg        RCU_SEC_FLAG_PLLI2SSEC: PLLI2S secure flag
      \arg        RCU_SEC_FLAG_RMVRSTSEC: RMVRST secure flag
      \arg        RCU_SEC_FLAG_BKPSEC: BKP secure flag
      \arg        RCU_SEC_FLAG_AHB1_PxSEC:(x=A,B,C): GPIO port x security flag
      \arg        RCU_SEC_FLAG_AHB1_WIFI: WIFI security flag
      \arg        RCU_SEC_FLAG_AHB1_SRAMx: (x=0,1,2,3): SRAMx security flag
      \arg        RCU_SEC_FLAG_AHB1_DMAx:(x=0,1): DMAx security flag
      \arg        RCU_SEC_FLAG_AHB2_DCI (not available for GD32W515TX series): DCI security flag
      \arg        RCU_SEC_FLAG_AHB2_PKCAU: PKCAU security flag
      \arg        RCU_SEC_FLAG_AHB2_CAU: CAU security flag
      \arg        RCU_SEC_FLAG_AHB2_HAU: HAU security flag
      \arg        RCU_SEC_FLAG_AHB2_TRNG: TRNG security flag
      \arg        RCU_SEC_FLAG_AHB3_SQPI: SQPI security flag
      \arg        RCU_SEC_FLAG_AHB2_QSPI: QSPI security flag
      \arg        RCU_SEC_FLAG_APB1_TIMERx: (x=0,1,2,3,4,5,15,16): TIMERx security flag
      \arg        RCU_SEC_FLAG_APB1_WWDGT:  WWDGT security flag
      \arg        RCU_SEC_FLAG_APB1_SPI1: SPI1 security flag
      \arg        RCU_SEC_FLAG_APB1_USARTx: (x=0,1,2): USARTx security flag
      \arg        RCU_SEC_FLAG_APB1_I2Cx: (x=0,1): I2Cx security flag
      \arg        RCU_SEC_FLAG_APB1_PMU: PMU security flag
      \arg        RCU_SEC_FLAG_APB2_ADC: ADC security flag
      \arg        RCU_SEC_FLAG_APB2_SDIO: SDIO security flag
      \arg        RCU_SEC_FLAG_APB2_SPI0: SPI0 security flag
      \arg        RCU_SEC_FLAG_APB2_SYSCFG: SYSCFG security flag
      \arg        RCU_SEC_FLAG_APB2_HPDF: HPDF security flag
      \arg        RCU_SEC_FLAG_APB2_RF: RF security flag 
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus rcu_security_flag_get(rcu_sec_flag_enum sec_flag)
{
    /* get the rcu interrupt flag */
    if(RESET != (RCU_REG_VAL(sec_flag) & BIT(RCU_BIT_POS(sec_flag)))){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      get the system clock, bus and peripheral clock frequency
    \param[in]  clock: the clock frequency which to get
                only one parameter can be selected which is shown as below:
      \arg        CK_SYS: system clock frequency
      \arg        CK_AHB: AHB clock frequency
      \arg        CK_APB1: APB1 clock frequency
      \arg        CK_APB2: APB2 clock frequency
      \arg        CK_USART0: USART0 clock frequency
      \arg        CK_USART2: USART2 clock frequency
      \arg        CK_I2C0: I2C0 clock frequency
    \param[out] none
    \retval     clock frequency of system, AHB, APB1, APB2
*/
uint32_t rcu_clock_freq_get(rcu_clock_freq_enum clock)
{
    uint32_t sws, ck_freq = 0U;
    uint32_t cksys_freq, ahb_freq, apb1_freq, apb2_freq;
    uint32_t usart2_freq = 0U, usart0_freq = 0U,i2c0_freq = 0U;    
    uint32_t pllpsc, plln, pllsel, pllp, ck_src, idx, clk_exp;
    
    /* exponent of AHB, APB1 and APB2 clock divider */
    const uint8_t ahb_exp[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
    const uint8_t apb1_exp[8] = {0, 0, 0, 0, 1, 2, 3, 4};
    const uint8_t apb2_exp[8] = {0, 0, 0, 0, 1, 2, 3, 4};

    sws = GET_BITS(RCU_CFG0, 2, 3);
    switch(sws){
    /* IRC16M is selected as CK_SYS */
    case SEL_IRC16M:
        cksys_freq = IRC16M_VALUE;
        break;
    /* HXTAL is selected as CK_SYS */
    case SEL_HXTAL:
        cksys_freq = HXTAL_VALUE;
        break;
    /* PLLP is selected as CK_SYS */
    case SEL_PLLP:
        /* get the value of PLLPSC[5:0] */
        pllpsc = GET_BITS(RCU_PLL, 0U, 5U);
        plln = GET_BITS(RCU_PLL, 6U, 14U);
        pllp = (GET_BITS(RCU_PLL, 16U, 17U) + 1U) * 2U;
        /* PLL clock source selection, HXTAL or IRC16M/2 */
        pllsel = (RCU_PLL & RCU_PLL_PLLSEL);
        if (RCU_PLLSRC_HXTAL == pllsel) {
            ck_src = HXTAL_VALUE;
        } else {
            ck_src = IRC16M_VALUE;
        }
        cksys_freq = ((ck_src / pllpsc) * plln)/pllp;
        break;
    /* IRC16M is selected as CK_SYS */
    default:
        cksys_freq = IRC16M_VALUE;
        break;
    }
    /* calculate AHB clock frequency */
    idx = GET_BITS(RCU_CFG0, 4, 7);
    clk_exp = ahb_exp[idx];
    ahb_freq = cksys_freq >> clk_exp;
    
    /* calculate APB1 clock frequency */
    idx = GET_BITS(RCU_CFG0, 10, 12);
    clk_exp = apb1_exp[idx];
    apb1_freq = ahb_freq >> clk_exp;
    
    /* calculate APB2 clock frequency */
    idx = GET_BITS(RCU_CFG0, 13, 15);
    clk_exp = apb2_exp[idx];
    apb2_freq = ahb_freq >> clk_exp;
    
    /* return the clocks frequency */
    switch(clock){
    case CK_SYS:
        ck_freq = cksys_freq;
        break;
    case CK_AHB:
        ck_freq = ahb_freq;
        break;
    case CK_APB1:
        ck_freq = apb1_freq;
        break;
    case CK_APB2:
        ck_freq = apb2_freq;
        break;
    case CK_USART0:
        /* calculate USART0 clock frequency */
        if(RCU_USART0SRC_CKAPB1 == (RCU_CFG1 & RCU_CFG1_USART0SEL)){
            usart0_freq = apb1_freq;
        }else if(RCU_USART0SRC_CKSYS == (RCU_CFG1 & RCU_CFG1_USART0SEL)){
            usart0_freq = cksys_freq;
        }else if(RCU_USART0SRC_LXTAL == (RCU_CFG1 & RCU_CFG1_USART0SEL)){
            usart0_freq = LXTAL_VALUE;
        }else if(RCU_USART0SRC_IRC16M == (RCU_CFG1 & RCU_CFG1_USART0SEL)){
            usart0_freq = IRC16M_VALUE;
        }else{
        }
        ck_freq = usart0_freq;
        break;
    case CK_USART2:
        /* calculate USART2 clock frequency */
        if(RCU_USART2SRC_CKAPB2 == (RCU_CFG1 & RCU_CFG1_USART2SEL)){
            usart2_freq = apb2_freq;
        }else if(RCU_USART2SRC_CKSYS == (RCU_CFG1 & RCU_CFG1_USART2SEL)){
            usart2_freq = cksys_freq;
        }else if(RCU_USART2SRC_LXTAL == (RCU_CFG1 & RCU_CFG1_USART2SEL)){
            usart2_freq = LXTAL_VALUE;
        }else if(RCU_USART2SRC_IRC16M == (RCU_CFG1 & RCU_CFG1_USART2SEL)){
            usart2_freq = IRC16M_VALUE;
        }else{
        }
        ck_freq = usart2_freq;
        break;
    case CK_I2C0:
        /* calculate I2C0 clock frequency */
        if(RCU_I2C0SRC_CKAPB1 == (RCU_CFG1 & RCU_CFG1_I2C0SEL)){
            i2c0_freq = apb1_freq;
        }else if(RCU_I2C0SRC_CKSYS == (RCU_CFG1 & RCU_CFG1_I2C0SEL)){
            i2c0_freq = cksys_freq;
        }else if(RCU_I2C0SRC_IRC16M == (RCU_CFG1 & RCU_CFG1_I2C0SEL)){
            i2c0_freq = IRC16M_VALUE;
        }else{
        }
        ck_freq = i2c0_freq;
        break;
    default:
        break;
    }
    return ck_freq;
}
