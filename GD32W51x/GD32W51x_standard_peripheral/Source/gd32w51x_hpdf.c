/*!
    \file    gd32w51x_hpdf.c
    \brief   HPDF driver

    \version 2021-03-25, V1.0.0, firmware for GD32W51x
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    All rights reserved.

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

#include "gd32w51x_hpdf.h"
#include <stdlib.h>

#if defined (GD32W515PI) || defined (GD32W515P0)

/* HPDF register bit offset */
#define CH0CTL_CKOUTDIV_OFFSET              ((uint32_t)0x00000010U)               /*!< bit offset of CKOUTDIV in HPDf_CH0CTL */
#define CHyCTL_SPICKSS_OFFSET               ((uint32_t)0x00000002U)               /*!< bit offset of CKOUTDIV in HPDf_CH0CTL */
#define CHyCFG_DTRS_OFFSET                  ((uint32_t)0x00000003U)               /*!< bit offset of DTRS in HPDF_CH0CFG */
#define CHyCFG0_CALOFF_OFFSET               ((uint32_t)0x00000008U)               /*!< bit offset of CALOFF in HPDF_CH0CFG */
#define CHyCFG1_TMFOR_OFFSET                ((uint32_t)0x00000010U)               /*!< bit offset of TMFOR in CHyCFG1 */
#define FLTySFCFG_SFOR_OFFSET               ((uint32_t)0x00000010U)               /*!< bit offset of SFOR in FLTySFCFG */
#define FLTyIDATAT_IDATA_OFFSET             ((uint32_t)0x00000008U)               /*!< bit offset of IDATA in FLTyIDATA */
#define FLTyRDATAT_RDATA_OFFSET             ((uint32_t)0x00000008U)               /*!< bit offset of RDATA in FLTyRDATA */
#define FLTyTMHT_HTVAL_OFFSET               ((uint32_t)0x00000008U)               /*!< bit offset of HTVAL in FLTyTMHT */
#define FLTyTMLT_LTVAL_OFFSET               ((uint32_t)0x00000008U)               /*!< bit offset of LTVAL in FLTyTMLT */
#define FLTyEMMAX_MAXVAL_OFFSET             ((uint32_t)0x00000008U)               /*!< bit offset of MAXVAL in FLTyEMMAX */
#define FLTyEMMIN_MINVAL_OFFSET             ((uint32_t)0x00000008U)               /*!< bit offset of MINVAL in FLTyEMMIN */

#define HPDF_WRONG_HANDLE        while(1){}

/*!
    \brief      reset HPDF
    \param[in]  hpdf_periph: HPDF
    \param[out] none
    \retval     none
*/
void hpdf_deinit(void)
{
    /* reset HPDF */
    rcu_periph_reset_enable(RCU_HPDFRST);
    rcu_periph_reset_disable(RCU_HPDFRST);
}

/*!
    \brief      initialize the parameters of HPDF channel struct with the default values
    \param[in]  init_struct: the initialization data needed to initialize HPDF
                  serial_interface: EXTERNAL_CKIN, INTERNAL_CKOUT, HALF_CKOUT_FALLING_EDGE, HALF_CKOUT_RISING_EDGE
                  spi_ck_source: SPI_RISING_EDGE, SPI_FALLING_EDGE, MANCHESTER_CODE0, MANCHESTER_CODE1
                  malfunction_monitor: MM_DISABLE, MM_ENABLE
                  calibration_offset: calibration offset(-8388608 ~ 8388607)
                  right_bit_shift: data right bit-shift(0 ~ 31)
                  channel_multiplexer: SERIAL_INPUT, INTERNAL_INPUT
                  channel_pin_select: CHPINSEL_CURRENT, CHPINSEL_NEXT
                  ck_loss_detector: CLK_LOSS_DISABLE, CLK_LOSS_ENABLE
                  data_packing_mode: DPM_STANDARD_MODE, DPM_INTERLEAVED_MODE, DPM_DUAL_MODE
                  tm_filter: TM_FASTSINC, TM_SINC1, TM_SINC2, TM_SINC3
                  tm_filter_oversample: threshold monitor filter oversampling rate(0 ~ 31), AW_FLT_BYPASS=0
                  mm_break_signal: DISABLE, ENABLE
                  mm_counter_threshold: malfunction monitor counter threshold(0 ~ 255)
                  plsk_value: the number of serial input samples that will be skipped(0 ~ 63)
    \param[out] none
    \retval     none
*/
void hpdf_channel_struct_para_init(hpdf_channel_parameter_struct* init_struct)
{
    /* check whether the struct is empty */
    if(NULL == init_struct){
        HPDF_WRONG_HANDLE
    }
    /* set the struct with the default values */
    init_struct->serial_interface       = SPI_RISING_EDGE;
    init_struct->spi_ck_source          = EXTERNAL_CKIN;
    init_struct->malfunction_monitor    = MM_DISABLE;
    init_struct->calibration_offset     = 0;
    init_struct->right_bit_shift        = 0U;
    init_struct->channel_multiplexer    = SERIAL_INPUT; 
    init_struct->channel_pin_select     = CHPINSEL_CURRENT;
    init_struct->ck_loss_detector       = CLK_LOSS_DISABLE;
    init_struct->data_packing_mode      = DPM_STANDARD_MODE;
    init_struct->tm_filter              = TM_FASTSINC;
    init_struct->tm_filter_oversample   = TM_FLT_BYPASS;
    init_struct->mm_break_signal        = DISABLE;
    init_struct->mm_counter_threshold   = 0U;
    init_struct->plsk_value             = 0U;
}

/*!
    \brief      initialize the parameters of HPDF filter struct with the default values
    \param[in]  init_struct: the initialization data needed to initialize HPDF
                  sinc_filter: FLT_FASTSINC, FLT_SINC1, FLT_SINC2, FLT_SINC3, FLT_SINC4, FLT_SINC5
                  sinc_oversample: sinc filter oversampling rate(0 ~ 1023), FLT_SINC_BYPASS=0
                  integrator_oversample: integrator oversampling rate(0 ~ 255), INTEGRATOR_BYPASS=0
                  tm_fast_mode: TMFM_DISABLE, TMFM_ENABLE
                  tm_channel: TMCHEN_DISABLE, TMCHEN_CHANNEL0, TMCHEN_CHANNEL1, TMCHEN_CHANNEL0_1
                  tm_high_threshold: threshold monitor high threshold(-8388608 ~ 8388607)
                  tm_low_threshold: threshold monitor low threshold value(-8388608 ~ 8388607)
                  extreme_monitor_channel: EM_CHANNEL_DISABLE, EM_CHANNEL0, EM_CHANNEL1, EM_CHANNEL0_1
                  ht_break_signal: NO_TM_HT_BREAK, TM_HT_BREAK0, TM_HT_BREAK1, TM_HT_BREAK0_1
                  lt_break_signal: NO_TM_LT_BREAK, TM_LT_BREAK0, TM_LT_BREAK1, TM_LT_BREAK0_1
    \param[out] none
    \retval     none
*/
void hpdf_filter_struct_para_init(hpdf_filter_parameter_struct* init_struct)
{
    /* check whether the struct is empty */
    if(NULL == init_struct){
        HPDF_WRONG_HANDLE
    }
    /* set the struct with the default values */
    init_struct->sinc_filter                = FLT_FASTSINC;
    init_struct->sinc_oversample            = FLT_SINC_BYPASS;
    init_struct->integrator_oversample      = INTEGRATOR_BYPASS;
    init_struct->tm_fast_mode               = TMFM_DISABLE;
    init_struct->tm_channel                 = TMCHEN_DISABLE;
    init_struct->tm_high_threshold          = 0;
    init_struct->tm_low_threshold           = 0;
    init_struct->extreme_monitor_channel    = EM_CHANNEL_DISABLE;
    init_struct->ht_break_signal            = NO_TM_HT_BREAK;
    init_struct->lt_break_signal            = NO_TM_LT_BREAK;
}

/*!
    \brief      initialize the parameters of regular conversion struct with the default values
    \param[in]  init_struct: the initialization data needed to initialize HPDF
                  continuous_mode: RCCM_DISABLE, RCCM_ENABLE
                  fast_mode: FAST_DISABLE, FAST_ENABLE
                  rcdmaen: RCDMAEN_DISABLE, RCDMAEN_ENABLE
                  rcsyn: RCSYN_DISABLE, RCSYN_ENABLE
                  rcsyn: RCS_CHANNEL0, RCS_CHANNEL1
    \param[out] none
    \retval     none
*/
void hpdf_rc_struct_para_init(hpdf_rc_parameter_struct* init_struct)
{
    /* check whether the struct is empty */
    if(NULL == init_struct){
        HPDF_WRONG_HANDLE
    }
    /* set the struct with the default values */
    init_struct->continuous_mode        = RCCM_DISABLE;
    init_struct->fast_mode              = FAST_DISABLE;
    init_struct->rcdmaen                = RCDMAEN_DISABLE;
    init_struct->rcsyn                  = RCSYN_DISABLE;
    init_struct->rcs_channel            = RCS_CHANNEL0;
}

/*!
    \brief      initialize the parameters of inserted conversion struct with the default values
    \param[in]  init_struct: the initialization data needed to initialize HPDF
                  scmod: SCMOD_DISABLE, SCMOD_ENABLE
                  icdmaen: ICDMAEN_DISABLE, ICDMAEN_ENABLE
                  ic_channel_group: IGCSEL_CHANNEL0, IGCSEL_CHANNEL1, IGCSEL_CHANNEL0_1
                  icsyn: ICSYN_DISABLE, ICSYN_ENABLE
                  trigger_dege: TRG_DISABLE, RISING_EDGE_TRG, FALLING_EDGE_TRG, EDGE_TRG
                  trigger_signal: HPDF_ITRG0, HPDF_ITRG1, HPDF_ITRG2, HPDF_ITRG3, HPDF_ITRG24, HPDF_ITRG25, HPDF_ITRG26
    \param[out] none
    \retval     none
*/
void hpdf_ic_struct_para_init(hpdf_ic_parameter_struct* init_struct)
{
    /* check whether the struct is empty */
    if(NULL == init_struct){
        HPDF_WRONG_HANDLE
    }
    /* set the struct with the default values */
    init_struct->scmod              = SCMOD_DISABLE;
    init_struct->icdmaen            = ICDMAEN_DISABLE;
    init_struct->ic_channel_group   = IGCSEL_CHANNEL0;
    init_struct->icsyn              = ICSYN_DISABLE;
    init_struct->trigger_dege       = TRG_DISABLE;
    init_struct->trigger_signal     = HPDF_ITRG0;
}

/*!
    \brief      enable the HPDF module globally
    \param[in]  hpdf_periph: HPDF
    \param[out] none
    \retval     none
*/
void hpdf_enable(void)
{
    /* enable the HPDF module globally */
    HPDF_CHxCTL(CHANNEL0) |= HPDF_CH0CTL_HPDFEN;
}

/*!
    \brief      disable the HPDF module globally
    \param[in]  hpdf_periph: HPDF
    \param[out] none
    \retval     none
*/
void hpdf_disable(void)
{
    /* disable the HPDF module globally */
    HPDF_CHxCTL(CHANNEL0) &= ~HPDF_CH0CTL_HPDFEN;
}

/*!
    \brief      initialize the HPDF channel
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  init_struct: the initialization data needed to initialize HPDF channel
    \param[out] none
    \retval     none
*/
void hpdf_channel_init(hpdf_channel_enum channelx, hpdf_channel_parameter_struct* init_struct)
{
    uint32_t reg;
    /* configure the HPDF_CH0CTL */
    reg = HPDF_CHxCTL(channelx);
    reg &= ~(HPDF_CHxCTL_SPICKSS | HPDF_CHxCTL_SITYP | HPDF_CHxCTL_MMEN | HPDF_CHxCTL_CKLEN | HPDF_CHxCTL_CHPINSEL |\
             HPDF_CHxCTL_CMSD | HPDF_CHxCTL_DPM);
    reg |= (init_struct->spi_ck_source | init_struct->serial_interface | init_struct->malfunction_monitor |\
            init_struct->channel_multiplexer | init_struct->channel_pin_select | init_struct->ck_loss_detector |\
            init_struct->data_packing_mode);
    HPDF_CHxCTL(channelx) = reg;
    /* configure the HPDF_CH0CFG0 */
    reg = HPDF_CHxCFG0(channelx);
    reg &= ~(HPDF_CHxCFG0_CALOFF | HPDF_CHxCFG0_DTRS);
    reg |= (((uint32_t)init_struct-> calibration_offset << CHyCFG0_CALOFF_OFFSET) | (init_struct->right_bit_shift << CHyCFG_DTRS_OFFSET));
    HPDF_CHxCFG0(channelx) = reg;
    /* configure the HPDF_CH0CFG1 */
    if(init_struct->tm_filter_oversample > 0U){
        init_struct->tm_filter_oversample = init_struct->tm_filter_oversample - 1U;    
    }
    reg = HPDF_CHxCFG1(channelx);
    reg &= ~(HPDF_CHxCFG1_TMSFO | HPDF_CHxCFG1_TMFOR | HPDF_CHxCFG1_MMBSD | HPDF_CHxCFG1_MMCT);
    reg |= (init_struct->tm_filter | (init_struct->tm_filter_oversample << CHyCFG1_TMFOR_OFFSET) | init_struct->mm_break_signal |\
            init_struct->mm_counter_threshold);
    HPDF_CHxCFG1(channelx) = reg;
    /* configure the HPDF_CH0PS */
    reg = HPDF_CHxPS(channelx);
    reg &= ~(HPDF_CHxPS_PLSK);
    reg |= init_struct->plsk_value;
    HPDF_CHxPS(channelx) = reg;
}

/*!
    \brief      initialize the HPDF filter
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  init_struct: the initialization data needed to initialize HPDF filter
    \param[out] none
    \retval     none
*/
void hpdf_filter_init(hpdf_filter_enum filtery, hpdf_filter_parameter_struct* init_struct)
{
    uint32_t reg;
    /* configure the HPDF_FLT0CTL0 */
    HPDF_FLTyCTL0(filtery) |= init_struct->tm_fast_mode;
    /* configure the HPDF_FLT0CTL1 */
    reg = HPDF_FLTyCTL1(filtery);
    reg &= ~(HPDF_FLTyCTL1_TMCHEN | HPDF_FLTyCTL1_EMCS);
    reg |= (init_struct->tm_channel | init_struct->extreme_monitor_channel);
    HPDF_FLTyCTL1(filtery) = reg;
    /* configure the HPDF_FLT0SFCTL*/
    if(init_struct->sinc_oversample > 0U){
        init_struct->sinc_oversample = init_struct->sinc_oversample - 1U;    
    }
    if(init_struct->integrator_oversample > 0U){
        init_struct->integrator_oversample = init_struct->integrator_oversample - 1U;    
    }
    reg = HPDF_FLTySFCFG(filtery);
    reg &= ~(HPDF_FLTySFCFG_SFO | HPDF_FLTySFCFG_SFOR | HPDF_FLTySFCFG_IOR);
    reg |= (init_struct->sinc_filter | (init_struct->sinc_oversample << FLTySFCFG_SFOR_OFFSET) | init_struct->integrator_oversample);
    HPDF_FLTySFCFG(filtery) = reg;
    /* configure the HPDF_FLT0TMHT */
    reg = HPDF_FLTyTMHT(filtery);
    reg &= ~(HPDF_FLTyTMHT_HTVAL | HPDF_FLTyTMHT_HTBSD);
    reg |= (((uint32_t)init_struct->tm_high_threshold << FLTyTMHT_HTVAL_OFFSET) | init_struct->ht_break_signal);
    HPDF_FLTyTMHT(filtery) = reg;
    /* configure the HPDF_FLT0TMLT */
    reg = HPDF_FLTyTMLT(filtery);
    reg &= ~(HPDF_FLTyTMLT_LTVAL | HPDF_FLTyTMLT_LTBSD);
    reg |= (((uint32_t)init_struct->tm_low_threshold << FLTyTMLT_LTVAL_OFFSET) | init_struct->lt_break_signal);
    HPDF_FLTyTMLT(filtery) = reg;
}

/*!
    \brief      initialize the regular conversion
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  init_struct: the initialization data needed to initialize regular conversion
    \param[out] none
    \retval     none
*/
void hpdf_rc_init(hpdf_filter_enum filtery, hpdf_rc_parameter_struct* init_struct)
{
    uint32_t reg;
    /* configure the HPDF_FLT0CTL0 */
    reg = HPDF_FLTyCTL0(filtery);
    reg &= ~(HPDF_FLTyCTL0_FAST | HPDF_FLTyCTL0_RCS | HPDF_FLTyCTL0_RCDMAEN | HPDF_FLTyCTL0_RCSYN | HPDF_FLTyCTL0_RCCM);
    reg |= (init_struct->continuous_mode | init_struct->fast_mode | init_struct->rcdmaen | init_struct->rcsyn |\
            init_struct->rcs_channel);
    HPDF_FLTyCTL0(filtery) = reg;
}

/*!
    \brief      initialize the inserted conversion
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  init_struct: the initialization data needed to initialize inserted conversion
    \param[out] none
    \retval     none
*/
void hpdf_ic_init(hpdf_filter_enum filtery, hpdf_ic_parameter_struct* init_struct)
{
    uint32_t reg;
    /* configure the HPDF_FLT0CTL0 */
    reg = HPDF_FLTyCTL0(filtery);
    reg &= ~(HPDF_FLTyCTL0_ICTEEN | HPDF_FLTyCTL0_ICTSSEL | HPDF_FLTyCTL0_ICDMAEN | HPDF_FLTyCTL0_SCMOD |\
             HPDF_FLTyCTL0_ICSYN);
    reg |= (init_struct->trigger_dege | init_struct->trigger_signal | init_struct->icdmaen | init_struct->scmod |\
                init_struct->icsyn);
    HPDF_FLTyCTL0(filtery) = reg;
    /* configure the HPDF_FLT0IGCS */
    reg = HPDF_FLTyIGCS(filtery);
    reg &= ~HPDF_FLTyIGCS_IGCSEL; 
    reg |= init_struct->ic_channel_group;
    HPDF_FLTyIGCS(filtery) = reg;
}

/*!
    \brief      configure serial output clock
    \param[in]  source: the HPDF serial clock output source
                only one parameter can be selected which is shown as below:
    \arg          SERIAL_SYSTEM_CLK: serial clock output source is from system clock
    \arg          SERIAL_AUDIO_CLK: serial clock output source is from audio clock
    \param[in]  divider: serial clock output divider 0-255
    \param[in]  mode: serial clock output duty mode
                only one parameter can be selected which is shown as below:
    \arg          CKOUTDM_DISABLE: disable serial clock output duty mode
    \arg          CKOUTDM_ENABLE: enable serial clock output duty mode
    \param[out] none
    \retval     none
*/
void hpdf_clock_output_config(uint32_t source, uint8_t divider, uint32_t mode)
{
    uint32_t reg;
    reg = HPDF_CHxCTL(CHANNEL0);
    reg &= ~(HPDF_CH0CTL_CKOUTSEL | HPDF_CH0CTL_CKOUTSEL | HPDF_CH0CTL_CKOUTDM);
    /* configure serial output clock */
    reg |= (source | ((uint32_t)divider << CH0CTL_CKOUTDIV_OFFSET) | mode);
    HPDF_CHxCTL(CHANNEL0) = reg;
}

/*!
    \brief      configure serial clock output source
    \param[in]  source: the HPDF serial clock output source
    \arg          SERIAL_SYSTEM_CLK: serial clock output source is from system clock
    \arg          SERIAL_AUDIO_CLK: serial clock output source is from audio clock
    \param[out] none
    \retval     none
*/
void hpdf_clock_output_source_config(uint32_t source)
{
    uint32_t reg;
    reg = HPDF_CHxCTL(CHANNEL0);
    reg &= ~HPDF_CH0CTL_CKOUTSEL;
    reg |= source;
    HPDF_CHxCTL(CHANNEL0) = reg;
}

/*!
    \brief      disable serial clock output duty mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hpdf_clock_output_duty_mode_disable(void)
{
    /* make sure the HPDF_CH0CTL_HPDFEN=0 */
    if(RESET == (HPDF_CHxCTL(CHANNEL0) & HPDF_CH0CTL_HPDFEN)){
        HPDF_CHxCTL(CHANNEL0) &= ~CKOUTDM_ENABLE;
    }
}

/*!
    \brief      enable serial clock output duty mode
    \param[in]  none
    \param[out] none
    \retval     none
*/
void hpdf_clock_output_duty_mode_enable(void)
{
    /* make sure the HPDF_CH0CTL_HPDFEN=0 */
    if(RESET == (HPDF_CHxCTL(CHANNEL0) & HPDF_CH0CTL_HPDFEN)){
        HPDF_CHxCTL(CHANNEL0) |= CKOUTDM_ENABLE;
    }
}

/*!
    \brief      configure serial clock output divider
    \param[in]  divider: serial clock output divider 0-255
    \param[out] none
    \retval     none
*/
void hpdf_clock_output_divider_config(uint8_t divider)
{
    uint32_t reg;
    /* make sure the HPDF_CH0CTL_HPDFEN=0 */
    if(RESET == (HPDF_CHxCTL(CHANNEL0) & HPDF_CH0CTL_HPDFEN)){
        reg = HPDF_CHxCTL(CHANNEL0);
        reg &= ~HPDF_CH0CTL_CKOUTDIV;
        reg |= ((uint32_t)divider << CH0CTL_CKOUTDIV_OFFSET);
        HPDF_CHxCTL(CHANNEL0) = reg;
    }
}

/*!
    \brief      enable the HPDF channel
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_channel_enable(hpdf_channel_enum channelx)
{
    HPDF_CHxCTL(channelx) |= HPDF_CHxCTL_CHEN;
}

/*!
    \brief      disable the HPDF channel
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_channel_disable(hpdf_channel_enum channelx)
{
    HPDF_CHxCTL(channelx) &= ~HPDF_CHxCTL_CHEN;
}

/*!
    \brief      configure SPI clock source
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  clock_source: SPI clock source
                only one parameter can be selected which is shown as below:
    \arg          EXTERNAL_CKIN: external input clock 
    \arg          INTERNAL_CKOUT: internal CKOUT clock 
    \arg          HALF_CKOUT_FALLING_EDGE: internal CKOUT clock, sampling point on each second CKOUT falling edge
    \arg          HALF_CKOUT_RISING_EDGE: internal CKOUT clock, sampling point on each second CKOUT rising edge
    \param[out] none
    \retval     none
*/
void hpdf_spi_clock_source_config(hpdf_channel_enum channelx, uint32_t clock_source)
{
    uint32_t reg;
    reg = HPDF_CHxCTL(channelx);
    /* make sure the CHEN=0 */
    if(RESET == (reg & HPDF_CHxCTL_CHEN)){
        reg &= ~HPDF_CHxCTL_SPICKSS;
        reg |= clock_source;
        HPDF_CHxCTL(channelx) = reg;
    }
}

/*!
    \brief      configure serial interface type
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  type: serial interface type
                only one parameter can be selected which is shown as below:
    \arg          SPI_RISING_EDGE: SPI interface, sample data on rising edge 
    \arg          SPI_FALLING_EDGE: SPI interface, sample data on rising edge
    \arg          MANCHESTER_CODE0: Manchester coded input: rising edge = logic 0, falling edge = logic 1
    \arg          MANCHESTER_CODE1: Manchester coded input: rising edge = logic 1, falling edge = logic 0
    \param[out] none
    \retval     none
*/
void hpdf_serial_interface_type_config(hpdf_channel_enum channelx, uint32_t type)
{
    uint32_t reg;
    reg = HPDF_CHxCTL(channelx);
    /* make sure the CHEN=0 */
    if(RESET == (reg & HPDF_CHxCTL_CHEN)){
        reg &= ~HPDF_CHxCTL_SITYP; 
        reg |= type;
        HPDF_CHxCTL(channelx) = reg;
    }
}

/*!
    \brief      disable malfunction monitor 
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_malfunction_monitor_disable(hpdf_channel_enum channelx)
{
    HPDF_CHxCTL(channelx) &= ~HPDF_CHxCTL_MMEN;
}
/*!
    \brief      enable malfunction monitor 
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_malfunction_monitor_enable(hpdf_channel_enum channelx)
{
    HPDF_CHxCTL(channelx) |= HPDF_CHxCTL_MMEN;
}

/*!
    \brief      disable clock loss detector 
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_clock_loss_disable(hpdf_channel_enum channelx)
{
    HPDF_CHxCTL(channelx) &= ~HPDF_CHxCTL_CKLEN;
}

/*!
    \brief      enable clock loss detector 
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_clock_loss_enable(hpdf_channel_enum channelx)
{
    HPDF_CHxCTL(channelx) |= HPDF_CHxCTL_CKLEN;
}

/*!
    \brief      disable channel inputs pins redirection
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_channel_pin_redirection_disable(hpdf_channel_enum channelx)
{
    /* make sure the CHEN=0 */
    if(RESET == (HPDF_CHxCTL(channelx) & HPDF_CHxCTL_CHEN)){
        HPDF_CHxCTL(channelx) &= ~HPDF_CHxCTL_CHPINSEL;
    }
}

/*!
    \brief      enable channel inputs pins redirection
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_channel_pin_redirection_enable(hpdf_channel_enum channelx)
{
    /* make sure the CHEN=0 */
    if(RESET == (HPDF_CHxCTL(channelx) & HPDF_CHxCTL_CHEN)){
        HPDF_CHxCTL(channelx) |= HPDF_CHxCTL_CHPINSEL;
    }
}

/*!
    \brief      configure channel multiplexer select input data source
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  data_source: input data source
                only one parameter can be selected which is shown as below:
    \arg          SERIAL_INPUT: input data source is taken from serial inputs 
    \arg          INTERNAL_INPUT: input data source is taken from internal HPDF_CHxPDI register
    \param[out] none
    \retval     none
*/
void hpdf_channel_multiplexer_config(hpdf_channel_enum channelx, uint32_t data_source)
{
    uint32_t reg;
    reg = HPDF_CHxCTL(channelx);
    /* make sure the CHEN=0 */
    if(RESET == (reg & HPDF_CHxCTL_CHEN)){
        reg &= ~HPDF_CHxCTL_CMSD;
        /* configure the input data source */
        reg |= data_source;
        HPDF_CHxCTL(channelx) = reg;
    }
}

/*!
    \brief      configure data packing mode
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  mode: parallel data packing mode
                only one parameter can be selected which is shown as below:
    \arg          DPM_STANDARD_MODE : standard mode
    \arg          DPM_INTERLEAVED_MODE: interleaved mode
    \arg          DPM_DUAL_MODE: dual mode
    \param[out] none
    \retval     none
*/
void hpdf_data_pack_mode_config(hpdf_channel_enum channelx, uint32_t mode)
{
    uint32_t reg;
    reg = HPDF_CHxCTL(channelx);
    /* make sure the CHEN=0 */
    if(RESET == (reg & HPDF_CHxCTL_CHEN)){
        reg &= ~HPDF_CHxCTL_DPM;
        /* configure the data packing mode */
        reg |= mode;
        HPDF_CHxCTL(channelx) = reg;
    }
}
/*!
    \brief      configure data right bit-shift
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  right_shift: the number of bits that determine the right shift(0-31)
    \param[out] none
    \retval     none
*/
void hpdf_data_right_bit_shift_config(hpdf_channel_enum channelx, uint8_t right_shift)
{
    uint32_t reg;
    /* make sure the CHEN=0 */
    if(RESET == (HPDF_CHxCTL(channelx) & HPDF_CHxCTL_CHEN)){
        reg = HPDF_CHxCFG0(channelx);
        reg &= ~HPDF_CHxCFG0_DTRS;
        /* configure the right shift */
        reg |= ((uint32_t)right_shift << CHyCFG_DTRS_OFFSET);
        HPDF_CHxCFG0(channelx) = reg;
    }
}

/*!
    \brief      configure calibration offset
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  offset: 24-bit calibration offset, must be in (-8388608~8388607)
    \param[out] none
    \retval     none
*/
void hpdf_calibration_offset_config(hpdf_channel_enum channelx, int32_t offset)
{
    uint32_t reg;
    reg = HPDF_CHxCFG0(channelx);
    reg &= ~HPDF_CHxCFG0_CALOFF;
    /* configure the calibration offset */
    reg |= ((uint32_t)offset << CHyCFG0_CALOFF_OFFSET);
    HPDF_CHxCFG0(channelx) = reg;
}

/*!
    \brief      configure malfunction monitor break signal
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  break_signal: malfunction monitor break signal distribution
                  only one parameter can be selected which is shown as below:
    \arg          NO_MM_BREAK: break signal is not distributed to malfunction monitor on channel
    \arg          MM_BREAK0: break signal 0 is distributed to malfunction monitor on channel
    \arg          MM_BREAK1: break signal 1 is distributed to malfunction monitor on channel
    \arg          MM_BREAK0_1: break signal 0 and 1 is distributed to malfunction monitor on channel
    \param[out] none
    \retval     none
*/
void hpdf_malfunction_break_signal_config(hpdf_channel_enum channelx, uint32_t break_signal)
{
    uint32_t reg;
    reg = HPDF_CHxCFG1(channelx);
    reg &= ~HPDF_CHxCFG1_MMBSD;
    /* configure the break signal */
    reg |= break_signal; 
    HPDF_CHxCFG1(channelx) = reg;
}

/*!
    \brief      configure malfunction monitor counter threshold
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  threshold: malfunction monitor counter threshold(0-255)
    \param[out] none
    \retval     none
*/
void hpdf_malfunction_counter_config(hpdf_channel_enum channelx, uint8_t threshold)
{
    uint32_t reg;
    reg = HPDF_CHxCFG1(channelx);
    reg &= ~HPDF_CHxCFG1_MMCT;
    /* configure the malfunction monitor counter threshold */
    reg |= threshold;
    HPDF_CHxCFG1(channelx) = reg;
}

/*!
    \brief      write the parallel data on standard mode of data packing
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  data: the parallel data 
    \param[out] none
    \retval     none
*/
void hpdf_write_parallel_data_standard_mode(hpdf_channel_enum channelx, int16_t data)
{
    uint32_t reg;
    /* make sure HPDF channel is used receive parallel data */
    if(INTERNAL_INPUT == (HPDF_CHxCTL(channelx) & INTERNAL_INPUT)){
        /* make sure the data pack of HPDF_CHxPDI register is standard mode */
        if(DPM_STANDARD_MODE == (HPDF_CHxCTL(channelx) & DPM_STANDARD_MODE)){
            reg = HPDF_CHxPDI(channelx);
            reg &= ~HPDF_CHxPDI_IDATA0;
            reg |= (uint16_t)data;
            HPDF_CHxPDI(channelx) = reg;
        }
    }
}

/*!
    \brief      write the parallel data on interleaved mode of data packing
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  data: the parallel data 
    \param[out] none
    \retval     none
*/
void hpdf_write_parallel_data_interleaved_mode(hpdf_channel_enum channelx, int32_t data)
{
    uint32_t reg;
    /* make sure HPDF channel is used receive parallel data */
    if(INTERNAL_INPUT == (HPDF_CHxCTL(channelx) & INTERNAL_INPUT)){
        /* make sure the data pack of HPDF_CH0PDI register is interleaved mode */
        if(DPM_INTERLEAVED_MODE == (HPDF_CHxCTL(channelx) & DPM_INTERLEAVED_MODE)){
            reg = HPDF_CHxPDI(channelx);
            reg &= ~(HPDF_CHxPDI_IDATA0 | HPDF_CHxPDI_IDATA1);
            reg |= (uint32_t)data;
            HPDF_CHxPDI(channelx) = reg;
        }
    }
}

/*!
    \brief      write the parallel data on dual mode of data packing  
    \param[in]  channelx: CHANNELy(y=0)
    \param[in]  data: the parallel data 
    \param[out] none
    \retval     none
*/
void hpdf_write_parallel_data_dual_mode(hpdf_channel_enum channelx, int32_t data)
{
    uint32_t reg;
    /* make sure HPDF channel is used receive parallel data */
    if(INTERNAL_INPUT == (HPDF_CHxCTL(CHANNEL0) & INTERNAL_INPUT)){
        /* make sure the data pack of HPDF_CH0PDI register is dual mode */
        if(DPM_DUAL_MODE == (HPDF_CHxCTL(CHANNEL0) & DPM_DUAL_MODE)){
            reg = HPDF_CHxPDI(CHANNEL0);
            reg &= ~(HPDF_CHxPDI_IDATA0 | HPDF_CHxPDI_IDATA1);
            reg |= (uint32_t)data;
            HPDF_CHxPDI(CHANNEL0) = reg;
        }
    }
}

/*!
    \brief      update the number of pulses to skip
    \param[in]  channelx: CHANNELy(y=0)
    \param[in]  number: the number of serial input samples that will be skipped
    \param[out] none
    \retval     none
*/
void hpdf_pulse_skip_update(hpdf_channel_enum channelx, uint8_t number)
{
    uint32_t reg;
    reg = HPDF_CHxPS(channelx);
    reg &= ~HPDF_CHxPS_PLSK;
    /* update the number of pulses to skip */
    reg |= (uint32_t)number;
    HPDF_CHxPS(channelx) = reg;
}

/*!
    \brief      read the number of pulses to skip
    \param[in]  channelx: CHANNELy(y=0)
    \param[out] none
    \retval     the number of pulses to skip
*/
uint8_t hpdf_pulse_skip_read(hpdf_channel_enum channelx)
{
    uint8_t val;
    /* read the number of pulses to skip */
    val = (uint8_t)HPDF_CHxPS(channelx);
    return val;
}

/*!
    \brief      enable filter
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_filter_enable(hpdf_filter_enum filtery)
{
    HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_FLTEN;
}

/*!
    \brief      disable filter
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_filter_disable(hpdf_filter_enum filtery)
{
    HPDF_FLTyCTL0(filtery) &= ~HPDF_FLTyCTL0_FLTEN;
}

/*!
    \brief      configure sinc filter order and oversample
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  order: sinc filter order
                only one parameter can be selected which is shown as below:
    \arg          FLT_FASTSINC: FastSinc filter type
    \arg          FLT_SINC1: Sinc1 filter type
    \arg          FLT_SINC2: Sinc2 filter type
    \arg          FLT_SINC3: Sinc3 filter type
    \arg          FLT_SINC4: Sinc4 filter type
    \arg          FLT_SINC5: Sinc5 filter type
    \param[in]  oversample: Sinc filter oversampling rate(1-1024)
    \param[out] none
    \retval     none
*/
void hpdf_filter_config(hpdf_filter_enum filtery, uint32_t order, uint16_t oversample)
{
    uint32_t reg;
    
    if(oversample > 0U){
        oversample = oversample - 1U;
    }
    /* make sure the FLTEN=0 */
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        reg = HPDF_FLTySFCFG(filtery);
        reg &= ~(HPDF_FLTySFCFG_SFO |HPDF_FLTySFCFG_SFOR);
        /* configure the sinc filter order and oversample */
        reg |= (order | ((uint32_t)oversample << FLTySFCFG_SFOR_OFFSET));
        HPDF_FLTySFCFG(filtery) = reg;
    }
}

/*!
    \brief      configure integrator oversampling rate
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  oversample: integrator oversampling rate(1-256)
    \param[out] none
    \retval     none
*/
void hpdf_integrator_oversample(hpdf_filter_enum filtery, uint8_t oversample)
{
    uint32_t reg;
    
    if(oversample > 0U){
        oversample = oversample - 1U;
    }
    /* make sure the FLTEN=0 */
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        reg = HPDF_FLTySFCFG(filtery);
        reg &= ~HPDF_FLTySFCFG_IOR;
        /* configure the integrator oversampling rate */
        reg |= (uint32_t)oversample;
        HPDF_FLTySFCFG(filtery) = reg;
    }
}

/*!
    \brief      configure threshold monitor filter order and oversample
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[in]  order: threshold monitor Sinc filter order
                only one parameter can be selected which is shown as below:
    \arg          TM_FASTSINC: FastSinc filter type
    \arg          TM_SINC1: Sinc1 filter type
    \arg          TM_SINC2: Sinc2 filter type
    \arg          TM_SINC3: Sinc3 filter type
    \param[in]  oversample: Sinc filter oversampling rate(1-32)
    \param[out] none
    \retval     none
*/
void hpdf_threshold_monitor_filter_config(hpdf_channel_enum channelx, uint32_t order, uint8_t oversample)
{
    uint32_t reg;
    
    if(oversample > 0U){
        oversample = oversample - 1U;
    }
    /* make sure the CHEN=0 */
    if(RESET == (HPDF_CHxCTL(channelx) & HPDF_CHxCTL_CHEN )){
        reg = HPDF_CHxCFG1(channelx);
        reg &= ~(HPDF_CHxCFG1_TMSFO | HPDF_CHxCFG1_TMFOR);
        /* configure the threshold monitor filter order and oversample rate */
        reg |= (order | ((uint32_t)oversample << CHyCFG1_TMFOR_OFFSET));
        HPDF_CHxCFG1(channelx) = reg;
   }
}

/*!
    \brief      read the threshold monitor filter data
    \param[in]  channelx: CHANNELx(x=0,1)
    \param[out] none
    \retval     the threshold monitor filter data
*/
int16_t hpdf_threshold_monitor_filter_read_data(hpdf_channel_enum channelx)
{
    int16_t val;
    val = (int16_t)HPDF_CHxTMFDT(channelx);
    return val;
}

/*!
    \brief      disable threshold monitor fast mode 
    \param[in]  filtery: HPDF_FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_threshold_monitor_fast_mode_disable(hpdf_filter_enum filtery)
{
    HPDF_FLTyCTL0(filtery) &= ~HPDF_FLTyCTL0_TMFM;
}

/*!
    \brief      enable threshold monitor fast mode
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_threshold_monitor_fast_mode_enable(hpdf_filter_enum filtery)
{
    HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_TMFM;
}

/*!
    \brief      configure threshold monitor channel
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  channel: which channel use threshold monitorx(x=0,1)
                only one parameter can be selected which is shown as below:
    \arg          TMCHEN_DISABLE: threshold monitorx is disabled on channel 0 and channel 1
    \arg          TMCHEN_CHANNEL0 threshold monitor x is enabled on channel 0
    \arg          TMCHEN_CHANNEL1: threshold monitor x is enabled on channel 1
    \arg          TMCHEN_CHANNEL0_1: threshold monitor x is enabled on channel 0 and channel 1
    \param[out] none
    \retval     none
*/
void hpdf_threshold_monitor_channel(hpdf_filter_enum filtery, uint32_t channel)
{
    uint32_t reg;
    reg = HPDF_FLTyCTL1(filtery);
    reg &= ~HPDF_FLTyCTL1_TMCHEN;
    /* configure the channel which threshold monitor watch on */
    reg |= channel;
    HPDF_FLTyCTL1(filtery) = reg;
}

/*!
    \brief      configure threshold monitor high threshold value
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  value: high threshold value(-8388608~8388607)
    \param[out] none
    \retval     none
*/
void hpdf_threshold_monitor_high_threshold(hpdf_filter_enum filtery, int32_t value)
{
    uint32_t reg;
    reg = HPDF_FLTyTMHT(filtery);
    reg &= ~HPDF_FLTyTMHT_HTVAL;
    /* write the signed value */
    reg |= (uint32_t)value << FLTyTMHT_HTVAL_OFFSET;
    HPDF_FLTyTMHT(filtery) = reg;
}

/*!
    \brief      configure threshold monitor low threshold value
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  value: low threshold value(-8388608~8388607)
    \param[out] none
    \retval     none
*/
void hpdf_threshold_monitor_low_threshold(hpdf_filter_enum filtery, int32_t value)
{
    uint32_t reg;
    reg = HPDF_FLTyTMLT(filtery);
    reg &= ~HPDF_FLTyTMLT_LTVAL;
    /* write the signed value */
    reg |= (uint32_t)value << FLTyTMLT_LTVAL_OFFSET ;
    HPDF_FLTyTMLT(filtery) = reg;
}

/*!
    \brief      configure threshold monitor high threshold event break signal
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]   break_signal: HPDF break signal
                 only one parameter can be selected which is shown as below:
    \arg          NO_TM_HT_BREAK: break signal is not distributed to an threshold monitor high threshold event
    \arg          TM_HT_BREAK0: break signal 0 is distributed to an threshold monitor high threshold event
    \arg          TM_HT_BREAK1: break signal 1 is distributed to an threshold monitor high threshold event
    \arg          TM_HT_BREAK0_1: break signal 0 and 1 is distributed to an threshold monitor high threshold event
    \param[out] none
    \retval     none
*/
void hpdf_high_threshold_break_signal(hpdf_filter_enum filtery, uint32_t break_signal)
{
    uint32_t reg;
    reg = HPDF_FLTyTMHT(filtery);
    reg &= ~HPDF_FLTyTMHT_HTBSD;
    /* configure the break signal */
    reg |= break_signal;
    HPDF_FLTyTMHT(filtery) = reg;
}

/*!
    \brief      configure threshold monitor low threshold event break signal
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]   break_signal: the HPDF break signal
                 only one parameter can be selected which is shown as below:
    \arg          NO_TM_LT_BREAK: break signal is not distributed to an threshold monitor high threshold event
    \arg          TM_LT_BREAK0: break signal 0 is distributed to an threshold monitor high threshold event
    \arg          TM_LT_BREAK1: break signal 1 is distributed to an threshold monitor high threshold event
    \arg          TM_LT_BREAK0_1: break signal 0 and 1 is distributed to an threshold monitor high threshold event
    \param[out] none
    \retval     none
*/
void hpdf_low_threshold_break_signal(hpdf_filter_enum filtery, uint32_t break_signal)
{
    uint32_t reg;
    reg = HPDF_FLTyTMLT(filtery);
    reg &= ~HPDF_FLTyTMLT_LTBSD;
    /* configure the break signal */
    reg |= break_signal;
    HPDF_FLTyTMLT(filtery) = reg;
}

/*!
    \brief      configure extremes monitor channel
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  channel: which channel use extremes monitor y (y=0,1)
                only one parameter can be selected which is shown as below:
    \arg          EM_CHANNEL_DISABLE: extremes monitor y does not accept data from channel 0 and channel 1
    \arg          EM_CHANNEL0: extremes monitor y accepts data from channel 0
    \arg          EM_CHANNEL1: extremes monitor y accepts data from channel 1
    \arg          EM_CHANNEL0_1: extremes monitor y accepts data from channel 0 and channel 1
    \param[out] none
    \retval     none
*/
void hpdf_extremes_monitor_channel(hpdf_filter_enum filtery, uint32_t channel)
{
    uint32_t reg;
    reg = HPDF_FLTyCTL1(filtery);
    reg &= ~HPDF_FLTyCTL1_EMCS;
    /* configure the channel which channel use extremes monitor */
    reg |= channel;
    HPDF_FLTyCTL1(filtery) = reg;
}

/*!
    \brief      get the extremes monitor maximum value
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     the maximum value
*/
int32_t hpdf_extremes_monitor_maximum_get(hpdf_filter_enum filtery)
{
    uint32_t val;
    /* get the maximum value */
    val = HPDF_FLTyEMMAX(filtery) >> FLTyEMMAX_MAXVAL_OFFSET;   
    /* get the sign of vlaue */
    if(val & 0x00800000U){
        val |= 0xFF000000U; 
    }
    return (int32_t)val;
}

/*!
    \brief      get the extremes monitor minimum value
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     the minimum value
*/
int32_t hpdf_extremes_monitor_minimum_get(hpdf_filter_enum filtery)
{
    uint32_t val;
    /* get the channel of maximum value */
    val = HPDF_FLTyEMMIN(filtery) >> FLTyEMMIN_MINVAL_OFFSET;
    /* get the sign of vlaue */
    if(val & 0x00800000U){
        val |= 0xFF000000U; 
    }
    return (int32_t)val; 
}

/*!
    \brief      disable regular conversions continuous mode
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_rc_continuous_disable(hpdf_filter_enum filtery)
{
    HPDF_FLTyCTL0(filtery) &= ~HPDF_FLTyCTL0_RCCM;
}

/*!
    \brief      enable regular conversions continuous mode 
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_rc_continuous_enable(hpdf_filter_enum filtery)
{
    HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_RCCM;
}

/*!
    \brief      start regular channel conversion by software
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_rc_start_by_software(hpdf_filter_enum filtery)
{
    HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_SRCS;
}

/*!
    \brief      disable regular conversion synchronously 
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_rc_syn_disable(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        HPDF_FLTyCTL0(filtery) &= ~HPDF_FLTyCTL0_RCSYN;
    }
}

/*!
    \brief      enable regular conversion synchronously
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_rc_syn_enable(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_RCSYN;
    }
}

/*!
    \brief      disable regular conversion DMA channel
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_rc_dma_disable(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
       HPDF_FLTyCTL0(filtery) &= ~HPDF_FLTyCTL0_RCDMAEN; 
    }
}

/*!
    \brief      enable regular conversion DMA channel 
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_rc_dma_enable(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
       HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_RCDMAEN; 
    }
}

/*!
    \brief      configure regular conversion channel
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  channelx: the HPDF channel
                only one parameter can be selected which is shown as below:
    \arg          CHANNEL0: the HPDF channel0
    \arg          CHANNEL1: the HPDF channel1
    \param[out] none
    \retval     none
*/
void hpdf_rc_channel_config(hpdf_filter_enum filtery, hpdf_channel_enum channelx)
{
    uint32_t reg;
    reg = HPDF_FLTyCTL0(filtery);
    reg &= HPDF_FLTyCTL0_RCS;
    reg |= HPDF_FLTyCTL0_RCS;
    HPDF_FLTyCTL0(filtery) = reg;
}

/*!
    \brief      disable regular conversion fast conversion mode
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_rc_fast_mode_disable(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        HPDF_FLTyCTL0(filtery) &= ~HPDF_FLTyCTL0_FAST;
    }
}

/*!
    \brief      enable regular conversion fast conversion mode
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_rc_fast_mode_enable(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_FAST;
    }
}

/*!
    \brief      get the regular conversions data
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     regular conversions data
*/
int32_t hpdf_rc_data_get(hpdf_filter_enum filtery)
{
    uint32_t val;
    /* get the signed data */
    val = HPDF_FLTyRDATA(filtery) >> FLTyRDATAT_RDATA_OFFSET;   
    /* get the sign of vlaue */
    if(val & 0x00800000U){
        val |= 0xFF000000U; 
    }
    return (int32_t)val; 
}

/*!
    \brief      get the channel of regular group channel most recently converted 
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     the channel
*/
uint8_t hpdf_rc_channel_get(hpdf_filter_enum filtery)
{
    uint8_t val;
    val = (uint8_t)HPDF_FLTyRDATA(filtery);
    val &= (uint8_t)HPDF_FLTyRDATA_RCCH;
    return val;
}

/*!
    \brief      start inserted channel conversion by software
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_ic_start_by_software(hpdf_filter_enum filtery)
{
    HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_SICC;
}

/*!
    \brief      disable inserted conversion synchronously 
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_ic_syn_disable(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        HPDF_FLTyCTL0(filtery) &= ~HPDF_FLTyCTL0_ICSYN;
    }
}

/*!
    \brief      enable inserted conversion synchronously 
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_ic_syn_enable(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_ICSYN;
    }
}

/*!
    \brief      disable inserted conversion DMA channel
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_ic_dma_disable(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
       HPDF_FLTyCTL0(filtery) &= ~HPDF_FLTyCTL0_ICDMAEN;
    }
}

/*!
    \brief      enable inserted conversion DMA channel
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_ic_dma_enable(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
       HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_ICDMAEN;
    }
}

/*!
    \brief      disable scan conversion mode
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_ic_scan_mode_disable(hpdf_filter_enum filtery)
{
     if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        HPDF_FLTyCTL0(filtery) &= ~HPDF_FLTyCTL0_SCMOD;
     }
}

/*!
    \brief      enable scan conversion mode
    \param[in]  filtery: HPDF_FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_ic_scan_mode_enable(hpdf_filter_enum filtery)
{
     if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        HPDF_FLTyCTL0(filtery) |= HPDF_FLTyCTL0_SCMOD;
     }
}

/*!
    \brief      disable inserted conversions trigger siganl
    \param[in]  filtery: FLTy(y=0,1)
    \param[out] none
    \retval     none
*/
void hpdf_ic_trigger_signal_disbale(hpdf_filter_enum filtery)
{
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        HPDF_FLTyCTL0(filtery) &= ~HPDF_FLTyCTL0_ICTEEN;
    }
}

/*!
    \brief      configure inserted conversions trigger siganl and trigger edge
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  trigger: inserted conversions trigger signal
                only one parameter can be selected which is shown as below:
    \arg          HPDF_ITRG0: TIMER1_TRGO is selected to start inserted conversion
    \arg          HPDF_ITRG1: TIMER2_TRGO is selected to start inserted conversion
    \arg          HPDF_ITRG2: TIMER3_TRGO is selected to start inserted conversion
    \arg          HPDF_ITRG3: TIMER4_TRGO is selected to start inserted conversion
    \arg          HPDF_ITRG24: EXTI11 is selected to start inserted conversion
    \arg          HPDF_ITRG25: EXTI15 is selected to start inserted conversion
    \arg          HPDF_ITRG26: TIMER5_TRGO is selected to start inserted conversion
    \param[in]  trigger_edge: inserted conversions trigger edge
                only one parameter can be selected which is shown as below:
    \arg          TRG_DISABLE: disable trigger siganl
    \arg          RISING_EDGE_TRG: rising edge on the trigger signal
    \arg          FALLING_EDGE_TRG: falling edge on the trigger signal
    \arg          EDGE_TRG: edge (rising edges and falling edges) on the trigger signal
    \param[out] none
    \retval     none
*/
void hpdf_ic_trigger_signal_config(hpdf_filter_enum filtery,  uint32_t trigger, uint32_t trigger_edge)
{
    uint32_t reg;
    /* make sure the FLTEN=0 */
    if(RESET == (HPDF_FLTyCTL0(filtery) & HPDF_FLTyCTL0_FLTEN)){
        reg = HPDF_FLTyCTL0(filtery);
        reg &= ~(HPDF_FLTyCTL0_ICTEEN | HPDF_FLTyCTL0_ICTSSEL);
        /* configure inserted conversions trigger siganl and trigger edge */
        reg |= (trigger | trigger_edge);
        HPDF_FLTyCTL0(filtery) = reg;
    }
}

/*!
    \brief      configure inserted group conversions channel
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  channel: the HPDF channel belongs to inserted group
                only one parameter can be selected which is shown as below:
    \arg          IGCSEL_CHANNEL0: channel0 belongs to the inserted group
    \arg          IGCSEL_CHANNEL1: channel1 belongs to the inserted group
    \arg          IGCSEL_CHANNEL0_1: channel0 and channel1 belongs to the inserted group
    \param[out] none
    \retval     none
*/
void hpdf_ic_channel_config(hpdf_filter_enum filtery, uint32_t channel)
{
    uint32_t reg;
    reg = HPDF_FLTyIGCS(filtery);
    reg &= ~HPDF_FLTyIGCS_IGCSEL;
    reg |= channel;
    HPDF_FLTyIGCS(filtery) = reg;
}

/*!
    \brief      get the inserted conversions data
    \param[in]  filtery: HPDF_FLTy(y=0,1)
    \param[out] none
    \retval     inserted conversions data
*/
int32_t hpdf_ic_data_get(hpdf_filter_enum filtery)
{
    uint32_t val;
    /* get the unsigned data  */
    val = HPDF_FLTyIDATA(filtery) >> FLTyIDATAT_IDATA_OFFSET;
    /* get the sign of vlaue */
    /* get the sign of vlaue */
    if(val & 0x00800000U){
        val |= 0xFF000000U; 
    }
    /* get the signed data */   
    return (int32_t)val; 
}

/*!
    \brief      get the channel of inserted group channel most recently converted 
    \param[in]  filtery: HPDF_FLTy(y=0,1)
    \param[out] none
    \retval     the channel
*/
uint8_t hpdf_ic_channel_get(hpdf_filter_enum filtery)
{
    uint8_t val;
    val = (uint8_t)HPDF_FLTyIDATA(filtery);
    val &= (uint8_t)HPDF_FLTyIDATA_ICCH;
    return val;
}

/*!
    \brief      get the HPDF flags
    \param[in]  filtery: FLTy(y=0,1)
    \param[in]  flag: HPDF flags, refer to hpdf_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        HPDF_FLAG_FLTy_ICEF: FLTy inserted conversion end flag
      \arg        HPDF_FLAG_FLTy_RCEF: FLTy regular conversion end flag
      \arg        HPDF_FLAG_FLTy_ICDOF: FLTy inserted conversion data overflow flag
      \arg        HPDF_FLAG_FLTy_RCDOF: FLTy regular conversion data overflow flag
      \arg        HPDF_FLAG_FLTy_TMEOF: FLTy threshold monitor event occurred flag
      \arg        HPDF_FLAG_FLTy_ICPF: FLTy inserted conversion in progress flag
      \arg        HPDF_FLAG_FLTy_RCPF: FLTy regular conversion in progress flag
      \arg        HPDF_FLAG_FLT0_CKLF0: clock signal is lost on channel 0 flag
      \arg        HPDF_FLAG_FLT0_CKLF1: clock signal is lost on channel 1 flag
      \arg        HPDF_FLAG_FLT0_MMF0: malfunction event occurred on channel 0 flag
      \arg        HPDF_FLAG_FLT0_MMF1: malfunction occurred on channel 1 flag
      \arg        HPDF_FLAG_FLTy_RCHPDT: FLTy inserted channel most recently converted 
      \arg        HPDF_FLAG_FLTy_LTF0: threshold monitor low threshold flag on channel 0 flag
      \arg        HPDF_FLAG_FLTy_LTF1: threshold monitor low threshold flag on channel 1 flag
      \arg        HPDF_FLAG_FLTy_HTF0: threshold monitor high threshold flag on channel 0 flag
      \arg        HPDF_FLAG_FLTy_HTF1: threshold monitor high threshold flag on channel 1 flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hpdf_flag_get(hpdf_filter_enum filtery, hpdf_flag_enum flag)
{
    if(FLT0 == filtery){
        /* get the flag in FLT0 register */
        if(RESET != (HPDF_REG_VAL(HPDF_FLT0, flag) & BIT(HPDF_BIT_POS(flag)))){
            return SET;
        }else{
            return RESET;
        }
    }else{
        /* get the flag in FLT1 register */
        if(RESET != (HPDF_REG_VAL(HPDF_FLT1, flag) & BIT(HPDF_BIT_POS(flag)))){
            return SET;
        }else{
            return RESET;
        }
    }
}

/*!
    \brief      clear the HPDF flags
    \param[in]  hpdf_fltx: FLTy(y=0,1)
    \param[in]  flag: HPDF flags, refer to hpdf_flag_enum
                only one parameter can be selected which is shown as below:
      \arg        HPDF_FLAG_FLTy_ICEF: FLTy inserted conversion end flag
      \arg        HPDF_FLAG_FLTy_RCEF: FLTy regular conversion end flag
      \arg        HPDF_FLAG_FLTy_ICDOF: FLTy inserted conversion data overflow flag
      \arg        HPDF_FLAG_FLTy_RCDOF: FLTy regular conversion data overflow flag
      \arg        HPDF_FLAG_FLTy_TMEOF: FLTy threshold monitor event occurred flag
      \arg        HPDF_FLAG_FLT0_CKLF0: clock signal is lost on channel 0 flag
      \arg        HPDF_FLAG_FLT0_CKLF1: clock signal is lost on channel 1 flag
      \arg        HPDF_FLAG_FLT0_MMF0: malfunction event occurred on channel 0 flag
      \arg        HPDF_FLAG_FLT0_MMF1: malfunction event occurred on channel 1 flag
      \arg        HPDF_FLAG_FLTy_LTF0: threshold monitor low threshold flag on channel 0 flag
      \arg        HPDF_FLAG_FLTy_LTF1: threshold monitor low threshold flag on channel 1 flag
      \arg        HPDF_FLAG_FLTy_HTF0: threshold monitor high threshold flag on channel 0 flag
      \arg        HPDF_FLAG_FLTy_HTF1: threshold monitor high threshold flag on channel 1 flag
    \param[out] none
    \retval     none
*/
void hpdf_flag_clear(hpdf_filter_enum filtery, hpdf_flag_enum flag)
{
    uint32_t val;
    
    /* get the flag position */
    val = BIT(HPDF_BIT_POS(flag));
    /* make sure the range of flag position */
    val = (uint8_t)(((val >= BIT(HPDF_BIT_POS(HPDF_FLAG_FLTy_LTF0))) && (val <= BIT(HPDF_BIT_POS(HPDF_FLAG_FLTy_HTF1)))) ? SET : RESET);
    if(HPDF_FLAG_FLTy_TMEOF == flag){
        /* clear the threshold monitor flag */
        HPDF_FLTyTMFC(filtery)|= (HPDF_FLTyTMFC_HTFC | HPDF_FLTyTMFC_LTFC); 
    }else{
        switch (val){
        case SET:
            if(HPDF_FLAG_FLTy_ICEF == flag){
                /* clear inserted conversion end flag */
                HPDF_FLTyIDATA(filtery);
            }else if(HPDF_FLAG_FLTy_RCEF == flag){
                /* clear regular conversion end flag */
                HPDF_FLTyRDATA(filtery);
            }else{
                /* clear threshold monitor high threshold flag */
                HPDF_FLTyTMFC(filtery) |= BIT(HPDF_BIT_POS(flag));
            }
            break;
        case RESET:
            /* clear the flag by HPDF_FLTyINTC register */ 
            HPDF_FLTyINTC(filtery) |= BIT(HPDF_BIT_POS(flag));
            break;
        default :
            break;
        }
    }
}

/*!
    \brief      enable HPDF interrupt
    \param[in]  hpdf_fltx: FLTy(y=0,1)
    \param[in]  interrupt: HPDF interrupts, refer to hpdf_interrput_enum
                only one parameter can be selected which is shown as below:
      \arg        HPDF_INT_FLTy_ICEIE: FLTy inserted conversion end interrupt enable
      \arg        HPDF_INT_FLTy_RCEIE: FLTy regular conversion end interrupt enable
      \arg        HPDF_INT_FLTy_ICDOIE: FLTy inserted conversion data overflow interrupt enable
      \arg        HPDF_INT_FLTy_RCDOIE: FLTy regular conversion data overflow interrupt enable
      \arg        HPDF_INT_FLTy_TMEOIE: FLTy threshold monitor interrupt enable
      \arg        HPDF_INT_FLT0_MMIE: malfunction monitor interrupt enable
      \arg        HPDF_INT_FLT0_CKLIE: clock loss interrupt enable
    \param[out] none
    \retval     none
*/
void hpdf_interrupt_enable(hpdf_filter_enum filtery, hpdf_interrput_enum interrupt)
{
    if(FLT0 == filtery){
        HPDF_REG_VAL(HPDF_FLT0, interrupt) |= BIT(HPDF_BIT_POS(interrupt));
    }else{
        HPDF_REG_VAL(HPDF_FLT1, interrupt) |= BIT(HPDF_BIT_POS(interrupt));
    }
}

/*!
    \brief      disable HPDF interrupt
    \param[in]  hpdf_fltx: FLTy(y=0,1)
    \param[in]  interrupt: HPDF interrupts, refer to hpdf_interrput_enum
                only one parameter can be selected which is shown as below:
      \arg        HPDF_INT_FLTy_ICEIE: FLTy inserted conversion interrupt enable
      \arg        HPDF_INT_FLTy_RCEIE: FLTy regular conversion interrupt enable
      \arg        HPDF_INT_FLTy_ICDOIE: FLTy inserted conversion data overflow interrupt enable
      \arg        HPDF_INT_FLTy_RCDOIE: FLTy regular conversion data overflow interrupt enable
      \arg        HPDF_INT_FLTy_TMEOIE: FLTy threshold monitor interrupt enable
      \arg        HPDF_INT_FLT0_MMIE: malfunction monitor interrupt enable
      \arg        HPDF_INT_FLT0_CKLIE: clock loss interrupt enable
    \param[out] none
    \retval     none
*/
void hpdf_interrupt_disable(hpdf_filter_enum filtery, hpdf_interrput_enum interrupt)
{
    if(FLT0 == filtery){
        HPDF_REG_VAL(HPDF_FLT0, interrupt) &= BIT(HPDF_BIT_POS(interrupt));
    }else{
        HPDF_REG_VAL(HPDF_FLT1, interrupt) &= BIT(HPDF_BIT_POS(interrupt));
    }
}

/*!
    \brief      get the HPDF interrupt flags
    \param[in]  hpdf_fltx: FLTy(y=0,1)
    \param[in]  interrupt: HPDF flags, refer to hpdf_interrput_enum
                only one parameter can be selected which is shown as below:
      \arg        HPDF_INT_FLAG_FLTy_ICEF: FLTy inserted conversion end interrupt flag
      \arg        HPDF_INT_FLAG_FLTy_RCEF: FLTy regular conversion end interrupt flag
      \arg        HPDF_INT_FLAG_FLTy_ICDOF: FLTy inserted conversion data overflow interrupt flag
      \arg        HPDF_INT_FLAG_FLTy_RCDOF: FLTy regular conversion data overflow interrupt flag
      \arg        HPDF_INT_FLAG_FLTy_TMEOF: FLTy threshold monitor event occurred interrupt flag
      \arg        HPDF_INT_FLAG_FLT0_CKLF0: clock signal is lost on channel 0 interrupt flag
      \arg        HPDF_INT_FLAG_FLT0_CKLF1: clock signal is lost on channel 1 interrupt flag
      \arg        HPDF_INT_FLAG_FLT0_MMF0: malfunction event occurred on channel 0 interrupt flag
      \arg        HPDF_INT_FLAG_FLT0_MMF1: malfunction event occurred on channel 1 interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus hpdf_interrupt_flag_get(hpdf_filter_enum filtery, hpdf_interrput_flag_enum int_flag)
{
    uint32_t int_enable = 0U, flagstatus = 0U;
    if(FLT0 == filtery){
        /* get the interrupt enable bit status */
        int_enable = (HPDF_REG_VAL(HPDF_FLT0, int_flag) & BIT(HPDF_BIT_POS(int_flag)));
        /* get the interrupt enable bit status */
        flagstatus = (HPDF_REG_VAL2(HPDF_FLT0, int_flag) & BIT(HPDF_BIT_POS2(int_flag)));
        if(flagstatus && int_enable){
            return SET;
        }else{
            return RESET; 
        }
    }else{
        /* get the interrupt enable bit status */
        int_enable = (HPDF_REG_VAL(HPDF_FLT1, int_flag) & BIT(HPDF_BIT_POS(int_flag)));
        /* get the interrupt enable bit status */
        flagstatus = (HPDF_REG_VAL2(HPDF_FLT1, int_flag) & BIT(HPDF_BIT_POS2(int_flag)));
        if(flagstatus && int_enable){
            return SET;
        }else{
            return RESET; 
        }
    }
}

/*!
    \brief      clear the HPDF interrupt flags
    \param[in]  hpdf_fltx: FLTy(y=0,1)
    \param[in]  interrupt: HPDF flags, refer to hpdf_interrput_enum
                only one parameter can be selected which is shown as below:
      \arg        HPDF_INT_FLAG_FLTy_ICEF: FLTy inserted conversion end interrupt flag
      \arg        HPDF_INT_FLAG_FLTy_RCEF: FLTy regular conversion end interrupt flag
      \arg        HPDF_INT_FLAG_FLTy_ICDOF: FLTy inserted conversion data overflow interrupt flag
      \arg        HPDF_INT_FLAG_FLTy_RCDOF: FLTy regular conversion data overflow interrupt flag
      \arg        HPDF_INT_FLAG_FLTy_TMEOF: FLTy threshold monitor event occurred interrupt flag
      \arg        HPDF_INT_FLAG_FLT0_CKLF0: clock signal is lost on channel0 interrupt flag
      \arg        HPDF_INT_FLAG_FLT0_CKLF1: clock signal is lost on channel1 interrupt flag
      \arg        HPDF_INT_FLAG_FLT0_MMF0: malfunction event occurred on channel0 interrupt flag
      \arg        HPDF_INT_FLAG_FLT0_MMF1: malfunction event occurred on channel1 interrupt flag
    \param[out] none
    \retval     none
*/
void hpdf_interrupt_flag_clear(hpdf_filter_enum filtery, hpdf_interrput_flag_enum int_flag)
{
    if(HPDF_INT_FLAG_FLTy_ICEF == int_flag){
        /* read the inserted conversion data */
        HPDF_FLTyIDATA(filtery);
    }else if(HPDF_INT_FLAG_FLTy_RCEF == int_flag){
        /* read the regular conversion data */
        HPDF_FLTyRDATA(filtery);
    }else{
        HPDF_FLTyINTC(filtery) |= BIT(HPDF_BIT_POS2(int_flag)); 
    }
}

#endif /* GD32W515PI and GD32W515P0 */ 
