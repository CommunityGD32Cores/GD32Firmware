/*!
    \file    gd32w51x_icache.c
    \brief   ICACHE driver
    
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

#include "gd32w51x_icache.h"

/*!
    \brief      enable icache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void icache_enable(void)
{
    if(ENABLE != (ICACHE_CTL & ICACHE_CTL_EN)){ 
        ICACHE_CTL |= ICACHE_CTL_EN;
    }
}

/*!
    \brief      disable icache
    \param[in]  none
    \param[out] none
    \retval     none
*/
void icache_disable(void)
{
    /* disable memory remap */
    icache_remap_disable(ICACHE_REMAP_REGION_0);
    icache_remap_disable(ICACHE_REMAP_REGION_1);
    icache_remap_disable(ICACHE_REMAP_REGION_2);
    icache_remap_disable(ICACHE_REMAP_REGION_3);
    /* reset and disable monitor */
    icache_monitor_reset(ICACHE_MONITOR_RESET_HIT_MISS);
    icache_monitor_disable(ICACHE_MONITOR_HIT_MISS);
    /* disable icache */
    ICACHE_CTL &= ICACHE_CTL_DEFAULT;
}

/*!
    \brief      enable the icache monitor
    \param[in]  monitor_source: the monitor to be enabled
                only one parameter can be selected which is shown as below:
      \arg        ICACHE_MONITOR_HIT: hit monitor
      \arg        ICACHE_MONITOR_MISS: miss monitor
      \arg        ICACHE_MONITOR_HIT_MISS: hit and miss monitor     
    \param[out] none
    \retval     none
*/
void icache_monitor_enable(uint32_t monitor_source)
{
    ICACHE_CTL |= monitor_source;
}

/*!
    \brief      disable the icache monitor
    \param[in]  monitor_source: the monitor to be disabled
                only one parameter can be selected which is shown as below:
      \arg        ICACHE_MONITOR_HIT: hit monitor
      \arg        ICACHE_MONITOR_MISS: miss monitor
      \arg        ICACHE_MONITOR_HIT_MISS: hit and miss monitor   
    \param[out] none
    \retval     none
*/
void icache_monitor_disable(uint32_t monitor_source)
{
    ICACHE_CTL &= ~monitor_source;
}

/*!
    \brief      reset the icache monitor
    \param[in]  reset_monitor_source: the monitor to be reset
                only one parameter can be selected which is shown as below:
      \arg        ICACHE_MONITOR_RESET_HIT: hit monitor
      \arg        ICACHE_MONITOR_RESET_MISS: miss monitor
      \arg        ICACHE_MONITOR_RESET_HIT_MISS: hit and miss monitor
    \param[out] none
    \retval     none
*/
void icache_monitor_reset(uint32_t reset_monitor_source)
{
    ICACHE_CTL |= reset_monitor_source;
    ICACHE_CTL &= ~reset_monitor_source;
}

/*!
    \brief      configure icache way (associativity mode) 
    \param[in]  none
    \param[out] none 
    \retval     ErrStatus: status of result(SUCCESS or ERROR)
*/
ErrStatus icache_way_configure(void)
{
    if(ENABLE == (ICACHE_CTL & ICACHE_CTL_EN)){ 
        return ERROR;
    }else{
        ICACHE_CTL |= ICACHE_CTL_AMSEL;
    }
    return SUCCESS;
}

/*!
    \brief      select icache burst type 
    \param[in]  burst_type: output burst type
                only one parameter can be selected which is shown as below:
      \arg        ICACHE_WRAP_BURST: icache WRAP burst mode
      \arg        ICACHE_INCR_BURST: icache INCR burst mode
    \param[out] none
    \retval     ErrStatus: status of result(SUCCESS or ERROR)
*/
ErrStatus icache_burst_type_select(uint32_t burst_type)
{
    if(ENABLE == (ICACHE_CTL & ICACHE_CTL_EN)){ 
        return ERROR;
    }else{
        /* select burst type */
        if (ICACHE_WRAP_BURST == burst_type){
            ICACHE_CTL &= ~ICACHE_CTL_BSTT;
        }else{
            ICACHE_CTL |= ICACHE_CTL_BSTT;
        }
        return SUCCESS;
    }
}

/*!
    \brief      invalidate icache 
    \param[in]  none
    \param[out] none
    \retval     ErrStatus: status of result(SUCCESS or ERROR)
*/
ErrStatus icache_invalidation(void)
{
    if(ENABLE == (ICACHE_CTL & ICACHE_CTL_EN)){ 
        return ERROR;
    }
    
    icache_interrupt_flag_clear(ICACHE_ENDC_FLAG);
    /* set the invalidation */
    ICACHE_CTL |= ICACHE_CTL_INVAL;
    /* wait until the busy end */
    while (RESET == (ICACHE_STAT & ICACHE_STAT_END)){
    }
    
    return SUCCESS;
}

/*!
    \brief      get the hit monitor value
    \param[in]  none
    \param[out] none
    \retval     hit value
*/
uint32_t icache_hitvalue_get(void)
{
    return ICACHE_HMC;
}

/*!
    \brief      get the miss monitor value
    \param[in]  none
    \param[out] none
    \retval     miss value
*/
uint32_t icache_missvalue_get(void)
{
    return ICACHE_MMC;
}

/*!
    \brief      enable the icache remap function   
    \param[in]  icache_remap_struct : structure of icache remap parameters
    \param[out] none
    \retval     ErrStatus: status of result(SUCCESS or ERROR)
*/
ErrStatus icache_remap_enable(icache_remap_struct * icache_remap_config)
{
    uint32_t val = 0x00U;
  
    /* the remap can not be configured when icache en is 1 */
    if(ENABLE == (ICACHE_CTL & ICACHE_CTL_EN)){ 
        return ERROR;
    }else{
        /* select the icache region */
        uint32_t reg = (ICACHE + 0x20U + (0x04U * icache_remap_config->region_num));
        /* check if the region is already configured */
        if(0U != (REG32(reg) & 0x1000U)){
            return ERROR;
        }else{
            /* set the base address */
            val |= ((icache_remap_config->base_address & 0x1FFE00000U) >> 21U);
            /* set the remap address */
            val |= ((icache_remap_config->remap_address & 0xFFE00000U) >> 5U);
            /* set the remap size select the master and burst type */
            val |= ((icache_remap_config->remap_size << 9) | (icache_remap_config->master_sel << 28U) | (icache_remap_config->burst_type << 31U));
            REG32(reg) |= val;
            /* enable remap */
            REG32(reg) |= ICACHE_CFGx_EN;
          
            return SUCCESS;
        }
    }
}

/*!
    \brief      disable the icache remap function
    \param[in]  region_num : the remap region to be disabled
    \param[out] none
    \retval     ErrStatus: status of result(SUCCESS or ERROR)
*/
ErrStatus icache_remap_disable(uint32_t region_num)
{
    if(ENABLE == (ICACHE_CTL & ICACHE_CTL_EN)){ 
        return ERROR;
    }else{
        /* select the icache region */
        __IO uint32_t reg = REG32(ICACHE + 0x20U + (0x04U * region_num));
        reg &= ~ICACHE_CFGx_EN;
        return SUCCESS;
    }
}

/*!
    \brief      get icache flag
    \param[in]  flag: the icache flag to be get
                only one parameter can be selected which is shown as below:
      \arg        ICACHE_BUSY_FLAG: icache busy flag
      \arg        ICACHE_END_FLAG: icache busy end flag
      \arg        ICACHE_ERR_FLAG: icache error flag
    \param[out] none
    \retval     FlagStatus: status of result(SET or RESET)
*/
FlagStatus icache_flag_get(uint32_t flag)
{
    __IO uint32_t reg = 0U;
    reg = ICACHE_STAT;
    /* check the status of interrupt source */
    if (RESET != (reg & flag)){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear icache flag
    \param[in]  icache_flag: the icache flag to be cleared
                only one parameter can be selected which is shown as below:
      \arg        ICACHE_ENDC_FLAG: icache busy end clear flag
      \arg        ICACHE_ERRC_FLAG: icache error clear flag 
    \param[out] none
    \retval     none
*/
void icache_flag_clear(uint32_t flag)
{
    ICACHE_FC |= flag;
}

/*!
    \brief      enable icache interrupt
    \param[in]  interrupt: the interrupt to be enabled
                only one parameter can be selected which is shown as below:
      \arg        ICACHE_ENDIE: icache busy end interrupt
      \arg        ICACHE_ERRIE: icache error interrupt
    \param[out] none
    \retval     none
*/
void icache_interrupt_enable(uint32_t interrupt)
{
    ICACHE_INTEN |= interrupt;
}

/*!
    \brief      disable icache interrupt
    \param[in]  interrupt: the interrupt to be disabled
                only one parameter can be selected which is shown as below:
      \arg        ICACHE_ENDIE: icache busy end interrupt
      \arg        ICACHE_ERRIE: icache error interrupt
    \param[out] none
    \retval     none
*/
void icache_interrupt_disable(uint32_t interrupt)
{
    ICACHE_INTEN &= ~interrupt;
}

/*!
    \brief      get icache interrupt flag
    \param[in]  interrupt: the interrupt flag to be get
                only one parameter can be selected which is shown as below:
      \arg        ICACHE_END_FLAG: icache busy end interrupt flag
      \arg        ICACHE_ERR_FLAG: icache error interrupt flag
    \param[out] none
    \retval     FlagStatus: status of result(SET or RESET)
*/
FlagStatus icache_interrupt_flag_get(uint32_t interrupt)
{
    __IO uint32_t reg = 0U;
    reg = ICACHE_STAT;
    /* check the status of interrupt source */
    if(RESET != (reg & interrupt)){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear icache interrupt flag
    \param[in]  interrupt: the interrupt flag to be cleared
                only one parameter can be selected which is shown as below:
      \arg        ICACHE_ENDC_FLAG: icache busy end interrupt clear flag
      \arg        ICACHE_ERRC_FLAG: icache error interrupt clear flag
    \param[out] none
    \retval     none
*/
void icache_interrupt_flag_clear(uint32_t interrupt)
{
    ICACHE_FC |= interrupt;
}

