/*!
    \file    gd32w51x_tzpcu.c
    \brief   TZPCU driver

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

#include "gd32w51x_tzpcu.h"


#define LSB_16BIT_MASK                     ((uint16_t)0xFFFFU)      /*!< LSB 16-bit mask */
#define PERIPH_LOCATION_MASK               ((uint32_t)0x00200000U)  /*!< peripherals location mask */
#define PERIPH_REGNUM_POS                  28U 
#define TZSPC_SAM0_ADDR                    ((TZSPC) + 0x00000010U)
#define TZSPC_PAM0_ADDR                    ((TZSPC) + 0x00000020U)

/* definitions for TZPCU TZSPC all peripherals register values */
#define TZSPC_PERIPH_CFG0_ALL              (0xC400CDDFU)
#define TZSPC_PERIPH_CFG1_ALL              (0x0001FF9AU)
#define TZSPC_PERIPH_CFG2_ALL              (0xFC800000U)
#define TZSPC_PERIPH_PAMCFG2_ALL           (0xFE800000U)
#define EXT_MEM_SIZE                       (0x08000000U)            /*!< QSPI flash and SQPI PSRAM are both 128M */
#define START_ADDRESS_POS                  0U
#define START_ADDRESS_MASK                 (0x00003FFFU)
#define LENGTH_POS                         16U
#define PERIPH_OFFSET                      16U
#define LENGTH_MASK                        (0x7FFF0000U)
#define TZIAC_INTEN0_ALL                   (0xC400CDDFU)
#define TZIAC_INTEN1_ALL                   (0x1EF9FF9AU)
#define TZIAC_INTEN2_ALL                   (0xFF800FF3U)

/*!
    \brief      configure peripherals secure attributes
    \param[in]  periph: peripheral 
                only one parameter can be selected which is shown as below:
      \arg        TZPCU_PERIPH_TIMER1: TIMER1 peripheral
      \arg        TZPCU_PERIPH_TIMER2: TIMER2 peripheral
      \arg        TZPCU_PERIPH_TIMER3: TIMER3 peripheral
      \arg        TZPCU_PERIPH_TIMER4: TIMER4 peripheral
      \arg        TZPCU_PERIPH_TIMER5: TIMER5 peripheral
      \arg        TZPCU_PERIPH_WWDGT: WWDGT peripheral
      \arg        TZPCU_PERIPH_FWDGT: FWDGT peripheral
      \arg        TZPCU_PERIPH_SPI1: SPI1 peripheral
      \arg        TZPCU_PERIPH_USART1: USART1 peripheral
      \arg        TZPCU_PERIPH_USART2: USART2 peripheral
      \arg        TZPCU_PERIPH_I2C0: I2C0 peripheral
      \arg        TZPCU_PERIPH_I2C1: I2C1 peripheral
      \arg        TZPCU_PERIPH_USBFS: USBFS peripheral
      \arg        TZPCU_PERIPH_TIMER0: TIMER0 peripheral
      \arg        TZPCU_PERIPH_SPI0: SPI0 peripheral
      \arg        TZPCU_PERIPH_USART0: USART0 peripheral
      \arg        TZPCU_PERIPH_TIMER15: TIMER15 peripheral
      \arg        TZPCU_PERIPH_TIMER16: TIMER16 peripheral
      \arg        TZPCU_PERIPH_HPDF: HPDF peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_PERIPH_CRC: CRC peripheral
      \arg        TZPCU_PERIPH_TSI: TSI peripheral
      \arg        TZPCU_PERIPH_ICACHE: ICACHE peripheral
      \arg        TZPCU_PERIPH_ADC: ADC peripheral
      \arg        TZPCU_PERIPH_CAU: CAU peripheral
      \arg        TZPCU_PERIPH_HAU: HAU peripheral
      \arg        TZPCU_PERIPH_TRNG: TRNG peripheral
      \arg        TZPCU_PERIPH_PKCAU: PKCAU peripheral
      \arg        TZPCU_PERIPH_SDIO: SDIO peripheral
      \arg        TZPCU_PERIPH_EFUSE: EFUSE peripheral
      \arg        TZPCU_PERIPH_DBG: DBG peripheral(only for privilege attributes)
      \arg        TZPCU_PERIPH_SQPI_PSRAMREG: SQPI PSRAMREG peripheral
      \arg        TZPCU_PERIPH_QSPI_FLASHREG: QSPI FLASHREG peripheral
      \arg        TZPCU_PERIPH_WIFI_RF: WIFI RF peripheral
      \arg        TZPCU_PERIPH_I2S1_ADD: I2S1_ADD peripheral
      \arg        TZPCU_PERIPH_DCI: DCI peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_PERIPH_WIFI: WIFI peripheral
      \arg        TZPCU_PERIPH_ALL: all peripherals
    \param[in]  attributes: security and privilege attributes
                one or more parameters can be selected which are shown as below:
      \arg        TZPCU_SEC or TZPCU_NSEC: peripheral secure attributes is secure or non-secure
      \arg        TZPCU_PRIV or TZPCU_NPRIV: peripheral privilege attributes is privilege or non-privilege
    \param[out] none
    \retval     none
*/
void tzpcu_tzspc_peripheral_attributes_config(uint32_t periph, uint32_t attributes)
{
    uint32_t position = 0U, reg_value = 0U;

    if(TZPCU_PERIPH_ALL == (periph & TZPCU_PERIPH_ALL)){
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
        /* secure configuration */
        if(TZPCU_SEC == (attributes & TZPCU_SEC)){
            TZPCU_TZSPC_SAM_CFG0 |= TZSPC_PERIPH_CFG0_ALL;
            TZPCU_TZSPC_SAM_CFG1 |= TZSPC_PERIPH_CFG1_ALL;
            TZPCU_TZSPC_SAM_CFG2 |= TZSPC_PERIPH_CFG2_ALL;
        }else{
            if(TZPCU_NSEC == (attributes & TZPCU_NSEC)){
                TZPCU_TZSPC_SAM_CFG0 &= ~TZSPC_PERIPH_CFG0_ALL;
                TZPCU_TZSPC_SAM_CFG1 &= ~TZSPC_PERIPH_CFG1_ALL;
                TZPCU_TZSPC_SAM_CFG2 &= ~TZSPC_PERIPH_CFG2_ALL;
            }
        }
#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */
        
        /* privilege configuration */
        if(TZPCU_PRIV == (attributes & TZPCU_PRIV)){
            TZPCU_TZSPC_PAM_CFG0 |= TZSPC_PERIPH_CFG0_ALL;
            TZPCU_TZSPC_PAM_CFG1 |= TZSPC_PERIPH_CFG1_ALL;
            TZPCU_TZSPC_PAM_CFG2 |= TZSPC_PERIPH_PAMCFG2_ALL;
        }else{
            if(TZPCU_NPRIV == (attributes & TZPCU_NPRIV)){
                TZPCU_TZSPC_PAM_CFG0 &= ~TZSPC_PERIPH_CFG0_ALL;
                TZPCU_TZSPC_PAM_CFG1 &= ~TZSPC_PERIPH_CFG1_ALL;
                TZPCU_TZSPC_PAM_CFG2 &= ~TZSPC_PERIPH_PAMCFG2_ALL;
            }
        }
    }else{
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
        if(TZPCU_PERIPH_DBG != periph){
            /* get TZSPC_SAM_CFG register value */
            reg_value = REG32(TZSPC_SAM0_ADDR + ((periph >> PERIPH_REGNUM_POS) << 2U));
            position = periph & LSB_16BIT_MASK;
            if(periph & PERIPH_LOCATION_MASK){
                position = position << PERIPH_OFFSET;
            }

            /* secure configuration */
            if(TZPCU_SEC == (attributes & TZPCU_SEC)){
                reg_value |= position;
            }else if(TZPCU_NSEC == (attributes & TZPCU_NSEC)){
                reg_value &= ~position;
            }

            /* set TZSPC_SAM_CFG register value */
            REG32(TZSPC_SAM0_ADDR + ((periph >> PERIPH_REGNUM_POS) << 2U)) = reg_value;
        }
#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */

        /* get TZSPC_PAM_CFG register value */
        reg_value = REG32(TZSPC_PAM0_ADDR + ((periph >> PERIPH_REGNUM_POS ) << 2U));
        position = periph & LSB_16BIT_MASK;
        if(periph & PERIPH_LOCATION_MASK){
            position = position << PERIPH_OFFSET;
        }

        /* privilege configuration */
        if(TZPCU_PRIV == (attributes & TZPCU_PRIV)){
            reg_value |= position;
        }else{
          if(TZPCU_NPRIV == (attributes & TZPCU_NPRIV)){
            reg_value &= ~position;
          }
        }

        /* set TZSPC_PAM_CFG register value */
        REG32(TZSPC_PAM0_ADDR + ((periph >> PERIPH_REGNUM_POS ) << 2U)) = reg_value;
    }
}

/*!
    \brief      get peripherals secure attributes
    \param[in]  periph: peripheral 
                only one parameter can be selected which is shown as below:
      \arg        TZPCU_PERIPH_TIMER1: TIMER1 peripheral
      \arg        TZPCU_PERIPH_TIMER2: TIMER2 peripheral
      \arg        TZPCU_PERIPH_TIMER3: TIMER3 peripheral
      \arg        TZPCU_PERIPH_TIMER4: TIMER4 peripheral
      \arg        TZPCU_PERIPH_TIMER5: TIMER5 peripheral
      \arg        TZPCU_PERIPH_WWDGT: WWDGT peripheral
      \arg        TZPCU_PERIPH_FWDGT: FWDGT peripheral
      \arg        TZPCU_PERIPH_SPI1: SPI1 peripheral
      \arg        TZPCU_PERIPH_USART1: USART1 peripheral
      \arg        TZPCU_PERIPH_USART2: USART2 peripheral
      \arg        TZPCU_PERIPH_I2C0: I2C0 peripheral
      \arg        TZPCU_PERIPH_I2C1: I2C1 peripheral
      \arg        TZPCU_PERIPH_USBFS: USBFS peripheral
      \arg        TZPCU_PERIPH_TIMER0: TIMER0 peripheral
      \arg        TZPCU_PERIPH_SPI0: SPI0 peripheral
      \arg        TZPCU_PERIPH_USART0: USART0 peripheral
      \arg        TZPCU_PERIPH_TIMER15: TIMER15 peripheral
      \arg        TZPCU_PERIPH_TIMER16: TIMER16 peripheral
      \arg        TZPCU_PERIPH_HPDF: HPDF peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_PERIPH_CRC: CRC peripheral
      \arg        TZPCU_PERIPH_TSI: TSI peripheral
      \arg        TZPCU_PERIPH_ICACHE: ICACHE peripheral
      \arg        TZPCU_PERIPH_ADC: ADC peripheral
      \arg        TZPCU_PERIPH_CAU: CAU peripheral
      \arg        TZPCU_PERIPH_HAU: HAU peripheral
      \arg        TZPCU_PERIPH_TRNG: TRNG peripheral
      \arg        TZPCU_PERIPH_PKCAU: PKCAU peripheral
      \arg        TZPCU_PERIPH_SDIO: SDIO peripheral
      \arg        TZPCU_PERIPH_EFUSE: EFUSE peripheral
      \arg        TZPCU_PERIPH_DBG: DBG peripheral(only for privilege attributes)
      \arg        TZPCU_PERIPH_SQPI_PSRAMREG: SQPI PSRAMREG peripheral
      \arg        TZPCU_PERIPH_QSPI_FLASHREG: QSPI FLASHREG peripheral
      \arg        TZPCU_PERIPH_WIFI_RF: WIFI RF peripheral
      \arg        TZPCU_PERIPH_I2S1_ADD: I2S1_ADD peripheral
      \arg        TZPCU_PERIPH_DCI: DCI peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_PERIPH_WIFI: WIFI peripheral
    \param[out] none
    \retval     peripheral attributes: TZPCU_SEC/TZPCU_NSEC or TZPCU_PRIV/TZPCU_NPRIV
*/
uint32_t tzpcu_tzspc_peripheral_attributes_get(uint32_t periph)
{
    uint32_t position = 0U, reg_value = 0U, attributes = 0U;
    
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
    if(TZPCU_PERIPH_DBG != periph){
        /* get TZSPC_SAM_CFG register value */
        reg_value = REG32(TZSPC_SAM0_ADDR + ((periph >> PERIPH_REGNUM_POS) << 2U));
        position = periph & LSB_16BIT_MASK;
        if(periph & PERIPH_LOCATION_MASK){
            position = position << PERIPH_OFFSET;
        }

        /* secure configuration */
        if(0U != (reg_value & position)){
            attributes |= TZPCU_SEC;
        }else{
            attributes |= TZPCU_NSEC;
        }
    }
#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */
        
    /* get TZSPC_PAM_CFG register value */
    reg_value = REG32(TZSPC_PAM0_ADDR + ((periph >> PERIPH_REGNUM_POS ) << 2U));
    position = periph & LSB_16BIT_MASK;
    if(periph & PERIPH_LOCATION_MASK){
        position = position << PERIPH_OFFSET;
    }

    /* privilege configuration */
    if(0U != (reg_value & position)){
        attributes |= TZPCU_PRIV;
    }else{
        attributes |= TZPCU_NPRIV;
    }
    return attributes;
}

#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
/*!
    \brief      initialize the TZPCU non-secure mark struct with the default values
    \param[in]  none
    \param[out] init_struct:
                  memory_type: QSPI_FLASH_MEM or SQPI_PSRAM_MEM
                  region_number: NON_SECURE_MARK_REGION0, NON_SECURE_MARK_REGION1, NON_SECURE_MARK_REGION2 or NON_SECURE_MARK_REGION3
                  start_address: external memory non-secure area start address
                  length: external memory non-secure area length
    \retval     none
*/
void tzpcu_non_secure_mark_struct_para_init(tzpcu_non_secure_mark_struct* init_struct)
{
    /* set the TZPCU non-secure mark struct with the default values */
    init_struct->memory_type         = QSPI_FLASH_MEM;
    init_struct->region_number       = NON_SECURE_MARK_REGION0;
    init_struct->start_address       = 0U;
    init_struct->length              = 0U;
}

/*!
    \brief      configure external memory non-secure mark
    \param[in]  p_non_secure_mark:
                  memory_type: QSPI_FLASH_MEM or SQPI_PSRAM_MEM
                  region_number: NON_SECURE_MARK_REGION0, NON_SECURE_MARK_REGION1, NON_SECURE_MARK_REGION2 or NON_SECURE_MARK_REGION3
                  start_address: external memory non-secure area start address
                  length: external memory non-secure area length
    \param[out] none
    \retval     ErrStatus: ERROR or SUCCESS
*/
ErrStatus tzpcu_tzspc_emnsm_config(tzpcu_non_secure_mark_struct *p_non_secure_mark)
{
    uint32_t reg_value, reg_address;

    if(QSPI_FLASH_MEM == p_non_secure_mark->memory_type){
        /* set TZSPC_TZMMPC0_NSMR0 address to reg_address */
        reg_address = (uint32_t)(&TZPCU_TZSPC_TZMMPC0_NSM0);
    }else if(SQPI_PSRAM_MEM == p_non_secure_mark->memory_type){
        /* set TZSPC_TZMMPC1_NSMR0 address to reg_address */
        reg_address = (uint32_t)(&TZPCU_TZSPC_TZMMPC1_NSM0);
    }

    if((p_non_secure_mark->start_address + p_non_secure_mark->length) > EXT_MEM_SIZE){
        return ERROR;
    }

    reg_value = ((p_non_secure_mark->start_address / EXT_MEM_GRANULARITY) << START_ADDRESS_POS) & START_ADDRESS_MASK;
    reg_value |= ((p_non_secure_mark->length / EXT_MEM_GRANULARITY) << LENGTH_POS) & LENGTH_MASK;
    
    REG32(reg_address + ((p_non_secure_mark->region_number) << 2U)) = reg_value;

    return SUCCESS;
}

/*!
    \brief      lock TZSPC items
    \param[in]  none
    \param[out] none
    \retval     none
*/
void tzpcu_tzspc_items_lock(void)
{
    uint32_t reg_value;

    /* get TZSPC control register value */
    reg_value = TZPCU_TZSPC_CTL;
    reg_value |= TZPCU_TZSPC_CTL_LK;

    /* set TZSPC control register value */
    TZPCU_TZSPC_CTL = reg_value;
}

/*!
    \brief      configure debug type
    \param[in]  dbg_type: debug type
                only one parameter can be selected which is shown as below:
      \arg        INVASIVE_DEBUG: invasive debug
      \arg        NON_INVASIVE_DEBUG: non-invasive debug
      \arg        SECURE_INVASIVE_DEBUG: secure invasive debug
      \arg        SECURE_NON_INVASIVE_DEBUG: secure non-Invasive debug
    \param[in]  config_value: ENABLE or DISABLE
    \param[out] none
    \retval     none
*/
void tzpcu_tzspc_dbg_config(uint32_t dbg_type, ControlStatus config_value)
{
    uint32_t reg_value;

    /* get TZSPC debug configuration register */
    reg_value = TZPCU_TZSPC_DBG_CFG;
    if(ENABLE == config_value){
        reg_value |= dbg_type;
    }else{
        reg_value &= ~dbg_type;
    }

    /* set TZSPC debug configuration register */
    TZPCU_TZSPC_DBG_CFG = reg_value;
}

#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */

/*!
    \brief      lock the control register of the TZBMPC sub-block
    \param[in]  tzbmpx: TZBMPCx(x=0,1,2,3)
    \param[out] none
    \retval     none
*/
void tzpcu_tzbmpc_lock(uint32_t tzbmpx)
{
    uint32_t reg_value;

    /* get TZSPC TZBMPC_CTL configuration register */
    reg_value = TZPCU_TZBMPC_CTL(tzbmpx);
    reg_value |= TZPCU_TZBMPC_CTL_LK;

    /* set TZSPC TZBMPC_CTL configuration register */
    TZPCU_TZBMPC_CTL(tzbmpx) = reg_value;
}

/*!
    \brief      configure default security state
    \param[in]  tzbmpx: TZBMPCx(x=0,1,2,3)
    \param[in]  sec_state: security state
                only one parameter can be selected which is shown as below:
      \arg        DEFAULT_STATE: default state
      \arg        INVERT_STATE: invert the state
    \param[out] none
    \retval     none
*/
void tzpcu_tzbmpc_security_state_config(uint32_t tzbmpx, uint32_t sec_state)
{
    uint32_t reg_value;

    /* get TZSPC TZBMPC_CTL configuration register */
    reg_value = TZPCU_TZBMPC_CTL(tzbmpx);
    
    if(DEFAULT_STATE == sec_state){
        reg_value &= ~TZPCU_TZBMPC_CTL_SECSTATCFG;
    }else{
      if(INVERT_STATE == sec_state){
        reg_value |= TZPCU_TZBMPC_CTL_SECSTATCFG;
      }
    }

    /* set TZSPC TZBMPC_CTL configuration register */
    TZPCU_TZBMPC_CTL(tzbmpx) = reg_value;
}

/*!
    \brief      configure secure access state
    \param[in]  tzbmpx: TZBMPCx(x=0,1,2,3)
    \param[in]  sec_illaccess_state: secure illegal access state
                only one parameter can be selected which is shown as below:
      \arg        SECURE_ILLEGAL_ACCESS_ENABLE: secure read/write access non-secure SRAM is illegal
      \arg        SECURE_ILLEGAL_ACCESS_DISABLE: secure read/write access non-secure SRAM is legal
    \param[out] none
    \retval     none
*/
void tzpcu_tzbmpc_secure_access_config(uint32_t tzbmpx, uint32_t sec_illaccess_state)
{
    uint32_t reg_value;

    /* get TZSPC TZBMPC_CTL configuration register */
    reg_value = TZPCU_TZBMPC_CTL(tzbmpx);

    if(SECURE_ILLEGAL_ACCESS_ENABLE == sec_illaccess_state){
        reg_value &= ~TZPCU_TZBMPC_CTL_SRWACFG;
    }else{
      if(SECURE_ILLEGAL_ACCESS_DISABLE == sec_illaccess_state){
        reg_value |= TZPCU_TZBMPC_CTL_SRWACFG;
      }
    }

    /* set TZSPC TZBMPC_CTL configuration register */
    TZPCU_TZBMPC_CTL(tzbmpx) = reg_value;
}

/*!
    \brief      configure block secure access mode
    \param[in]  tzbmpx: TZBMPCx(x=0,1,2,3)
    \param[in]  block_pos_num: block position number
                for TZBMPC0 and TZBMPC1 block position number is 0-255;
                for TZBMPC2 block position number is 0-511;
                for TZBMPC3 block position number is 0-799
    \param[in]  access_mode: block secure access mode
                only one parameter can be selected which is shown as below:
      \arg        BLOCK_SECURE_ACCESS_MODE_SEC: block secure access mode is secure
      \arg        BLOCK_SECURE_ACCESS_MODE_NSEC: block secure access mode is non-secure
    \param[out] none
    \retval     none
*/
void tzpcu_tzbmpc_block_secure_access_mode_config(uint32_t tzbmpx, uint32_t block_pos_num, uint32_t access_mode)
{
    uint32_t reg_value, integer, mod;

    integer = block_pos_num / 32U;
    mod = block_pos_num % 32U;
    /* get TZSPC TZBMPC_VEC configuration register */
    reg_value = TZPCU_TZBMPC_VEC(tzbmpx, integer);

    if(BLOCK_SECURE_ACCESS_MODE_SEC == access_mode){
        reg_value |= 1U << mod;
    }else{
      if(BLOCK_SECURE_ACCESS_MODE_NSEC == access_mode){
        reg_value &= ~(1U << mod );
      }
    }

    /* set TZSPC TZBMPC_VEC configuration register */
    TZPCU_TZBMPC_VEC(tzbmpx, integer) = reg_value;
}

/*!
    \brief      lock configure union block secure access mode
    \param[in]  tzbmpx: TZBMPCx(x=0,1,2,3)
    \param[in]  union_block_position_num: union block position number
                for TZBMPC0 and TZBMPC1 union block position number is 0-7;
                for TZBMPC2 union block position number is 0-15;
                for TZBMPC3 union block position number is 0-23
    \param[out] none
    \retval     none
*/
void tzpcu_tzbmpc_union_block_lock(uint32_t tzbmpx, uint32_t union_block_position_num)
{
    uint32_t reg_value;

    /* get TZSPC TZBMPC_LOCK1 configuration register */
    reg_value = TZPCU_TZBMPC_LOCK0(tzbmpx);

    reg_value |= 1U << union_block_position_num;

    /* set TZSPC TZBMPC_LOCK1 configuration register */
    TZPCU_TZBMPC_LOCK0(tzbmpx) = reg_value;
}

/*!
    \brief      enable illegal access interrupt
    \param[in]  periph: peripheral 
                only one parameter can be selected which is shown as below:
      \arg        TZPCU_PERIPH_TIMER1: TIMER1 peripheral
      \arg        TZPCU_PERIPH_TIMER2: TIMER2 peripheral
      \arg        TZPCU_PERIPH_TIMER3: TIMER3 peripheral
      \arg        TZPCU_PERIPH_TIMER4: TIMER4 peripheral
      \arg        TZPCU_PERIPH_TIMER5: TIMER5 peripheral
      \arg        TZPCU_PERIPH_WWDGT: WWDGT peripheral
      \arg        TZPCU_PERIPH_FWDGT: FWDGT peripheral
      \arg        TZPCU_PERIPH_SPI1: SPI1 peripheral
      \arg        TZPCU_PERIPH_USART1: USART1 peripheral
      \arg        TZPCU_PERIPH_USART2: USART2 peripheral
      \arg        TZPCU_PERIPH_I2C0: I2C0 peripheral
      \arg        TZPCU_PERIPH_I2C1: I2C1 peripheral
      \arg        TZPCU_PERIPH_USBFS: USBFS peripheral
      \arg        TZPCU_PERIPH_TIMER0: TIMER0 peripheral
      \arg        TZPCU_PERIPH_SPI0: SPI0 peripheral
      \arg        TZPCU_PERIPH_USART0: USART0 peripheral
      \arg        TZPCU_PERIPH_TIMER15: TIMER15 peripheral
      \arg        TZPCU_PERIPH_TIMER16: TIMER16 peripheral
      \arg        TZPCU_PERIPH_HPDF: HPDF peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_PERIPH_CRC: CRC peripheral
      \arg        TZPCU_PERIPH_TSI: TSI peripheral
      \arg        TZPCU_PERIPH_ICACHE: ICACHE peripheral
      \arg        TZPCU_PERIPH_ADC: ADC peripheral
      \arg        TZPCU_PERIPH_CAU: CAU peripheral
      \arg        TZPCU_PERIPH_HAU: HAU peripheral
      \arg        TZPCU_PERIPH_TRNG: TRNG peripheral
      \arg        TZPCU_PERIPH_PKCAU: PKCAU peripheral
      \arg        TZPCU_PERIPH_SDIO: SDIO peripheral
      \arg        TZPCU_PERIPH_RTC: RTC peripheral
      \arg        TZPCU_PERIPH_PMU: PMU peripheral
      \arg        TZPCU_PERIPH_SYSCFG: SYSCFG peripheral
      \arg        TZPCU_PERIPH_DMA0: DMA0 peripheral
      \arg        TZPCU_PERIPH_DMA1: DMA1 peripheral
      \arg        TZPCU_PERIPH_RCU: RCU peripheral
      \arg        TZPCU_PERIPH_FLASH: FLASH peripheral
      \arg        TZPCU_PERIPH_FMC: FLASH REG peripheral
      \arg        TZPCU_PERIPH_EXTI: EXTI peripheral
      \arg        TZPCU_PERIPH_TZSPC: TZSPC peripheral
      \arg        TZPCU_PERIPH_TZIAC: TZIAC peripheral
      \arg        TZPCU_PERIPH_SRAM0: SRAM0 peripheral
      \arg        TZPCU_PERIPH_TZBMPC0_REG: ZBMPC0 register
      \arg        TZPCU_PERIPH_SRAM1: SRAM1 peripheral
      \arg        TZPCU_PERIPH_TZBMPC1_REG: TZBMPC1 register
      \arg        TZPCU_PERIPH_SRAM2: SRAM2 peripheral
      \arg        TZPCU_PERIPH_TZBMPC2_REG: TZBMPC2 register
      \arg        TZPCU_PERIPH_SRAM3: SRAM3 peripheral
      \arg        TZPCU_PERIPH_TZBMPC3_REG: TZBMPC3 register
      \arg        TZPCU_PERIPH_EFUSE: EFUSE peripheral
      \arg        TZPCU_PERIPH_SQPI_PSRAM: SQPI_PSRAM peripheral
      \arg        TZPCU_PERIPH_QSPI_FLASH: QSPI_FLASH peripheral
      \arg        TZPCU_PERIPH_SQPI_PSRAMREG: SQPI PSRAMREG peripheral
      \arg        TZPCU_PERIPH_QSPI_FLASHREG: QSPI FLASHREG peripheral
      \arg        TZPCU_PERIPH_WIFI_RF: WIFI RF peripheral
      \arg        TZPCU_PERIPH_I2S1_ADD: I2S1_ADD peripheral
      \arg        TZPCU_PERIPH_DCI: DCI peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_PERIPH_WIFI: WIFI peripheral
      \arg        TZPCU_PERIPH_ALL: all peripherals
    \param[out] none
    \retval     none
*/
void tzpcu_tziac_interrupt_enable(uint32_t periph)
{
    uint32_t position = 0U, reg_value = 0U;

     if(TZPCU_PERIPH_ALL == (periph & TZPCU_PERIPH_ALL)){
        /* enable all interrupt */
        TZPCU_TZIAC_INTEN0 |= TZIAC_INTEN0_ALL;
        TZPCU_TZIAC_INTEN1 |= TZIAC_INTEN1_ALL;
        TZPCU_TZIAC_INTEN2 |= TZIAC_INTEN2_ALL;
    }else{
        /* get TZIAC_INTEN register value */
        reg_value = REG32(TZIAC + ((periph >> PERIPH_REGNUM_POS ) << 2U));

        position = periph & LSB_16BIT_MASK;
        if(periph & PERIPH_LOCATION_MASK){
            position = position << PERIPH_OFFSET;
        }

        reg_value |= position;

        /* set TZIAC_INTEN register value */
        REG32(TZIAC + ((periph >> PERIPH_REGNUM_POS ) << 2U)) = reg_value;
    }
}

/*!
    \brief      disable illegal access interrupt
    \param[in]  periph: peripheral 
                only one parameter can be selected which is shown as below:
      \arg        TZPCU_PERIPH_TIMER1: TIMER1 peripheral
      \arg        TZPCU_PERIPH_TIMER2: TIMER2 peripheral
      \arg        TZPCU_PERIPH_TIMER3: TIMER3 peripheral
      \arg        TZPCU_PERIPH_TIMER4: TIMER4 peripheral
      \arg        TZPCU_PERIPH_TIMER5: TIMER5 peripheral
      \arg        TZPCU_PERIPH_WWDGT: WWDGT peripheral
      \arg        TZPCU_PERIPH_FWDGT: FWDGT peripheral
      \arg        TZPCU_PERIPH_SPI1: SPI1 peripheral
      \arg        TZPCU_PERIPH_USART1: USART1 peripheral
      \arg        TZPCU_PERIPH_USART2: USART2 peripheral
      \arg        TZPCU_PERIPH_I2C0: I2C0 peripheral
      \arg        TZPCU_PERIPH_I2C1: I2C1 peripheral
      \arg        TZPCU_PERIPH_USBFS: USBFS peripheral
      \arg        TZPCU_PERIPH_TIMER0: TIMER0 peripheral
      \arg        TZPCU_PERIPH_SPI0: SPI0 peripheral
      \arg        TZPCU_PERIPH_USART0: USART0 peripheral
      \arg        TZPCU_PERIPH_TIMER15: TIMER15 peripheral
      \arg        TZPCU_PERIPH_TIMER16: TIMER16 peripheral
      \arg        TZPCU_PERIPH_HPDF: HPDF peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_PERIPH_CRC: CRC peripheral
      \arg        TZPCU_PERIPH_TSI: TSI peripheral
      \arg        TZPCU_PERIPH_ICACHE: ICACHE peripheral
      \arg        TZPCU_PERIPH_ADC: ADC peripheral
      \arg        TZPCU_PERIPH_CAU: CAU peripheral
      \arg        TZPCU_PERIPH_HAU: HAU peripheral
      \arg        TZPCU_PERIPH_TRNG: TRNG peripheral
      \arg        TZPCU_PERIPH_PKCAU: PKCAU peripheral
      \arg        TZPCU_PERIPH_SDIO: SDIO peripheral
      \arg        TZPCU_PERIPH_RTC: RTC peripheral
      \arg        TZPCU_PERIPH_PMU: PMU peripheral
      \arg        TZPCU_PERIPH_SYSCFG: SYSCFG peripheral
      \arg        TZPCU_PERIPH_DMA0: DMA0 peripheral
      \arg        TZPCU_PERIPH_DMA1: DMA1 peripheral
      \arg        TZPCU_PERIPH_RCU: RCU peripheral
      \arg        TZPCU_PERIPH_FLASH: FLASH peripheral
      \arg        TZPCU_PERIPH_FMC: FLASH REG peripheral
      \arg        TZPCU_PERIPH_EXTI: EXTI peripheral
      \arg        TZPCU_PERIPH_TZSPC: TZSPC peripheral
      \arg        TZPCU_PERIPH_TZIAC: TZIAC peripheral
      \arg        TZPCU_PERIPH_SRAM0: SRAM0 peripheral
      \arg        TZPCU_PERIPH_TZBMPC0_REG: ZBMPC0 REG peripheral
      \arg        TZPCU_PERIPH_SRAM1: SRAM1 peripheral
      \arg        TZPCU_PERIPH_TZBMPC1_REG: TZBMPC1 REG peripheral
      \arg        TZPCU_PERIPH_SRAM2: SRAM2 peripheral
      \arg        TZPCU_PERIPH_TZBMPC2_REG: TZBMPC2 REG peripheral
      \arg        TZPCU_PERIPH_SRAM3: SRAM3 peripheral
      \arg        TZPCU_PERIPH_TZBMPC3_REG: TZBMPC3 REG peripheral
      \arg        TZPCU_PERIPH_EFUSE: EFUSE peripheral
      \arg        TZPCU_PERIPH_SQPI_PSRAM: SQPI_PSRAM peripheral
      \arg        TZPCU_PERIPH_QSPI_FLASH: QSPI_FLASH peripheral
      \arg        TZPCU_PERIPH_SQPI_PSRAMREG: SQPI PSRAMREG peripheral
      \arg        TZPCU_PERIPH_QSPI_FLASHREG: QSPI FLASHREG peripheral
      \arg        TZPCU_PERIPH_WIFI_RF: WIFI RF peripheral
      \arg        TZPCU_PERIPH_I2S1_ADD: I2S1_ADD peripheral
      \arg        TZPCU_PERIPH_DCI: DCI peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_PERIPH_WIFI: WIFI peripheral
      \arg        TZPCU_PERIPH_ALL: all peripherals
    \param[in]  attributes: security and privilege attributes
                one or more parameters can be selected which are shown as below:
      \arg        TZPCU_SEC or TZPCU_NSEC
      \arg        TZPCU_PRIV or TZPCU_NPRIV
    \param[out] none
    \retval     none
*/
void tzpcu_tziac_interrupt_disable(uint32_t periph)
{
    uint32_t position = 0U, reg_value = 0U;

     if(0U != (periph & TZPCU_PERIPH_ALL)){
        /* disable all interrupt */
        TZPCU_TZIAC_INTEN0 &= ~TZIAC_INTEN0_ALL;
        TZPCU_TZIAC_INTEN1 &= ~TZIAC_INTEN1_ALL;
        TZPCU_TZIAC_INTEN2 &= ~TZIAC_INTEN2_ALL;
    }else{
        /* get TZIAC_INTEN register value */
        reg_value = REG32(TZIAC + ((periph >> PERIPH_REGNUM_POS ) << 2U));

        position = periph & LSB_16BIT_MASK;
        if(periph & PERIPH_LOCATION_MASK){
            position = position << PERIPH_OFFSET;
        }

        reg_value &= ~position;

        /* set TZIAC_INTEN register value */
        REG32(TZIAC + ((periph >> PERIPH_REGNUM_POS ) << 2U)) = reg_value;
    }
}

/*!
    \brief      get illegal access interrupt flag
    \param[in]  periph_flag: peripheral interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        TZPCU_TZIAC_FLAG_TIMER1: TIMER1 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER2: TIMER2 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER3: TIMER3 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER4: TIMER4 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER5: TIMER5 peripheral
      \arg        TZPCU_TZIAC_FLAG_WWDGT: WWDGT peripheral
      \arg        TZPCU_TZIAC_FLAG_FWDGT: FWDGT peripheral
      \arg        TZPCU_TZIAC_FLAG_SPI1: SPI1 peripheral
      \arg        TZPCU_TZIAC_FLAG_USART1: USART1 peripheral
      \arg        TZPCU_TZIAC_FLAG_USART2: USART2 peripheral
      \arg        TZPCU_TZIAC_FLAG_I2C0: I2C0 peripheral
      \arg        TZPCU_TZIAC_FLAG_I2C1: I2C1 peripheral
      \arg        TZPCU_TZIAC_FLAG_USBFS: USBFS peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER0: TIMER0 peripheral
      \arg        TZPCU_TZIAC_FLAG_SPI0: SPI0 peripheral
      \arg        TZPCU_TZIAC_FLAG_USART0: USART0 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER15: TIMER15 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER16: TIMER16 peripheral
      \arg        TZPCU_TZIAC_FLAG_HPDF: HPDF peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_TZIAC_FLAG_CRC: CRC peripheral
      \arg        TZPCU_TZIAC_FLAG_TSI: TSI peripheral
      \arg        TZPCU_TZIAC_FLAG_ICACHE: ICACHE peripheral
      \arg        TZPCU_TZIAC_FLAG_ADC: ADC peripheral
      \arg        TZPCU_TZIAC_FLAG_CAU: CAU peripheral
      \arg        TZPCU_TZIAC_FLAG_HAU: HAU peripheral
      \arg        TZPCU_TZIAC_FLAG_TRNG: TRNG peripheral
      \arg        TZPCU_TZIAC_FLAG_PKCAU: PKCAU peripheral
      \arg        TZPCU_TZIAC_FLAG_SDIO: SDIO peripheral
      \arg        TZPCU_TZIAC_FLAG_RTC: RTC peripheral
      \arg        TZPCU_TZIAC_FLAG_PMU: PMU peripheral
      \arg        TZPCU_TZIAC_FLAG_SYSCFG: SYSCFG peripheral
      \arg        TZPCU_TZIAC_FLAG_DMA0: DMA0 peripheral
      \arg        TZPCU_TZIAC_FLAG_DMA1: DMA1 peripheral
      \arg        TZPCU_TZIAC_FLAG_RCU: RCU peripheral
      \arg        TZPCU_TZIAC_FLAG_FLASH: FLASH peripheral
      \arg        TZPCU_TZIAC_FLAG_FMC: FLASH REG peripheral
      \arg        TZPCU_TZIAC_FLAG_EXTI: EXTI peripheral
      \arg        TZPCU_TZIAC_FLAG_TZSPC: TZSPC peripheral
      \arg        TZPCU_TZIAC_FLAG_TZIAC: TZIAC peripheral
      \arg        TZPCU_TZIAC_FLAG_SRAM0: SRAM0 peripheral
      \arg        TZPCU_TZIAC_FLAG_TZBMPC0_REG: TZBMPC0 REG peripheral
      \arg        TZPCU_TZIAC_FLAG_SRAM1: SRAM1 peripheral
      \arg        TZPCU_TZIAC_FLAG_TZBMPC1_REG: TZBMPC1 REG peripheral
      \arg        TZPCU_TZIAC_FLAG_SRAM2: SRAM2 peripheral
      \arg        TZPCU_TZIAC_FLAG_TZBMPC2_REG: TZBMPC2 REG peripheral
      \arg        TZPCU_TZIAC_FLAG_SRAM3: SRAM3 peripheral
      \arg        TZPCU_TZIAC_FLAG_TZBMPC3_REG: TZBMPC3 REG peripheral
      \arg        TZPCU_TZIAC_FLAG_EFUSE: EFUSE peripheral
      \arg        TZPCU_TZIAC_FLAG_SQPI_PSRAM: SQPI_PSRAM peripheral
      \arg        TZPCU_TZIAC_FLAG_QSPI_FLASH: QSPI_FLASH peripheral
      \arg        TZPCU_TZIAC_FLAG_SQPI_PSRAMREG: SQPI PSRAMREG peripheral
      \arg        TZPCU_TZIAC_FLAG_QSPI_FLASHREG: QSPI FLASHREG peripheral
      \arg        TZPCU_TZIAC_FLAG_WIFI_RF: WIFI RF peripheral
      \arg        TZPCU_TZIAC_FLAG_I2S1_ADD: I2S1_ADD peripheral
      \arg        TZPCU_TZIAC_FLAG_DCI: DCI peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_TZIAC_FLAG_WIFI: WIFI peripheral
    \param[out] none
    \retval     NO_ILLEGAL_ACCESS_PENDING or ILLEGAL_ACCESS_PENDING
*/
uint32_t tzpcu_tziac_flag_get(uint32_t periph_flag)
{
    uint32_t position = 0U, reg_value = 0U, flag_status = 0U;

    /* get TZPCU_TZIAC_STAT0 register value */
    reg_value = REG32((uint32_t)(&TZPCU_TZIAC_STAT0) + ((periph_flag >> PERIPH_REGNUM_POS ) << 2U));

    position = periph_flag & LSB_16BIT_MASK;
    if(periph_flag & PERIPH_LOCATION_MASK){
        position = position << PERIPH_OFFSET;
    }

    /* get flag status */
    if(0U != (reg_value & position)){
        flag_status = ILLEGAL_ACCESS_PENDING;
    }else{
        flag_status = NO_ILLEGAL_ACCESS_PENDING;
    }
    return flag_status;
}

/*!
    \brief      clear illegal access interrupt flag
    \param[in]  periph_flag: peripheral interrupt flag
                only one parameter can be selected which is shown as below:
      \arg        TZPCU_TZIAC_FLAG_TIMER1: TIMER1 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER2: TIMER2 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER3: TIMER3 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER4: TIMER4 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER5: TIMER5 peripheral
      \arg        TZPCU_TZIAC_FLAG_WWDGT: WWDGT peripheral
      \arg        TZPCU_TZIAC_FLAG_FWDGT: FWDGT peripheral
      \arg        TZPCU_TZIAC_FLAG_SPI1: SPI1 peripheral
      \arg        TZPCU_TZIAC_FLAG_USART1: USART1 peripheral
      \arg        TZPCU_TZIAC_FLAG_USART2: USART2 peripheral
      \arg        TZPCU_TZIAC_FLAG_I2C0: I2C0 peripheral
      \arg        TZPCU_TZIAC_FLAG_I2C1: I2C1 peripheral
      \arg        TZPCU_TZIAC_FLAG_USBFS: USBFS peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER0: TIMER0 peripheral
      \arg        TZPCU_TZIAC_FLAG_SPI0: SPI0 peripheral
      \arg        TZPCU_TZIAC_FLAG_USART0: USART0 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER15: TIMER15 peripheral
      \arg        TZPCU_TZIAC_FLAG_TIMER16: TIMER16 peripheral
      \arg        TZPCU_TZIAC_FLAG_HPDF: HPDF peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_TZIAC_FLAG_CRC: CRC peripheral
      \arg        TZPCU_TZIAC_FLAG_TSI: TSI peripheral
      \arg        TZPCU_TZIAC_FLAG_ICACHE: ICACHE peripheral
      \arg        TZPCU_TZIAC_FLAG_ADC: ADC peripheral
      \arg        TZPCU_TZIAC_FLAG_CAU: CAU peripheral
      \arg        TZPCU_TZIAC_FLAG_HAU: HAU peripheral
      \arg        TZPCU_TZIAC_FLAG_TRNG: TRNG peripheral
      \arg        TZPCU_TZIAC_FLAG_PKCAU: PKCAU peripheral
      \arg        TZPCU_TZIAC_FLAG_SDIO: SDIO peripheral
      \arg        TZPCU_TZIAC_FLAG_RTC: RTC peripheral
      \arg        TZPCU_TZIAC_FLAG_PMU: PMU peripheral
      \arg        TZPCU_TZIAC_FLAG_SYSCFG: SYSCFG peripheral
      \arg        TZPCU_TZIAC_FLAG_DMA0: DMA0 peripheral
      \arg        TZPCU_TZIAC_FLAG_DMA1: DMA1 peripheral
      \arg        TZPCU_TZIAC_FLAG_RCU: RCU peripheral
      \arg        TZPCU_TZIAC_FLAG_FLASH: FLASH peripheral
      \arg        TZPCU_TZIAC_FLAG_FMC: FLASH REG peripheral
      \arg        TZPCU_TZIAC_FLAG_EXTI: EXTI peripheral
      \arg        TZPCU_TZIAC_FLAG_TZSPC: TZSPC peripheral
      \arg        TZPCU_TZIAC_FLAG_TZIAC: TZIAC peripheral
      \arg        TZPCU_TZIAC_FLAG_SRAM0: SRAM0 peripheral
      \arg        TZPCU_TZIAC_FLAG_TZBMPC0_REG: ZBMPC0 REG peripheral
      \arg        TZPCU_TZIAC_FLAG_SRAM1: SRAM1 peripheral
      \arg        TZPCU_TZIAC_FLAG_TZBMPC1_REG: TZBMPC1 REG peripheral
      \arg        TZPCU_TZIAC_FLAG_SRAM2: SRAM2 peripheral
      \arg        TZPCU_TZIAC_FLAG_TZBMPC2_REG: TZBMPC2 REG peripheral
      \arg        TZPCU_TZIAC_FLAG_SRAM3: SRAM3 peripheral
      \arg        TZPCU_TZIAC_FLAG_TZBMPC3_REG: TZBMPC3 REG peripheral
      \arg        TZPCU_TZIAC_FLAG_EFUSE: EFUSE peripheral
      \arg        TZPCU_TZIAC_FLAG_SQPI_PSRAM: SQPI_PSRAM peripheral
      \arg        TZPCU_TZIAC_FLAG_QSPI_FLASH: QSPI_FLASH peripheral
      \arg        TZPCU_TZIAC_FLAG_SQPI_PSRAMREG: SQPI PSRAMREG peripheral
      \arg        TZPCU_TZIAC_FLAG_QSPI_FLASHREG: QSPI FLASHREG peripheral
      \arg        TZPCU_TZIAC_FLAG_WIFI_RF: WIFI RF peripheral
      \arg        TZPCU_TZIAC_FLAG_I2S1_ADD: I2S1_ADD peripheral
      \arg        TZPCU_TZIAC_FLAG_DCI: DCI peripheral(not support on GD32W515TX series devices)
      \arg        TZPCU_TZIAC_FLAG_WIFI: WIFI peripheral
      \arg        TZPCU_TZIAC_FLAG_ALL: all peripherals
    \param[out] none
    \retval     none
*/
void tzpcu_tziac_flag_clear(uint32_t periph_flag)
{
    uint32_t position = 0U;

    if(0U != (periph_flag & TZPCU_TZIAC_FLAG_ALL)){
        /* clear all interrupt flag */
        TZPCU_TZIAC_STATC0 = TZIAC_INTEN0_ALL;
        TZPCU_TZIAC_STATC1 = TZIAC_INTEN1_ALL;
        TZPCU_TZIAC_STATC2 = TZIAC_INTEN2_ALL;
    }else{
        position = periph_flag & LSB_16BIT_MASK;
        if(periph_flag & PERIPH_LOCATION_MASK){
            position = position << PERIPH_OFFSET;
        }

        /* clear interrupt flag */
        REG32((uint32_t)(&TZPCU_TZIAC_STATC0) + ((periph_flag >> PERIPH_REGNUM_POS) << 2U)) = position;
    }
}
