/*!
    \file    gd32w51x_fmc.h
    \brief   definitions for the FMC

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

#ifndef GD32W51X_FMC_H
#define GD32W51X_FMC_H

#include "gd32w51x.h"

/* FMC and option bytes definition */
#define FMC                             FMC_BASE                                         /*!< FMC base address */

/* registers definitions */
/* non-secure registers */
#define FMC_KEY                         REG32(FMC + 0x00000004U)                         /*!< FMC unlock key register */
#define FMC_OBKEY                       REG32(FMC + 0x00000008U)                         /*!< FMC option bytes unlock key register */
#define FMC_STAT                        REG32(FMC + 0x0000000CU)                         /*!< FMC status register */
#define FMC_CTL                         REG32(FMC + 0x00000010U)                         /*!< FMC control register */
#define FMC_ADDR                        REG32(FMC + 0x00000014U)                         /*!< FMC address register */
#define FMC_OBSTAT                      REG32(FMC + 0x0000001CU)                         /*!< FMC option bytes status register */
/* secure registers */
#define FMC_SECKEY                      REG32(FMC + 0x00000024U)                         /*!< FMC secure unlock key register */
#define FMC_SECSTAT                     REG32(FMC + 0x0000002CU)                         /*!< FMC secure status register */
#define FMC_SECCTL                      REG32(FMC + 0x00000030U)                         /*!< FMC secure control register */
#define FMC_SECADDR                     REG32(FMC + 0x00000034U)                         /*!< FMC secure address register */
/* non-secure registers */
#define FMC_OBR                         REG32(FMC + 0x00000040U)                         /*!< FMC option byte register */
#define FMC_OBUSER                      REG32(FMC + 0x00000044U)                         /*!< FMC option byte user register */
/* secure registers */
#define FMC_SECMCFG0                    REG32(FMC + 0x00000048U)                         /*!< FMC secure mark configuration register 0 */
#define FMC_DMP0                        REG32(FMC + 0x0000004CU)                         /*!< FMC secure dedicated mark protection register 0*/
/* non-secure register */
#define FMC_OBWRP0                      REG32(FMC + 0x00000050U)                         /*!< FMC option byte write protection area register 0 */
#define FMC_SECMCFG1                    REG32(FMC + 0x00000054U)                         /*!< FMC secure mark configuration register 1 */
#define FMC_DMP1                        REG32(FMC + 0x00000058U)                         /*!< FMC secure dedicated mark protection register 1 */
/* non-secure register */
#define FMC_OBWRP1                      REG32(FMC + 0x0000005CU)                         /*!< FMC option byte write protection area register 1 */
/* non-secure register */
#define FMC_SECMCFG2                    REG32(FMC + 0x00000060U)                         /*!< FMC secure mark configuration register 2 */
#define FMC_SECMCFG3                    REG32(FMC + 0x00000064U)                         /*!< FMC secure mark configuration register 3 */
/* secure registers when TZEN = 1 */
#define FMC_NODEC0                      REG32(FMC + 0x00000070U)                         /*!< FMC NO-RTDEC region register 0 */
#define FMC_NODEC1                      REG32(FMC + 0x00000074U)                         /*!< FMC NO-RTDEC region register 1 */
#define FMC_NODEC2                      REG32(FMC + 0x00000078U)                         /*!< FMC NO-RTDEC region register 2 */
#define FMC_NODEC3                      REG32(FMC + 0x0000007CU)                         /*!< FMC NO-RTDEC region register 3 */
#define FMC_OFRG                        REG32(FMC + 0x00000080U)                         /*!< FMC offset region register */
#define FMC_OFVR                        REG32(FMC + 0x00000084U)                         /*!< FMC offset value register */
/* secure registers */
#define FMC_DMPCTL                      REG32(FMC + 0x0000008CU)                         /*!< FMC DMP control register */
/* when TZEN = 1, this register can be read by secure and non-secure access and it is write-protected against non-secure write access when the flash is secure */
#define FMC_PRIVCFG                     REG32(FMC + 0x00000090U)                         /*!< FMC privilege configuration register */
/* non-secure register */
#define FMC_PID                         REG32(FMC + 0x00000100U)                         /*!< FMC product ID register */

/* bits definitions */
/* FMC_KEY */
#define FMC_KEY_KEY                     BITS(0,31)                                       /*!< FMC_CTL unlock key */

/* FMC_OBKEY */
#define FMC_OBKEY_OBKEY                 BITS(0,31)                                       /*!< option bytes unlock key */

/* FMC_STAT */
#define FMC_STAT_BUSY                   BIT(0)                                           /*!< flash busy flag */
#define FMC_STAT_OBERR                  BIT(3)                                           /*!< option bytes error flag */
#define FMC_STAT_WPERR                  BIT(4)                                           /*!< erase/program protection error flag */
#define FMC_STAT_ENDF                   BIT(5)                                           /*!< end of operation flag */
 
/* FMC_CTL */
#define FMC_CTL_PG                      BIT(0)                                           /*!< main flash program command */
#define FMC_CTL_PER                     BIT(1)                                           /*!< main flash page erase command */
#define FMC_CTL_MER                     BIT(2)                                           /*!< main flash mass erase command */
#define FMC_CTL_START                   BIT(6)                                           /*!< send erase command to FMC */
#define FMC_CTL_LK                      BIT(7)                                           /*!< FMC_CTL lock */
#define FMC_CTL_OBWEN                   BIT(9)                                           /*!< FMC_SECMx (x=0,1,2,3) / FMC_NODECx (x=0,1,2,3) / FMC_OFRG / FMC_OFVR write enable */
#define FMC_CTL_ERRIE                   BIT(10)                                          /*!< error interrupt enable */
#define FMC_CTL_ENDIE                   BIT(12)                                          /*!< end of operation interrupt enable */
#define FMC_CTL_OBSTART                 BIT(14)                                          /*!< option bytes modification start */
#define FMC_CTL_OBRLD                   BIT(15)                                          /*!< option bytes reload */

/* FMC_ADDR */
#define FMC_ADDR_ADDR                   BITS(0,31)                                       /*!< address of flash to be erased/programmed */

/* FMC_OBSTAT */
#define FMC_OBSTAT_SPC_P5               BIT(0)                                           /*!< flash security protection level 0.5 state */
#define FMC_OBSTAT_SPC                  BIT(1)                                           /*!< flash security protection level 1 state */
#define FMC_OBSTAT_WP                   BIT(2)                                           /*!< write/erase protection state */
#define FMC_OBSTAT_TZEN_STAT            BIT(3)                                           /*!< trustzone state */
#define FMC_OBSTAT_NQSPI                BIT(4)                                           /*!< memory structure is FMC mode or QSPI mode */
#define FMC_OBSTAT_FMCOB                BIT(5)                                           /*!< whether the option byte exist or not */

/* FMC_SECKEY */
#define FMC_SECKEY_SECKEY               BITS(0,31)                                       /*!< FMC_SECCTL unlock key */

/* FMC_SECSTAT */
#define FMC_SECSTAT_SECBUSY             BIT(0)                                           /*!< flash secure busy flag */
#define FMC_SECSTAT_SECERR              BIT(3)                                           /*!< flash secure error flag */
#define FMC_SECSTAT_SECWPERR            BIT(4)                                           /*!< secure erase/program protection error flag */
#define FMC_SECSTAT_SECENDF             BIT(5)                                           /*!< secure end of operation flag */
 
/* FMC_SECCTL */
#define FMC_SECCTL_SECPG                BIT(0)                                           /*!< main flash secure program command */
#define FMC_SECCTL_SECPER               BIT(1)                                           /*!< main flash secure page erase command */
#define FMC_SECCTL_SECMER               BIT(2)                                           /*!< main flash secure mass erase command */
#define FMC_SECCTL_SECSTART             BIT(6)                                           /*!< send secure erase command to FMC */
#define FMC_SECCTL_SECLK                BIT(7)                                           /*!< FMC_SECCTL lock */
#define FMC_SECCTL_SECERRIE             BIT(10)                                          /*!< secure error interrupt enable */
#define FMC_SECCTL_SECENDIE             BIT(12)                                          /*!< secure end of operation interrupt enable */

/* FMC_SECADDR */
#define FMC_SECADDR_SECADDR             BITS(0,31)                                       /*!< secure address of flash to be erased/programmed */

/* FMC_OBR */
#define FMC_OBR_SPC                     BITS(0,7)                                        /*!< option bytes security protection code */
#define FMC_OBR_SRAM1_RST               BIT(12)                                          /*!< option bytes SRAM1 erase when system reset */
#define FMC_OBR_TZEN                    BIT(15)                                          /*!< option bytes trustzone enable */

/* FMC_OBUSER */
#define FMC_OBUSER_USER                 BITS(0,15)                                       /*!< option bytes user code */

/* FMC_SECMCFG0 */
#define FMC_SECMCFG0_SECM0_SPAGE        BITS(0,9)                                        /*!< start page of secure mark area 0 */
#define FMC_SECMCFG0_SECM0_EPAGE        BITS(16,25)                                      /*!< end page of secure mark area 0 */

/* FMC_DMP0 */
#define FMC_DMP0_DMP0_EPAGE             BITS(0,9)                                        /*!< end page of DMP area 0 */
#define FMC_DMP0_DMP0EN                 BIT(31)                                          /*!< DMP area 0 enable */

/* FMC_OBWRP0 */
#define FMC_OBWRP0_WRP0_SPAGE           BITS(0,9)                                        /*!< start page of write protection area 0 */
#define FMC_OBWRP0_WRP0_EPAGE           BITS(16,25)                                      /*!< end page of write protection area 0 */

/* FMC_SECMCFG1 */
#define FMC_SECMCFG1_SECM1_SPAGE        BITS(0,9)                                        /*!< start page of secure mark area 1 */
#define FMC_SECMCFG1_SECM1_EPAGE        BITS(16,25)                                      /*!< end page of secure mark area 1 */

/* FMC_DMP1 */
#define FMC_DMP1_DMP1_EPAGE             BITS(0,9)                                        /*!< end page of DMP area 1 */
#define FMC_DMP1_DMP1EN                 BIT(31)                                          /*!< DMP area 1 enable */

/* FMC_OBWRP1 */
#define FMC_OBWRP1_WRP1_SPAGE           BITS(0,9)                                        /*!< start page of write protection area 1 */
#define FMC_OBWRP1_WRP1_EPAGE           BITS(16,25)                                      /*!< end page of write protection area 1 */

/* FMC_SECMCFG2 */
#define FMC_SECMCFG2_SECM2_SPAGE        BITS(0,9)                                        /*!< start page of secure mark area 2 */
#define FMC_SECMCFG2_SECM2_EPAGE        BITS(16,25)                                      /*!< end page of secure mark area 2 */

/* FMC_SECMCFG3 */
#define FMC_SECMCFG3_SECM3_SPAGE        BITS(0,9)                                        /*!< start page of secure mark area 3 */
#define FMC_SECMCFG3_SECM3_EPAGE        BITS(16,25)                                      /*!< end page of secure mark area 3 */

/* FMC_NODEC0 */
#define FMC_NODEC0_NODEC0_SPAGE         BITS(0,9)                                        /*!< start page of NO-RTDEC region 0 */
#define FMC_NODEC0_NODEC0_EPAGE         BITS(16,25)                                      /*!< end page of NO-RTDEC region 0 */

/* FMC_NODEC1 */
#define FMC_NODEC1_NODEC1_SPAGE         BITS(0,9)                                        /*!< start page of NO-RTDEC region 1 */
#define FMC_NODEC1_NODEC1_EPAGE         BITS(16,25)                                      /*!< end page of NO-RTDEC region 1 */

/* FMC_NODEC2 */
#define FMC_NODEC2_NODEC2_SPAGE         BITS(0,9)                                        /*!< start page of NO-RTDEC region 2 */
#define FMC_NODEC2_NODEC2_EPAGE         BITS(16,25)                                      /*!< end page of NO-RTDEC region 2 */

/* FMC_NODEC3 */
#define FMC_NODEC3_NODEC3_SPAGE         BITS(0,9)                                        /*!< start page of NO-RTDEC region 3 */
#define FMC_NODEC3_NODEC3_EPAGE         BITS(16,25)                                      /*!< end page of NO-RTDEC region 3 */

/* FMC_OFRG */
#define FMC_OFRG_OF_SPAGE               BITS(0,12)                                       /*!< start page of offset region */
#define FMC_OFRG_OF_EPAGE               BITS(16,28)                                      /*!< end page of offset region */

/* FMC_OFVR */
#define FMC_OFVR_OF_VALUE               BITS(0,12)                                       /*!< offset value */

/* FMC_DMPCTL */
#define FMC_DMPCTL_DMP0_ACCCFG          BIT(0)                                           /*!< DMP area 0 access configuration */
#define FMC_DMPCTL_DMP1_ACCCFG          BIT(1)                                           /*!< DMP area 1 access configuration */

/* FMC_PRIV_CFG */
#define FMC_PRIVCFG_FMC_PRIV            BIT(0)                                           /*!< privileged access only */

/* FMC_PID */
#define FMC_PID_PID                     BITS(0,31)                                       /*!< product ID */

/* constants definitions */
/* fmc state */
typedef enum
{
    FMC_READY,                                                                           /*!< the operation has been completed */
    FMC_BUSY,                                                                            /*!< the operation is in progress */
    FMC_WPERR,                                                                           /*!< erase/program protection error */
    FMC_TOERR,                                                                           /*!< timeout error */
    FMC_OBERR,                                                                           /*!< option bytes error */
    FMC_SECERR,                                                                          /*!< secure error, only available in secure mode */
}fmc_state_enum;

/* unlock key */
#define UNLOCK_KEY0                     ((uint32_t)0x45670123U)                          /*!< unlock key 0 */
#define UNLOCK_KEY1                     ((uint32_t)0xCDEF89ABU)                          /*!< unlock key 1 */
#define UNLOCK_SECKEY0                  ((uint32_t)0x45670123U)                          /*!< secure unlock key 0 */
#define UNLOCK_SECKEY1                  ((uint32_t)0xCDEF89ABU)                          /*!< secure unlock key 1 */

/* read protection configuration */
#define FMC_NSPC                        ((uint8_t)0xAAU)                                 /*!< no security protection */
#define FMC_SPC_P0_5                    ((uint8_t)0x55U)                                 /*!< security protection level 0.5 */
#define FMC_SPC_P1                      ((uint8_t)0xCCU)                                 /*!< security protection level 1 */

/* option byte wrtie protection area register x(x=0,1) */
#define OBWRP_INDEX0                    ((uint32_t)0x00000000U)                          /*!< FMC_OBWRP0 register index */
#define OBWRP_INDEX1                    ((uint32_t)0x00000001U)                          /*!< FMC_OBWRP1 register index */

/* secure mark configuration register x(x=0,1,2,3) */
#define SECM_INDEX0                     ((uint32_t)0x00000000U)                          /*!< FMC_SECMCFG0 register index */
#define SECM_INDEX1                     ((uint32_t)0x00000001U)                          /*!< FMC_SECMCFG1 register index */
#define SECM_INDEX2                     ((uint32_t)0x00000002U)                          /*!< FMC_SECMCFG2 register index */
#define SECM_INDEX3                     ((uint32_t)0x00000003U)                          /*!< FMC_SECMCFG3 register index */

/* DMP mark configuration register x(x=0,1) */
#define DMP_INDEX0                      ((uint32_t)0x00000000U)                          /*!< FMC_DMP0 register index */
#define DMP_INDEX1                      ((uint32_t)0x00000001U)                          /*!< FMC_DMP1 register index */

/* NO RTDEC configuration register x(x=0,1,2,3) */
#define NODEC_INDEX0                    ((uint32_t)0x00000000U)                          /*!< FMC_NODEC0 register index */
#define NODEC_INDEX1                    ((uint32_t)0x00000001U)                          /*!< FMC_NODEC1 register index */
#define NODEC_INDEX2                    ((uint32_t)0x00000002U)                          /*!< FMC_NODEC2 register index */
#define NODEC_INDEX3                    ((uint32_t)0x00000003U)                          /*!< FMC_NODEC3 register index */

/* FMC flags */
#define OB_FLAG_NSPC                    ((uint32_t)0x00000000U)                          /*!< flash security protection level 0 state */
#define OB_FLAG_SPC1                    FMC_OBSTAT_SPC                                   /*!< flash security protection level 0.5 state */
#define OB_FLAG_SPC0_5                  FMC_OBSTAT_SPC_P5                                /*!< flash security protection level 1 state */

/* FMC interrupt enable */
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
#define FMC_INT_ERR                     FMC_SECCTL_SECERRIE                              /*!< FMC secure error interrupt enable */
#define FMC_INT_END                     FMC_SECCTL_SECENDIE                              /*!< FMC secure end of operation interrupt enable */
#else
#define FMC_INT_ERR                     FMC_CTL_ERRIE                                    /*!< FMC error interrupt enable */
#define FMC_INT_END                     FMC_CTL_ENDIE                                    /*!< FMC end of operation interrupt enable */
#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */

/* FMC flags */
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
#define FMC_FLAG_BUSY                   FMC_SECSTAT_SECBUSY                              /*!< FMC busy flag */
#define FMC_FLAG_SECERR                 FMC_SECSTAT_SECERR                               /*!< FMC secure error flag */
#define FMC_FLAG_WPERR                  FMC_SECSTAT_SECWPERR                             /*!< FMC secure erase/program protection error flag */
#define FMC_FLAG_END                    FMC_SECSTAT_SECENDF                              /*!< FMC secure end of operation flag */
#else
#define FMC_FLAG_BUSY                   FMC_STAT_BUSY                                    /*!< FMC busy flag */
#define FMC_FLAG_WPERR                  FMC_STAT_WPERR                                   /*!< FMC erase/program protection error flag */
#define FMC_FLAG_END                    FMC_STAT_ENDF                                    /*!< FMC end of operation flag */
#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */
#define FMC_FLAG_OBERR                  FMC_STAT_OBERR                                   /*!< option bytes error flag */

/* FMC interrupt flags */
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
#define FMC_INT_FLAG_SECERR             FMC_SECSTAT_SECERR                               /*!< FMC secure error interrupt flag */
#define FMC_INT_FLAG_WPERR              FMC_SECSTAT_SECWPERR                             /*!< FMC secure erase/program protection error interrupt flag */
#define FMC_INT_FLAG_END                FMC_SECSTAT_SECENDF                              /*!< FMC secure end of operation interrupt flag */
#else
#define FMC_INT_FLAG_WPERR              FMC_STAT_WPERR                                   /*!< FMC erase/program protection error interrupt flag */
#define FMC_INT_FLAG_END                FMC_STAT_ENDF                                    /*!< FMC end of operation interrupt flag */
#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */

/* FMC timeout */
#define FMC_TIMEOUT_COUNT              ((uint32_t)0x01000000U)                           /*!< FMC timeout count value */

/* function declarations */
#ifndef GD32W515P0
/* FMC main memory programming functions */
/* unlock the main FMC operation */
void fmc_unlock(void);
/* lock the main FMC operation */
void fmc_lock(void);
/* FMC erase page */
fmc_state_enum fmc_page_erase(uint32_t page_address);
/* FMC erase whole chip */
fmc_state_enum fmc_mass_erase(void);
/* FMC program a word at the corresponding address */
fmc_state_enum fmc_word_program(uint32_t address, uint32_t data);
/* FMC program continuously at the corresponding address */
fmc_state_enum fmc_continuous_program(uint32_t address, uint32_t data[], uint32_t size);
#endif /* GD32W515PI and GD32W515TX */

/* enable SRAM1 reset automatically function */
void sram1_reset_enable(void);
/* disable SRAM1 reset automatically function */
void sram1_reset_disable(void);
/* enable the privileged access */
void fmc_privilege_enable(void);
/* disable the privileged access */
void fmc_privilege_disable(void);

/* FMC option bytes programming functions */
/* unlock the option bytes operation */
void ob_unlock(void);
/* lock the option bytes operation */
void ob_lock(void);

#ifndef GD32W515P0
/* send option bytes modification start command */
void ob_start(void);
/* reload option bytes */
void ob_reload(void);
/* configure the option bytes security protection */
fmc_state_enum ob_security_protection_config(uint8_t ob_spc);
/* enable trustzone */
fmc_state_enum ob_trustzone_enable(void);
/* disable trustzone */
ErrStatus ob_trustzone_disable(void);
/* program option bytes USER */
fmc_state_enum ob_user_write(uint16_t ob_user);
/* configure write protection pages */
fmc_state_enum ob_write_protection_config(uint32_t wrp_spage, uint32_t wrp_epage, uint32_t wrp_register_index);
/* configure secure mark pages */
void ob_secmark_config(uint32_t secm_spage, uint32_t secm_epage, uint32_t secm_register_index);
#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
/* enable DMP region access right */
void ob_dmp_access_enable(uint32_t dmp_register_index);
/* disable DMP region access right */
void ob_dmp_access_disable(uint32_t dmp_register_index);
#endif /* defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3) */
/* configure DMP secure pages */
ErrStatus ob_dmp_config(uint32_t dmp_epage, uint32_t dmp_register_index);
/* enable DMP function */
fmc_state_enum ob_dmp_enable(uint32_t dmp_register_index);
/* disable DMP function */
fmc_state_enum ob_dmp_disable(uint32_t dmp_register_index);
/* configure NO-RTDEC pages */
void fmc_no_rtdec_config(uint32_t nodec_spage, uint32_t nodec_epage, uint32_t nodec_register_index);
#endif /* GD32W515PI and GD32W515TX */

/* FMC read offset function */
/* configure offset region */
void fmc_offset_region_config(uint32_t of_spage, uint32_t of_epage);
/* configure offset value */
void fmc_offset_value_config(uint32_t of_value);

#ifndef GD32W515P0
/* get option bytes write protection state, only applies to get the status of write/erase protection setting by EFUSE */
FlagStatus ob_write_protection_get(void);
/* get the value of option bytes USER */
uint16_t ob_user_get(void);
#endif /* GD32W515PI and GD32W515TX */

/* get option bytes security protection state */
FlagStatus ob_security_protection_flag_get(uint32_t spc_state);
/* get trustzone state */
FlagStatus ob_trustzone_state_get(void);
/* get the state of MCU memory structure is FMC mode or QSPI mode */
FlagStatus ob_memory_mode_state_get(void);
/* get the state of whether the option byte exist or not */
FlagStatus ob_exist_state_get(void);

#ifndef GD32W515P0
/* FMC interrupts and flags management functions */
/* get FMC flag status */
FlagStatus fmc_flag_get(uint32_t flag);
/* clear the FMC flag */
void fmc_flag_clear(uint32_t flag);
/* enable FMC interrupt */
void fmc_interrupt_enable(uint32_t interrupt);
/* disable FMC interrupt */
void fmc_interrupt_disable(uint32_t interrupt);
/* get FMC interrupt flag */
FlagStatus fmc_interrupt_flag_get(uint32_t flag);
/* clear FMC interrupt flag */
void fmc_interrupt_flag_clear(uint32_t flag);
#endif /* GD32W515PI and GD32W515TX */

#endif /* GD32W51X_FMC_H */
