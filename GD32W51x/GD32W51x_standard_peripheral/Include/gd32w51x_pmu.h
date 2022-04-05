/*!
    \file    gd32w51x_pmu.h
    \brief   definitions for the PMU

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

#ifndef GD32W51X_PMU_H
#define GD32W51X_PMU_H

#include "gd32w51x.h"

/* PMU definitions */
#define PMU                           PMU_BASE                              /*!< PMU base address */

/* registers definitions */
#define PMU_CTL0                      REG32((PMU) + 0x00U)                  /*!< PMU control register 0 */
#define PMU_CS0                       REG32((PMU) + 0x04U)                  /*!< PMU control and status register 0 */
#define PMU_CTL1                      REG32((PMU) + 0x08U)                  /*!< PMU control register 1 */
#define PMU_CS1                       REG32((PMU) + 0x0CU)                  /*!< PMU control and status register 1 */
#define PMU_RFCTL                     REG32((PMU) + 0x20U)                  /*!< PMU RF control register */
#define PMU_SECCFG                    REG32((PMU) + 0x30U)                  /*!< PMU secure configuration register */
#define PMU_PRICFG                    REG32((PMU) + 0x34U)                  /*!< PMU privilege configuration register */

/* bits definitions */
/* PMU_CTL0 */
#define PMU_CTL0_LDOLP                BIT(0)                                /*!< LDO low power mode */
#define PMU_CTL0_STBMOD               BIT(1)                                /*!< standby mode */
#define PMU_CTL0_WURST                BIT(2)                                /*!< wakeup flag reset */
#define PMU_CTL0_STBRST               BIT(3)                                /*!< standby flag reset */
#define PMU_CTL0_LVDEN                BIT(4)                                /*!< low voltage detector enable */
#define PMU_CTL0_LVDT                 BITS(5,7)                             /*!< low voltage detector threshold */
#define PMU_CTL0_BKPWEN               BIT(8)                                /*!< backup domain write enable */
#define PMU_CTL0_VLVDEN               BIT(9)                                /*!< vdda low voltage detect enable */
#define PMU_CTL0_LDLP                 BIT(10)                               /*!< low-driver mode when use low power LDO */
#define PMU_CTL0_LDNP                 BIT(11)                               /*!< low-driver mode when use normal power LDO */
#define PMU_CTL0_LDOVS                BITS(14,15)                           /*!< LDO output voltage select */
#define PMU_CTL0_LDEN                 BITS(18,19)                           /*!< low-driver mode enable in deep-sleep mode */

/* PMU_CS0 */
#define PMU_CS0_WUF                   BIT(0)                                /*!< wakeup flag */
#define PMU_CS0_STBF                  BIT(1)                                /*!< standby flag */
#define PMU_CS0_LVDF                  BIT(2)                                /*!< VDD low voltage detector status flag */
#define PMU_CS0_VLVDF                  BIT(3)                               /*!< VDDA low voltage detector status flag */
#define PMU_CS0_WUPEN0                BIT(8)                                /*!< wakeup pin0 enable */
#define PMU_CS0_WUPEN1                BIT(9)                                /*!< wakeup pin1 enable */
#define PMU_CS0_WUPEN2                BIT(10)                               /*!< wakeup pin2 enable */
#define PMU_CS0_WUPEN3                BIT(11)                               /*!< wakeup pin3 enable */
#define PMU_CS0_LDOVSRF               BIT(14)                               /*!< LDO voltage select ready flag */
#define PMU_CS0_LDRF                  BITS(18,19)                           /*!< Low-driver mode ready flag */

/* PMU_CTL1 */
#define PMU_CTL1_WPEN                 BIT(1)                                /*!< WIFI power enable */
#define PMU_CTL1_WPSLEEP              BIT(2)                                /*!< WIFI go to sleep */
#define PMU_CTL1_WPWAKE               BIT(3)                                /*!< WIFI wakeup */
#define PMU_CTL1_SRAM1PSLEEP          BIT(5)                                /*!< SRAM1 go to sleep */
#define PMU_CTL1_SRAM1PWAKE           BIT(6)                                /*!< SRAM1 wakeup */
#define PMU_CTL1_SRAM2PSLEEP          BIT(9)                                /*!< SRAM2 go to sleep */
#define PMU_CTL1_SRAM2PWAKE           BIT(10)                               /*!< SRAM2 wakeup */
#define PMU_CTL1_SRAM3PSLEEP          BIT(13)                               /*!< SRAM3 go to sleep */
#define PMU_CTL1_SRAM3PWAKE           BIT(14)                               /*!< SRAM3 wakeup */

/* PMU_CS1 */
#define PMU_CS1_WPS_SLEEP             BIT(2)                                /*!< WIFI is in sleep state */
#define PMU_CS1_WPS_ACTIVE            BIT(3)                                /*!< WIFI is in active state */
#define PMU_CS1_SRAM1PS_SLEEP         BIT(5)                                /*!< SRAM1 is in sleep state */
#define PMU_CS1_SRAM1PS_ACTIVE        BIT(6)                                /*!< SRAM1 is in active state */
#define PMU_CS1_SRAM2PS_SLEEP         BIT(9)                                /*!< SRAM2 is in sleep state */
#define PMU_CS1_SRAM2PS_ACTIVE        BIT(10)                               /*!< SRAM2 is in active state */
#define PMU_CS1_SRAM3PS_SLEEP         BIT(13)                               /*!< SRAM3 is in sleep state */
#define PMU_CS1_SRAM3PS_ACTIVE        BIT(14)                               /*!< SRAM3 is in active state */

/* PMU_RFCTL */
#define PMU_RFCTL_RFSWEN              BIT(0)                                /*!< RF software sequence enable */
#define PMU_RFCTL_RFFS                BIT(1)                                /*!< Software force start, open RF power */
#define PMU_RFCTL_RFFC                BIT(2)                                /*!< Software force close, close RF power */

/* PMU_SECCFG */
#define PMU_SECCFG_WUP0SEC            BIT(0)                                /*!< WKUP pin 0 security */
#define PMU_SECCFG_WUP1SEC            BIT(1)                                /*!< WKUP pin 1 security */
#define PMU_SECCFG_WUP2SEC            BIT(2)                                /*!< WKUP pin 2 security */
#define PMU_SECCFG_WUP3SEC            BIT(3)                                /*!< WKUP pin 3 security */
#define PMU_SECCFG_LPMSEC             BIT(8)                                /*!< Low-power mode security */
#define PMU_SECCFG_VDMSEC             BIT(9)                                /*!< Voltage detection and monitoring security */
#define PMU_SECCFG_DBPSEC             BIT(10)                               /*!< Backup domain write access security */
#define PMU_SECCFG_LPSSEC             BIT(11)                               /*!< WIFI_sleep and SRAM_sleep mode security */
#define PMU_SECCFG_RFSEC              BIT(16)                               /*!< RF security */

/* PMU_PRICFG */
#define PMU_PRICFG_PRIV               BIT(0)                                /*!< privileged access only */

/* constants definitions */
/* PMU low voltage detector threshold definitions */
#define CTL0_LVDT(regval)             (BITS(5,7)&((uint32_t)(regval)<<5))
#define PMU_LVDT_0                    CTL0_LVDT(0)                          /*!< voltage threshold is 2.1V */
#define PMU_LVDT_1                    CTL0_LVDT(1)                          /*!< voltage threshold is 2.3V */
#define PMU_LVDT_2                    CTL0_LVDT(2)                          /*!< voltage threshold is 2.4V */
#define PMU_LVDT_3                    CTL0_LVDT(3)                          /*!< voltage threshold is 2.6V */
#define PMU_LVDT_4                    CTL0_LVDT(4)                          /*!< voltage threshold is 2.7V */
#define PMU_LVDT_5                    CTL0_LVDT(5)                          /*!< voltage threshold is 2.9V */
#define PMU_LVDT_6                    CTL0_LVDT(6)                          /*!< voltage threshold is 3.0V */
#define PMU_LVDT_7                    CTL0_LVDT(7)                          /*!< voltage threshold is 3.1V */

/* PMU LDO output voltage select definitions */
#define CTL0_LDOVS(regval)            (BITS(14,15)&((uint32_t)(regval)<<14))
#define PMU_LDOVS_LOW                 CTL0_LDOVS(0)                         /*!< LDO output voltage low mode */
#define PMU_LDOVS_HIGH                CTL0_LDOVS(2)                         /*!< LDO output voltage high mode */

/* PMU low-driver mode when use low power LDO */
#define CTL0_LDLP(regval)             (BIT(10)&((uint32_t)(regval)<<10))
#define PMU_NORMALDR_LOWPWR           CTL0_LDLP(0)                          /*!< normal driver when use low power LDO */
#define PMU_LOWDR_LOWPWR              CTL0_LDLP(1)                          /*!< low-driver mode enabled when LDEN is 11 and use low power LDO */

/* PMU low-driver mode when use normal power LDO */
#define CTL0_LDNP(regval)             (BIT(11)&((uint32_t)(regval)<<11))
#define PMU_NORMALDR_NORMALPWR        CTL0_LDNP(0)                          /*!< normal driver when use normal power LDO */
#define PMU_LOWDR_NORMALPWR           CTL0_LDNP(1)                          /*!< low-driver mode enabled when LDEN is 11 and use normal power LDO */

/* PMU low-drive mode ready flag definitions */
#define CS0_LDRF(regval)              (BITS(18,19)&((uint32_t)(regval)<<18))
#define PMU_LDRF_NORMAL               CS0_LDRF(0)                           /*!< normal driver in deep-sleep mode */
#define PMU_LDRF_LOWDRIVER            CS0_LDRF(3)                           /*!< low-driver mode in deep-sleep mode */

/* PMU flag definitions */
#define PMU_FLAG_WAKEUP               PMU_CS0_WUF                           /*!< wakeup flag status */
#define PMU_FLAG_STANDBY              PMU_CS0_STBF                          /*!< standby flag status */
#define PMU_FLAG_LVD                  PMU_CS0_LVDF                          /*!< LVD flag status */
#define PMU_FLAG_VLVD                 PMU_CS0_VLVDF                          /*!< VDDA low votage flag */
#define PMU_FLAG_LDOVSRF              PMU_CS0_LDOVSRF                       /*!< LDO voltage select ready flag */
#define PMU_FLAG_LDRF                 PMU_CS0_LDRF                          /*!< low-driver mode ready flag */
#define PMU_FLAG_WIFI_SLEEP           (BIT(31) | PMU_CS1_WPS_SLEEP)        /*!< WIFI is in sleep state */
#define PMU_FLAG_WIFI_ACTIVE          (BIT(31) | PMU_CS1_WPS_ACTIVE)       /*!< WIFI is in active state */
#define PMU_FLAG_SRAM1_SLEEP          (BIT(31) | PMU_CS1_SRAM1PS_SLEEP)     /*!< SRAM1 is in sleep state */
#define PMU_FLAG_SRAM1_ACTIVE         (BIT(31) | PMU_CS1_SRAM1PS_ACTIVE)    /*!< SRAM1 is in active state */
#define PMU_FLAG_SRAM2_SLEEP          (BIT(31) | PMU_CS1_SRAM2PS_SLEEP)     /*!< SRAM2 is in sleep state */
#define PMU_FLAG_SRAM2_ACTIVE         (BIT(31) | PMU_CS1_SRAM2PS_ACTIVE)    /*!< SRAM2 is in active state */
#define PMU_FLAG_SRAM3_SLEEP          (BIT(31) | PMU_CS1_SRAM3PS_SLEEP)     /*!< SRAM3 is in sleep state */
#define PMU_FLAG_SRAM3_ACTIVE         (BIT(31) | PMU_CS1_SRAM3PS_ACTIVE)    /*!< SRAM3 is in active state */

/* PMU wakeup pin definitions */
#define PMU_WAKEUP_PIN0               PMU_CS0_WUPEN0                        /*!< WKUP Pin 0 (PA2) enable */
#define PMU_WAKEUP_PIN1               PMU_CS0_WUPEN1                        /*!< WKUP Pin 1 (PA15) enable */
#define PMU_WAKEUP_PIN2               PMU_CS0_WUPEN2                        /*!< WKUP Pin 2 (PB2) enable */
#define PMU_WAKEUP_PIN3               PMU_CS0_WUPEN3                        /*!< WKUP Pin 3 (PA12) enable */

/* PMU WIFI SRAM control */
#define PMU_WIFI_SLEEP                PMU_CTL1_WPSLEEP                     /*!< WIFI go to sleep */
#define PMU_WIFI_WAKE                 PMU_CTL1_WPWAKE                      /*!< WIFI wakeup */
#define PMU_SRAM1_SLEEP               PMU_CTL1_SRAM1PSLEEP                  /*!< SRAM1 go to sleep */
#define PMU_SRAM1_WAKE                PMU_CTL1_SRAM1PWAKE                   /*!< SRAM1 wakeup */
#define PMU_SRAM2_SLEEP               PMU_CTL1_SRAM2PSLEEP                  /*!< SRAM2 go to sleep */
#define PMU_SRAM2_WAKE                PMU_CTL1_SRAM2PWAKE                   /*!< SRAM2 wakeup */
#define PMU_SRAM3_SLEEP               PMU_CTL1_SRAM3PSLEEP                  /*!< SRAM3 go to sleep */
#define PMU_SRAM3_WAKE                PMU_CTL1_SRAM3PWAKE                   /*!< SRAM3 wakeup */

/* PMU LDO definitions */
#define PMU_LDO_NORMAL                ((uint32_t)0x00000000U)               /*!< LDO normal work when PMU enter deepsleep mode */
#define PMU_LDO_LOWPOWER              PMU_CTL0_LDOLP                        /*!< LDO work at low power status when PMU enter deepsleep mode */

/* Low-driver mode in deep-sleep mode */
#define PMU_LOWDRIVER_DISABLE         ((uint32_t)0x00000000U)               /*!< Low-driver mode disable in deep-sleep mode */
#define PMU_LOWDRIVER_ENABLE          PMU_CTL0_LDEN                         /*!< Low-driver mode enable in deep-sleep mode */

/* RF force definitions */
#define PMU_RF_FORCE_OPEN             PMU_RFCTL_RFFS                        /*!< Software force start, open RF power */
#define PMU_RF_FORCE_CLOSE            PMU_RFCTL_RFFC                        /*!< Software force close, close RF power */

/* RF sequence definitions */
#define PMU_RF_SOFTWARE               PMU_RFCTL_RFSWEN                      /*!< RF software sequence enable */
#define PMU_RF_HARDWARE               ((uint32_t)0x00000000U)               /*!< RF hardware sequence enable */

/* PMU security definitions */
#define PMU_SEC_WAKEUP_PIN0           PMU_SECCFG_WUP0SEC                    /*!< WKUP pin 0 security */
#define PMU_SEC_WAKEUP_PIN1           PMU_SECCFG_WUP1SEC                    /*!< WKUP pin 1 security */
#define PMU_SEC_WAKEUP_PIN2           PMU_SECCFG_WUP2SEC                    /*!< WKUP pin 2 security */
#define PMU_SEC_WAKEUP_PIN3           PMU_SECCFG_WUP3SEC                    /*!< WKUP pin 3 security */
#define PMU_SEC_LPLDO_DPSLP_STB       PMU_SECCFG_LPMSEC                     /*!< Low-power mode security */
#define PMU_SEC_LVD_VLVD              PMU_SECCFG_VDMSEC                     /*!< Voltage detection and monitoring security */
#define PMU_SEC_BACKUP_WRITE          PMU_SECCFG_DBPSEC                     /*!< Backup domain write access security */
#define PMU_SEC_WIFI_SRAM_CONTROL     PMU_SECCFG_LPSSEC                     /*!< WIFI_sleep and SRAM_sleep mode security */
#define PMU_SEC_RF_SEQUENCE           PMU_SECCFG_RFSEC                      /*!< RF security */

/* PMU flag reset definitions */
#define PMU_FLAG_RESET_WAKEUP         PMU_CTL0_WURST                        /*!< wakeup flag reset */
#define PMU_FLAG_RESET_STANDBY        PMU_CTL0_STBRST                       /*!< standby flag reset */

/* PMU command constants definitions */
#define WFI_CMD                       ((uint8_t)0x00U)                      /*!< use WFI command */
#define WFE_CMD                       ((uint8_t)0x01U)                      /*!< use WFE command */

/* function declarations */
/* reset PMU register */
void pmu_deinit(void);

/* LVD & PVM functions */
/* select low voltage detector threshold */
void pmu_lvd_select(uint32_t lvdt_n);
/* disable PMU LVD */
void pmu_lvd_disable(void);
/* enable PMU VLVD */
void pmu_vlvd_enable(void);
/* disable PMU VLVD */
void pmu_vlvd_disable(void);

/* LDO functions */
/* select LDO output voltage */
void pmu_ldo_output_select(uint32_t ldo_output);

/* set PMU mode */
/* PMU work at sleep mode */
void pmu_to_sleepmode(uint8_t sleepmodecmd);
/* PMU work at deep-sleep mode */
void pmu_to_deepsleepmode(uint32_t ldo, uint32_t lowdrive, uint8_t deepsleepmodecmd);
/* PMU work at standby mode */
void pmu_to_standbymode(uint8_t standbymodecmd);

/* wakeup pin related functions */
/* enable PMU wakeup pin */
void pmu_wakeup_pin_enable(uint32_t wakeup_pin);
/* disable PMU wakeup pin */
void pmu_wakeup_pin_disable(uint32_t wakeup_pin);

/* backup related functions */
/* backup domain write enable */
void pmu_backup_write_enable(void);
/* backup domain write disable */
void pmu_backup_write_disable(void);

/* WIFI & SRAM functions */
/* enable WIFI power */
void pmu_wifi_power_enable(void);
/* disable WIFI power */
void pmu_wifi_power_disable(void);
/* WIFI & SRAM low power control */
void pmu_wifi_sram_control(uint32_t wifi_sram);

/* RF functions */
/* RF sequence force open/close */
void pmu_rf_force_enable(uint32_t force);
/* disable RF sequence open/close force */
void pmu_rf_force_disable(uint32_t force);
/* RF sequence configuration */
void pmu_rf_sequence_config(uint32_t mode);

/* security & privilege functions */
/* enable the security attribution */
void pmu_security_enable(uint32_t security);
/* disable the security attribution */
void pmu_security_disable(uint32_t security);
/* enable the privileged access */
void pmu_privilege_enable(void);
/* disable the privileged access */
void pmu_privilege_disable(void);

/* flag functions */
/* reset flag bit */
void pmu_flag_reset(uint32_t flag_reset);
/* get flag status */
FlagStatus pmu_flag_get(uint32_t pmu_flag);

#endif /* GD32W51X_PMU_H */
