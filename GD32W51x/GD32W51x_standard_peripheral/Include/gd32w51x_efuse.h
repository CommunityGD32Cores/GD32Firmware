/*!
    \file    gd32w51x_efuse.h
    \brief   definitions for the EFUSE

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

#ifndef GD32W51X_EFUSE_H
#define GD32W51X_EFUSE_H

#include "gd32w51x.h"

/* EFUSE definitions */
#define EFUSE                              (EFUSE_BASE)                      /*!< EFUSE base address */

/* registers definitions */
#define EFUSE_CS                           REG32(EFUSE + 0x00000000U)        /*!< EFUSE control and status register */
#define EFUSE_ADDR                         REG32(EFUSE + 0x00000004U)        /*!< EFUSE address register */
#define EFUSE_CTL                          REG32(EFUSE + 0x00000008U)        /*!< EFUSE control register */
#define EFUSE_TZCTL                        REG32(EFUSE + 0x0000000CU)        /*!< EFUSE trustzone control register */
#define EFUSE_FP_CTL                       REG32(EFUSE + 0x00000010U)        /*!< EFUSE flash protection control register */
#define EFUSE_USER_CTL                     REG32(EFUSE + 0x00000014U)        /*!< EFUSE user control register */

#define EFUSE_MCU_INIT_DATA0               REG32(EFUSE + 0x00000018U)        /*!< EFUSE mcu initialization data register0 */
#define EFUSE_MCU_INIT_DATA1               REG32(EFUSE + 0x0000001CU)        /*!< EFUSE mcu initialization data register1 */
#define EFUSE_MCU_INIT_DATA2               REG32(EFUSE + 0x00000020U)        /*!< EFUSE mcu initialization data register2 */

#define EFUSE_AES_KEY0                     REG32(EFUSE + 0x00000024U)        /*!< EFUSE firmware AES key register0 */
#define EFUSE_AES_KEY1                     REG32(EFUSE + 0x00000028U)        /*!< EFUSE firmware AES key register1 */
#define EFUSE_AES_KEY2                     REG32(EFUSE + 0x0000002CU)        /*!< EFUSE firmware AES key register2 */
#define EFUSE_AES_KEY3                     REG32(EFUSE + 0x00000030U)        /*!< EFUSE firmware AES key register3 */

#define EFUSE_ROTPK_KEY0                   REG32(EFUSE + 0x00000034U)        /*!< EFUSE RoTPK key register0 */
#define EFUSE_ROTPK_KEY1                   REG32(EFUSE + 0x00000038U)        /*!< EFUSE RoTPK key register1 */
#define EFUSE_ROTPK_KEY2                   REG32(EFUSE + 0x0000003CU)        /*!< EFUSE RoTPK key register2 */
#define EFUSE_ROTPK_KEY3                   REG32(EFUSE + 0x00000040U)        /*!< EFUSE RoTPK key register3 */
#define EFUSE_ROTPK_KEY4                   REG32(EFUSE + 0x00000044U)        /*!< EFUSE RoTPK key register4 */
#define EFUSE_ROTPK_KEY5                   REG32(EFUSE + 0x00000048U)        /*!< EFUSE RoTPK key register5 */
#define EFUSE_ROTPK_KEY6                   REG32(EFUSE + 0x0000004CU)        /*!< EFUSE RoTPK key register6 */
#define EFUSE_ROTPK_KEY7                   REG32(EFUSE + 0x00000050U)        /*!< EFUSE RoTPK key register7 */

#define EFUSE_DP0                          REG32(EFUSE + 0x00000054U)        /*!< EFUSE debug password register0 */
#define EFUSE_DP1                          REG32(EFUSE + 0x00000058U)        /*!< EFUSE debug password register1 */

#define EFUSE_IAK_RSS0                     REG32(EFUSE + 0x0000005CU)        /*!< EFUSE IAK key or RSS register0 */
#define EFUSE_IAK_RSS1                     REG32(EFUSE + 0x00000060U)        /*!< EFUSE IAK key or RSS register1 */
#define EFUSE_IAK_RSS2                     REG32(EFUSE + 0x00000064U)        /*!< EFUSE IAK key or RSS register2 */
#define EFUSE_IAK_RSS3                     REG32(EFUSE + 0x00000068U)        /*!< EFUSE IAK key or RSS register3 */
#define EFUSE_IAK_RSS4                     REG32(EFUSE + 0x0000006CU)        /*!< EFUSE IAK key or RSS register4 */
#define EFUSE_IAK_RSS5                     REG32(EFUSE + 0x00000070U)        /*!< EFUSE IAK key or RSS register5 */
#define EFUSE_IAK_RSS6                     REG32(EFUSE + 0x00000074U)        /*!< EFUSE IAK key or RSS register6 */
#define EFUSE_IAK_RSS7                     REG32(EFUSE + 0x00000078U)        /*!< EFUSE IAK key or RSS register7 */
#define EFUSE_IAK_RSS8                     REG32(EFUSE + 0x0000007CU)        /*!< EFUSE IAK key or RSS register8 */
#define EFUSE_IAK_RSS9                     REG32(EFUSE + 0x00000080U)        /*!< EFUSE IAK key or RSS register9 */
#define EFUSE_IAK_RSS10                    REG32(EFUSE + 0x00000084U)        /*!< EFUSE IAK key or RSS register10 */
#define EFUSE_IAK_RSS11                    REG32(EFUSE + 0x00000088U)        /*!< EFUSE IAK key or RSS register11 */
#define EFUSE_IAK_RSS12                    REG32(EFUSE + 0x0000008CU)        /*!< EFUSE IAK key or RSS register12 */
#define EFUSE_IAK_RSS13                    REG32(EFUSE + 0x00000090U)        /*!< EFUSE IAK key or RSS register13 */
#define EFUSE_IAK_RSS14                    REG32(EFUSE + 0x00000094U)        /*!< EFUSE IAK key or RSS register14 */
#define EFUSE_IAK_RSS15                    REG32(EFUSE + 0x00000098U)        /*!< EFUSE IAK key or RSS register15 */

#define EFUSE_PUID0                        REG32(EFUSE + 0x0000009CU)        /*!< EFUSE product UID register0 */
#define EFUSE_PUID1                        REG32(EFUSE + 0x000000A0U)        /*!< EFUSE product UID register1 */
#define EFUSE_PUID2                        REG32(EFUSE + 0x000000A4U)        /*!< EFUSE product UID register2 */
#define EFUSE_PUID3                        REG32(EFUSE + 0x000000A8U)        /*!< EFUSE product UID register3 */

#define EFUSE_HUK_KEY0                     REG32(EFUSE + 0x000000ACU)        /*!< EFUSE HUK key register0 */
#define EFUSE_HUK_KEY1                     REG32(EFUSE + 0x000000B0U)        /*!< EFUSE HUK key register1 */
#define EFUSE_HUK_KEY2                     REG32(EFUSE + 0x000000B4U)        /*!< EFUSE HUK key register2 */
#define EFUSE_HUK_KEY3                     REG32(EFUSE + 0x000000B8U)        /*!< EFUSE HUK key register3 */

#define EFUSE_RF_DATA0                     REG32(EFUSE + 0x000000BCU)        /*!< EFUSE RF data register0 */
#define EFUSE_RF_DATA1                     REG32(EFUSE + 0x000000C0U)        /*!< EFUSE RF data register1 */
#define EFUSE_RF_DATA2                     REG32(EFUSE + 0x000000C4U)        /*!< EFUSE RF data register2 */
#define EFUSE_RF_DATA3                     REG32(EFUSE + 0x000000C8U)        /*!< EFUSE RF data register3 */
#define EFUSE_RF_DATA4                     REG32(EFUSE + 0x000000CCU)        /*!< EFUSE RF data register4 */
#define EFUSE_RF_DATA5                     REG32(EFUSE + 0x000000D0U)        /*!< EFUSE RF data register5 */
#define EFUSE_RF_DATA6                     REG32(EFUSE + 0x000000D4U)        /*!< EFUSE RF data register6 */
#define EFUSE_RF_DATA7                     REG32(EFUSE + 0x000000D8U)        /*!< EFUSE RF data register7 */
#define EFUSE_RF_DATA8                     REG32(EFUSE + 0x000000DCU)        /*!< EFUSE RF data register8 */
#define EFUSE_RF_DATA9                     REG32(EFUSE + 0x000000E0U)        /*!< EFUSE RF data register9 */
#define EFUSE_RF_DATA10                    REG32(EFUSE + 0x000000E4U)        /*!< EFUSE RF data register10 */
#define EFUSE_RF_DATA11                    REG32(EFUSE + 0x000000E8U)        /*!< EFUSE RF data register11 */

#define EFUSE_USER_DATA0                   REG32(EFUSE + 0x000000ECU)        /*!< EFUSE user data register0 */
#define EFUSE_USER_DATA1                   REG32(EFUSE + 0x000000F0U)        /*!< EFUSE user data register1 */
#define EFUSE_USER_DATA2                   REG32(EFUSE + 0x000000F4U)        /*!< EFUSE user data register2 */
#define EFUSE_USER_DATA3                   REG32(EFUSE + 0x000000F8U)        /*!< EFUSE user data register3 */
#define EFUSE_USER_DATA4                   REG32(EFUSE + 0x000000FCU)        /*!< EFUSE user data register4 */
#define EFUSE_USER_DATA5                   REG32(EFUSE + 0x00000100U)        /*!< EFUSE user data register5 */
#define EFUSE_USER_DATA6                   REG32(EFUSE + 0x00000104U)        /*!< EFUSE user data register6 */
#define EFUSE_USER_DATA7                   REG32(EFUSE + 0x00000108U)        /*!< EFUSE user data register7 */

#define EFUSE_PRE_TZEN                     REG32(EFUSE + 0x00000118U)        /*!< EFUSE Pre-TrustZone enable register */
#define EFUSE_TZ_BOOT_ADDR                 REG32(EFUSE + 0x00000120U)        /*!< EFUSE TrustZone boot address register */
#define EFUSE_NTZ_BOOT_ADDR                REG32(EFUSE + 0x00000124U)        /*!< EFUSE No-TrustZone boot address register */

/* bits definitions */
/* EFUSE_CS */
#define EFUSE_CS_EFSTR                     BIT(0)                            /*!< start EFUSE operation */
#define EFUSE_CS_EFRW                      BIT(1)                            /*!< selection of EFUSE operation */
#define EFUSE_CS_CFGRSS                    BIT(15)                           /*!< configuration bit of EFUSE_IAK_RSS register attribute */
#define EFUSE_CS_PGIF                      BIT(16)                           /*!< programming operation completion flag */
#define EFUSE_CS_RDIF                      BIT(17)                           /*!< read operation completion flag */
#define EFUSE_CS_OBERIF                    BIT(18)                           /*!< overstep boundary error flag */
#define EFUSE_CS_PGIE                      BIT(20)                           /*!< enable bit for programming operation completed interrupt */
#define EFUSE_CS_RDIE                      BIT(21)                           /*!< enable bit for read operation completed interrupt */
#define EFUSE_CS_OBERIE                    BIT(22)                           /*!< enable bit for overstep boundary error interrupt */
#define EFUSE_CS_PGIC                      BIT(24)                           /*!< clear bit for programming operation completed interrupt flag */
#define EFUSE_CS_RDIC                      BIT(25)                           /*!< clear bit for read operation completed interrupt flag */
#define EFUSE_CS_OBERIC                    BIT(26)                           /*!< clear bit for overstep boundary error interrupt flag */

/* EFUSE_ADDR */
#define EFUSE_ADDR_EFADDR                  BITS(0,7)                         /*!< read or write EFUSE data start address */
#define EFUSE_ADDR_EFSIZE                  BITS(8,14)                        /*!< read or write EFUSE data size */

/* EFUSE_CTL */
#define EFUSE_CTL_EFSB                     BIT(0)                            /*!< boot from secure boot */
#define EFUSE_CTL_EFBOOTLK                 BIT(1)                            /*!< EFUSE_CTL register bit[5:2] lock bit */
#define EFUSE_CTL_EFBOOT1                  BIT(2)                            /*!< EFUSE boot1 */
#define EFUSE_CTL_SWBOOT1                  BIT(3)                            /*!< enable bit of EFUSE boot1 */
#define EFUSE_CTL_EFBOOT0                  BIT(4)                            /*!< EFUSE boot0 */
#define EFUSE_CTL_SWBOOT0                  BIT(5)                            /*!< enable bit of EFUSE boot0 */

/* EFUSE_TZCTL */
#define EFUSE_TZCTL_TZEN                   BIT(0)                            /*!< trustzone enable bit */
#define EFUSE_TZCTL_NDBG                   BIT(1)                            /*!< debugging permission setting */
#define EFUSE_TZCTL_ROTLK                  BIT(2)                            /*!< EFUSE_ROTPK_KEY register lock bit */
#define EFUSE_TZCTL_RFLK                   BIT(3)                            /*!< EFUSE_RF_DATA register lock bit */
#define EFUSE_TZCTL_IRLK                   BIT(4)                            /*!< EFUSE_IAK_RSS register lock bit */
#define EFUSE_TZCTL_DPLK                   BIT(5)                            /*!< EFUSE_DP register lock bit */
#define EFUSE_TZCTL_VFIMG                  BIT(6)                            /*!< verify firmware image */
#define EFUSE_TZCTL_VFCERT                 BIT(7)                            /*!< verify firmware certificate */

/* EFUSE_FP_CTL */
#define EFUSE_FP_CTL_FP                    BITS(0,7)                         /*!< EFUSE flash protection value */

/* EFUSE_USER_CTL */
#define EFUSE_USER_CTL_HWDG                BIT(0)                            /*!< free watchdog timer selection */
#define EFUSE_USER_CTL_NRSTSTDBY           BIT(1)                            /*!< the option of entry standby mode after reset */
#define EFUSE_USER_CTL_NRSTDPSLP           BIT(2)                            /*!< the option of entry deep sleep mode after reset */
#define EFUSE_USER_CTL_EFOPLK              BIT(3)                            /*!< EFUSE_FP_CTL and EFUSE_USER_CTL register lock bit */
#define EFUSE_USER_CTL_MCUINITLK           BIT(4)                            /*!< EFUSE_MCU_INIT_DATA register lock bit */
#define EFUSE_USER_CTL_AESEN               BIT(5)                            /*!< lock EFUSE_AES_KEY register and enable AES decrypt function */
#define EFUSE_USER_CTL_UDLK                BIT(6)                            /*!< EFUSE_USER_DATA register lock bit */

/* EFUSE_MCU_INIT_DATA */
#define EFUSE_MCU_INIT_DATA_INITDATA       BITS(0,31)                        /*!< EFUSE mcu_init value */

/* EFUSE_AES_KEY */
#define EFUSE_AES_KEY_AESKEY               BITS(0,31)                        /*!< EFUSE AES key value */

/* EFUSE_ROTPK_KEY */
#define EFUSE_ROTPK_KEY_RKEY               BITS(0,31)                        /*!< EFUSE ROTPK or its hash value */

/* EFUSE_DP */
#define EFUSE_DP_DP                        BITS(0,31)                        /*!< EFUSE Debug password value */

/* EFUSE_IAK_RSS */
#define EFUSE_IAK_RSS_IAKRSS               BITS(0,31)                        /*!< EFUSE IAK/RSS value */

/* EFUSE_PUID */
#define EFUSE_PUID_UID                     BITS(0,31)                        /*!< EFUSE MCU UID value */

/* EFUSE_HUK_KEY */
#define EFUSE_HUK_KEY_HKEY                 BITS(0,31)                        /*!< EFUSE HUK value */

/* EFUSE_RF_DATA */
#define EFUSE_RF_DATA_RFDATA               BITS(0,31)                        /*!< EFUSE RF data value */

/* EFUSE_USER_DATA */
#define EFUSE_USER_DATA_USERDATA           BITS(0,31)                        /*!< EFUSE USER_DATA value */

/* EFUSE_PRE_TZEN */
#define EFUSE_PRE_TZEN_STZEN               BIT(0)                            /*!< enable Trustzone function by software */

/* EFUSE_TZ_BOOT_ADDR */
#define EFUSE_TZ_BOOT_ADDR_TZBOOTADDR      BITS(0,31)                        /*!< boot from the address when TrustZone is enabled */

/* EFUSE_NTZ_BOOT_ADDR */
#define EFUSE_NTZ_BOOT_ADDR_NTZBOOTADDR    BITS(0,31)                        /*!< boot from the address when TrustZone is disabled */

/* constants definitions */
/* EFUSE flag enum */
typedef enum
{
    EFUSE_PGIF   = EFUSE_CS_PGIF,                                /*!< programming operation completion flag */
    EFUSE_RDIF   = EFUSE_CS_RDIF,                                /*!< read operation completion flag */
    EFUSE_OBERIF = EFUSE_CS_OBERIF,                              /*!< overstep boundary error flag */
}efuse_flag_enum;

/* EFUSE flag clear enum */
typedef enum
{
    EFUSE_PGIC   = EFUSE_CS_PGIC,                                /*!< clear programming operation completion flag */
    EFUSE_RDIC   = EFUSE_CS_RDIC,                                /*!< clear read operation completion flag */
    EFUSE_OBERIC = EFUSE_CS_OBERIC,                              /*!< clear overstep boundary error flag */
}efuse_clear_flag_enum;

/* EFUSE interrupt enum */
typedef enum
{
    EFUSE_INTEN_PG   = EFUSE_CS_PGIE,                            /*!< programming operation completion interrupt */
    EFUSE_INTEN_RD   = EFUSE_CS_RDIE,                            /*!< read operation completion interrupt */
    EFUSE_INTEN_OBER = EFUSE_CS_OBERIE,                          /*!< overstep boundary error interrupt */
}efuse_int_enum;

/* EFUSE interrupt flag enum */
typedef enum
{
    EFUSE_INT_PGIF   = EFUSE_CS_PGIF,                            /*!< programming operation completion interrupt flag */
    EFUSE_INT_RDIF   = EFUSE_CS_RDIF,                            /*!< read operation completion interrupt flag */
    EFUSE_INT_OBERIF = EFUSE_CS_OBERIF,                          /*!< overstep boundary error interrupt flag */
}efuse_int_flag_enum;

/* EFUSE interrupt flag clear enum */
typedef enum
{
    EFUSE_INT_PGIC   = EFUSE_CS_PGIC,                            /*!< clear programming operation completion interrupt flag */
    EFUSE_INT_RDIC   = EFUSE_CS_RDIC,                            /*!< clear read operation completion interrupt flag */
    EFUSE_INT_OBERIC = EFUSE_CS_OBERIC,                          /*!< clear overstep boundary error interrupt flag */
}efuse_clear_int_flag_enum;

/* EFUSE trustzone state */
typedef enum
{
    EFUSE_TZ,                                                    /*!< trustzone enable */
    EFUSE_N_TZ,                                                  /*!< trustzone disable */
}efuse_tz_enum;

#define EFUSE_MAX_SIZE            ((uint32_t)0x00000040U)        /*!< the maximum length of a single field in EFUSE */

#define EFUSE_CTL_ADDR            ((uint32_t)0x00000000U)        /*!< EFUSE control address */
#define TZ_CTL_ADDR               ((uint32_t)0x00000001U)        /*!< trustzone control address */
#define FP_ADDR                   ((uint32_t)0x00000002U)        /*!< flash protection address */
#define USER_CTL_ADDR             ((uint32_t)0x00000003U)        /*!< user control address */
#define MCU_INIT_ADDR             ((uint32_t)0x00000004U)        /*!< MCU initialization parameters address */
#define AES_KEY_ADDR              ((uint32_t)0x00000010U)        /*!< AES key address */
#define ROTPK_ADDR                ((uint32_t)0x00000020U)        /*!< ROTPK address */
#define DP_ADDR                   ((uint32_t)0x00000040U)        /*!< Debug password address */
#define IAK_ADDR                  ((uint32_t)0x00000048U)        /*!< IAK address */
#define MCU_UID_ADDR              ((uint32_t)0x00000088U)        /*!< MCU unique ID address */
#define HUK_ADDR                  ((uint32_t)0x00000098U)        /*!< HUK address */
#define RF_DATA_ADDR              ((uint32_t)0x000000A8U)        /*!< RF data address */
#define USER_DATA_ADDR            ((uint32_t)0x000000D8U)        /*!< user data address */
#define RESERVED_ADDR             ((uint32_t)0x000000F8U)        /*!< reserved address */

/* function declarations */
/* EFUSE operation functions */
/* read EFUSE value */
ErrStatus efuse_read(uint32_t ef_addr, uint32_t size, uint32_t buf[]);
/* write EFUSE */
ErrStatus efuse_write(uint32_t ef_addr, uint32_t size, uint32_t buf[]);
/* boot configuration */
ErrStatus efuse_boot_config(uint32_t size, uint8_t bt_value[]);
/* trustzone control configuration */
ErrStatus efuse_tz_control_config(uint32_t size, uint8_t tz_ctl[]);
#ifdef GD32W515P0
/* flash protection configuration */
ErrStatus efuse_fp_config(uint32_t size, uint8_t fp_value[]);
#endif /* GD32W515P0 */
/* write mcu initialization parameters */
ErrStatus efuse_mcu_init_data_write(uint32_t size, uint32_t buf[]);
/* write AES key */
ErrStatus efuse_aes_key_write(uint32_t size, uint32_t buf[]);
/* write ROTPK key */
ErrStatus efuse_rotpk_key_write(uint32_t size, uint32_t buf[]);
/* write debug password */
ErrStatus efuse_dp_write(uint32_t size, uint32_t buf[]);
/* write IAK key */
ErrStatus efuse_iak_write(uint32_t size, uint32_t buf[]);
/* write user data */
ErrStatus efuse_user_data_write(uint32_t size, uint32_t buf[]);
#ifdef GD32W515P0
/* enable trustzone by software */
void efuse_software_trustzone_enable(void);
/* disable trustzone by software */
void efuse_software_trustzone_disable(void);
#endif /* GD32W515P0 */
/* get boot address information */
uint32_t efuse_boot_address_get(efuse_tz_enum tz);

/* flag and interrupt functions */
/* check EFUSE flag is set or not */
FlagStatus efuse_flag_get(efuse_flag_enum efuse_flag);
/* clear EFUSE pending flag */
void efuse_flag_clear(efuse_clear_flag_enum efuse_cflag);
/* enable EFUSE interrupt */
void efuse_interrupt_enable(efuse_int_enum source);
/* disable EFUSE interrupt */
void efuse_interrupt_disable(efuse_int_enum source);
/* check EFUSE interrupt flag is set or not */
FlagStatus efuse_interrupt_flag_get(efuse_int_flag_enum int_flag);
/* clear EFUSE pending interrupt flag */
void efuse_interrupt_flag_clear(efuse_clear_int_flag_enum int_cflag);

#endif /* GD32W51X_EFUSE_H */
