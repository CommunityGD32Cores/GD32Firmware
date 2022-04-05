/*!
    \file    gd32w51x_pkcau.c
    \brief   PKCAU driver
    
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

#include "gd32w51x_pkcau.h"

/* read result from PKCAU RAM */
static void pkcau_memcpy_value(uint32_t offset, uint32_t value);
/* copy operand with EOS or ROS to PKCAU RAM */
static void pkcau_memcpy_operand(uint32_t offset, const uint8_t operand[], uint32_t size);

/*!
    \brief      reset PKCAU
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pkcau_deinit(void)
{
    rcu_periph_reset_enable(RCU_PKCAURST);
    rcu_periph_reset_disable(RCU_PKCAURST);
}

/*!
    \brief      initialize montgomery parameter structure with a default value
    \param[in]  init_para: initialize montgomery parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_mont_struct_para_init(pkcau_mont_parameter_struct* init_para)
{
    /* initialize the member of montgomery parameter structure with the default value */
    init_para->modulus_n   = 0U;
    init_para->modulus_len = 0U;
}

/*!
    \brief      initialize modular parameter structure with a default value
    \param[in]  init_para: initialize modular parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_mod_struct_para_init(pkcau_mod_parameter_struct* init_para)
{
    /* initialize the member of modular parameter structure with the default value */
    init_para->oprd_a      = 0U;
    init_para->oprd_b      = 0U;
    init_para->modulus_n   = 0U;
    init_para->modulus_len = 0U;
}

/*!
    \brief      initialize modular exponentation parameter structure with a default value
    \param[in]  init_para: initialize modular exponentation parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_mod_exp_struct_para_init(pkcau_mod_exp_parameter_struct* init_para)
{
    /* initialize the member of modular exponentation parameter structure with the default value */
    init_para->oprd_a      = 0U;
    init_para->exp_e       = 0U;
    init_para->e_len       = 0U;
    init_para->modulus_n   = 0U;
    init_para->modulus_len = 0U;
    init_para->mont_para   = 0U;
}

/*!
    \brief      initialize modular inversion parameter structure with a default value
    \param[in]  init_para: initialize modular inversion parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_mod_inver_struct_para_init(pkcau_mod_inver_parameter_struct* init_para)
{
    /* initialize the member of modular inversion parameter structure with the default value */
    init_para->oprd_a      = 0U;
    init_para->modulus_n   = 0U;
    init_para->modulus_len = 0U;
}

/*!
    \brief      initialize arithmetic parameter structure with a default value
    \param[in]  init_para: initialize arithmetic parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_arithmetic_struct_para_init(pkcau_arithmetic_parameter_struct* init_para)
{
    /* initialize the member of arithmetic parameter structure with the default value */
    init_para->oprd_a   = 0U;
    init_para->oprd_b   = 0U;
    init_para->oprd_len = 0U;
}

/*!
    \brief      initialize CRT parameter structure with a default value
    \param[in]  init_para: initialize CRT parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_crt_struct_para_init(pkcau_crt_parameter_struct* init_para)
{
    /* initialize the member of arithmetic parameter structure with the default value */
    init_para->oprd_a    = 0U;
    init_para->oprd_len  = 0U;
    init_para->oprd_dp   = 0U;
    init_para->oprd_dq   = 0U;
    init_para->oprd_qinv = 0U;
    init_para->oprd_p    = 0U;
    init_para->oprd_q    = 0U;
}

/*!
    \brief      initialize ECC curve parameter structure with a default value
    \param[in]  init_para: initialize ECC curve parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_ec_group_struct_para_init(pkcau_ec_group_parameter_struct* init_para)
{
    /* initialize the member of ECC curve parameter structure with the default value */
    init_para->modulus_p     = 0U;
    init_para->coff_a        = 0U;
    init_para->coff_b        = 0U;
    init_para->base_point_x  = 0U;
    init_para->base_point_y  = 0U;
    init_para->order_n       = 0U;
    init_para->a_sign        = 0U;
    init_para->modulus_p_len = 0U;
    init_para->order_n_len   = 0U;
    init_para->multi_k       = 0U;
    init_para->k_len         = 0U;
    init_para->mont_para     = 0U;
}

/*!
    \brief      initialize point parameter structure with a default value
    \param[in]  init_para: initialize point parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_point_struct_para_init(pkcau_point_parameter_struct* init_para)
{
    /* initialize the member of point parameter structure with the default value */
    init_para->point_x = 0U;
    init_para->point_y = 0U;
}

/*!
    \brief      initialize signature parameter structure with a default value
    \param[in]  init_para: initialize signature parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_signature_struct_para_init(pkcau_signature_parameter_struct* init_para)
{
    /* initialize the member of signature parameter structure with the default value */
    init_para->sign_r = 0U;
    init_para->sign_s = 0U;
}

/*!
    \brief      initialize hash parameter structure with a default value
    \param[in]  init_para: initialize hash parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_hash_struct_para_init(pkcau_hash_parameter_struct* init_para)
{
    /* initialize the member of hash parameter structure with the default value */
    init_para->hash_z     = 0U;
    init_para->hash_z_len = 0U;
}

/*!
    \brief      initialize modular reduction parameter structure with a default value
    \param[in]  init_para: initialize modular reduction parameter struct
    \param[out] none
    \retval     none
*/
void pkcau_mod_reduc_struct_para_init(pkcau_mod_reduc_parameter_struct* init_para)
{
    /* initialize the member of modular reduction parameter structure with the default value */
    init_para->oprd_a      = 0U;
    init_para->oprd_a_len  = 0U;
    init_para->modulus_n   = 0U;
    init_para->modulus_len = 0U;
}

/*!
    \brief      enable PKCAU
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pkcau_enable(void)
{
    PKCAU_CTL |= PKCAU_CTL_PKCAUEN;
}

/*!
    \brief      disable PKCAU
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pkcau_disable(void)
{
    PKCAU_CTL &= ~PKCAU_CTL_PKCAUEN;
}

/*!
    \brief      start operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pkcau_start(void)
{
    PKCAU_CTL |= PKCAU_CTL_START;
}

/*!
    \brief      configure the PKCAU operation mode
    \param[in]  mode: PKCAU operation mode
                only one parameter can be selected which is shown as below:
      \arg        PKCAU_MODE_MOD_EXP: Montgomery parameter computation then modular exponentiation
      \arg        PKCAU_MODE_MONT_PARAM: Montgomery parameter computation only
      \arg        PKCAU_MODE_MOD_EXP_FAST: modular exponentiation only
      \arg        PKCAU_MODE_CRT_EXP: RSA CRT exponentiation
      \arg        PKCAU_MODE_MOD_INVERSION: modular inversion
      \arg        PKCAU_MODE_ARITHMETIC_ADD: arithmetic addition
      \arg        PKCAU_MODE_ARITHMETIC_SUB: arithmetic subtraction
      \arg        PKCAU_MODE_ARITHMETIC_MUL: arithmetic multiplication
      \arg        PKCAU_MODE_ARITHMETIC_COMP: arithmetic comparison
      \arg        PKCAU_MODE_MOD_REDUCTION: modular reduction
      \arg        PKCAU_MODE_MOD_ADD: modular addition
      \arg        PKCAU_MODE_MOD_SUB: modular subtraction
      \arg        PKCAU_MODE_MONT_MUL: Montgomery multiplication
      \arg        PKCAU_MODE_ECC_MUL: Montgomery parameter computation then ECC scalar multiplication
      \arg        PKCAU_MODE_ECC_MUL_FAST: ECC scalar multiplication only
      \arg        PKCAU_MODE_ECDSA_SIGN: ECDSA sign
      \arg        PKCAU_MODE_ECDSA_VERIFICATION: ECDSA verification
      \arg        PKCAU_MODE_POINT_CHECK: point on elliptic curve Fp check
    \retval     none
*/
void pkcau_mode_set(uint32_t mode)
{
    PKCAU_CTL &= ~PKCAU_CTL_MODESEL;
    PKCAU_CTL |= mode;
}

/*!
    \brief      execute montgomery parameter operation
    \param[in]  mont_para: PKCAU montgomery parameter struct
                  modulus_n: modulus value n
                  modulus_len: modulus length in byte
    \param[out] none
    \retval     none
*/
void pkcau_mont_param_operation(pkcau_mont_parameter_struct *mont_para)
{
    uint32_t n_bit_len = (mont_para->modulus_len) * 8U;
    
    /* write the modulus length in bit to PKCAU RAM */
    pkcau_memcpy_value(0x00000404U, n_bit_len);
    
    /* write the modulus value n to PKCAU RAM */
    pkcau_memcpy_operand(0x00000D5CU, mont_para->modulus_n, mont_para->modulus_len);
    /* configure the operation mode */
    pkcau_mode_set(PKCAU_MODE_MONT_PARAM);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      execute modular operation, include modular addition, modular subtraction and montgomery multiplication
    \param[in]  mont_para: PKCAU modular parameter struct
                  oprd_a: operand A
                  oprd_b: operand B
                  modulus_n: modulus value n
                  modulus_len: modulus length in byte
    \param[in]  mode: modular operation mode
                only one parameter can be selected which is shown as below:
      \arg        PKCAU_MODE_MOD_ADD: modular addition
      \arg        PKCAU_MODE_MOD_SUB: modular subtraction
      \arg        PKCAU_MODE_MONT_MUL: Montgomery multiplication
    \param[out] none
    \retval     none
*/
void pkcau_mod_operation(pkcau_mod_parameter_struct *mod_para, uint32_t mode)
{
    uint32_t bit_len = mod_para->modulus_len * 8U;

    /* write the modulus length in bit to PKCAU RAM */
    pkcau_memcpy_value(0x00000404U, bit_len);
    
    /* write the operand A, operand B and modulus value n to PKCAU RAM */
    pkcau_memcpy_operand(0x000008B4U, mod_para->oprd_a, mod_para->modulus_len);
    pkcau_memcpy_operand(0x00000A44U, mod_para->oprd_b, mod_para->modulus_len);
    pkcau_memcpy_operand(0x00000D5CU, mod_para->modulus_n, mod_para->modulus_len);
    /* configure the operation mode */
    pkcau_mode_set(mode);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      execute modular exponentation operation
    \param[in]  mont_para: PKCAU exponentation parameter struct for fast mode
                  oprd_a: operand A
                  exp_e: exponent e
                  e_len: exponent length in byte
                  modulus_n: modulus value n
                  modulus_len: modulus length in byte
                  mont_para: montgomery parameter R2 mod n
    \param[in]  mode: modular exponentation operation mode
                only one parameter can be selected which is shown as below:
      \arg        PKCAU_MODE_MOD_EXP: montgomery parameter computation then modular exponentiation
      \arg        PKCAU_MODE_MOD_EXP_FAST: modular exponentiation only
    \param[out] none
    \retval     none
*/
void pkcau_mod_exp_operation(pkcau_mod_exp_parameter_struct *mod_exp_para, uint32_t mode)
{
    uint32_t e_bit_len = mod_exp_para->e_len * 8U;
    uint32_t n_bit_len = mod_exp_para->modulus_len * 8U;
    
    /* write the exponent length and modulus length in bit to PKCAU RAM */
    pkcau_memcpy_value(0x00000400U, e_bit_len);
    pkcau_memcpy_value(0x00000404U, n_bit_len);
    
    /* write the operand A, exponent e and modulus value n to PKCAU RAM */
    pkcau_memcpy_operand(0x00000A44U, mod_exp_para->oprd_a, mod_exp_para->modulus_len);
    pkcau_memcpy_operand(0x00000BD0U, mod_exp_para->exp_e, mod_exp_para->e_len);
    pkcau_memcpy_operand(0x00000D5CU, mod_exp_para->modulus_n, mod_exp_para->modulus_len);
    /* write the montgomery parameter to PKCAU RAM */
    if(mode == PKCAU_MODE_MOD_EXP_FAST){
        pkcau_memcpy_operand(0x00000594U, mod_exp_para->mont_para, mod_exp_para->modulus_len);
    }
    /* configure the operation mode */
    pkcau_mode_set(mode);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      execute modular inversion operation
    \param[in]  mod_inver_para: PKCAU modular inversion parameter struct
                  oprd_a: operand A
                  modulus_n: modulus value n
                  modulus_len: modulus length in byte
    \param[out] none
    \retval     none
*/
void pkcau_mod_inver_operation(pkcau_mod_inver_parameter_struct *mod_inver_para)
{
    uint32_t bit_len = mod_inver_para->modulus_len * 8U;
    
    /* write the modulus length in bit to PKCAU RAM */
    pkcau_memcpy_value(0x00000404U, bit_len);
    
    /* write the operand A and modulus value n to PKCAU RAM */
    pkcau_memcpy_operand(0x000008B4U, mod_inver_para->oprd_a, mod_inver_para->modulus_len);
    pkcau_memcpy_operand(0x00000A44U, mod_inver_para->modulus_n, mod_inver_para->modulus_len);
    /* configure the operation mode */
    pkcau_mode_set(PKCAU_MODE_MOD_INVERSION);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      execute modular reduction operation
    \param[in]  mod_reduc_para: PKCAU modular reduction parameter struct
                  oprd_a: operand A
                  oprd_a_len: length of operand A in byte
                  modulus_n: modulus value n
                  modulus_len: modulus length in byte
    \param[out] none
    \retval     none
*/
void pkcau_mod_reduc_operation(pkcau_mod_reduc_parameter_struct *mod_reduc_para)
{
    uint32_t n_bit_len = mod_reduc_para->modulus_len * 8U;
    uint32_t a_bit_len = mod_reduc_para->oprd_a_len * 8U;
    
    /* write the modulus length and length of operand A in bit to PKCAU RAM */
    pkcau_memcpy_value(0x00000400U, a_bit_len);
    pkcau_memcpy_value(0x00000404U, n_bit_len);
    
    /* write the operand A and modulus value n to PKCAU RAM */
    pkcau_memcpy_operand(0x000008B4U, mod_reduc_para->oprd_a, mod_reduc_para->oprd_a_len);
    pkcau_memcpy_operand(0x00000A44U, mod_reduc_para->modulus_n, mod_reduc_para->modulus_len);
    /* configure the operation mode */
    pkcau_mode_set(PKCAU_MODE_MOD_REDUCTION);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      execute arithmetic operation
    \param[in]  arithmetic_para: PKCAU arithmetic parameter struct
                  oprd_a: operand A
                  oprd_b: operand B
                  oprd_len: length of operand in byte
    \param[in]  mode: arithmetic operation mode
                only one parameter can be selected which is shown as below:
      \arg        PKCAU_MODE_ARITHMETIC_ADD: arithmetic addition
      \arg        PKCAU_MODE_ARITHMETIC_SUB: arithmetic subtraction
      \arg        PKCAU_MODE_ARITHMETIC_MUL: arithmetic multiplication
      \arg        PKCAU_MODE_ARITHMETIC_COMP: arithmetic comparison
    \param[out] none
    \retval     none
*/
void pkcau_arithmetic_operation(pkcau_arithmetic_parameter_struct *arithmetic_para, uint32_t mode)
{
    uint32_t bit_len = arithmetic_para->oprd_len * 8U;
    
    /* write the length of operand in bit to PKCAU RAM */
    pkcau_memcpy_value(0x00000404U, bit_len);
    
    /* write the operand A and operand B to PKCAU RAM */
    pkcau_memcpy_operand(0x000008B4U, arithmetic_para->oprd_a, arithmetic_para->oprd_len);
    pkcau_memcpy_operand(0x00000A44U, arithmetic_para->oprd_b, arithmetic_para->oprd_len);
    /* configure the operation mode */
    pkcau_mode_set(mode);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      execute RSA CRT exponentation operation
    \param[in]  mod_reduc_para: PKCAU modular CRT parameter struct
                  oprd_a: operand A
                  modulus_len: modulus length in byte
                  oprd_dp: operand dp
                  oprd_dq:operand dq
                  oprd_qinv: operand qinv
                  oprd_p: prime operand p
                  oprd_q: prime operand q
    \param[out] none
    \retval     none
*/
void pkcau_crt_exp_operation(pkcau_crt_parameter_struct* crt_para)
{
    uint32_t max_len_in_bit = crt_para->oprd_len * 8U;
    
    /* write the modulus length in bit to PKCAU RAM */
    pkcau_memcpy_value(0x00000404U, max_len_in_bit); 
    
    /* write the operand dp, dq, qinv, p and q to PKCAU RAM */
    pkcau_memcpy_operand(0x0000065CU, crt_para->oprd_dp, crt_para->oprd_len/2U); 
    pkcau_memcpy_operand(0x00000BD0U, crt_para->oprd_dq, crt_para->oprd_len/2U);
    pkcau_memcpy_operand(0x000007ECU, crt_para->oprd_qinv, crt_para->oprd_len/2U); 
    pkcau_memcpy_operand(0x0000097CU, crt_para->oprd_p, crt_para->oprd_len/2U);
    pkcau_memcpy_operand(0x00000D5CU, crt_para->oprd_q, crt_para->oprd_len/2U);
    /* write the operand A to PKCAU RAM */
    pkcau_memcpy_operand(0x00000EECU, crt_para->oprd_a, crt_para->oprd_len);
    /* configure the operation mode */
    pkcau_mode_set(PKCAU_MODE_CRT_EXP);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      execute point check operation
    \param[in]  point_para: PKCAU point struct
                  point_x: point coordinate x
                  point_y: point coordinate y
    \param[in]  curve_group_para: PKCAU ECC curve paramter struct
                  modulus_p: curve modulus p
                  coff_a: curve coefficient a
                  coff_b: curve coefficient b
                  base_point_x: curve base point coordinate x
                  base_point_y: curve base point coordinate y
                  order_n: curve prime order n
                  a_sign: curve coefficient a sign
                  modulus_p_len: curve modulus p length in byte
                  order_n_len: curve prime order n length in byte
                  multi_k: scalar multiplier k
                  k_len: length of scalar multiplier k
    \param[out] none
    \retval     none
*/
void pkcau_point_check_operation(pkcau_point_parameter_struct* point_para, const pkcau_ec_group_parameter_struct* curve_group_para)
{
    uint32_t max_len_in_bit = curve_group_para->modulus_p_len * 8U;
    
    /* write the modulus length in bit to PKCAU RAM */
    pkcau_memcpy_value(0x00000404U, max_len_in_bit);
    /* write the sign of curve coefficient a to PKCAU RAM */
    pkcau_memcpy_value(0x00000408U, curve_group_para->a_sign);
    
    /* write the curve coefficient a, curve coefficient b and curve modulus p to PKCAU RAM */
    pkcau_memcpy_operand(0x0000040CU, curve_group_para->coff_a, curve_group_para->modulus_p_len);
    pkcau_memcpy_operand(0x000007FCU, curve_group_para->coff_b, curve_group_para->modulus_p_len);
    pkcau_memcpy_operand(0x00000460U, curve_group_para->modulus_p, curve_group_para->modulus_p_len);
    /* write the point coordinate x and point coordinate y to PKCAU RAM */
    pkcau_memcpy_operand(0x0000055CU, point_para->point_x, curve_group_para->modulus_p_len);
    pkcau_memcpy_operand(0x000005B0U, point_para->point_y, curve_group_para->modulus_p_len);
    /* configure the operation mode */
    pkcau_mode_set(PKCAU_MODE_POINT_CHECK);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      execute point multiplication operation
    \param[in]  point_para: PKCAU point struct
                  point_x: point coordinate x
                  point_y: point coordinate y
    \param[in]  curve_group_para: PKCAU ECC curve paramter struct
                  modulus_p: curve modulus p
                  coff_a: curve coefficient a
                  coff_b: curve coefficient b
                  base_point_x: curve base point coordinate x
                  base_point_y: curve base point coordinate y
                  order_n: curve prime order n
                  a_sign: curve coefficient a sign
                  modulus_p_len: curve modulus p length in byte
                  order_n_len: curve prime order n length in byte
                  multi_k: scalar multiplier k
                  k_len: length of scalar multiplier k
                  mont_para: montgomery parameter R2 mod n
    \param[in]  mode: point multiplication operation mode
                only one parameter can be selected which is shown as below:
      \arg        PKCAU_MODE_ECC_MUL: montgomery parameter computation then ECC scalar multiplication
      \arg        PKCAU_MODE_ECC_MUL_FAST: ECC scalar multiplication only
    \param[out] none
    \retval     none
*/
void pkcau_point_mul_operation(pkcau_point_parameter_struct *point_para, const pkcau_ec_group_parameter_struct* curve_group_para, uint32_t mode)
{
    uint32_t k_len = curve_group_para->k_len * 8U;
    uint32_t max_len_in_bit = curve_group_para->modulus_p_len * 8U;
    
    /* write the length of scalar multiplier k, curve modulus p length in bit and curve coefficient a sign to PKCAU RAM */
    pkcau_memcpy_value(0x00000400U, k_len);
    pkcau_memcpy_value(0x00000404U, max_len_in_bit);
    pkcau_memcpy_value(0x00000408U, curve_group_para->a_sign);
    
    /* write the curve coefficient a, curve modulus p, scalar multiplier k, point coordinate x and point coordinate y to PKCAU RAM */
    pkcau_memcpy_operand(0x0000040CU, curve_group_para->coff_a, curve_group_para->modulus_p_len);
    pkcau_memcpy_operand(0x00000460U, curve_group_para->modulus_p, curve_group_para->modulus_p_len);
    pkcau_memcpy_operand(0x00000508U, curve_group_para->multi_k, curve_group_para->k_len);
    pkcau_memcpy_operand(0x0000055CU, point_para->point_x, curve_group_para->modulus_p_len);
    pkcau_memcpy_operand(0x000005B0U, point_para->point_y, curve_group_para->modulus_p_len);
    if(mode == PKCAU_MODE_ECC_MUL_FAST){
        pkcau_memcpy_operand(0x000004B4U, curve_group_para->mont_para, curve_group_para->modulus_p_len);
    }
    /* configure the operation mode */
    pkcau_mode_set(mode);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      execute ECDSA sign operation
    \param[in]  p_key_d: private key d
    \param[in]  k: integer k
    \param[in]  hash_para: hash struct
                  hash_z: hash value z
                  hash_z_len: hash value z length in byte
    \param[in]  curve_group_para: PKCAU ECC curve paramter struct
                  modulus_p: curve modulus p
                  coff_a: curve coefficient a
                  coff_b: curve coefficient b
                  base_point_x: curve base point coordinate x
                  base_point_y: curve base point coordinate y
                  order_n: curve prime order n
                  a_sign: curve coefficient a sign
                  modulus_p_len: curve modulus p length in byte
                  order_n_len: curve prime order n length in byte
                  multi_k: scalar multiplier k
                  k_len: length of scalar multiplier k
                  mont_para: montgomery parameter R2 mod n
    \param[out] none
    \retval     none
*/
void pkcau_ecdsa_sign_operation(const uint8_t* p_key_d, const uint8_t* k, pkcau_hash_parameter_struct *hash_para, const pkcau_ec_group_parameter_struct* curve_group_para)
{
    uint32_t max_len_in_bit = curve_group_para->modulus_p_len * 8U;
    
    /* write the curve prime order n length, curve modulus p length in bit and curve coefficient a sign to PKCAU RAM */
    pkcau_memcpy_value(0x00000400U, curve_group_para->order_n_len * 8U);
    pkcau_memcpy_value(0x00000404U, max_len_in_bit);
    pkcau_memcpy_value(0x00000408U, curve_group_para->a_sign);
    
    /* write the curve coefficient a to PKCAU RAM */
    pkcau_memcpy_operand(0x0000040CU, curve_group_para->coff_a, curve_group_para->modulus_p_len);
    /* write the curve modulus p to PKCAU RAM */
    pkcau_memcpy_operand(0x00000460U, curve_group_para->modulus_p, curve_group_para->modulus_p_len);
    /* write the integer k to PKCAU RAM */
    pkcau_memcpy_operand(0x00000508U, k, curve_group_para->modulus_p_len);
    /* write the curve base point coordinate x to PKCAU RAM */
    pkcau_memcpy_operand(0x0000055CU, curve_group_para->base_point_x, curve_group_para->modulus_p_len);
    /* write the curve base point coordinate y to PKCAU RAM */
    pkcau_memcpy_operand(0x000005B0U, curve_group_para->base_point_y, curve_group_para->modulus_p_len);
    /* write the hash value z and hash value z length in byte to PKCAU RAM */
    pkcau_memcpy_operand(0x00000DE8U, hash_para->hash_z, hash_para->hash_z_len);
    /* write the private key d to PKCAU RAM */
    pkcau_memcpy_operand(0x00000E3CU, p_key_d, curve_group_para->modulus_p_len);
    /* write the curve prime order n to PKCAU RAM */
    pkcau_memcpy_operand(0x00000E94U, curve_group_para->order_n, curve_group_para->order_n_len);
    /* configure the operation mode */
    pkcau_mode_set(PKCAU_MODE_ECDSA_SIGN);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      execute ECDSA verification operation
    \param[in]  point_para: PKCAU point struct
                  point_x: point coordinate x
                  point_y: point coordinate y
    \param[in]  hash_para: hash struct
                  hash_z: hash value z
                  hash_z_len: hash value z length in byte
    \param[in]  signature_para: signature struct
                  sign_r: signature part r
                  sign_s: signature part s
    \param[in]  curve_group_para: PKCAU ECC curve paramter struct
                  modulus_p: curve modulus p
                  coff_a: curve coefficient a
                  coff_b: curve coefficient b
                  base_point_x: curve base point coordinate x
                  base_point_y: curve base point coordinate y
                  order_n: curve prime order n
                  a_sign: curve coefficient a sign
                  modulus_p_len: curve modulus p length in byte
                  order_n_len: curve prime order n length in byte
                  multi_k: scalar multiplier k
                  k_len: length of scalar multiplier k
                  mont_para: montgomery parameter R2 mod n
    \param[out] none
    \retval     none
*/
void pkcau_ecdsa_verification_operation(pkcau_point_parameter_struct *point_para, \
                              pkcau_hash_parameter_struct *hash_para, \
                              pkcau_signature_parameter_struct *signature_para, \
                              const pkcau_ec_group_parameter_struct* curve_group_para)
{
    uint32_t max_len_in_bit = curve_group_para->modulus_p_len * 8U;

    /* write the curve prime order n length, curve modulus p length in bit and curve coefficient a sign to PKCAU RAM */
    pkcau_memcpy_value(0x00000404U, curve_group_para->order_n_len * 8U);
    pkcau_memcpy_value(0x000004B4U, max_len_in_bit);
    pkcau_memcpy_value(0x0000045CU, curve_group_para->a_sign);
    
    /* write the curve coefficient a to PKCAU RAM */
    pkcau_memcpy_operand(0x00000460U, curve_group_para->coff_a, curve_group_para->modulus_p_len);
    /* write the curve modulus p to PKCAU RAM */
    pkcau_memcpy_operand(0x000004B8U, curve_group_para->modulus_p, curve_group_para->modulus_p_len);
    /* write the curve base point coordinate x to PKCAU RAM */
    pkcau_memcpy_operand(0x000005E8U, curve_group_para->base_point_x, curve_group_para->modulus_p_len);
    /* write the curve base point coordinate y to PKCAU RAM */
    pkcau_memcpy_operand(0x0000063CU, curve_group_para->base_point_y, curve_group_para->modulus_p_len);
    /* write the point coordinate x to PKCAU RAM */
    pkcau_memcpy_operand(0x00000F40U, point_para->point_x, curve_group_para->modulus_p_len);
    /* write the point coordinate y to PKCAU RAM */
    pkcau_memcpy_operand(0x00000F94U, point_para->point_y, curve_group_para->modulus_p_len);
    /* write the signature part r to PKCAU RAM */
    pkcau_memcpy_operand(0x000001098U, signature_para->sign_r, curve_group_para->modulus_p_len);
    /* write the signature part s to PKCAU RAM */
    pkcau_memcpy_operand(0x00000A44U, signature_para->sign_s, curve_group_para->modulus_p_len);
    /* write the hash value z and hash value z length in byte to PKCAU RAM */
    pkcau_memcpy_operand(0x00000FE8U, hash_para->hash_z, hash_para->hash_z_len);
    /* write the curve prime order n to PKCAU RAM */
    pkcau_memcpy_operand(0x00000D5CU, curve_group_para->order_n, curve_group_para->order_n_len);
    /* configure the operation mode */
    pkcau_mode_set(PKCAU_MODE_ECDSA_VERIFICATION);
    /* start computation */
    pkcau_start();
}

/*!
    \brief      read result from PKCAU RAM
    \param[in]  offset: RAM address offset to PKCAU base address
    \param[out] buf: big endian result buffer, the left most byte from the PKCAU should be in the first element of buffer
    \param[in]  size: number of byte to read
    \retval     none
*/
void pkcau_memread(uint32_t offset, uint8_t buf[], uint32_t size)
{
    uint32_t  data = 0U, i = 0U, j = 0U;
    
    /* read data from PKCAU RAM */
    while(size >= 4U){
        data = *(uint32_t*)((uint32_t)(PKCAU_BASE + offset + i));
        i = i + 4U;
        
        /* data in PKCAU RAM is big endian */
        buf[size-1U] = (uint8_t)(data & 0xffU);
        buf[size-2U] = (uint8_t)((data >> 8U) & 0xffU);
        buf[size-3U] = (uint8_t)((data >> 16U) & 0xffU);
        buf[size-4U] = (uint8_t)((data >> 24U) & 0xffU);
        size -= 4U;
    }
    /* read data from PKCAU RAM which is not a multiple of four */
    if(size > 0U){
        data = *(uint32_t*)((uint32_t)(PKCAU_BASE + offset + i));
        for(j = 0U; j < size; j++){
            buf[j] = (uint8_t)((data >> ((size-1U-j) * 8U)) & 0xffU);
        }
    }
}

/*!
    \brief      get PKCAU flag status
    \param[in]  flag: PKCAU flags
                only one parameter can be selected which is shown as below:
      \arg        PKCAU_FLAG_ADDRERR: address error flag
      \arg        PKCAU_FLAG_RAMERR: PKCAU RAM error flag
      \arg        PKCAU_FLAG_END: end of PKCAU operation flag
      \arg        PKCAU_FLAG_BUSY: busy flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus pkcau_flag_get(uint32_t flag)
{
    if(RESET != (PKCAU_STAT & flag)){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear PKCAU flag status
    \param[in]  flag: PKCAU flags
                one or more parameters can be selected which is shown as below:
      \arg        PKCAU_FLAG_ADDRERR: address error flag
      \arg        PKCAU_FLAG_RAMERR: PKCAU RAM error flag
      \arg        PKCAU_FLAG_END: end of PKCAU operation flag
    \param[out] none
    \retval     none
*/
void pkcau_flag_clear(uint32_t flag)
{
    switch(flag){
    /* clear address error flag */
    case PKCAU_FLAG_ADDRERR:
        PKCAU_STATC |= PKCAU_STATC_ADDRERRC;
        break;
    /* clear PKCAU RAM error flag */
    case PKCAU_FLAG_RAMERR:
        PKCAU_STATC |= PKCAU_STATC_RAMERRC;
        break;
    /* clear end of PKCAU operation flag */
    case PKCAU_FLAG_END:
        PKCAU_STATC |= PKCAU_STATC_ENDC;
        break;
    default :
        break;
    }
}

/*!
    \brief      enable PKCAU interrupt
    \param[in]  interrupt: interrupt type
                one or more parameters can be selected which is shown as below:
      \arg        PKCAU_INT_ADDRERR: address error interrupt
      \arg        PKCAU_INT_RAMERR: PKCAU RAM error interrupt
      \arg        PKCAU_INT_END: end of PKCAU operation interrupt
    \param[out] none
    \retval     none
*/
void pkcau_interrupt_enable(uint32_t interrupt)
{
    PKCAU_CTL |= interrupt;
}

/*!
    \brief      disable PKCAU interrupt
    \param[in]  interrupt: interrupt type
                one or more parameters can be selected which is shown as below:
      \arg        PKCAU_INT_ADDRERR: address error interrupt
      \arg        PKCAU_INT_RAMERR: PKCAU RAM error interrupt
      \arg        PKCAU_INT_END: end of PKCAU operation interrupt
    \param[out] none
    \retval     none
*/
void pkcau_interrupt_disable(uint32_t interrupt)
{
    PKCAU_CTL &= ~(interrupt);
}

/*!
    \brief      get PKCAU interrupt flag status
    \param[in]  int_flag: PKCAU interrupt flags
                only one parameter can be selected which is shown as below:
      \arg        PKCAU_INT_FLAG_ADDRERR: address error interrupt flag
      \arg        PKCAU_INT_FLAG_RAMERR: PKCAU RAM error interrupt flag
      \arg        PKCAU_INT_FLAG_END: end of PKCAU operation interrupt flag
    \param[out] none
    \retval     FlagStatus: SET or RESET
*/
FlagStatus pkcau_interrupt_flag_get(uint32_t int_flag)
{
    uint32_t reg1 = PKCAU_CTL;
    uint32_t reg2 = PKCAU_STAT;

    switch(int_flag){
    /* clear address error interrupt flag */
    case PKCAU_INT_FLAG_ADDRERR:
        reg1 = reg1 & PKCAU_CTL_ADDRERRIE;
        reg2 = reg2 & PKCAU_STAT_ADDRERR;
        break;
    /* clear RAM error interrupt flag */
    case PKCAU_INT_FLAG_RAMERR:
        reg1 = reg1 & PKCAU_CTL_RAMERRIE;
        reg2 = reg2 & PKCAU_STAT_RAMERR;
        break;
    /* clear end of PKCAU operation interrupt flag */
    case PKCAU_INT_FLAG_END:
        reg1 = reg1 & PKCAU_CTL_ENDIE;
        reg2 = reg2 & PKCAU_STAT_ENDF;
        break;
    default :
        break;
    }
    /*get PKCAU interrupt flag status */
    if(reg1 && reg2){
        return SET;
    }else{
        return RESET;
    }
}

/*!
    \brief      clear PKCAU interrupt flag status
    \param[in]  int_flag: PKCAU interrupt flags
                only one parameter can be selected which is shown as below:
      \arg        PKCAU_INT_FLAG_ADDRERR: address error interrupt flag
      \arg        PKCAU_INT_FLAG_RAMERR: PKCAU RAM error interrupt flag
      \arg        PKCAU_INT_FLAG_END: end of PKCAU operation interrupt flag
    \param[out] none
    \retval     none
*/
void pkcau_interrupt_flag_clear(uint32_t int_flag)
{
    switch(int_flag){
    /* clear address error interrupt flag */
    case PKCAU_INT_FLAG_ADDRERR:
        PKCAU_STATC |= PKCAU_STATC_ADDRERRC;
        break;
    /* clear PKCAU RAM error interrupt flag */
    case PKCAU_INT_FLAG_RAMERR:
        PKCAU_STATC |= PKCAU_STATC_RAMERRC;
        break;
    /* clear end of PKCAU operation interrupt flag */
    case PKCAU_INT_FLAG_END:
        PKCAU_STATC |= PKCAU_STATC_ENDC;
        break;
    default:
        break;
    }
}

/*!
    \brief      copy normal 32-bit value to PKCAU RAM
    \param[in]  offset: RAM address offset to PKCAU base address
    \param[in]  value: the value needed to write into PKCAU RAM
    \param[out] none
    \retval     none
*/
static void pkcau_memcpy_value(uint32_t offset, uint32_t value)
{
    uint32_t* addr = (uint32_t*)((uint32_t)(PKCAU_BASE + offset));
    *addr = value;
}

/*!
    \brief      copy operand with EOS or ROS to PKCAU RAM
    \param[in]  offset: RAM address offset to PKCAU base address
    \param[in]  operand: the big endian operand vector, the left most byte of the value should be in the first element of the vector
    \param[in]  size: operand vector length in byte
    \retval     none
*/
static void pkcau_memcpy_operand(uint32_t offset, const uint8_t operand[], uint32_t size)
{
    uint32_t data = 0U, i = 0U, j = 0U;

    while(size >= 4U){
        /* convert the big endian operand vector to little endian parameter to input to PKCAU RAM */
        data = (uint32_t)((uint32_t)operand[size-1U] | ((uint32_t)operand[size-2U] << 8U) | ((uint32_t)operand[size-3U] << 16U) | ((uint32_t)operand[size-4U] << 24U));
        *(uint32_t*)((uint32_t)(PKCAU_BASE + offset + i)) = data;
        i = i + 4U;
        size -= 4U;
    }
    /* convert the big endian operand vector to little endian parameter to input to PKCAU RAM which is not a multiple of four */
    if(size > 0U){
        data = 0U;
        for(j = 0U; j < size; j++){
            data = (uint32_t)((data << 8U) | (uint32_t)operand[j]);
        }
        *(uint32_t*)((uint32_t)(PKCAU_BASE + offset + i)) = data;
        i = i + 4U;
    }
    
    /* an additional word 0x00000000 is expected by the PKCAU */
    *(uint32_t*)((uint32_t)(PKCAU_BASE + offset + i)) = 0x00000000U;
}
