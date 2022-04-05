/*!
    \file    gd32w51x_cau_aes.c
    \brief   CAU AES driver
    
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

#include "gd32w51x_cau.h"

#define AESBSY_TIMEOUT    ((uint32_t)0x00010000U)
#define BLOCK_B0_MASK     ((uint8_t)0x07U)

/* AES key structure parameter config */
static void cau_aes_key_config(uint8_t *key, uint32_t keysize, cau_key_parameter_struct* cau_key_initpara);
/* fill data into data input register */
static ErrStatus cau_fill_data(uint8_t *input, uint32_t in_length);
/* AES calculate process */
static ErrStatus cau_aes_calculate(uint8_t *input, uint32_t in_length, uint8_t *output);

/*!
    \brief      encrypt and decrypt using AES in ECB mode
    \param[in]  cau_parameter: pointer to the input structure
                  alg_dir: algorithm dirctory
                    CAU_ENCRYPT, CAU_DECRYPT                  
                  key: key (in bytes)
                  key_size: key size in bits, must be either 128, 192 or 256
                  input: input data (in bytes)
                  in_length: input data length in bytes, must be a multiple of 16 bytes
    \param[out] output: pointer to the returned buffer
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus cau_aes_ecb(cau_parameter_struct *cau_parameter, uint8_t *output)
{
    ErrStatus ret = ERROR;
    cau_key_parameter_struct key_initpara;
    __IO uint32_t counter = 0U;
    uint32_t busystatus = 0U;
  
    /* key structure initialization */
    cau_key_struct_para_init(&key_initpara);
    /* AES key structure parameter config */
    cau_aes_key_config(cau_parameter->key, cau_parameter->key_size, &key_initpara);
    /* key initialization */
    cau_key_init(&key_initpara);  

    /* AES decryption */
    if(CAU_DECRYPT == cau_parameter->alg_dir){   
        /* flush the IN and OUT FIFOs */
        cau_fifo_flush();
        /* initialize the CAU peripheral */
        cau_init(CAU_DECRYPT, CAU_MODE_AES_KEY, CAU_SWAPPING_32BIT);      

        /* enable the CAU peripheral */
        cau_enable();

        /* wait until the busy flag is RESET */
        do{      
            busystatus = cau_flag_get(CAU_FLAG_BUSY);
            counter++;
        }while((AESBSY_TIMEOUT != counter) && (RESET != busystatus));

        if(RESET != busystatus){
            return ERROR;
        }
    }
    
    /* initialize the CAU peripheral */
    cau_init(cau_parameter->alg_dir, CAU_MODE_AES_ECB, CAU_SWAPPING_8BIT);
       
    /* flush the IN and OUT FIFOs */
    cau_fifo_flush();

    /* enable the CAU peripheral */
    cau_enable();
    /* AES calculate process */
    ret = cau_aes_calculate(cau_parameter->input, cau_parameter->in_length, output);
    /* disable the CAU peripheral */
    cau_disable();

    return ret;    
}

/*!
    \brief      encrypt and decrypt using AES in CBC mode
    \param[in]  cau_parameter: pointer to the input structure
                  alg_dir: algorithm dirctory
                    CAU_ENCRYPT, CAU_DECRYPT 
                  key: key (in bytes)
                  key_size: key size in bits, must be either 128, 192 or 256
                  iv: initialization vector, 16 bytes (in bytes)
                  input: input data (in bytes)
                  in_length: input data length in bytes, must be a multiple of 16 bytes
    \param[out] output: pointer to the returned buffer
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus cau_aes_cbc(cau_parameter_struct *cau_parameter, uint8_t *output)
{
    ErrStatus ret = ERROR;
    cau_key_parameter_struct key_initpara;
    cau_iv_parameter_struct iv_initpara;
    __IO uint32_t counter = 0U;
    uint32_t busystatus = 0U;
  
    uint32_t ivaddr = (uint32_t)cau_parameter->iv;

    /* key structure initialization */
    cau_key_struct_para_init(&key_initpara);
    /* AES key structure parameter config */
    cau_aes_key_config(cau_parameter->key, cau_parameter->key_size, &key_initpara);
    /* key initialization */
    cau_key_init(&key_initpara);  

    /* AES decryption */
    if(CAU_DECRYPT == cau_parameter->alg_dir){   
        /* flush the IN and OUT FIFOs */
        cau_fifo_flush();
        /* initialize the CAU peripheral */
        cau_init(CAU_DECRYPT, CAU_MODE_AES_KEY, CAU_SWAPPING_32BIT);      

        /* enable the CAU peripheral */
        cau_enable();

        /* wait until the busy flag is RESET */
        do{      
            busystatus = cau_flag_get(CAU_FLAG_BUSY);
            counter++;
        }while((AESBSY_TIMEOUT != counter) && (RESET != busystatus));

        if(RESET != busystatus){
            return ERROR;
        }
    }
    
    /* initialize the CAU peripheral */
    cau_init(cau_parameter->alg_dir, CAU_MODE_AES_CBC, CAU_SWAPPING_8BIT);
    
    /* vectors initialization */
    iv_initpara.iv_0_high = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_0_low = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_1_high = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_1_low = __REV(*(uint32_t*)(ivaddr));
    cau_iv_init(&iv_initpara);    

    /* flush the IN and OUT FIFOs */
    cau_fifo_flush();

    /* enable the CAU peripheral */
    cau_enable();
    /* AES calculate process */
    ret = cau_aes_calculate(cau_parameter->input, cau_parameter->in_length, output);
    /* disable the CAU peripheral */
    cau_disable();

    return ret;
}

/*!
    \brief      encrypt and decrypt using AES in CTR mode
    \param[in]  cau_parameter: pointer to the input structure
                  alg_dir: algorithm dirctory
                    CAU_ENCRYPT, CAU_DECRYPT 
                  key: key (in bytes)
                  key_size: key size in bits, must be either 128, 192 or 256
                  iv: initialization vector, 16 bytes (in bytes)
                  input: input data (in bytes)
                  in_length: input data length in bytes, must be a multiple of 16 bytes
    \param[out] output: pointer to the returned buffer
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus cau_aes_ctr(cau_parameter_struct *cau_parameter, uint8_t *output)
{
    ErrStatus ret = ERROR;
    cau_key_parameter_struct key_initpara;
    cau_iv_parameter_struct iv_initpara;       
    uint32_t ivaddr = (uint32_t)cau_parameter->iv;

    /* key structure initialization */
    cau_key_struct_para_init(&key_initpara);
    /* initialize the CAU peripheral */
    cau_init(cau_parameter->alg_dir, CAU_MODE_AES_CTR, CAU_SWAPPING_8BIT);  
  
    /* AES key structure parameter config */
    cau_aes_key_config(cau_parameter->key, cau_parameter->key_size, &key_initpara);
    /* key initialization */
    cau_key_init(&key_initpara);
  
    /* vectors initialization */
    iv_initpara.iv_0_high = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_0_low = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_1_high = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_1_low = __REV(*(uint32_t*)(ivaddr));
    cau_iv_init(&iv_initpara);

    /* flush the IN and OUT FIFOs */
    cau_fifo_flush();

    /* enable the CAU peripheral */
    cau_enable();
    /* AES calculate process */
    ret = cau_aes_calculate(cau_parameter->input, cau_parameter->in_length, output);
    /* disable the CAU peripheral */
    cau_disable();

    return ret;
}

/*!
    \brief      encrypt and decrypt using AES in CFB mode
    \param[in]  cau_parameter: pointer to the input structure
                  alg_dir: algorithm dirctory
                    CAU_ENCRYPT, CAU_DECRYPT 
                  key: key (in bytes)
                  key_size: key size in bits, must be either 128, 192 or 256
                  iv: initialization vector, 16 bytes (in bytes)
                  input: input data (in bytes)
                  in_length: input data length in bytes, must be a multiple of 16 bytes
    \param[out] output: pointer to the returned buffer
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus cau_aes_cfb(cau_parameter_struct *cau_parameter, uint8_t *output)
{
    ErrStatus ret = ERROR;
    cau_key_parameter_struct key_initpara;
    cau_iv_parameter_struct iv_initpara;        
    uint32_t ivaddr = (uint32_t)cau_parameter->iv;

    /* key structure initialization */
    cau_key_struct_para_init(&key_initpara);
    /* initialize the CAU peripheral */
    cau_init(cau_parameter->alg_dir, CAU_MODE_AES_CFB, CAU_SWAPPING_8BIT);  
  
    /* AES key structure parameter config */
    cau_aes_key_config(cau_parameter->key, cau_parameter->key_size, &key_initpara);
    /* key initialization */
    cau_key_init(&key_initpara);
  
    /* vectors initialization */
    iv_initpara.iv_0_high = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_0_low = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_1_high = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_1_low = __REV(*(uint32_t*)(ivaddr));
    cau_iv_init(&iv_initpara);

    /* flush the IN and OUT FIFOs */
    cau_fifo_flush();
    /* enable the CAU peripheral */
    cau_enable();
    /* AES calculate process */
    ret = cau_aes_calculate(cau_parameter->input, cau_parameter->in_length, output);
    /* disable the CAU peripheral */
    cau_disable();

    return ret;
}

/*!
    \brief      encrypt and decrypt using AES in OFB mode
    \param[in]  cau_parameter: pointer to the input structure
                  alg_dir: algorithm dirctory
                    CAU_ENCRYPT, CAU_DECRYPT 
                  key: key (in bytes)
                  key_size: key size in bits, must be either 128, 192 or 256
                  iv: initialization vector, 16 bytes (in bytes)
                  input: input data (in bytes)
                  in_length: input data length in bytes, must be a multiple of 16 bytes
    \param[out] output: pointer to the returned buffer
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus cau_aes_ofb(cau_parameter_struct *cau_parameter, uint8_t *output)
{
    ErrStatus ret = ERROR;
    cau_key_parameter_struct key_initpara;
    cau_iv_parameter_struct iv_initpara;        
    uint32_t ivaddr = (uint32_t)cau_parameter->iv;

    /* key structure initialization */
    cau_key_struct_para_init(&key_initpara);
    /* initialize the CAU peripheral */
    cau_init(cau_parameter->alg_dir, CAU_MODE_AES_OFB, CAU_SWAPPING_8BIT);  
  
    /* AES key structure parameter config */
    cau_aes_key_config(cau_parameter->key, cau_parameter->key_size, &key_initpara);
    /* key initialization */
    cau_key_init(&key_initpara);
  
    /* vectors initialization */
    iv_initpara.iv_0_high = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_0_low = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_1_high = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_1_low = __REV(*(uint32_t*)(ivaddr));
    cau_iv_init(&iv_initpara);

    /* flush the IN and OUT FIFOs */
    cau_fifo_flush();
    /* enable the CAU peripheral */
    cau_enable();
    /* AES calculate process */
    ret = cau_aes_calculate(cau_parameter->input, cau_parameter->in_length, output);
    /* disable the CAU peripheral */
    cau_disable();

    return ret;
}

/*!
    \brief      encrypt and decrypt using AES in GCM mode
    \param[in]  cau_parameter: pointer to the input structure
                  alg_dir: algorithm dirctory
                    CAU_ENCRYPT, CAU_DECRYPT 
                  key: key (in bytes)
                  key_size: key size in bits, must be either 128, 192 or 256
                  iv: initialization vector, 16 bytes (in bytes)
                  input: input data (in bytes)
                  in_length: input data length in bytes, must be a multiple of 16 bytes
                  aad: additional authentication data (in bytes)
                  aad_size: aad size in bytes, must be a multiple of 16 bytes
    \param[out] output: pointer to the returned output data buffer
    \param[out] tag: pointer to the returned tag buffer
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus cau_aes_gcm(cau_parameter_struct *cau_parameter, uint8_t *output, uint8_t *tag)
{
    ErrStatus ret = SUCCESS;
    cau_key_parameter_struct key_initpara;
    cau_iv_parameter_struct iv_initpara;
    uint32_t aadlength_h = cau_parameter->aad_size >> 29;
    uint32_t aadlength_l = cau_parameter->aad_size << 3;
    uint32_t inputlength_h = cau_parameter->in_length >> 29; 
    uint32_t inputlength_l = cau_parameter->in_length << 3;               
    uint32_t ivaddr = (uint32_t)cau_parameter->iv;
    uint32_t tagaddr = (uint32_t)tag;

    /* key structure initialization */
    cau_key_struct_para_init(&key_initpara);
    /* initialize the CAU peripheral */
    cau_init(cau_parameter->alg_dir, CAU_MODE_AES_GCM, CAU_SWAPPING_8BIT);  
  
    /* AES key structure parameter config */
    cau_aes_key_config(cau_parameter->key, cau_parameter->key_size, &key_initpara);
    /* key initialization */
    cau_key_init(&key_initpara);
  
    /* vectors initialization */
    iv_initpara.iv_0_high = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_0_low = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_1_high = __REV(*(uint32_t*)(ivaddr));
    ivaddr += 4U;
    iv_initpara.iv_1_low = __REV(*(uint32_t*)(ivaddr));
    cau_iv_init(&iv_initpara);

    /*========================= prepare phase=========================*/
    /* select prepare phase */
    cau_phase_config(CAU_PREPARE_PHASE);
    /* enable the CAU peripheral */
    cau_enable();
    /* wait for CAUEN bit to be 0 */
    while(ENABLE == cau_enable_state_get()){
    }

    /*========================= AAD phase=======================*/
    if((uint32_t)0 != cau_parameter->aad_size){
        /* Select AAD phase */
        cau_phase_config(CAU_AAD_PHASE);
        /* flush the IN and OUT FIFOs */
        cau_fifo_flush();
        /* enable the CAU peripheral */
        cau_enable();
        
        ret = cau_fill_data(cau_parameter->aad, cau_parameter->aad_size);

        if(ERROR == ret){
            return ret;
        }    
    }
    
    /*========================= encryption/decryption phase=======================*/
    if((uint32_t)0 != cau_parameter->in_length){
        /* Select encryption/decryption phase */
        cau_phase_config(CAU_ENCRYPT_DECRYPT_PHASE);
        /* flush the IN and OUT FIFOs */
        cau_fifo_flush();        
        /* enable the CAU peripheral */
        cau_enable();
        
        /* AES calculate process */
        ret = cau_aes_calculate(cau_parameter->input, cau_parameter->in_length, output);

        if(ERROR == ret){
            return ret; 
        }    
    }
    
    /*========================= tag phase=======================*/
    /* select tag phase */
    cau_phase_config(CAU_TAG_PHASE);
    /* flush the IN and OUT FIFOs */
    cau_fifo_flush();     
    /* enable the CAU peripheral */
    cau_enable();

    if(DISABLE == cau_enable_state_get()){
        return ERROR;    
    }
    
    cau_data_write(__REV(aadlength_h));
    cau_data_write(__REV(aadlength_l));
    cau_data_write(__REV(inputlength_h));
    cau_data_write(__REV(inputlength_l));
    
    /* wait until the ONE flag is set */
    while(RESET == cau_flag_get(CAU_FLAG_OUTFIFO_NO_EMPTY)){
    }

    /* read the tag in the OUT FIFO */
    *(uint32_t*)(tagaddr) = cau_data_read();
    tagaddr += 4U;
    *(uint32_t*)(tagaddr) = cau_data_read();
    tagaddr += 4U;
    *(uint32_t*)(tagaddr) = cau_data_read();
    tagaddr += 4U;
    *(uint32_t*)(tagaddr) = cau_data_read();
    tagaddr += 4U;    

    /* disable the CAU peripheral */
    cau_disable();

    return ret;
}

/*!
    \brief      encrypt and decrypt using AES in CCM mode
    \param[in]  cau_parameter: pointer to the input structure
                  alg_dir: algorithm dirctory
                    CAU_ENCRYPT, CAU_DECRYPT 
                  key: key (in bytes)
                  key_size: key size in bytes, must be either 128, 192 or 256
                  iv: initialization vector (in bytes)
                  iv_size: iv size in bytes, must be 7 - 13
                  input: input data (in bytes)
                  in_length: input data length in bytes, must be a multiple of 16 bytes
                  aad: additional authentication data [in & out] (in bytes)
                  aad_size: aad size [in & out] (in bytes)
    \param[in]  tag_size: tag size (in bytes)
    \param[out] output: pointer to the returned output data buffer
    \param[out] tag: pointer to the returned tag buffer
    \param[out] aad_buf: pointer to the user buffer used when formatting aad block
    \retval     ErrStatus: SUCCESS or ERROR
*/
ErrStatus cau_aes_ccm(cau_parameter_struct *cau_parameter, uint8_t *output, uint8_t tag[], uint32_t tag_size, uint8_t aad_buf[])
{
    ErrStatus ret = ERROR;
    cau_key_parameter_struct key_initpara;
    cau_iv_parameter_struct iv_initpara;
    uint32_t inputaddr  = (uint32_t)cau_parameter->input;
    uint32_t inputsize = cau_parameter->in_length;
    uint32_t aadaddr = (uint32_t)cau_parameter->aad;    
    uint32_t aadsize = cau_parameter->aad_size;
    uint32_t ivaddr = (uint32_t)cau_parameter->iv;
    uint32_t ivsize = cau_parameter->iv_size;
    uint32_t temptag[4] = {0U};
    uint32_t outputaddr = (uint32_t)output;
    uint32_t i = 0U, plen = 0U;
    uint32_t head_index = 0U;
    uint8_t blockb0[16U] = {0U};
    uint8_t counter[16U] = {0U};
    uint32_t ctraddr = (uint32_t)counter;
    uint32_t b0addr = (uint32_t)blockb0;    

    /*===============formatting the aad block=================*/
    if((uint32_t)0 != aadsize){
        /* check that the aad length is lower than 2^16 - 2^8 = 65536 - 256 = 65280 */
        if(aadsize < 65280U){
            aad_buf[head_index++] = (uint8_t) ((aadsize >> 8U) & 0xFFU);
            aad_buf[head_index++] = (uint8_t) ((aadsize) & 0xFFU);
            aadsize += 2U;
        }else{
            /* aad is encoded as 0xff || 0xfe || [aadsize]32, i.e., six octets */
            aad_buf[head_index++] = 0xFFU;
            aad_buf[head_index++] = 0xFEU;
            aad_buf[head_index++] = (uint8_t)((aadsize & 0xff000000U) >> 24U);
            aad_buf[head_index++] = (uint8_t)((aadsize & 0x00ff0000U) >> 16U);
            aad_buf[head_index++] = (uint8_t)((aadsize & 0x0000ff00U) >> 8U);
            aad_buf[head_index++] = (uint8_t)(aadsize & 0x000000ffU);
            aadsize += 6U;
        }
        /* Copy the aad buffer in internal buffer "HBuffer" */
        for(i = 0U; i < aadsize; i++){
            aad_buf[head_index++] = *(uint8_t*)((uint32_t)(aadaddr + i));
        }
        /* Check if the aad size is modulo 16 */
        if ((aadsize % 16U) != 0U){
            /* pad the aad buffer with 0s till the aad_buf length is multiple of 16 */
            for(i = aadsize; i <= ((aadsize/16U) + 1U) * 16U; i++){
                aad_buf[i] = 0U;
            }
            /* Set the aad size to multiple of 16 */
            aadsize = ((aadsize/16U) + 1U) * 16U;
        }
        /* set the pointer aadaddr to aad_buf */
        aadaddr = (uint32_t)aad_buf;
    }    
    /*=================formatting the block B0===================*/
    if(aadsize != 0U){
        blockb0[0U] = 0x40U;
    }
    
    /* the byte length for payload length expressing, which plus the ivsize must equal to 15 bytes */
    plen = 15U - ivsize;
    /* flags byte */
    blockb0[0] |= 0U | (((((uint8_t)tag_size - 2U) / 2U) & 0x07U) << 3U) | (((uint8_t)plen - 1U) & 0x07U);
  
    for(i = 0U; i < ivsize; i++){
      blockb0[i+1U] = *(uint8_t*)((uint32_t)(ivaddr + i));
    }
    
    /* if the byte length for payload length expressing is more than 4 bytes */
    if(plen > 4U){
        /* pad the blockb0 after vectors, and before the last 4 bytes */
        for( ; i < 11U; i++){
            blockb0[i+1U] = 0U;
        }
        blockb0[12U] = (uint8_t)((inputsize >> 24U) & 0xFFU);
        blockb0[13U] = (uint8_t)((inputsize >> 16U) & 0xFFU); 
        blockb0[14U] = (uint8_t)((inputsize >> 8U) & 0xFFU);
        blockb0[15U] = (uint8_t)(inputsize & 0xFFU);        
    }else{
        /* the payload length is expressed in plen bytes */
        for( ; i < 15U; i++){
            blockb0[i+1U] = (uint8_t)((inputsize >> ((plen - 1U) * 8U)) & 0xFFU);
            plen--;
        }
    }
  
    /*=======formatting the counter CTR1 for encryption/decription=======*/
    /* byte 0: bits 0-2 contain the same encoding of q as in B0 */    
    counter[0] = blockb0[0] & BLOCK_B0_MASK;
    for(i = 1U; i < ivsize + 1U; i++){
        counter[i] = blockb0[i];
    }
    /* counter CTR1 is initialized to 1 */
    counter[15] |= 0x01U;   
    
    /*========================= prepare phase=========================*/
    /* flush the IN and OUT FIFOs */
    cau_fifo_flush();    
    /* clear CAUEN bit to ensure CAU is disable */
    cau_disable();
    
    /* key structure initialization */
    cau_key_struct_para_init(&key_initpara);
    /* initialize the CAU peripheral */
    cau_init(cau_parameter->alg_dir, CAU_MODE_AES_CCM, CAU_SWAPPING_8BIT);  
    /* select prepare phase */
    cau_phase_config(CAU_PREPARE_PHASE);
  
    /* AES key structure parameter config */
    cau_aes_key_config(cau_parameter->key, cau_parameter->key_size, &key_initpara);
    /* key initialization */
    cau_key_init(&key_initpara);
  
    /* vectors initialization */
    iv_initpara.iv_0_high = __REV(*(uint32_t*)(ctraddr));
    ctraddr += 4U;
    iv_initpara.iv_0_low = __REV(*(uint32_t*)(ctraddr));
    ctraddr += 4U;
    iv_initpara.iv_1_high = __REV(*(uint32_t*)(ctraddr));
    ctraddr += 4U;
    iv_initpara.iv_1_low = __REV(*(uint32_t*)(ctraddr));
    cau_iv_init(&iv_initpara);

    /* enable the CAU peripheral */
    cau_enable();    
    
    /* write block B0 in the In FIFO */
    cau_data_write(*(uint32_t*)(b0addr));
    b0addr += 4U;
    cau_data_write(*(uint32_t*)(b0addr));
    b0addr += 4U;
    cau_data_write(*(uint32_t*)(b0addr));
    b0addr += 4U;
    cau_data_write(*(uint32_t*)(b0addr));
    
    /* wait for CAUEN bit to be 0 */
    while(ENABLE == cau_enable_state_get()){
    }

    /*========================= AAD phase=======================*/
    if((uint32_t)0U != aadsize){
        /* select AAD phase */
        cau_phase_config(CAU_AAD_PHASE);
        /* enable the CAU peripheral */
        cau_enable();
        
        ret = cau_fill_data((uint8_t*)aadaddr, aadsize);

        if(ERROR == ret){
            return ret;
        }    
    }
    
    /*========================= encryption/decryption phase=======================*/
    if((uint32_t)0U != inputsize){
        /* select encryption/decryption phase */
        cau_phase_config(CAU_ENCRYPT_DECRYPT_PHASE);
        /* enable the CAU peripheral */
        cau_enable();
        
        /* AES calculate process */
        ret = cau_aes_calculate((uint8_t*)inputaddr, inputsize, (uint8_t*)outputaddr);

        if(ERROR == ret){
            return ret;   
        }    
    }
    
    /*========================= tag phase=======================*/
    /* select tag phase */
    cau_phase_config(CAU_TAG_PHASE);
    /* enable the CAU peripheral */
    cau_enable();

    if(DISABLE == cau_enable_state_get()){
        return ERROR;    
    }
    
    ctraddr = (uint32_t)counter;
    
    cau_data_write(*(uint32_t*)(ctraddr));
    ctraddr += 4U;
    cau_data_write(*(uint32_t*)(ctraddr));
    ctraddr += 4U;
    cau_data_write(*(uint32_t*)(ctraddr));
    ctraddr += 4U;
    /* reset bit 0 (after 8-bit swap) is equivalent to reset bit 24 (before 8-bit swap) */
    cau_data_write(*(uint32_t*)(ctraddr) & 0xfeffffffU);

    /* wait until the ONE flag is set */
    while(RESET == cau_flag_get(CAU_FLAG_OUTFIFO_NO_EMPTY)){
    }

    /* read the tag in the OUT FIFO */
    temptag[0U] = cau_data_read();
    temptag[1U] = cau_data_read();
    temptag[2U] = cau_data_read();
    temptag[3U] = cau_data_read(); 
    
    for(i = 0U; i < tag_size; i++){
/*        tag[i] = *((uint8_t*)temptag + i);*/
        tag[i] = (uint8_t)(temptag[i/4U] >> (8U * (i % 4U)));
    }

    /* disable the CAU peripheral */
    cau_disable();

    return ret;
}
/*!
    \brief      AES key structure parameter config
    \param[in]  key: key used for AES algorithm (in bytes)
    \param[in]  keysize: length of the key in bits, must be either 128, 192 or 256 
    \param[out] cau_key_initpara: key init parameter struct
                  key_0_high: key 0 high
                  key_0_low:  key 0 low
                  key_1_high: key 1 high
                  key_1_low:  key 1 low
                  key_2_high: key 2 high
                  key_2_low:  key 2 low
                  key_3_high: key 3 high
                  key_3_low:  key 3 low
    \retval     none
*/
static void cau_aes_key_config(uint8_t *key, uint32_t keysize, cau_key_parameter_struct* cau_key_initpara)
{
    uint32_t keyaddr = (uint32_t)key;
  
    switch(keysize){
    case 128:        
        cau_aes_keysize_config(CAU_KEYSIZE_128BIT);
        cau_key_initpara->key_2_high = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_2_low = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_3_high = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_3_low = __REV(*(uint32_t*)(keyaddr));
        break;
    case 192:
        cau_aes_keysize_config(CAU_KEYSIZE_192BIT);
        cau_key_initpara->key_1_high = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_1_low = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_2_high = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_2_low = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_3_high = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_3_low = __REV(*(uint32_t*)(keyaddr));
        break;
    case 256:
        cau_aes_keysize_config(CAU_KEYSIZE_256BIT);
        cau_key_initpara->key_0_high = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_0_low = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_1_high = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_1_low = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_2_high = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_2_low = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_3_high = __REV(*(uint32_t*)(keyaddr));
        keyaddr += 4U;
        cau_key_initpara->key_3_low = __REV(*(uint32_t*)(keyaddr));
        break;
    default:
        break;
    }
}

/*!
    \brief      fill data into data input register
    \param[in]  input: pointer to the input buffer (in bytes)    
    \param[in]  in_length: length of the input buffer in bytes, must be a multiple of 16 bytes    
    \retval     ErrStatus: SUCCESS or ERROR
*/
static ErrStatus cau_fill_data(uint8_t *input, uint32_t in_length)
{
    uint32_t inputaddr  = (uint32_t)input;
    uint32_t i = 0U;
    __IO uint32_t counter = 0U;
    uint32_t busystatus = 0U;

    if(DISABLE == cau_enable_state_get()){
        return ERROR;
    }
        
    for(i = 0U; i < in_length; i += 16U){    
        /* wait until the IEM flag is set */
        while(RESET == cau_flag_get(CAU_FLAG_INFIFO_EMPTY)){
        }

        /* write data to the IN FIFO */
        cau_data_write(*(uint32_t*)(inputaddr));
        inputaddr += 4U;
        cau_data_write(*(uint32_t*)(inputaddr));
        inputaddr += 4U;
        cau_data_write(*(uint32_t*)(inputaddr));
        inputaddr += 4U;
        cau_data_write(*(uint32_t*)(inputaddr));
        inputaddr += 4U;      
    }
    /* wait until the complete message has been processed */
    counter = 0U;
    do{        
        busystatus = cau_flag_get(CAU_FLAG_BUSY);
        counter++;
    }while((AESBSY_TIMEOUT != counter) && (RESET != busystatus));  
   
    if(RESET != busystatus){       
        return ERROR;
    }
    
    return SUCCESS;
}

/*!
    \brief      AES calculate process
    \param[in]  input: pointer to the input buffer     
    \param[in]  in_length: length of the input buffer in bytes, must be a multiple of 16 bytes  
    \param[out]  output: pointer to the returned buffer    
    \retval     ErrStatus: SUCCESS or ERROR
*/
static ErrStatus cau_aes_calculate(uint8_t *input, uint32_t in_length, uint8_t *output)
{
    uint32_t inputaddr  = (uint32_t)input;
    uint32_t outputaddr = (uint32_t)output;
    uint32_t i = 0U;
    __IO uint32_t counter = 0U;
    uint32_t busystatus = 0U;  
  
    /* the clock is not enabled or there is no embeded CAU peripheral */           
    if(DISABLE == cau_enable_state_get()){     
        return ERROR;
    }

    for(i = 0U; i < in_length; i += 16U){ 
        /* wait until the IEM flag is set */
        while(RESET == cau_flag_get(CAU_FLAG_INFIFO_EMPTY)){
        }        
        /* write data to the IN FIFO */
        cau_data_write(*(uint32_t*)(inputaddr));
        inputaddr += 4U;
        cau_data_write(*(uint32_t*)(inputaddr));
        inputaddr += 4U;
        cau_data_write(*(uint32_t*)(inputaddr));
        inputaddr += 4U;
        cau_data_write(*(uint32_t*)(inputaddr));
        inputaddr += 4U;

        /* wait until the complete message has been processed */
        counter = 0U;
        do{        
            busystatus = cau_flag_get(CAU_FLAG_BUSY);
            counter++;
        }while((AESBSY_TIMEOUT != counter) && (RESET != busystatus));

        if(RESET != busystatus){       
            return ERROR;
        }else{
            /* wait until the ONE flag is set */
            while(RESET == cau_flag_get(CAU_FLAG_OUTFIFO_NO_EMPTY)){
            }
            
            /* read the output block from the output FIFO */
            *(uint32_t*)(outputaddr) = cau_data_read();
            outputaddr += 4U;
            *(uint32_t*)(outputaddr) = cau_data_read();
            outputaddr += 4U;
            *(uint32_t*)(outputaddr) = cau_data_read();
            outputaddr += 4U;
            *(uint32_t*)(outputaddr) = cau_data_read();
            outputaddr += 4U;          
        }
    }
  
    return SUCCESS;
}
