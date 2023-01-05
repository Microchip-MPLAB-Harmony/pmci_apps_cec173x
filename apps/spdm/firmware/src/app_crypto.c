/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    app_crypto.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/
#include "app_crypto.h"
#include "spdm_pkt_prcs.h"
#include "spdm_task.h"
#include "spdm.h"
#include "sb_gpio.h"

struct mchphash mchp_hash;
uint8_t hw_config_buff[SRAM_MBOX_HW_CONFIG_SIZE] __attribute__((aligned(8)));
extern SPDM_BSS2_ATTR SPDM_CONTEXT *spdmContext;
SPDM_BSS0_ATTR HASH_CTX_DATA ctx_ptr;
SPDM_BSS0_ATTR uint32_t mchp_pk_cnx_mem_spdm_module[PK_CNX_MEM_SIZE/4];
SPDM_BSS0_ATTR struct mchp_pk_cnx* init_pke;
SPDM_BSS0_ATTR mchp_op ptr_pvt_key, ptr_rnd, ptr_hash;
SPDM_BSS0_ATTR uint8_t otp_array[286];
SPDM_BSS0_ATTR uint16_t sig_length;

/******************************************************************************/
/** sb_sram_rlog_read - Read data from SRAM ROM log
 * @param byte index - offset in RLOG data memory
 * @param efuse_buffer - Buffer to read data into
 * @param length - length of RLOG data to be read
 * @return uint8_t 0(success), 1(fail).
 ******************************************************************************/
uint8_t sb_sram_rlog_read(uint16_t byte_index, uint8_t *buffer, uint16_t length)
{
    uint8_t *rlog_ptr;
    uint16_t loopCount = 0u;
    uint8_t retVal = 0;

    do
    {		
        rlog_ptr = (uint8_t *) SRAM_ROM_LOG_START_LOCATION_A1;

        // Read Data
        for (loopCount = 0; loopCount < length; loopCount++)
        {
            buffer[loopCount] = rlog_ptr[(loopCount + byte_index)];
        }

    }
    while(0);

    return retVal;
}

/******************************************************************************/
/** Wrapper function of SPDM module to call the sha operations function in sb dgst
* @param uint8_t *buf_ptr - pointer to input data buffer
* @param uint32_t len - data length
* @param SPDM_CONTEXT *spdmContext -SPDM module context
* @return SUCCESS/FAILURE
*******************************************************************************/
uint8_t spdm_crypto_ops_calc_hash(uint8_t *buf_ptr, uint32_t len, SPDM_CONTEXT *spdmContext)
{   
    uint8_t ret_val;
    ret_val = do_sha(SHA_MODE_384, &buf_ptr[0], len, &spdmContext->sha_digest[0]);
    return ret_val;
}

/******************************************************************************/
/** do_sha();
* Perform the sha operation
* @param hash_algo - hash algorithm code
* @param input_buf - input data
* @param input_len - length of data
* @param output - buffer to copy the hash output
* @return 0 on success or error codes (SHA_RET_CODE)
*******************************************************************************/
SHA_RET_CODE do_sha(uint8_t hash_algo, uint8_t *input_buf, uint32_t input_len, uint8_t *output)
{
    SHA_RET_CODE ret = SHA_ERROR;
    int8_t opn_sts =0u;

    switch (hash_algo)
    {
        case SHA_MODE_256:
            opn_sts = mchp_hash_create_sha256(&mchp_hash);
            break;
        case SHA_MODE_384:
            opn_sts = mchp_hash_create_sha384(&mchp_hash);
            break;
        default:
            opn_sts = 1;
            break;
    }

    if (opn_sts == 0) {
        /* add data to DMA descriptors */
        opn_sts = mchp_hash_feed(&mchp_hash, (const char *)input_buf, (size_t)input_len);
		if (opn_sts == 0) {
			opn_sts = mchp_hash_digest(&mchp_hash, (char *)output);
            
            if(opn_sts == 0)
            {
                opn_sts = mchp_hash_wait(&mchp_hash);
                if (opn_sts != MCHP_OK) {
		            return SHA_ERROR;
	            }
                ret = SHA_SUCCESS;
                do
                {
                    opn_sts = mchp_hash_wait(&mchp_hash);
                }while(opn_sts == HASH_HW_BUSY);
            }
            else
            {
                /* Hash digest update failed */
            }            
		}
        else
        {
            /* Hash feed failed */
        }
    }
    else
    {
        /* Hash initialization failed */
    }

    return( ret );    
}

/******************************************************************************
 * efuse_read_data - Read data from efuse
 * @param byte index - offset in efuse data memory
 * @param efuse_buffer - Buffer to read data into
 * @param length - length of efuse data to be read
 * @param rev - read the value in reverse
 * @return uint8_t 0(success), 1(fail).
 ******************************************************************************/
uint8_t efuse_read_data(uint16_t byte_index, uint8_t* efuse_buffer, uint16_t length)
{
    uint16_t loopCount = 0u;
    uint8_t retVal = 0;

    do
    {
        if ((byte_index + length - 1) >= 1024)   //Will try to access an out of range index
        {
            /* Exited read, access out of range */
            retVal = 1;
            break;
        }
        //Attempt to read data, stop if any error occurs
        for (loopCount = 0; loopCount < length; loopCount++)
        {
            if (!api_efuse_byte_read((byte_index + loopCount), &efuse_buffer[loopCount]))
            {
                retVal = 1;
                break;  /* SOTG3-930 if read fail log error continue to read other OTP's  */
            }
        }
    }
    while(0);

    return retVal;
}


/******************************************************************************
 ** get_cert2_base_address()
 * This function is to get Certificate 2 Address
 * @param cert_ptr         Pointer to hold certificate 2 address
 * @return                 None
 * @note
 ******************************************************************************/
void get_cert2_base_address(uint32_t *cert_ptr)
{
    uint8_t efuse_data = 0x00;

    if (0 == efuse_read_data(CERT2_BASE_ADDR_OTP_OFFSET, &efuse_data, 1))
    {
        *cert_ptr = efuse_data;
    }
    if (0 == efuse_read_data(CERT2_BASE_ADDR_OTP_OFFSET + 1, &efuse_data, 1))
    {
        *cert_ptr = ((*cert_ptr << 8) | efuse_data);
    }
    if (0 == efuse_read_data(CERT2_BASE_ADDR_OTP_OFFSET + 2, &efuse_data, 1))
    {
        *cert_ptr = ((*cert_ptr << 8) | efuse_data);
    }
    if (0 == efuse_read_data(CERT2_BASE_ADDR_OTP_OFFSET + 3, &efuse_data, 1))
    {
        *cert_ptr = ((*cert_ptr << 8) | efuse_data);
    }
}

/******************************************************************************/
/** sb_sram_mbox_read - Read data from SRAM Mailbox
 * @param byte index - offset in MBOX data memory
 * @param efuse_buffer - Buffer to read data into
 * @param length - length of MBOX data to be read
 * @return uint8_t 0(success), 1(fail).
 ******************************************************************************/
uint8_t sb_sram_mbox_read(uint16_t byte_index, uint8_t *buffer, uint16_t length)
{
    uint8_t *mbox_ptr;
    uint16_t loopCount = 0u;
    uint8_t retVal = 0;

    do
    {
        if ((byte_index + length - 1) >= SRAM_MBOX_SIZE)   //Will try to access an out of range index
        {
            retVal = SRAM_MBOX_READ_FAIL;
            break;
        }

        mbox_ptr = (uint8_t *)(SRAM_MBOX_ADDRESS_START);

        // Read Data
        for (loopCount = 0; loopCount < length; loopCount++)
        {
            buffer[loopCount] = mbox_ptr[(loopCount + byte_index)];
        }

    }
    while(0);

    return retVal;
}

/******************************************************************************/
/** sb_sram_mbox_rom_hash_read - Reads rom hash info from SRAM Mailbox
 * @param buffer - Buffer to read data into
 * @return uint8_t 0(success), 1(fail).
 ******************************************************************************/
uint8_t sb_sram_mbox_rom_hash_read(uint8_t *buffer)
{
    uint8_t ret_val = 0;

    if(sb_sram_mbox_read(ROM_HASH_MAILBOX_OFFSET, buffer, SRAM_MBOX_ROM_HASH_SIZE))
    {
        ret_val = 1;
    }

    return ret_val;
}

/******************************************************************************/
/** sb_sram_mbox_ec_fw_hash_read - Reads EC_FW hash mailbox info from SRAM Mailbox
 * @param buffer - Buffer to read data into
 * @return uint8_t 0(success), 1(fail).
 ******************************************************************************/
uint8_t sb_sram_mbox_ec_fw_hash_read(uint8_t *buffer)
{
    uint8_t ret_val = 0;

    if(sb_sram_mbox_read(FW_EC_HASH_MAILBOX_OFFSET, buffer, SRAM_MBOX_EC_FW_HASH_SIZE))
    {
        ret_val = 1;
    }

    return ret_val;
}

/******************************************************************************/
/** sb_sram_mbox_hw_config_read - Reads HW config. mailbox info from SRAM Mailbox
 * @param buffer - Buffer to read data into
 * @return uint8_t 0(success), 1(fail).
 ******************************************************************************/
uint8_t sb_sram_mbox_hw_config_read(uint8_t *buffer)
{
    uint8_t ret_val = 0;

    if(sb_sram_mbox_read(HW_CONFIG_MAILBOX_OFFSET, buffer, SRAM_MBOX_HW_CONFIG_SIZE))
    {
        ret_val = 1;
    }

    return ret_val;
}

/******************************************************************************/
/** spdm_get_measurements
 * This function can be used to get hash of measurement data. 
 * @param buff_ptr         Pointer to hold the measurement data
 * @param index            measurement id i.e. 1,2,3,4
 * @return                 None
 * @note
 ******************************************************************************/
void spdm_get_measurements(uint8_t *buffer_ptr, uint8_t index)
{
    switch (index)
    {
        case INDEX1:
            sb_sram_mbox_rom_hash_read(buffer_ptr);
            break;

        case INDEX2:
            sb_sram_mbox_ec_fw_hash_read(buffer_ptr);
            break;

        case INDEX3:
            // read hw config data
            sb_sram_mbox_hw_config_read(&hw_config_buff[0]);

            hw_config_buff[0] &= 0x1F; // Mask from 0 to 5th bit

            // calculate hash of hw config data
            spdm_crypto_ops_calc_hash(&hw_config_buff[0], HW_CONFIG_DATA_SZ, spdmContext);

            // store the hash
            memcpy(buffer_ptr, &spdmContext->sha_digest[0], SHA384_BYTES);
            break;

        case INDEX4:
            // Read OTP 576 to 863, except 857 & 858
            efuse_read_data(576, &otp_array[0], 281);
            efuse_read_data(859, &otp_array[281], 5);

            // calculate hash of FW config data
            spdm_crypto_ops_calc_hash(&otp_array[0], sizeof(otp_array), spdmContext);

            // store the hash
            memcpy(buffer_ptr, &spdmContext->sha_digest[0], SHA384_BYTES);

            break;
    }
}

/******************************************************************************/
/** Function of SPDM module to initialize hash engine with context states
* @param HASH_CTX_DATA *ctx_ptr - internal buffer to store relevant HASH engine structures
* @return SUCCESS/FAILURE
*******************************************************************************/
uint8_t spdm_crypto_ops_hash_ctx_init(HASH_CTX_DATA *ctx_ptr)
{   
    uint8_t rc;
    struct mchphash *cptr = &ctx_ptr->h_ctx;
    /*
	 * initialize hash structure pointer to HW instance
	 * allocate a descriptor to load hash configuration word
	 */
    mchp_hash_create_sha384(cptr);
    ctx_ptr->bufsz = SHA_ALGO_384_SZ;//hash block size
    
    /* copy Hash algorithm initial value to statebuf */
    mchp_hash_init_state(cptr, &ctx_ptr->h_state, (char *)ctx_ptr->statebuf);
    
    /* allocate a descriptor to load hash algorithm initial state */
    rc = mchp_hash_resume_state(cptr, &ctx_ptr->h_state);
    if (rc != MCHP_OK) {
       return rc;
    }
    return rc;
}

/******************************************************************************/
/** Function of SPDM module to save state of hash engine with intermediate hash
* @param HASH_CTX_DATA *ctx_ptr - internal buffer to store relevant HASH engine structures
* @param uint8_t *data - Pointer to data buffer
* @param UINT16  datasz - size of data 
* @return SUCCESS/FAILURE
*******************************************************************************/
uint8_t spdm_crypto_ops_save_state_hash(HASH_CTX_DATA *ctx_ptr, uint8_t *data, uint16_t datasz)
{
	struct mchphash *cptr = &ctx_ptr->h_ctx;
	int rc = MCHP_OK;
	
	if (ctx_ptr->resume) {
		ctx_ptr->resume = false;
		/* Get HW registers and reset descriptor count to 0 */
		mchp_hash_create_sha384(cptr);
	
		/* allocate a descriptor to load hash algorithm initial state */
		rc = mchp_hash_resume_state(cptr, &ctx_ptr->h_state);
		if (rc != MCHP_OK) {
			return rc;
		}
	}
	
	/* Allocate descriptor to fetch data */
	if (datasz) {
		rc = mchp_hash_feed(cptr, (char *)data, datasz);
	} else {
		rc = mchp_hash_feed(cptr, (char *)ctx_ptr->buf, ctx_ptr->buflen);
	}
	
	if (rc != MCHP_OK) {
		return rc;
	}
	
	/* starts engine */
	rc = mchp_hash_save_state(cptr);
	if (rc != MCHP_OK) {
		return rc;
	}
	
	rc = mchp_hash_wait(cptr);
	if (rc != MCHP_OK) {
		return rc;
	}
	
	/* NOTE: mchp_hash_wait on success clears struct mchphash DMA context.
	 * struct mchphash must be re-initialized via mchp_hash_resume or
	 * starting a new hash sequence via mchp_hash_create_xxx.
	 */
	ctx_ptr->resume = true;
	
	return rc;
}
/******************************************************************************/
/** Function of SPDM module to calculate intermediate hash and update buffer
* @param ctx_ptr - internal buffer to store relevant HASH engine structures
* @param buf - 8 bit Pointer to data buffer
* @param dlen - size of data
* @return SUCCESS(0)/FAILURE
*******************************************************************************/
uint8_t spdm_crypto_ops_hash_ctx_update_buf(uint8_t *buf, HASH_CTX_DATA *ctx_ptr, uint16_t dlen)
{
    uint16_t clen = 0;
    uint8_t rc = 0;
    char *bptr = (char *)ctx_ptr->buf;

	/* mhp->bufsz is the hash algorithm block size */
	while (dlen) {
		if ((ctx_ptr->buflen == 0) && (dlen >= ctx_ptr->bufsz)) {
			/* update dlen % blksz blocks and copy remaining to buffer */
			clen = dlen & ~(ctx_ptr->bufsz - 1U);
			rc = spdm_crypto_ops_save_state_hash(ctx_ptr, buf, clen);
			if (rc != MCHP_OK) {
				return rc;
			}
			dlen -= clen;
			buf += clen;
			if (dlen) {
				memcpy(bptr, buf, dlen);
				ctx_ptr->buflen = dlen;
				buf += dlen;
				dlen = 0;
			}
		} else { /* either buf not empty or dlen < blksz */
			/* copy data to buffer and update buffer length */
			clen = ctx_ptr->bufsz - ctx_ptr->buflen;
			if (dlen < clen) {
				clen = dlen;
			}
			memcpy((bptr + ctx_ptr->buflen), buf, clen);
			dlen -= clen;
			ctx_ptr->buflen += clen;
            if(dlen)
			{
                buf += clen;
            }
		}

		/* if mhp->buf is full run the engine */
		if (ctx_ptr->buflen == ctx_ptr->bufsz) {
			rc = spdm_crypto_ops_save_state_hash(ctx_ptr, NULL, 0);
			if (rc != MCHP_OK) {
				return rc;
			}
			memset(bptr, 0, ctx_ptr->bufsz);
			ctx_ptr->buflen = 0;
		}
	}

	return MCHP_OK;
}

/******************************************************************************/
/** Function of SPDM module to calculate the final hash with the previous states hashes
* @param HASH_CTX_DATA *ctx_ptr - internal buffer to store relevant HASH engine structures
* @param uint8_t *digest - output digest buffer
* @return SUCCESS/FAILURE
*******************************************************************************/
uint8_t spdm_crypto_ops_hash_ctx_final(HASH_CTX_DATA *ctx_ptr, uint8_t *digest)
{
	uint8_t rc;

	struct mchphash *cptr = &ctx_ptr->h_ctx;

	if (ctx_ptr->resume) {
		ctx_ptr->resume = false;
		/* Get HW registers and reset descriptor count to 0 */
		mchp_hash_create_sha384(cptr);
        
		/* allocate a descriptor to load hash algorithm initial state */
		rc = mchp_hash_resume_state(cptr, &ctx_ptr->h_state);
		if (rc != MCHP_OK) {
			return rc;
		}
	}

	if (ctx_ptr->buflen) {
		rc = mchp_hash_feed(cptr, (const char *)ctx_ptr->buf, ctx_ptr->buflen);
		if (rc != MCHP_OK) {
			return rc;
		}
	}

	rc = mchp_hash_digest(cptr, (char *)digest);
	if (rc != MCHP_OK) {
		return rc;
	}

	/* clears pointer to HW registers on completion */
	rc = mchp_hash_wait(cptr);

	return rc;
}

/******************************************************************************/
/** Function of SPDM module called for hashing the requests/responses for challenge and measurements commands
* @param SPDM_CONTEXT *spdmContext - SPDM module context
* @return SUCCESS/FAILURE
*******************************************************************************/
uint8_t spdm_crypto_ops_run_time_hashing(uint8_t *buff, uint32_t length, SPDM_CONTEXT *spdmContext)
{   
    uint8_t rc = 0x00;
    
    if(NULL == spdmContext)
    {
        return 0xff;
    }
    
    switch(spdmContext->get_requests_state)
    {
        case HASH_INIT_MODE:
            /*
	          * initialize hash structure pointer to HW instance
	          * allocate a descriptor to load hash configuration word
	        */
            rc = spdm_crypto_ops_hash_ctx_init(&ctx_ptr);
            if (rc != MCHP_OK) {
		     return rc;
	        }
            break;
        case RUN_TIME_HASH_MODE:
            //populate internal buffer (128 bytes) upto 128 (block size) 
            //feed to hash engine and calculate intermediate hash if internal buffer reaches max 128 bytes
            rc = spdm_crypto_ops_hash_ctx_update_buf(buff, &ctx_ptr, length);
	        if (rc != MCHP_OK) {
		        return rc;
	        }
            break;
        case END_OF_HASH:
            rc = spdm_crypto_ops_hash_ctx_final(&ctx_ptr, &spdmContext->sha_digest[0]);
	        if (rc != MCHP_OK) {
		        return rc;
	        }
            //cleaning up the hash engine context and internal buffers in case
            //switch back to init state as we got the final digest
            spdmContext->get_requests_state = HASH_INIT_MODE;
            memset((char *)&ctx_ptr.buf, 0, ctx_ptr.bufsz);
			ctx_ptr.buflen = 0;
            break;
        default:
            /* Invalid state */
            break;
            
    }
	return rc;
}

/******************************************************************************/
/** sb_sram_mbox_devAK_cert_read - Reads Device AK certificate info from SRAM Mailbox
 * @param buffer - Buffer to read data into
 * @return uint8_t 0(success), 1(fail).
 ******************************************************************************/
uint8_t sb_sram_mbox_devAK_cert_read(uint16_t start_offset, uint8_t *buffer, uint8_t cert_num, uint16_t num_of_bytes)
{
    uint8_t ret_val = 0;
    uint16_t offset = (cert_num == 0) ? SRAM_MBOX_DEVAK_CERT_OFFSET : SRAM_MBOX_DEVAK_CA_CERT_OFFSET;
    uint16_t byte_index = (uint16_t)((offset + start_offset) & UINT16_MAX);
    if(sb_sram_mbox_read(byte_index, buffer, num_of_bytes))
    {
        ret_val = 1;
    }

    return ret_val;
}


/** timer_delay - 
 * @param delay in timer count units
 * @notes Input Capture Compare free run counter is operating 
 * at (3 MHz +/- 2%) [2.94, 3.06] MHz, [0.340, 0.327] us.
 * Assume +2% to insure delays meet the minimum.
 * 1 us = 1/0.327 = 3.06 counts
 * num_us = num_us * 3 = num_us * (2+1) = (num_us << 1) + num_us
 */
void timer_delay(uint32_t counts_to_delay)
{ 
    volatile uint32_t diff, curr_cnt;
    uint32_t  start_cnt;
    
    if (counts_to_delay) {
        
        start_cnt = CCT_REGS->CCT_FREE_RUN;
        
        do {
            curr_cnt = CCT_REGS->CCT_FREE_RUN;
            if (curr_cnt >= start_cnt) {
                diff = curr_cnt - start_cnt;
            } else {
                diff = curr_cnt + (0xfffffffful - start_cnt);
            }
        } while (diff < counts_to_delay);
    }
}


/* Input Capture Compare free run counter is operating at (3 MHz +/- 2%)
 * [2.94, 3.06] MHz, [0.340, 0.327] us
 * Assume +2% to insure delays meet the minimum.
 * 1 ms = 3060 (0x0BF4) counts
 * count = ms * 3060 = ms * (3072 - 12) = ms * (0xC00 - 0x0C)
 * = ms * (0x800 + 0x400 - (0x08 + 0x04))
 * = (ms * 0x800) + (ms * 0x400) - (ms * 0x08) - (ms * 0x04)
 * = (ms << 11) + (ms << 10) - (ms << 3) - (ms << 2)
 */
void timer_delay_ms(uint32_t num_ms)
{
    uint32_t counts = (num_ms << 11ul) + (num_ms << 10ul) - (num_ms << 3ul) - (num_ms << 2ul);
    
    timer_delay(counts);

    return;
}

void timer_delay_us(uint32_t num_us)
{
    uint32_t counts = num_us + (num_us << 1ul) + (num_us >> 4ul);
    
    timer_delay(counts);

    return;
}

/******************************************************************************/
/** platform_serial_flash_check_done_status();
* Check if the previous execution of serial flash operation is done
* @param qmspi_status - pointer to store qmspi status values
* @return command success (0) or error (1)
*******************************************************************************/
uint8_t platform_serial_flash_check_done_status(uint32_t *qmspi_status)
{
    uint16_t count = 0;
    uint8_t ret_val = 1, done_status = 0;
    
    count = QMSPI_STATUS_RETRY_COUNT * 10;  // multiplying by 10 since waiting 100us in below loop
    do 
    {
        done_status = api_qmspi_is_done_status(qmspi_status, 2);

        //timer_delay_ms(1);
        timer_delay_us(100);
        count--;
        
    }while ((!done_status) && (count > 0));
    
    ret_val = 1;
    if (done_status)
    {
        ret_val = 0;
    }
    else 
    {
        /* pltform serial flash check done sts */
    }
    
    return ret_val;
    
}

/******************************************************************************/
/** spdm_read_certificate
 * This function can be used to read the certificate data and store it in buffer.
 * @param address          Address of certificate
 * @param buff_ptr         Pointer to hold certificate data
 * @param length           Length of the certificate to be read
 * @param certificate_no   Certificate number
 * @return                 None
 * @note
 * ############################################################################
 * *****************************************************************************/
uint8_t spdm_read_certificate(uint32_t address, 
                                  uint8_t *buff_ptr,
                                  uint32_t length,
                                  uint8_t certificate_num)
{
    uint32_t qmspi_status = 0;
    if (certificate_num == 0 || certificate_num == 1)
    {
        sb_sram_mbox_devAK_cert_read(address, buff_ptr, certificate_num, length);
    } else {
        api_qmspi_port_ctrl(INT_SPI_PORT_ID, SPI_IO_FD_DUAL, DEV_ENABLE);
        api_qmspi_port_drv_slew(INT_SPI_PORT_ID, SPI_IO_FD_DUAL, (GPIO_DRV_4MA) | (GPIO_SLEW_FAST));
        api_qmspi_init(QMSPI_SPI_MODE_0, QMSPI_FREQ_48M, NONE, INT_SPI_PORT_NO);
        gpio_init_pp_output_high(PIN_QSPI0_CS0);
        gpio_init_pp_output_high(PIN_QSPI0_CS1);
        gpio_init_pp_output_high(PIN_QSPI1_CS0);
        gpio_init_pp_output_high(PIN_QSPI1_CS1);
        gpio_init_pp_output_high(PIN_INT_QSPI0_CS);

        api_qmspi_flash_read_24_32_ldma(0, address, length, (uint32_t)buff_ptr, 0);
        gpio_output_set(PIN_INT_QSPI0_CS, GPIO_ALT_OUT_DIS, 0 );
        api_qmspi_start(0, 0);
        platform_serial_flash_check_done_status(&qmspi_status);
        gpio_output_set(PIN_INT_QSPI0_CS, GPIO_ALT_OUT_DIS, 1 );
    }
}

/******************************************************************************/
/** Wrapper function of SPDM module to call the ndrng block operations function in sb dgst
* @param buff - 8 bit pointer to rng output buffer
* @param bytes - length of buff
* @return SUCCESS(0)/FAILURE
*******************************************************************************/
uint8_t spdm_crypto_ops_gen_random_no(uint8_t *buff, uint8_t bytes)
{
    int8_t ret_status = 0x00;

    mchp_ndrng_enable(0);
    mchp_ndrng_soft_reset();

    ret_status = mchp_ndrng_init(true, NDRNG_FIFO_WAKEUP_LVL, NDRNG_OFF_TIMER_VAL,
                                         NDRNG_NB_128BIT_BLOCKS, NDRNG_CLKDIV,
                                         NDRNG_INIT_WAIT_VAL);

    mchp_ndrng_enable(1);

    ret_status = mchp_ndrng_read_bytes_nh(buff, bytes);

    return ret_status;
}

/******************************************************************************/
/** Wrapper function of SPDM module to initialize the pke engine local to spdm module
* @param none
* @return SUCCESS/FAILURE
*******************************************************************************/
uint8_t spdm_crypto_ops_pke_engine_init(void)
{
	uint8_t ret_value = 0;

	/* Initialize PK and library */
	init_pke = mchp_pk_init(mchp_pk_cnx_mem_spdm_module, PK_CNX_MEM_SIZE);

	if (init_pke == NULL) {
		ret_value = 0xff;
	}

	return ret_value;
}

/******************************************************************************/
/** Wrapper function of SPDM module to generate ECDSA signature
* @param SPDM_CONTEXT *spdmContext -SPDM module context
* @return SUCCESS/FAILURE
*******************************************************************************/
uint32_t spdm_crypto_ops_gen_signature()
{  
    uint32_t return_status = 0x00;
    // uint8_t arr[2 * CURVE_384_SZ] = {0};
    sig_length = 2 * CURVE_384_SZ;

    if(NULL == spdmContext)
    {
        return 0xff;
    }

    //read the Device AK PVT key from sram mailbox; @127100
    sb_sram_mbox_read(DEV_AK_CERTIFICATE_OFFSET, &pvt_key[0], PVT_KEY_CODE_LENGTH);
    
    return_status = bk_ecdsa_sign((bk_ecc_private_key_code_t *)&pvt_key[0], false, 
		    			&hash_of_req_buffer[0], SHA384_BYTES, true, 
					    &(ecdsa_signature.ecdsa_signature[0]),
                        &sig_length);
    
    return return_status;
}

