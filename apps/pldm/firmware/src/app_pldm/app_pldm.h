/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_smb_drv.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_SMB_DRV_Initialize" and "APP_SMB_DRV_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_SMB_DRV_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

#ifndef _APP_PLDM_H
#define _APP_PLDM_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>
#include "common.h"



#ifndef MCHP_EXTRA_IN_DESCS
#define MCHP_EXTRA_IN_DESCS 0U
#endif

#define HASH_STATE_SZ (128U + 128U)
#define HASH_BUF_SZ 128U
#define SHA_ALGO_384_SZ 128U


typedef enum {
    SHA_SUCCESS,
    SHA_ERROR,
    SHA_INIT_ERROR,
    SHA_UPDATE_ERROR,
    SHA_UPDATE_TIMEOUT,
    SHA_FINALIZE_ERROR,
    SHA_DONE_TIMEOUT,
    SHA_BUSY_TIMEOUT,
    SHA_PARAM_ERROR
} SHA_RET_CODE;


/** A cryptomaster DMA descriptor */
struct mchpdesc {
    char *addr;
    struct mchpdesc *next;
    uint32_t sz;
    uint32_t dmatag;
};

/** Input and output descriptors and related state for cmdma */
struct mchp_dmaslot
{
    uint32_t cfg;
    uint8_t extramem[26];
    struct mchpdesc indescs[6 + MCHP_EXTRA_IN_DESCS];
    struct mchpdesc outdescs[5];
};

/** DMA controller
 *
 * For internal use only. Don't access directly.
 */
struct mchp_dmactl {
    struct mchp_regs *regs;
    struct mchpdesc *mapped_in;
    struct mchpdesc *mapped_out;
    struct mchp_dmaslot dmamem;
};


struct mchphashstate {
    char *m;
    size_t sz;
};

struct mchphash {
    struct mchpdesc *d;
    struct mchphashstate *h;
    const struct hashalgo *algo;
    uint32_t cntindescs;
    struct mchp_dmactl dma;
    uint32_t feedsz;
};

typedef struct HASH_CTX_DATA
{
    struct mchphash h_ctx;
    struct mchphashstate h_state;
    uint32_t statebuf[HASH_STATE_SZ / 4];
    bool resume;
    uint32_t bufsz;
    uint32_t buflen;
    uint32_t buf[HASH_BUF_SZ / 4];
} HASH_CTX_DATA;

struct mchp_buf {
     size_t sz;
     char *bytes;
};

typedef struct mchp_buf mchp_op;

struct mchp_regs
{
   char *base;
};

/** ec firmware image id **/
typedef enum
{
    ECFW_IMG_TAG0,
    ECFW_IMG_TAG1,
    ECFW_IMG_APCFG0,
    ECFW_IMG_APCFG1
} EC_FW_IMAGE_ID;

typedef struct mchp_pk_accel mchp_pk_accel;

struct mchp_pk_dreq
{
   mchp_pk_accel *req;
   int status;
};

/** Description of the (hardware) accelerator capabilities and features.*/
struct mchp_pk_capabilities {
   /** Maximum pending requests at any time. */
   int maxpending;
   /** Maximum operand size for prime field operands. 0 when not supported. */
   int max_gfp_opsz;
   /** Maximum operand size for elliptic curve operands. 0 when not supported. */
   int max_ecc_opsz;
   /** Maximum operand size for binary field operands. 0 when not supported. */
   int max_gfb_opsz;
};
#define NUM_PK_INST 1 /* MCHP */

struct mchp_pk_accel
{
   struct mchp_regs regs;
   char *cryptoram;
   int op_size;
   const struct mchp_pk_cmd_def *cmd;
   struct mchp_pk_cnx *cnx;
   const char *outputops[4];
   void *userctxt;
};

struct mchp_pk_cnx {
   struct mchp_pk_accel instances[NUM_PK_INST];
   struct mchp_pk_capabilities caps;
};

/** Elliptic curve configuration and parameters.
 *
 * To be used only via the functions in ec_curves.h The internal members of the
 * structure should not be accessed directly.
 */
struct mchp_pk_ecurve {
    uint32_t curveflags;
    int sz;
    const char *params;
    int offset;
    struct mchp_pk_cnx *cnx;
};

/* Start of ROM APIs definitions */

/** Prepare a new SHA256 hash
 *
 * The user allocated mchphash object 'c' will contain the internal state
 * needed to run the hash. After creation, it shall be passed to
 * mchp_hash_feed() or mchp_hash_digest().
 */
typedef int (*mchp_hash_create_sha256_td)(struct mchphash *c);

/** Prepare a new SHA384 hash
 *
 * The user allocated mchphash object 'c' will contain the internal state
 * needed to run the hash. After creation, it shall be passed to
 * mchp_hash_feed() or mchp_hash_digest().
 */
typedef int (*mchp_hash_create_sha384_td)(struct mchphash *c);

/** Assign data to be hashed.
 *
 * If this function is called with size of message 0, no data will be assigned
 * to be hashed.
 * This function can be called multiple times to assemble pieces of the message
 * scattered in the memory, if the called in excess, error MCHP_ERR_TOO_BIG
 * will be returned.
 * Inputs:
 *  - 'msg': data to be hashed.
 *  - 'sz': size of 'msg' in bytes.
 */
typedef int (*mchp_hash_feed_td)(struct mchphash *c, const char *msg, size_t sz);

/** Starts the hashing operation.
 *
 * Outputs:
 *  - 'digest': computed digest.
 */
typedef int (*mchp_hash_digest_td)(struct mchphash *c, char *digest);

/** Return the hashing status.
 *
 * If the operation is still ongoing, return MCHP_ERR_HW_PROCESSING.
 * In that case, the user can retry later. After any other error or
 * MCHP_OK, the hashing is completed and the created hash operation 'c'
 * has been freed. After that, 'c' cannot be used again without
 * starting a new operation with a MCHP_hash_create_sha*() call.
 */
typedef int (*mchp_hash_status_td)(struct mchphash *c);

/** Wait until the hashing finishes.
 *
 * When done, return the operations status. See sx_hash_status().
 */
typedef int (*mchp_hash_wait_td)(struct mchphash *c);

/** Initialize the hash context saving
 *
 * Inputs:
 *  - 'dmamem': address of the location to save the state
 */
typedef void (*mchp_hash_init_state_td)(struct mchphash *c, struct mchphashstate *h, char *dmamem);
typedef int (*mchp_hash_resume_state_td)(struct mchphash *c, struct mchphashstate *h);
typedef int (*mchp_hash_save_state_td)(struct mchphash *c);
typedef uint8_t (*api_efuse_byte_read_td)(uint16_t byte_idx, uint8_t* out_data);
typedef void (*mchp_ndrng_enable_td)(bool enable);
typedef void (*mchp_ndrng_soft_reset_td)(void);
typedef int (*mchp_ndrng_init_td)(bool enable_conditioning,
         uint32_t fifo_wake_thresh,
         uint32_t ring_pwr_down_delay,
         uint32_t nb_cond,
         uint32_t rng_clk_div,
         uint32_t rng_init_wait);
/**
 *  \name mchp_pk_get_curve_nistp384_td
 *
 *  \param [in] mchp_pk_cnx structure memory initialised - struct mchp_pk_cnx *cnx
 *
 *  \return mchp_pk_ecurve - for nistp384
 *
 *  \details This API is used to return curve structure - nistp384.
 */
typedef struct mchp_pk_ecurve (*mchp_pk_get_curve_nistp384_td)(struct mchp_pk_cnx *cnx);
typedef int (*mchp_ndrng_read_bytes_nh_td)(uint8_t *dest, int nbytes);
/*
 * Initialize PK block
 * Clear Crypto PCR sleep enable
 * Clears PK shared crypto memory(SCM)
 * Initializes struct sx_pk_cnx with memory provided by caller.
 * Memory must be aligned >= 4 bytes and have size >= PK_CNX_MEM_SIZE
 * Returns on success:
 * Pointer to struct sx_pk_cnx (should be cnxmem)
 * Returns on error:
 * NULL
 */
typedef struct mchp_pk_cnx *(*mchp_pk_init_td)(uint32_t *cnxmem, size_t cnxmsz);
typedef struct mchp_pk_dreq (*mchp_async_ecdsa_generate_go_td)(
      const struct mchp_pk_ecurve *curve,
      const mchp_op *d,
      const mchp_op *k,
      const mchp_op *h,
      mchp_op *r,
      mchp_op *s);

typedef uint8_t bk_ecc_private_key_code_t[116];
typedef uint8_t bk_ecc_public_key_code_t[180];
/** Wait until the current operation finishes.
 *
 * After the operation finishes, return the operation status code.
 */

typedef int (*mchp_pk_wait_td)(struct mchp_pk_accel *req);
typedef void (*mchp_async_ecdsa_generate_end_td)(struct mchp_pk_accel *req, mchp_op *r, mchp_op *s);
typedef void (*api_qmspi_start_td)(uint16_t ien_mask, uint8_t port );
typedef int (*bk_ecdsa_sign_td)(
    const bk_ecc_private_key_code_t * const private_key_code,
    const bool                              deterministic_signature,
    const uint8_t                   * const message,
    const uint32_t                          message_length,
    const bool                              message_is_hash,
    uint8_t                   * const signature,
    uint16_t                  * const signature_length
);

#define   mchp_ndrng_read_bytes_nh            ((mchp_ndrng_read_bytes_nh_td                       )         0x0001239)
#define   mchp_ndrng_soft_reset            ((mchp_ndrng_soft_reset_td                          )         0x0001215)
#define   mchp_ndrng_init            ((mchp_ndrng_init_td                                )         0x000121d)
#define   mchp_ndrng_enable            ((mchp_ndrng_enable_td                              )         0x0001225)


#define   mchp_hash_create_sha256            ((mchp_hash_create_sha256_td                        )         0x0001181)
#define   mchp_hash_create_sha384            ((mchp_hash_create_sha384_td                        )         0x0001185)
#define   mchp_hash_feed            ((mchp_hash_feed_td                                 )         0x000119d)
#define   mchp_hash_digest            ((mchp_hash_digest_td                               )         0x00011a1)
#define   mchp_hash_wait            ((mchp_hash_wait_td                                 )         0x00011a5)
#define  api_efuse_byte_read              (( api_efuse_byte_read_td)0x0000f009)
#define   mchp_hash_init_state            ((mchp_hash_init_state_td                           )         0x0001191)
#define   mchp_hash_resume_state            ((mchp_hash_resume_state_td                         )         0x0001195)
#define   mchp_hash_save_state            ((mchp_hash_save_state_td                           )         0x0001199)
#define   mchp_pk_get_curve_nistp384            ((mchp_pk_get_curve_nistp384_td                     )         0x0001079)
#define   mchp_pk_init            ((mchp_pk_init_td                                   )         0x00010ad)
#define   mchp_async_ecdsa_generate_go            ((mchp_async_ecdsa_generate_go_td                   )         0x0001005)
#define   mchp_pk_wait            ((mchp_pk_wait_td                                   )         0x0001119)
#define   mchp_async_ecdsa_generate_end            ((mchp_async_ecdsa_generate_end_td                  )         0x0001009)
#define bk_ecdsa_sign                       ((bk_ecdsa_sign_td                                     )0x0001f435)



/******************************************************************************/
/** do_sha();
* Perform the sha operation
* @param hash_algo - hash algorithm code
* @param input_buf - input data
* @param input_len - length of data
* @param output - buffer to copy the hash output
* @return 0 on success or error codes (SHA_RET_CODE)
*******************************************************************************/
SHA_RET_CODE do_sha(uint8_t hash_algo, uint8_t *input_buf, uint32_t input_len, uint8_t *output);

#endif /* _APP_SMB_DRV_H */

/*******************************************************************************
 End of File
 */

