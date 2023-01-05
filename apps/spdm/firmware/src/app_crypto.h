/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app_crypto.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_SMB_DRV_Initialize" and "APP_SMB_DRV_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_SMB_DRV_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/
#ifndef _APP_CRYPTO_H
#define _APP_CRYPTO_H

#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>

#define SHA384_BYTES               48U

//Retry count for polling 'api_qmspi_is_done_status' in milliseconds
#define QMSPI_STATUS_RETRY_COUNT   200
#define DEV_AK_CERTIFICATE_OFFSET 0xA30

/* Internal Flash PIN */
#define PIN_INT_QSPI0_CS        GPIO_PIN_GPIO230
#define PIN_QSPI0_CS0           GPIO_PIN_GPIO055
#define PIN_QSPI0_CS1           GPIO_PIN_GPIO002
#define PIN_QSPI1_CS0           GPIO_PIN_GPIO124
#define PIN_QSPI1_CS1           GPIO_PIN_GPIO120

/* QMSPI configs */
#define INT_SPI_PORT_ID 2U
#define SPI_IO_FD_DUAL  0x0EU
#define DEV_ENABLE   1U
#define GPIO_DRV_4MA (1ul << 4)
#define GPIO_SLEW_FAST 1U
#define QMSPI_SPI_MODE_0 0x00U
#define QMSPI_FREQ_48M 2U
#define NONE     0U
#define INT_SPI_PORT_NO 0x00U

#ifndef MCHP_EXTRA_IN_DESCS
#define MCHP_EXTRA_IN_DESCS 0U
#endif

#define SHA_MODE_256    (3U)
#define SHA_MODE_384    (4U)
#define SHA_MODE_512    (5U)


#define MCHP_OK 0U
#define HASH_HW_BUSY -1

#define ROM_HASH_MAILBOX_OFFSET   (0x804U)
#define SRAM_MBOX_ROM_HASH_SIZE   (48U)
#define SRAM_MBOX_SIZE            0xC00U
#define SRAM_MBOX_ADDRESS_START   0x126800U
#define FW_EC_HASH_MAILBOX_OFFSET (0x8c8U)
#define SRAM_MBOX_EC_FW_HASH_SIZE (48U)
#define SRAM_MBOX_HW_CONFIG_SIZE  (0x04U)

#define SRAM_MBOX_DEVAK_CERT_OFFSET                   (0U)
#define SRAM_MBOX_DEVAK_CA_CERT_OFFSET                (0x400U)

/*
 * 0x01 - Out of range
 * 0x00 - Success in read
 */
#define SRAM_MBOX_READ_SUCCESS                (0U)
#define SRAM_MBOX_READ_FAIL                   (1U)

#define HW_CONFIG_MAILBOX_OFFSET                      (0x8c7U)
#define SRAM_MBOX_HW_CONFIG_SIZE                      (0x04U)

#define NDRNG_FIFO_WAKEUP_LVL	(8U)	/* number of 128-bit blocks */

#define NDRNG_OFF_TIMER_VAL	(0U)

#define NDRNG_CLKDIV		(0U)

#define NDRNG_INIT_WAIT_VAL	(512U)

#define NDRNG_NB_128BIT_BLOCKS	(4U)	/* conditioning number of 128-bit blocks */

#define RLOG_LOAD_FROM_TAG0             (0x08000000U)
#define RLOG_LOAD_FROM_TAG1             (0x10000000U)
#define ROM_EVENT_RLOG_OFFSET                         (0x1F4U)
#define RLOG_ROM_EVENT_SIZE                           (4U)
#define SRAM_ROM_LOG_START_LOCATION_A1                   (0x00126200U)

#define TAG0_ADDRESS 0x20000U
#define PK_CNX_MEM_SIZE		64U

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
typedef uint8_t (*api_qmspi_port_ctrl_td)(uint8_t port_id, uint8_t width_mask, uint8_t enable);
typedef uint8_t (*api_qmspi_port_drv_slew_td)(uint8_t port_id, uint8_t width_mask, uint8_t drv_slew);
typedef void (*api_qmspi_init_td)(uint8_t spi_mode, uint8_t freq_div, uint8_t ifc, uint8_t port);
typedef uint32_t(*api_qmspi_flash_read_24_32_ldma_td)(uint8_t cmd_id, uint32_t spi_addr, uint32_t nbytes,
        uint32_t maddr, uint8_t port);
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
 * 	Pointer to struct sx_pk_cnx (should be cnxmem)
 * Returns on error:
 *	NULL
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
typedef uint8_t (*api_qmspi_is_done_status_td)(uint32_t *pstatus, uint8_t port);
typedef int (*bk_ecdsa_sign_td)(
    const bk_ecc_private_key_code_t * const private_key_code,
    const bool                              deterministic_signature,
    const uint8_t                   * const message,
    const uint32_t                          message_length,
    const bool                              message_is_hash,
    uint8_t                   * const signature,
    uint16_t                  * const signature_length
);

#define	mchp_ndrng_read_bytes_nh				((mchp_ndrng_read_bytes_nh_td                       )			0x0001239)
#define	mchp_ndrng_soft_reset				((mchp_ndrng_soft_reset_td                          )			0x0001215)
#define	mchp_ndrng_init				((mchp_ndrng_init_td                                )			0x000121d)
#define	mchp_ndrng_enable				((mchp_ndrng_enable_td                              )			0x0001225)
#define api_qmspi_flash_read_24_32_ldma            ((api_qmspi_flash_read_24_32_ldma_td)0x0000f241)
#define api_qmspi_init                        ((api_qmspi_init_td) 0x0000f225)
#define api_qmspi_port_drv_slew                   ((api_qmspi_port_drv_slew_td) 0x0000f219)
#define api_qmspi_port_ctrl                   ((api_qmspi_port_ctrl_td)    0x0000f215)
#define	mchp_hash_create_sha256				((mchp_hash_create_sha256_td                        )			0x0001181)
#define	mchp_hash_create_sha384				((mchp_hash_create_sha384_td                        )			0x0001185)
#define	mchp_hash_feed				((mchp_hash_feed_td                                 )			0x000119d)
#define	mchp_hash_digest				((mchp_hash_digest_td                               )			0x00011a1)
#define	mchp_hash_wait				((mchp_hash_wait_td                                 )			0x00011a5)
#define  api_efuse_byte_read              (( api_efuse_byte_read_td)0x0000f009)
#define	mchp_hash_init_state				((mchp_hash_init_state_td                           )			0x0001191)
#define	mchp_hash_resume_state				((mchp_hash_resume_state_td                         )			0x0001195)
#define	mchp_hash_save_state				((mchp_hash_save_state_td                           )			0x0001199)
#define	mchp_pk_get_curve_nistp384				((mchp_pk_get_curve_nistp384_td                     )			0x0001079)
#define	mchp_pk_init				((mchp_pk_init_td                                   )			0x00010ad)
#define	mchp_async_ecdsa_generate_go				((mchp_async_ecdsa_generate_go_td                   )			0x0001005)
#define	mchp_pk_wait				((mchp_pk_wait_td                                   )			0x0001119)
#define	mchp_async_ecdsa_generate_end				((mchp_async_ecdsa_generate_end_td                  )			0x0001009)
#define api_qmspi_start                          ((api_qmspi_start_td) 0x0000f231)
#define api_qmspi_is_done_status                ((api_qmspi_is_done_status_td)0x0000f221)
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

#endif /* _APP_CRYPTO_H */