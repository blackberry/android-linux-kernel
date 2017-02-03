/*****************************************************************************
* Copyright (c) 2014 BlackBerry Limited, All rights reserved.
*
* This software is provided as-is. BlackBerry hereby disclaims all conditions,
* endorsements, guarantees, representations, or warranties of any kind, express
* or implied, including without limitation any conditions, endorsements,
* guarantees, representations or warranties of durability, fitness for a
* particular purpose or use, merchantability, merchantable quality,
* noninfringement, dealing, or usage of trade.
*
* Filename:    tzbb_protocol_public.h
*
* Description: Internal communication protocol between the BlackBerry TZ app
*              and HLOS.  This file needs to be kept in sync in both locations
*
****************************************************************************/
#ifndef __TZBB_PROTOCOL_PUBLIC_H__
#define __TZBB_PROTOCOL_PUBLIC_H__

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------
#define BASE64_LEN(x)   ((((x)+2)/3)*4)

// Previously defined in stp_handler as BSIS_SEED_DATA_VERSION
#define BSIS_PRIVATE_KEY_STORE_VERSION  0x100

#define BIDE_PRIVATE_KEY_STORE_VERSION  0x200


// sizes defined in bytes
#define AES_BLOCK_SIZE                  16
#define AES128_IV_SIZE                  16
#define SHA256_SIZE                     32
#define SHA512_SIZE                     64
#define HMAC_SHA256_SIZE                SHA256_SIZE

#define ECC256_PRIV_KEY_SIZE            32      // ECC256 private key size
#define ECC256_SIG_SIZE                 64      // ECC256 signature size
#define ECC256_PUBL_KEY_SIZE            66      // ECC256 public key size
#define ECC256_ECDH_SHARED_SECRET_SIZE  32      // ECC256 D-H size

#define ECC521_PRIV_KEY_SIZE            66      // ECC521 private key size
#define ECC521_SIG_SIZE                 136     // ECC521 signature size
#define ECC521_PUBL_KEY_SIZE            136     // ECC521 public key size
#define ECC521_ECDH_SHARED_SECRET_SIZE  66      // ECC521 D-H size

// Gap left between ECC private key (66 bytes) and sha512 digest in the private
// key store (make it multiple of AES block size)
#define GAP_14_ALIGN                    14

// version, iv, encrypted private and hash.
#define PRIVATE_KEY_STORE_SIZE          ( sizeof( bsis_priv_key_store_t ) )

// version, iv, encrypted private and hash.
#define BIDE_PRIVATE_KEY_STORE_SIZE     ( sizeof( bide_priv_key_store_t ) )

// BIDE nonce size
#define BIDE_NONCE_SIZE                 64
// BIDE maximum TZ report size in characters
#define BIDE_MAX_REPORT_SIZE            1024
// BIDE maximum CSR size in bytes
#define MAX_PKCS10_SIZE                 1024
// Size of PIN (64-bit value as string - eg "0000000012345678")
#define BBPIN_SIZE                      16

//BIDE TZ_CMD_BIDE_ADD_SECTION flags
#define TZ_SECTION_FLAG_REMOVABLE                       (1<<0)      /* Section hashed in trustzone can be unloaded by TZ_CMD_BIDE_REMOVE_SECTION */

// BIDE services
#define TZ_CMD_BIDE_ADD_SECTION                         2000
#define TZ_CMD_BIDE_GENERATE_KEYPAIR                    2001
#define TZ_CMD_BIDE_SIGN_DATA                           2002
#define TZ_CMD_BIDE_GENERATE_NONCE                      2003
#define TZ_CMD_BIDE_REMOVE_SECTION                      2004

//------------------------------------------------------------------------------
// Helper types
//------------------------------------------------------------------------------
// Private key store for HLOS storage including encrypted private key and key-related data
typedef struct {
    uint32_t version;
    uint8_t iv[AES128_IV_SIZE];
    uint8_t key_and_hash[ECC521_PRIV_KEY_SIZE + GAP_14_ALIGN + SHA512_SIZE];
} bsis_priv_key_store_t;

// BIDE private key store for HLOS storage including encrypted private key and key-related data
typedef struct {
    uint32_t version;
    uint8_t iv[AES128_IV_SIZE];
    uint8_t key_and_hash[ECC256_PRIV_KEY_SIZE + SHA512_SIZE];
} bide_priv_key_store_t;

/* This struct is to be hashed by HLOS and compared against the internal TA result
 * before allowing bide_validate_and_sign to proceed. bide_generate_nonce will provide
 * the session value and the counter should increment from zero with each successful
 * call to bide_validate_and_sign. */
typedef struct {
    uint8_t session[BIDE_NONCE_SIZE];
    uint64_t counter;
} nonce_data_t;

//------------------------------------------------------------------------------
// BIDE services
//------------------------------------------------------------------------------
/**
 * [IN] Generate BIDE session nonce
 */
typedef struct {
    uint32_t cmd;
} bide_generate_nonce_in_t;

/**
 * [OUT] Generate BIDE session nonce
 * nonce - generated 8-byte session nonce
 */
typedef struct {
    uint32_t status;
    uint8_t nonce[BIDE_NONCE_SIZE];
} bide_generate_nonce_out_t;

/**
 * [IN] Add new memory section to include in validation
 * nonce_hash - sha512 hash of session nonce and counter (nonce_data_t)
 * addr - starting physical address of the section to include
 * size - size of section in bytes
 * flags - special details of the memory section (not presently used)
 * mem_hash - pre-calculated hash of memory for comparison
 */
typedef struct {
    uint32_t cmd;
    uint8_t nonce_hash[SHA512_SIZE];
    uint32_t addr;
    uint32_t size;
    uint32_t flags;
    uint8_t mem_hash[SHA256_SIZE];
} bide_add_section_in_t;

/**
 * [OUT] Add new memory section to include in validation
 * num_sections - number of added sections including the one added in this call (if successful)
 */
typedef struct {
    uint32_t status;
    uint32_t num_sections;
} bide_add_section_out_t;

/**
 * [IN] Generate a new signing key and certificate signing request for BIDE
 * nonce_hash - sha512 hash of session nonce and counter (nonce_data_t)
 * pin - device PIN to be included in the CSR
 * bsis_priv_key_store - input BSIS private key store for signing
 */
typedef struct {
    uint32_t cmd;
    uint8_t nonce_hash[SHA512_SIZE];
    uint8_t pin[BBPIN_SIZE];
    bsis_priv_key_store_t bsis_priv_key_store;
} bide_generate_keypair_in_t;
/**
 * [OUT] Generate a new signing key and certificate signing request for BIDE
 * pkcs10 - buffer containing the signed PKCS10 CSR (ASN.1 DER encoding)
 * pkcs10_size - actual length of pkcs10 in bytes
 * priv_key_store - encrypted private key store to be stored in HLOS
 */
typedef struct {
    uint32_t status;
    uint8_t pkcs10[MAX_PKCS10_SIZE];
    uint32_t pkcs10_size;
    bide_priv_key_store_t priv_key_store;
} bide_generate_keypair_out_t;

/**
 * [IN] Perform BIDE validation and generate a signed BIDE TZ report
 * nonce_hash - sha512 hash of session nonce and counter (nonce_data_t)
 * priv_key_store - encrypted private key store to use for signing
 * report_hash - base64-encoded hash of the HLOS report or other data to be included in the BIDE TZ report
 */
typedef struct {
    uint32_t cmd;
    uint8_t nonce_hash[SHA512_SIZE];
    bide_priv_key_store_t priv_key_store;
    char report_hash[BASE64_LEN(SHA256_SIZE) + 1];
} bide_sign_data_in_t;

/**
 * [OUT] Perform BIDE validation and sign data
 * report - generated TZ report with bide_sign_data_in_t.report_hash and integrity status
 * report_hash - sha256 hash of the generated TZ report used in signing
 * signature - ECC256 signature of bide_sign_data_out_t.report_hash
 */
typedef struct {
    uint32_t status;
    char report[BIDE_MAX_REPORT_SIZE];
    uint8_t report_hash[SHA256_SIZE];
    uint8_t signature[ECC256_SIG_SIZE];
} bide_sign_data_out_t;

/**
 * [IN] Remove memory section in validation
 * nonce_hash - sha512 hash of session nonce and counter (nonce_data_t)
 * addr - starting physical address of the section to include
 * size - size of section in bytes
 */
typedef struct {
    uint32_t cmd;
    uint8_t nonce_hash[SHA512_SIZE];
    uint32_t addr;
    uint32_t size;
} bide_remove_section_in_t;

/**
 * [OUT] Remove memory section in validation
 * num_sections - number of added sections excluding the one removed in this call (if successful)
 */
typedef struct {
    uint32_t status;
    uint32_t num_sections;
} bide_remove_section_out_t;

#endif
