#ifndef __HC32_LL_HASH_EX_H__
#define __HC32_LL_HASH_EX_H__

#include "hc32_ll_hash.h"

#ifdef __cplusplus
extern "C" {
#endif

// SHA256 context structure for incremental calculation
typedef struct {
    uint8_t  buffer[64];          // Accumulation data buffer
    uint32_t buffer_len;          // Valid bytes in buffer
    uint64_t total_len;           // Total bytes processed (excluding buffer)
    uint8_t  started;             // Whether the first group has been written
    uint8_t  finished;            // Whether calculation has been completed
} stc_hash_ctx_t;

// HMAC-SHA256 context structure for incremental calculation
typedef struct {
    stc_hash_ctx_t inner_ctx;     // Inner hash context: H(ipad_key || message)
    stc_hash_ctx_t outer_ctx;     // Outer hash context: H(opad_key || inner_hash)
    uint8_t k0[64];               // Processed key (fixed 64 bytes)
    uint8_t state;                // 0=uninitialized, 1=initialized, 2=completed
} stc_hmac_ctx_t;

/* ==================== SHA256 Incremental API ==================== */

/**
 * @brief  Initialize SHA256 context, reset hardware, prepare for incremental calculation.
 * @param  [in] pCtx  Pointer to the SHA256 context structure.
 * @retval LL_OK on success, other values indicate error codes.
 */
int32_t HASH_Init_Ex(stc_hash_ctx_t *pCtx);

/**
 * @brief  Append data to SHA256 calculation (can be called multiple times).
 * @param  [in] pCtx    Pointer to the SHA256 context structure.
 * @param  [in] pData   Pointer to the input data buffer.
 * @param  [in] u32Len  Length of input data in bytes.
 * @retval LL_OK on success, other values indicate error codes.
 */
int32_t HASH_Update_Ex(stc_hash_ctx_t *pCtx, const uint8_t *pData, uint32_t u32Len);

/**
 * @brief  Finalize SHA256 calculation and output the 32-byte message digest.
 * @param  [in]  pCtx          Pointer to the SHA256 context structure.
 * @param  [out] pu8MsgDigest  Pointer to the output buffer for the digest (at least 32 bytes).
 * @retval LL_OK on success, other values indicate error codes.
 */
int32_t HASH_Final_Ex(stc_hash_ctx_t *pCtx, uint8_t *pu8MsgDigest);

/* ==================== HMAC-SHA256 Incremental API ==================== */

/**
 * @brief  Initialize HMAC context with the given secret key.
 * @param  [out] pCtx       Pointer to the HMAC context structure.
 * @param  [in]  pKey       Pointer to the secret key buffer.
 * @param  [in]  u32KeyLen  Length of the secret key in bytes.
 * @retval LL_OK on success, other values indicate error codes.
 */
int32_t HMAC_Init_Ex(stc_hmac_ctx_t *pCtx, const uint8_t *pKey, uint32_t u32KeyLen);

/**
 * @brief  Append message data to HMAC calculation (can be called multiple times).
 * @param  [in] pCtx    Pointer to the HMAC context structure.
 * @param  [in] pData   Pointer to the message data buffer.
 * @param  [in] u32Len  Length of message data in bytes.
 * @retval LL_OK on success, other values indicate error codes.
 */
int32_t HMAC_Update_Ex(stc_hmac_ctx_t *pCtx, const uint8_t *pData, uint32_t u32Len);

/**
 * @brief  Finalize HMAC calculation and output the 32-byte message digest.
 * @param  [in]  pCtx          Pointer to the HMAC context structure.
 * @param  [out] pu8MsgDigest  Pointer to the output buffer for the digest (at least 32 bytes).
 * @retval LL_OK on success, other values indicate error codes.
 */
int32_t HMAC_Final_Ex(stc_hmac_ctx_t *pCtx, uint8_t *pu8MsgDigest);

#ifdef __cplusplus
}
#endif

#endif /* __HC32_LL_HASH_EX_H__ */