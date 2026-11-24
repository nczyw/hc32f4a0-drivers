#ifndef __HC32_LL_AES_EX_H__
#define __HC32_LL_AES_EX_H__

#include "hc32_ll_aes.h"

#ifdef __cplusplus
extern "C" {
#endif

#if (LL_AES_ENABLE == DDL_ON)

#define AES_EX_BLOCK_SIZE       (16U)

#define AES_EX_MODE_ECB         (0U)
#define AES_EX_MODE_CBC         (1U)
#define AES_EX_MODE_CTR         (2U)

#define AES_EX_DIR_ENCRYPT      (0U)
#define AES_EX_DIR_DECRYPT      (1U)

/* AES streaming context. All state belongs to the caller. */
typedef struct {
    uint8_t  key[32];          /* AES-128/192/256 key */
    uint8_t  iv[16];           /* CBC chaining value or CTR counter */
    uint8_t  buffer[16];       /* Partial/last block or CTR keystream */
    uint64_t total_len;        /* Total input bytes */
    uint32_t buffer_len;       /* Mode-dependent buffer state */
    uint8_t  key_size;         /* Key size in bytes */
    uint8_t  mode;             /* AES_EX_MODE_xxx */
    uint8_t  dir;              /* AES_EX_DIR_xxx */
    uint8_t  started;          /* Context initialized */
    uint8_t  finished;         /* Final operation completed */
} stc_aes_ctx_t;

/**
 * @brief  Initialize an AES streaming context.
 * @param  [out] pCtx       Pointer to the caller-owned context.
 * @param  [in]  u8Mode     AES_EX_MODE_ECB, AES_EX_MODE_CBC or AES_EX_MODE_CTR.
 * @param  [in]  u8Dir      AES_EX_DIR_ENCRYPT or AES_EX_DIR_DECRYPT.
 *                          Ignored for CTR, which always uses block encryption.
 * @param  [in]  pu8Key     Pointer to the AES key.
 * @param  [in]  u8KeySize  AES_KEY_SIZE_16BYTE/24BYTE/32BYTE.
 * @param  [in]  pu8Iv      CBC IV or initial CTR counter. May be NULL for ECB.
 * @retval LL_OK on success, LL_ERR_INVD_PARAM on invalid parameters.
 */
int32_t AES_Init_Ex(stc_aes_ctx_t *pCtx, uint8_t u8Mode, uint8_t u8Dir,
                    const uint8_t *pu8Key, uint8_t u8KeySize,
                    const uint8_t *pu8Iv);

/**
 * @brief  Process an input chunk.
 *         ECB/CBC encryption accepts arbitrary lengths and buffers partial blocks.
 *         ECB/CBC decryption requires complete blocks and retains the last block
 *         for PKCS#7 validation in AES_Final_Ex(). CTR accepts arbitrary lengths
 *         and produces exactly the same number of output bytes.
 * @param  [in,out] pCtx        Pointer to the AES context.
 * @param  [in]     pu8InData   Pointer to the input data.
 * @param  [in]     u32InLen    Input length in bytes, greater than zero.
 * @param  [out]    pu8OutData  Output buffer, sized for the processed blocks.
 * @param  [out]    pu32OutLen  Number of bytes written to pu8OutData.
 * @retval LL_OK on success, otherwise an LL_ERR_xxx code.
 */
int32_t AES_Update_Ex(stc_aes_ctx_t *pCtx, const uint8_t *pu8InData,
                      uint32_t u32InLen, uint8_t *pu8OutData,
                      uint32_t *pu32OutLen);

/**
 * @brief  Finish an AES streaming operation.
 *         ECB/CBC encryption appends PKCS#7 padding and outputs one final block.
 *         ECB/CBC decryption processes the retained last block, strictly checks
 *         PKCS#7 padding and outputs the unpadded bytes. CTR produces no output.
 *         On success, sensitive context data is cleared.
 * @param  [in,out] pCtx        Pointer to the AES context.
 * @param  [out]    pu8OutData  Output buffer (at least 16 bytes for ECB/CBC).
 *                              May be NULL for CTR.
 * @param  [out]    pu32OutLen  Number of bytes written to pu8OutData.
 * @retval LL_OK on success, otherwise an LL_ERR_xxx code.
 */
int32_t AES_Final_Ex(stc_aes_ctx_t *pCtx, uint8_t *pu8OutData,
                     uint32_t *pu32OutLen);

#endif /* LL_AES_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* __HC32_LL_AES_EX_H__ */
