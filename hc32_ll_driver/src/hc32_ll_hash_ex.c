#include "hc32_ll_hash_ex.h"
#include "hc32_ll_utility.h"


#define HASH_GROUP_SIZE         64U
#define HASH_GROUP_SIZE_WORD    (HASH_GROUP_SIZE / 4U)
#define HASH_LAST_GROUP_SIZE_MAX 56U
#define HASH_TIMEOUT            6000U
#define HASH_MSG_DIGEST_SIZE_WORD 8U

#define IPAD_CONST 0x36U
#define OPAD_CONST 0x5CU

static int32_t HASH_WaitStart(void);
static void HASH_WriteDataGroup(const uint8_t *pu8Data);
static void HASH_MemSet(uint8_t *pDest, uint8_t val, uint32_t len);
static void HASH_MemCopy(uint8_t *pDest, const uint8_t *pSrc, uint32_t len);
static int32_t HASH_ProcessBuffer(stc_hash_ctx_t *pCtx, uint8_t bLast);
static int32_t HMAC_PrepareKey(const uint8_t *pKey, uint32_t u32KeyLen, uint8_t *k0);

/**
 * @brief  Wait for the HASH operation to complete by polling the START flag.
 *         This function blocks until the START bit in the CR register is cleared,
 *         indicating that the current hash calculation has finished. A timeout
 *         mechanism prevents infinite waiting.
 *
 * @retval int32_t   LL_OK on success, or LL_ERR_TIMEOUT if the hardware does
 *                   not respond within the predefined timeout period.
 */
static int32_t HASH_WaitStart(void)
{
    uint32_t u32TimeOut = HASH_TIMEOUT;
    while (READ_REG32_BIT(CM_HASH->CR, HASH_CR_START) != 0UL) {
        if (--u32TimeOut == 0UL) {
            return LL_ERR_TIMEOUT;
        }
    }
    return LL_OK;
}


/**
 * @brief  Write one 64-byte data block to the HASH data registers.
 *         The input data is copied byte-by-byte to a temporary variable to avoid
 *         alignment issues, then byte-reversed (little-endian to big-endian) and
 *         written to the HASH data registers DR0~DR15.
 *
 * @param  [in] pu8Data    Pointer to the 64-byte data block to be written.
 * @retval None
 */
static void HASH_WriteDataGroup(const uint8_t *pu8Data)
{
    __IO uint32_t *regDR = &CM_HASH->DR0;    
    uint32_t u32Data;

    for (uint8_t i = 0U; i < HASH_GROUP_SIZE_WORD; i++) {
        HASH_MemCopy((uint8_t *)&u32Data,
                     &pu8Data[i * 4U],
                     sizeof(u32Data));

        regDR[i] = __REV(u32Data);          
    }
}


/**
 * @brief  Fill a memory area with a constant byte value.
 *         This function sets the first 'len' bytes of the memory area pointed
 *         to by 'pDest' to the specified value 'val'.
 *
 * @param  [out] pDest     Pointer to the destination memory area.
 * @param  [in]  val       Byte value to fill the memory area.
 * @param  [in]  len       Number of bytes to fill.
 * @retval None
 */
static void HASH_MemSet(uint8_t *pDest, uint8_t val, uint32_t len)
{
    while (len--) *pDest++ = val;
}


/**
 * @brief  Copy a memory area from source to destination.
 *         This function copies 'len' bytes from the source memory area
 *         pointed to by 'pSrc' to the destination memory area pointed
 *         to by 'pDest'. The memory areas must not overlap.
 *
 * @param  [out] pDest     Pointer to the destination memory area.
 * @param  [in]  pSrc      Pointer to the source memory area.
 * @param  [in]  len       Number of bytes to copy.
 * @retval None
 */
static void HASH_MemCopy(uint8_t *pDest, const uint8_t *pSrc, uint32_t len)
{
    while (len--) *pDest++ = *pSrc++;
}

/**
 * @brief  Process the current buffer content and write it to HASH hardware.
 *         This function handles both normal (non-last) and last data blocks.
 *         For non-last blocks, the buffer must be full (64 bytes). For the last
 *         block, it performs SHA256 padding: appends 0x80, zeros, and the 64-bit
 *         message length in bits. If the remaining data length is 56 or more,
 *         two blocks are sent: one with 0x80 and zeros, and another with the
 *         length field.
 *
 * @param  [in,out] pCtx   Pointer to the HASH context structure.
 * @param  [in]     bLast  Flag indicating whether this is the last block.
 *                         - 0: Normal block (buffer must be full)
 *                         - 1: Last block (perform padding and finalize)
 *
 * @retval int32_t   LL_OK on success, LL_ERR_INVD_PARAM if buffer is not full
 *                   for non-last block, or LL_ERR_TIMEOUT if hardware timeout.
 */
static int32_t HASH_ProcessBuffer(stc_hash_ctx_t *pCtx, uint8_t bLast)
{
    uint8_t fillBuf[HASH_GROUP_SIZE];
    uint32_t len = pCtx->buffer_len;
    int32_t ret;

    if (len == 0U && !bLast) {
        return LL_OK;
    }

    if (bLast) {
        HASH_MemSet(fillBuf, 0U, HASH_GROUP_SIZE);
        if (len > 0U) {
            HASH_MemCopy(fillBuf, pCtx->buffer, len);
        }
        fillBuf[len] = 0x80U;   

        uint64_t bitLen = pCtx->total_len << 3;   

        if (len < HASH_LAST_GROUP_SIZE_MAX) {
            for (uint8_t i = 0; i < 8; i++) {
                fillBuf[56 + i] = (uint8_t)(bitLen >> (i * 8));
            }
            HASH_WriteDataGroup(fillBuf);
            uint32_t flags = HASH_CR_KMSG_END | HASH_FLAG_CLR_ALL;
            if (!pCtx->started) {
                flags |= HASH_CR_FST_GRP;
            }
            SET_REG32_BIT(CM_HASH->CR, flags | HASH_CR_START);   
            ret = HASH_WaitStart();
            if (ret != LL_OK) return ret;
        } else {
            HASH_WriteDataGroup(fillBuf);
            uint32_t flags = HASH_FLAG_CLR_ALL;
            if (!pCtx->started) {
                flags |= HASH_CR_FST_GRP;
            }
            SET_REG32_BIT(CM_HASH->CR, flags | HASH_CR_START);   
            ret = HASH_WaitStart();
            if (ret != LL_OK) return ret;
            pCtx->started = 1U;

            HASH_MemSet(fillBuf, 0U, HASH_GROUP_SIZE);
            for (uint8_t i = 0; i < 8; i++) {
                fillBuf[56 + i] = (uint8_t)(bitLen >> (i * 8));
            }
            HASH_WriteDataGroup(fillBuf);
            SET_REG32_BIT(CM_HASH->CR, HASH_CR_KMSG_END | HASH_FLAG_CLR_ALL | HASH_CR_START); // 关键修复
            ret = HASH_WaitStart();
            if (ret != LL_OK) return ret;
        }
        CLR_REG32_BIT(CM_HASH->CR, HASH_FLAG_CLR_ALL);
        pCtx->finished = 1U;
    } else {
        if (len != HASH_GROUP_SIZE) return LL_ERR_INVD_PARAM;
        HASH_WriteDataGroup(pCtx->buffer);
        uint32_t flags = HASH_FLAG_CLR_ALL;
        if (!pCtx->started) {
            flags |= HASH_CR_FST_GRP;
        }
        SET_REG32_BIT(CM_HASH->CR, flags | HASH_CR_START);   
        ret = HASH_WaitStart();
        if (ret != LL_OK) return ret;
        pCtx->started = 1U;
        pCtx->buffer_len = 0U;
    }
    return LL_OK;
}


/**
 * @brief  Initialize the HASH context for SHA256 calculation.
 *         This function resets the context structure, configures the HASH
 *         hardware in SHA256 mode, clears status flags, and waits for the
 *         hardware to become idle.
 *
 * @param  [out] pCtx     Pointer to the HASH context structure to be initialized.
 *
 * @retval int32_t   LL_OK on success, LL_ERR_INVD_PARAM if pCtx is NULL,
 *                   or LL_ERR_TIMEOUT if hardware timeout.
 */
int32_t HASH_Init_Ex(stc_hash_ctx_t *pCtx)
{
    if (pCtx == NULL) return LL_ERR_INVD_PARAM;

    pCtx->buffer_len = 0U;
    pCtx->total_len  = 0U;
    pCtx->started    = 0U;
    pCtx->finished   = 0U;
    HASH_MemSet(pCtx->buffer, 0U, HASH_GROUP_SIZE);

    (void)HASH_SetMode(HASH_MD_SHA256);
    (void)HASH_ClearStatus(HASH_FLAG_CLR_ALL);
    return HASH_WaitStart();
}


/**
 * @brief  Update the SHA256 calculation with new data.
 *         This function can be called multiple times to feed data incrementally.
 *         Data is accumulated in the internal buffer until a full 64-byte block
 *         is available, then it is processed by the HASH hardware.
 *
 * @param  [in,out] pCtx     Pointer to the HASH context structure.
 * @param  [in]     pData    Pointer to the input data buffer.
 * @param  [in]     u32Len   Length of the input data in bytes.
 *
 * @retval int32_t   LL_OK on success, LL_ERR_INVD_PARAM on invalid parameter,
 *                   or LL_ERR_TIMEOUT if hardware timeout.
 */
int32_t HASH_Update_Ex(stc_hash_ctx_t *pCtx, const uint8_t *pData, uint32_t u32Len)
{
    int32_t ret;
    uint32_t offset = 0U;

    if (pCtx == NULL || pData == NULL || u32Len == 0U) return LL_ERR_INVD_PARAM;
    if (pCtx->finished) return LL_ERR_INVD_PARAM;

    pCtx->total_len += u32Len;   

    while (offset < u32Len) {
        uint32_t space = HASH_GROUP_SIZE - pCtx->buffer_len;
        uint32_t copy = (u32Len - offset) < space ? (u32Len - offset) : space;
        HASH_MemCopy(&pCtx->buffer[pCtx->buffer_len], &pData[offset], copy);
        pCtx->buffer_len += copy;
        offset += copy;

        if (pCtx->buffer_len == HASH_GROUP_SIZE) {
            ret = HASH_ProcessBuffer(pCtx, 0U);
            if (ret != LL_OK) return ret;
        }
    }
    return LL_OK;
}

/**
 * @brief  Finalize the SHA256 calculation and output the message digest.
 *         This function processes any remaining data in the buffer with proper
 *         SHA256 padding (0x80, zeros, and 64-bit length), then reads the final
 *         32-byte hash result from the hardware registers. After this function
 *         is called, the context is marked as finished and cannot be reused
 *         for further updates.
 *
 * @param  [in,out] pCtx          Pointer to the HASH context structure.
 * @param  [out]    pu8MsgDigest  Pointer to the output buffer for the 32-byte
 *                                message digest. The buffer must have at least
 *                                32 bytes of space.
 *
 * @retval int32_t   LL_OK on success, LL_ERR_INVD_PARAM on invalid parameter,
 *                   or LL_ERR_TIMEOUT if hardware timeout.
 */
int32_t HASH_Final_Ex(stc_hash_ctx_t *pCtx, uint8_t *pu8MsgDigest)
{
    int32_t ret;
    if (pCtx == NULL || pu8MsgDigest == NULL) return LL_ERR_INVD_PARAM;
    if (pCtx->finished) return LL_ERR_INVD_PARAM;

    ret = HASH_ProcessBuffer(pCtx, 1U);
    if (ret != LL_OK) return ret;

    __IO uint32_t *regHR = &CM_HASH->HR7;
    uint32_t *pDigest = (uint32_t *)pu8MsgDigest;
    for (uint8_t i = 0U; i < HASH_MSG_DIGEST_SIZE_WORD; i++) {
        pDigest[i] = __REV(regHR[i]);
    }
    return LL_OK;
}


/**
 * @brief  Prepare the HMAC key material (k0) according to RFC 2104.
 *         If the key length is greater than 64 bytes, it is first hashed using
 *         SHA256, then padded with zeros to 64 bytes. Otherwise, the key is
 *         copied directly and padded with zeros to 64 bytes.
 *
 * @param  [in]  pKey       Pointer to the secret key buffer.
 * @param  [in]  u32KeyLen  Length of the secret key in bytes.
 * @param  [out] k0         Output buffer of 64 bytes for the prepared key.
 *
 * @retval int32_t   LL_OK on success, or LL_ERR_TIMEOUT if hardware timeout.
 */
static int32_t HMAC_PrepareKey(const uint8_t *pKey, uint32_t u32KeyLen, uint8_t *k0)
{
    uint8_t tmpDigest[32];
    int32_t ret;

    if (u32KeyLen > 64U) {
       
        ret = HASH_Calculate(pKey, u32KeyLen, tmpDigest);   
        if (ret != LL_OK) return ret;
        HASH_MemCopy(k0, tmpDigest, 32);
        HASH_MemSet(k0 + 32, 0U, 32); 
    } else {
        HASH_MemCopy(k0, pKey, u32KeyLen);
        HASH_MemSet(k0 + u32KeyLen, 0U, 64U - u32KeyLen);
    }
    return LL_OK;
}

/**
 * @brief  Initialize the HMAC context with the given secret key.
 *         This function prepares the key material (k0) according to RFC 2104,
 *         computes ipad_key = k0 XOR 0x36, and initializes the inner SHA256
 *         context with it. The context is then ready to accept message data
 *         via HMAC_Update_Ex().
 *
 * @param  [out] pCtx       Pointer to the HMAC context structure.
 * @param  [in]  pKey       Pointer to the secret key buffer.
 * @param  [in]  u32KeyLen  Length of the secret key in bytes.
 *
 * @retval int32_t   LL_OK on success, LL_ERR_INVD_PARAM on invalid parameter,
 *                   or LL_ERR_TIMEOUT if hardware timeout.
 */
int32_t HMAC_Init_Ex(stc_hmac_ctx_t *pCtx, const uint8_t *pKey, uint32_t u32KeyLen)
{
    int32_t ret;
    uint8_t ipad_key[64];

    if (pCtx == NULL || pKey == NULL || u32KeyLen == 0U) return LL_ERR_INVD_PARAM;

    ret = HMAC_PrepareKey(pKey, u32KeyLen, pCtx->k0);
    if (ret != LL_OK) return ret;

    for (uint8_t i = 0U; i < 64U; i++) {
        ipad_key[i] = pCtx->k0[i] ^ IPAD_CONST;
    }
    ret = HASH_Init_Ex(&pCtx->inner_ctx);
    if (ret != LL_OK) return ret;
    ret = HASH_Update_Ex(&pCtx->inner_ctx, ipad_key, 64);
    if (ret != LL_OK) return ret;

    pCtx->state = 1U;
    return LL_OK;
}

/**
 * @brief  Update the HMAC calculation with new message data.
 *         This function feeds message data into the inner SHA256 context.
 *         It can be called multiple times with arbitrary-length data chunks.
 *
 * @param  [in,out] pCtx     Pointer to the HMAC context structure.
 * @param  [in]     pData    Pointer to the message data buffer.
 * @param  [in]     u32Len   Length of the message data in bytes.
 *
 * @retval int32_t   LL_OK on success, LL_ERR_INVD_PARAM on invalid parameter,
 *                   or LL_ERR_TIMEOUT if hardware timeout.
 */
int32_t HMAC_Update_Ex(stc_hmac_ctx_t *pCtx, const uint8_t *pData, uint32_t u32Len)
{
    if (pCtx == NULL || pData == NULL || u32Len == 0U) return LL_ERR_INVD_PARAM;
    if (pCtx->state != 1U) return LL_ERR_INVD_PARAM;

    return HASH_Update_Ex(&pCtx->inner_ctx, pData, u32Len);
}

/**
 * @brief  Finalize the HMAC calculation and output the message digest.
 *         This function completes the HMAC computation by:
 *         1. Finalizing the inner SHA256 context to get inner_hash
 *         2. Computing opad_key = k0 XOR 0x5C
 *         3. Initializing the outer SHA256 context with opad_key
 *         4. Appending inner_hash to the outer context
 *         5. Finalizing the outer context to produce the final HMAC digest
 *
 * @param  [in,out] pCtx          Pointer to the HMAC context structure.
 * @param  [out]    pu8MsgDigest  Pointer to the output buffer for the 32-byte
 *                                HMAC digest. The buffer must have at least
 *                                32 bytes of space.
 *
 * @retval int32_t   LL_OK on success, LL_ERR_INVD_PARAM on invalid parameter,
 *                   or LL_ERR_TIMEOUT if hardware timeout.
 */
int32_t HMAC_Final_Ex(stc_hmac_ctx_t *pCtx, uint8_t *pu8MsgDigest)
{
    int32_t ret;
    uint8_t inner_hash[32];
    uint8_t opad_key[64];

    if (pCtx == NULL || pu8MsgDigest == NULL) return LL_ERR_INVD_PARAM;
    if (pCtx->state != 1U) return LL_ERR_INVD_PARAM;

    ret = HASH_Final_Ex(&pCtx->inner_ctx, inner_hash);
    if (ret != LL_OK) return ret;

    for (uint8_t i = 0U; i < 64U; i++) {
        opad_key[i] = pCtx->k0[i] ^ OPAD_CONST;
    }
    ret = HASH_Init_Ex(&pCtx->outer_ctx);
    if (ret != LL_OK) return ret;
    ret = HASH_Update_Ex(&pCtx->outer_ctx, opad_key, 64);
    if (ret != LL_OK) return ret;

    ret = HASH_Update_Ex(&pCtx->outer_ctx, inner_hash, 32);
    if (ret != LL_OK) return ret;

    ret = HASH_Final_Ex(&pCtx->outer_ctx, pu8MsgDigest);
    if (ret != LL_OK) return ret;

    pCtx->state = 2U;
    return LL_OK;
}