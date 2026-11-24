#include "hc32_ll_aes_ex.h"

#if (LL_AES_ENABLE == DDL_ON)

#define AES_EX_KEY_SIZE_MAX     (32U)
#define AES_EX_BLOCK_WORDS      (AES_EX_BLOCK_SIZE / 4U)

static void AES_EX_MemCopy(uint8_t *pu8Dest, const uint8_t *pu8Src, uint32_t u32Len);
static void AES_EX_MemSet(uint8_t *pu8Dest, uint8_t u8Val, uint32_t u32Len);
static void AES_EX_ClearContext(stc_aes_ctx_t *pCtx);
static int32_t AES_EX_CryptBlock(stc_aes_ctx_t *pCtx, const uint8_t *pu8In, uint8_t *pu8Out);
static int32_t AES_EX_UpdateBlockEncrypt(stc_aes_ctx_t *pCtx, const uint8_t *pu8InData,
                                         uint32_t u32InLen, uint8_t *pu8OutData,
                                         uint32_t *pu32OutLen);
static int32_t AES_EX_UpdateBlockDecrypt(stc_aes_ctx_t *pCtx, const uint8_t *pu8InData,
                                         uint32_t u32InLen, uint8_t *pu8OutData,
                                         uint32_t *pu32OutLen);
static int32_t AES_EX_UpdateCtr(stc_aes_ctx_t *pCtx, const uint8_t *pu8InData,
                                uint32_t u32InLen, uint8_t *pu8OutData,
                                uint32_t *pu32OutLen);
static int32_t AES_EX_GenerateCtrBlock(stc_aes_ctx_t *pCtx);
static void AES_EX_IncrementCtr(uint8_t *pu8Ctr);

static void AES_EX_MemCopy(uint8_t *pu8Dest, const uint8_t *pu8Src, uint32_t u32Len)
{
    while (u32Len-- > 0U) {
        *pu8Dest++ = *pu8Src++;
    }
}

static void AES_EX_MemSet(uint8_t *pu8Dest, uint8_t u8Val, uint32_t u32Len)
{
    while (u32Len-- > 0U) {
        *pu8Dest++ = u8Val;
    }
}

static void AES_EX_ClearContext(stc_aes_ctx_t *pCtx)
{
    volatile uint8_t *pu8Ctx = (volatile uint8_t *)pCtx;
    uint32_t i;

    for (i = 0U; i < (uint32_t)sizeof(*pCtx); i++) {
        pu8Ctx[i] = 0U;
    }
}

/* Process one ECB/CBC block. Aligned local words avoid unaligned AES driver access. */
static int32_t AES_EX_CryptBlock(stc_aes_ctx_t *pCtx, const uint8_t *pu8In, uint8_t *pu8Out)
{
    uint32_t au32In[AES_EX_BLOCK_WORDS];
    uint32_t au32Out[AES_EX_BLOCK_WORDS];
    uint8_t *pu8InAligned = (uint8_t *)au32In;
    uint8_t *pu8OutAligned = (uint8_t *)au32Out;
    int32_t i32Ret;
    uint32_t i;

    if ((pCtx->mode == AES_EX_MODE_CBC) && (pCtx->dir == AES_EX_DIR_ENCRYPT)) {
        for (i = 0U; i < AES_EX_BLOCK_SIZE; i++) {
            pu8InAligned[i] = pu8In[i] ^ pCtx->iv[i];
        }
    } else {
        AES_EX_MemCopy(pu8InAligned, pu8In, AES_EX_BLOCK_SIZE);
    }

    if (pCtx->dir == AES_EX_DIR_ENCRYPT) {
        i32Ret = AES_Encrypt(pu8InAligned, AES_EX_BLOCK_SIZE,
                             pCtx->key, pCtx->key_size, pu8OutAligned);
    } else {
        i32Ret = AES_Decrypt(pu8InAligned, AES_EX_BLOCK_SIZE,
                             pCtx->key, pCtx->key_size, pu8OutAligned);
    }
    if (i32Ret != LL_OK) {
        return i32Ret;
    }

    if (pCtx->mode == AES_EX_MODE_CBC) {
        if (pCtx->dir == AES_EX_DIR_ENCRYPT) {
            AES_EX_MemCopy(pu8Out, pu8OutAligned, AES_EX_BLOCK_SIZE);
            AES_EX_MemCopy(pCtx->iv, pu8OutAligned, AES_EX_BLOCK_SIZE);
        } else {
            for (i = 0U; i < AES_EX_BLOCK_SIZE; i++) {
                pu8Out[i] = pu8OutAligned[i] ^ pCtx->iv[i];
            }
            AES_EX_MemCopy(pCtx->iv, pu8In, AES_EX_BLOCK_SIZE);
        }
    } else {
        AES_EX_MemCopy(pu8Out, pu8OutAligned, AES_EX_BLOCK_SIZE);
    }

    return LL_OK;
}

static int32_t AES_EX_UpdateBlockEncrypt(stc_aes_ctx_t *pCtx, const uint8_t *pu8InData,
                                         uint32_t u32InLen, uint8_t *pu8OutData,
                                         uint32_t *pu32OutLen)
{
    uint32_t u32Offset = 0U;
    uint32_t u32OutLen = 0U;
    uint32_t u32Copy;
    int32_t i32Ret;

    if (pCtx->buffer_len >= AES_EX_BLOCK_SIZE) {
        return LL_ERR_INVD_PARAM;
    }

    if (pCtx->buffer_len > 0U) {
        u32Copy = AES_EX_BLOCK_SIZE - pCtx->buffer_len;
        if (u32Copy > u32InLen) {
            u32Copy = u32InLen;
        }
        AES_EX_MemCopy(&pCtx->buffer[pCtx->buffer_len], pu8InData, u32Copy);
        pCtx->buffer_len += u32Copy;
        u32Offset = u32Copy;

        if (pCtx->buffer_len < AES_EX_BLOCK_SIZE) {
            *pu32OutLen = 0U;
            return LL_OK;
        }

        i32Ret = AES_EX_CryptBlock(pCtx, pCtx->buffer, pu8OutData);
        if (i32Ret != LL_OK) {
            return i32Ret;
        }
        pCtx->buffer_len = 0U;
        u32OutLen = AES_EX_BLOCK_SIZE;
    }

    while ((u32InLen - u32Offset) >= AES_EX_BLOCK_SIZE) {
        i32Ret = AES_EX_CryptBlock(pCtx, &pu8InData[u32Offset],
                                   &pu8OutData[u32OutLen]);
        if (i32Ret != LL_OK) {
            return i32Ret;
        }
        u32Offset += AES_EX_BLOCK_SIZE;
        u32OutLen += AES_EX_BLOCK_SIZE;
    }

    if (u32Offset < u32InLen) {
        pCtx->buffer_len = u32InLen - u32Offset;
        AES_EX_MemCopy(pCtx->buffer, &pu8InData[u32Offset], pCtx->buffer_len);
    }

    *pu32OutLen = u32OutLen;
    return LL_OK;
}

/* Keep the last ciphertext block until Final so PKCS#7 can be removed safely. */
static int32_t AES_EX_UpdateBlockDecrypt(stc_aes_ctx_t *pCtx, const uint8_t *pu8InData,
                                         uint32_t u32InLen, uint8_t *pu8OutData,
                                         uint32_t *pu32OutLen)
{
    uint32_t u32Offset = 0U;
    uint32_t u32OutLen = 0U;
    int32_t i32Ret;

    if (((u32InLen % AES_EX_BLOCK_SIZE) != 0U) ||
        ((pCtx->buffer_len != 0U) && (pCtx->buffer_len != AES_EX_BLOCK_SIZE))) {
        return LL_ERR_INVD_PARAM;
    }

    if (pCtx->buffer_len == AES_EX_BLOCK_SIZE) {
        i32Ret = AES_EX_CryptBlock(pCtx, pCtx->buffer, pu8OutData);
        if (i32Ret != LL_OK) {
            return i32Ret;
        }
        pCtx->buffer_len = 0U;
        u32OutLen = AES_EX_BLOCK_SIZE;
    }

    while ((u32InLen - u32Offset) > AES_EX_BLOCK_SIZE) {
        i32Ret = AES_EX_CryptBlock(pCtx, &pu8InData[u32Offset],
                                   &pu8OutData[u32OutLen]);
        if (i32Ret != LL_OK) {
            return i32Ret;
        }
        u32Offset += AES_EX_BLOCK_SIZE;
        u32OutLen += AES_EX_BLOCK_SIZE;
    }

    AES_EX_MemCopy(pCtx->buffer, &pu8InData[u32Offset], AES_EX_BLOCK_SIZE);
    pCtx->buffer_len = AES_EX_BLOCK_SIZE;
    *pu32OutLen = u32OutLen;
    return LL_OK;
}

static void AES_EX_IncrementCtr(uint8_t *pu8Ctr)
{
    int32_t i;

    for (i = (int32_t)AES_EX_BLOCK_SIZE - 1; i >= 0; i--) {
        pu8Ctr[i]++;
        if (pu8Ctr[i] != 0U) {
            break;
        }
    }
}

static int32_t AES_EX_GenerateCtrBlock(stc_aes_ctx_t *pCtx)
{
    uint32_t au32Counter[AES_EX_BLOCK_WORDS];
    uint32_t au32Stream[AES_EX_BLOCK_WORDS];
    int32_t i32Ret;

    AES_EX_MemCopy((uint8_t *)au32Counter, pCtx->iv, AES_EX_BLOCK_SIZE);
    i32Ret = AES_Encrypt((const uint8_t *)au32Counter, AES_EX_BLOCK_SIZE,
                         pCtx->key, pCtx->key_size, (uint8_t *)au32Stream);
    if (i32Ret != LL_OK) {
        return i32Ret;
    }

    AES_EX_MemCopy(pCtx->buffer, (const uint8_t *)au32Stream, AES_EX_BLOCK_SIZE);
    AES_EX_IncrementCtr(pCtx->iv);
    pCtx->buffer_len = 0U;
    return LL_OK;
}

static int32_t AES_EX_UpdateCtr(stc_aes_ctx_t *pCtx, const uint8_t *pu8InData,
                                uint32_t u32InLen, uint8_t *pu8OutData,
                                uint32_t *pu32OutLen)
{
    uint32_t u32Offset = 0U;
    uint32_t u32Copy;
    uint32_t i;
    int32_t i32Ret;

    if (pCtx->buffer_len > AES_EX_BLOCK_SIZE) {
        return LL_ERR_INVD_PARAM;
    }

    while (u32Offset < u32InLen) {
        if (pCtx->buffer_len == AES_EX_BLOCK_SIZE) {
            i32Ret = AES_EX_GenerateCtrBlock(pCtx);
            if (i32Ret != LL_OK) {
                return i32Ret;
            }
        }

        u32Copy = AES_EX_BLOCK_SIZE - pCtx->buffer_len;
        if (u32Copy > (u32InLen - u32Offset)) {
            u32Copy = u32InLen - u32Offset;
        }
        for (i = 0U; i < u32Copy; i++) {
            pu8OutData[u32Offset + i] = pu8InData[u32Offset + i] ^
                                        pCtx->buffer[pCtx->buffer_len + i];
        }
        pCtx->buffer_len += u32Copy;
        u32Offset += u32Copy;
    }

    *pu32OutLen = u32InLen;
    return LL_OK;
}

int32_t AES_Init_Ex(stc_aes_ctx_t *pCtx, uint8_t u8Mode, uint8_t u8Dir,
                    const uint8_t *pu8Key, uint8_t u8KeySize,
                    const uint8_t *pu8Iv)
{
    if (pCtx == NULL) {
        return LL_ERR_INVD_PARAM;
    }
    AES_EX_MemSet((uint8_t *)pCtx, 0U, (uint32_t)sizeof(*pCtx));

    if ((pu8Key == NULL) ||
        ((u8KeySize != AES_KEY_SIZE_16BYTE) &&
         (u8KeySize != AES_KEY_SIZE_24BYTE) &&
         (u8KeySize != AES_KEY_SIZE_32BYTE)) ||
        (u8Mode > AES_EX_MODE_CTR) ||
        ((u8Mode != AES_EX_MODE_CTR) && (u8Dir > AES_EX_DIR_DECRYPT)) ||
        ((u8Mode != AES_EX_MODE_ECB) && (pu8Iv == NULL))) {
        return LL_ERR_INVD_PARAM;
    }

    AES_EX_MemCopy(pCtx->key, pu8Key, u8KeySize);
    pCtx->key_size = u8KeySize;
    pCtx->mode = u8Mode;
    pCtx->dir = (u8Mode == AES_EX_MODE_CTR) ? AES_EX_DIR_ENCRYPT : u8Dir;
    if (u8Mode != AES_EX_MODE_ECB) {
        AES_EX_MemCopy(pCtx->iv, pu8Iv, AES_EX_BLOCK_SIZE);
    }
    if (u8Mode == AES_EX_MODE_CTR) {
        pCtx->buffer_len = AES_EX_BLOCK_SIZE;
    }
    pCtx->started = 1U;
    return LL_OK;
}

int32_t AES_Update_Ex(stc_aes_ctx_t *pCtx, const uint8_t *pu8InData,
                      uint32_t u32InLen, uint8_t *pu8OutData,
                      uint32_t *pu32OutLen)
{
    int32_t i32Ret;

    if (pu32OutLen != NULL) {
        *pu32OutLen = 0U;
    }
    if ((pCtx == NULL) || (pu32OutLen == NULL) || (pu8InData == NULL) ||
        (pu8OutData == NULL) || (u32InLen == 0U) ||
        (pCtx->started != 1U) || (pCtx->finished != 0U)) {
        return LL_ERR_INVD_PARAM;
    }

    if (pCtx->mode == AES_EX_MODE_CTR) {
        i32Ret = AES_EX_UpdateCtr(pCtx, pu8InData, u32InLen,
                                  pu8OutData, pu32OutLen);
    } else if (pCtx->dir == AES_EX_DIR_ENCRYPT) {
        i32Ret = AES_EX_UpdateBlockEncrypt(pCtx, pu8InData, u32InLen,
                                           pu8OutData, pu32OutLen);
    } else {
        i32Ret = AES_EX_UpdateBlockDecrypt(pCtx, pu8InData, u32InLen,
                                           pu8OutData, pu32OutLen);
    }

    if (i32Ret == LL_OK) {
        pCtx->total_len += u32InLen;
    }
    return i32Ret;
}

int32_t AES_Final_Ex(stc_aes_ctx_t *pCtx, uint8_t *pu8OutData,
                     uint32_t *pu32OutLen)
{
    uint32_t au32Plain[AES_EX_BLOCK_WORDS];
    uint8_t *pu8Plain = (uint8_t *)au32Plain;
    uint32_t u32OutLen = 0U;
    uint8_t u8Padding;
    uint32_t i;
    int32_t i32Ret = LL_OK;

    if (pu32OutLen != NULL) {
        *pu32OutLen = 0U;
    }
    if ((pCtx == NULL) || (pu32OutLen == NULL) ||
        (pCtx->started != 1U) || (pCtx->finished != 0U) ||
        (pCtx->mode > AES_EX_MODE_CTR)) {
        return LL_ERR_INVD_PARAM;
    }

    if (pCtx->mode == AES_EX_MODE_CTR) {
        AES_EX_ClearContext(pCtx);
        pCtx->finished = 1U;
        return LL_OK;
    }
    if (pu8OutData == NULL) {
        return LL_ERR_INVD_PARAM;
    }

    if (pCtx->dir == AES_EX_DIR_ENCRYPT) {
        if (pCtx->buffer_len >= AES_EX_BLOCK_SIZE) {
            return LL_ERR_INVD_PARAM;
        }
        u8Padding = (uint8_t)(AES_EX_BLOCK_SIZE - pCtx->buffer_len);
        AES_EX_MemSet(&pCtx->buffer[pCtx->buffer_len], u8Padding, u8Padding);
        i32Ret = AES_EX_CryptBlock(pCtx, pCtx->buffer, pu8OutData);
        if (i32Ret != LL_OK) {
            return i32Ret;
        }
        u32OutLen = AES_EX_BLOCK_SIZE;
    } else {
        if (pCtx->buffer_len != AES_EX_BLOCK_SIZE) {
            return LL_ERR_INVD_PARAM;
        }
        i32Ret = AES_EX_CryptBlock(pCtx, pCtx->buffer, pu8Plain);
        if (i32Ret != LL_OK) {
            return i32Ret;
        }

        u8Padding = pu8Plain[AES_EX_BLOCK_SIZE - 1U];
        if ((u8Padding == 0U) || (u8Padding > AES_EX_BLOCK_SIZE)) {
            i32Ret = LL_ERR_INVD_PARAM;
        } else {
            for (i = 0U; i < u8Padding; i++) {
                if (pu8Plain[AES_EX_BLOCK_SIZE - 1U - i] != u8Padding) {
                    i32Ret = LL_ERR_INVD_PARAM;
                    break;
                }
            }
        }
        if (i32Ret == LL_OK) {
            u32OutLen = AES_EX_BLOCK_SIZE - u8Padding;
            AES_EX_MemCopy(pu8OutData, pu8Plain, u32OutLen);
        }
    }

    if (i32Ret == LL_OK) {
        *pu32OutLen = u32OutLen;
    }
    AES_EX_ClearContext(pCtx);
    pCtx->finished = 1U;
    return i32Ret;
}

#endif /* LL_AES_ENABLE */
