/**
 *******************************************************************************
 * @file  ev_hc32f4a0_lqfp176_wm8731.c
 * @brief This file provides configure functions for wm8731 of the board
 *        EV_HC32F4A0_LQFP176.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2023-09-30       CDT             Rename local variables: stcWm8731Config -> m_stcWm8731Config
   2024-11-08       CDT             Optimize using DMA linked list transfer
 @endverbatim
 *******************************************************************************
 * Copyright (C) 2022-2025, Xiaohua Semiconductor Co., Ltd. All rights reserved.
 *
 * This software component is licensed by XHSC under BSD 3-Clause license
 * (the "License"); You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                    opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "ev_hc32f4a0_lqfp176_wm8731.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @addtogroup EV_HC32F4A0_LQFP176
 * @{
 */

/**
 * @defgroup EV_HC32F4A0_LQFP176_WM8731 EV_HC32F4A0_LQFP176 WM8731
 * @{
 */

#if ((BSP_WM8731_ENABLE == DDL_ON) && (BSP_EV_HC32F4A0_LQFP176 == BSP_EV_HC32F4XX))

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
/**
 * @defgroup EV_HC32F4A0_LQFP176_WM8731_Local_Variables EV_HC32F4A0_LQFP176 WM8731 Local Variables
 * @{
 */
static stc_wm8731_ll_t m_stcWm8731Config = {0};
static uint32_t m_u32AudioDataBuf[2U][BSP_WM8731_DMA_BLK_LEN];
static stc_dma_llp_descriptor_t m_stcTxLlpDesc[2U] = {0};
static stc_dma_llp_descriptor_t m_stcRxLlpDesc[2U] = {0};
static __IO uint8_t m_u8Wm8731StopFlag = 1U;
/**
 * @}
 */

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @defgroup EV_HC32F4A0_LQFP176_WM8731_Local_Functions EV_HC32F4A0_LQFP176 WM8731 Local Functions
 * @{
 */

/**
 * @brief  Initializes I2C for WM8731.
 * @param  None
 * @retval None
 */
static void BSP_WM8731_I2C_Init(void)
{
    stc_gpio_init_t stcGpioInit;

    /* Configuration I2C GPIO */
    (void)GPIO_StructInit(&stcGpioInit);
    (void)GPIO_Init(BSP_WM8731_I2C_SCL_PORT, BSP_WM8731_I2C_SCL_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_WM8731_I2C_SDA_PORT, BSP_WM8731_I2C_SDA_PIN, &stcGpioInit);
    GPIO_SetFunc(BSP_WM8731_I2C_SCL_PORT, BSP_WM8731_I2C_SCL_PIN, BSP_WM8731_I2C_SCL_FUNC);
    GPIO_SetFunc(BSP_WM8731_I2C_SDA_PORT, BSP_WM8731_I2C_SDA_PIN, BSP_WM8731_I2C_SDA_FUNC);
    /* Enable I2C Peripheral*/
    FCG_Fcg1PeriphClockCmd(BSP_WM8731_I2C_FCG, ENABLE);
    (void)BSP_I2C_Init(BSP_WM8731_I2C_UNIT);
}

/**
 * @brief  BSP WM8731 write data.
 * @param  [in] pu8Reg:                 The register value to be write.
 * @param  [in] pu8Buf:                 The pointer to the buffer contains the data to be write.
 * @param  [in] u32Len:                 Buffer size in byte.
 * @retval None
 */
static void BSP_WM8731_I2C_Write(const uint8_t *pu8Reg, const uint8_t *pu8Buf, uint32_t u32Len)
{
    (void)BSP_I2C_Write(BSP_WM8731_I2C_UNIT, BSP_WM8731_DEV_ADDR, pu8Reg, BSP_WM8731_REG_ADDR_LEN, pu8Buf, u32Len);
}
/**
 * @}
 */

/**
 * @defgroup EV_HC32F4A0_LQFP176_WM8731_Global_Functions EV_HC32F4A0_LQFP176 WM8731 Global Functions
 * @{
 */
/**
 * @brief  De-initialize the WM8731.
 * @retval None
 */
void BSP_WM8731_DeInit(void)
{
    (void)WM8731_Reset(&m_stcWm8731Config);
    BSP_I2S_DeInit();
    /* Reset the low layer of WM8731 */
    m_stcWm8731Config.Delay         = NULL;
    m_stcWm8731Config.Init          = NULL;
    m_stcWm8731Config.Write         = NULL;
    m_stcWm8731Config.Read          = NULL;
}

/**
 * @brief  Initialize the WM8731.
 * @param  u8InputDevice:               Specifies the input device.
 *         This parameter can be one of the following values:
 *           @arg WM8731_INPUT_DEVICE_NONE:         No device input
 *           @arg WM8731_INPUT_DEVICE_MICROPHONE:   Microphone input
 *           @arg WM8731_INPUT_DEVICE_LINE:         Line in intput
 * @param  u8OutputDevice:              Specifies the output device.
 *         This parameter can be one of the following values:
 *           @arg WM8731_OUTPUT_DEVICE_NONE:        No device output
 *           @arg WM8731_OUTPUT_DEVICE_LINE:        Line in output
 *           @arg WM8731_OUTPUT_DEVICE_HEADPHONE:   Headphone output
 *           @arg WM8731_OUTPUT_DEVICE_BOTH:        Line in and Headphone output
 * @param  u8Volume:                    Specifies the volume of input and output, Range is 0 to 100.
 * @param  u32AudioFreq:                Specifies the audio frequency of the communication.
 *         This parameter can be one of the following values:
 *           @arg WM8731_AUDIO_FREQ_8K:     8K
 *           @arg WM8731_AUDIO_FREQ_32K:    32K
 *           @arg WM8731_AUDIO_FREQ_48K:    48K
 *           @arg WM8731_AUDIO_FREQ_96K:    96K
 * @param  u8DataWidth:                 Specifies the data width.
 *         This parameter can be one of the following values:
 *           @arg WM8731_DATA_WIDTH_16BIT:  16Bits
 *           @arg WM8731_DATA_WIDTH_20BIT:  20Bits
 *           @arg WM8731_DATA_WIDTH_24BIT:  24Bits
 *           @arg WM8731_DATA_WIDTH_32BIT:  32Bits
 * @retval None
 */
int32_t BSP_WM8731_Init(uint8_t u8InputDevice, uint8_t u8OutputDevice,
                        uint8_t u8Volume, uint32_t u32AudioFreq, uint8_t u8DataWidth)
{
    stc_wm8731_init_t stcWm8731Init;

    /* Configuration the low layer of WM8731 */
    m_stcWm8731Config.Delay         = DDL_DelayMS;
    m_stcWm8731Config.Init          = BSP_WM8731_I2C_Init;
    m_stcWm8731Config.Write         = BSP_WM8731_I2C_Write;
    m_stcWm8731Config.Read          = NULL;
    /* Configuration the WM8731 */
    stcWm8731Init.u8InputDevice  = u8InputDevice;
    stcWm8731Init.u8OutputDevice = u8OutputDevice;
    stcWm8731Init.u8OutputSrc    = WM8731_OUTPUT_SRC_DAC;
    stcWm8731Init.u32AudioFreq   = u32AudioFreq;
    stcWm8731Init.u8Volume       = u8Volume;
    stcWm8731Init.u8DataForamt   = WM8731_DATA_FORMAT_PHILLIPS;
    stcWm8731Init.u8DataWidth    = u8DataWidth;
    (void)WM8731_Init(&m_stcWm8731Config, &stcWm8731Init);
    /* Play audio */
    (void)WM8731_Play(&m_stcWm8731Config);
    /* Init I2S */
    return BSP_I2S_Init(u32AudioFreq);
}

/**
 * @brief  Starts recording and playing audio.
 * @param  None
 * @retval None
 */
void BSP_WM8731_Record_And_Play(void)
{
    if (0U == m_u8Wm8731StopFlag) {
        return;
    }

    (void)DMA_SetDestAddr(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SDIN_CH, (uint32_t)(&m_u32AudioDataBuf[1U][0U]));
    (void)DMA_SetTransCount(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SDIN_CH, BSP_WM8731_DMA_BLK_LEN);
    (void)DMA_SetBlockSize(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SDIN_CH, BSP_WM8731_DMA_BLK_SIZE);
    DMA_SetLlpAddr(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SDIN_CH, (uint32_t)&m_stcRxLlpDesc[0U]);
    (void)DMA_ChCmd(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SDIN_CH, ENABLE);

    (void)DMA_SetSrcAddr(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SD_CH, (uint32_t)(&m_u32AudioDataBuf[0U][0U]));
    (void)DMA_SetTransCount(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SD_CH, BSP_WM8731_DMA_BLK_LEN);
    (void)DMA_SetBlockSize(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SD_CH, BSP_WM8731_DMA_BLK_SIZE);
    DMA_SetLlpAddr(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SD_CH, (uint32_t)&m_stcTxLlpDesc[0U]);
    (void)DMA_ChCmd(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SD_CH, ENABLE);

    if (RESET != I2S_GetStatus(BSP_WM8731_I2S_UNIT, I2S_FLAG_RX_ERR | I2S_FLAG_TX_ERR)) {
        I2S_ClearStatus(BSP_WM8731_I2S_UNIT, I2S_FLAG_RX_ERR | I2S_FLAG_TX_ERR);
    }
    I2S_SetReceiveFIFOLevel(BSP_WM8731_I2S_UNIT, I2S_RECEIVE_LVL3);
    I2S_SetTransFIFOLevel(BSP_WM8731_I2S_UNIT, I2S_TRANS_LVL3);
    I2S_FuncCmd(BSP_WM8731_I2S_UNIT, (I2S_FUNC_RX | I2S_FUNC_TX), ENABLE);

    m_u8Wm8731StopFlag = 0U;
}

/**
 * @brief  Stops audio playing and recording.
 * @param  None
 * @retval None
 */
void BSP_WM8731_Stop(void)
{
    (void)DMA_ChCmd(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SD_CH, DISABLE);
    (void)DMA_ChCmd(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SDIN_CH, DISABLE);

    I2S_FuncCmd(BSP_WM8731_I2S_UNIT, (I2S_FUNC_RX | I2S_FUNC_TX), DISABLE);
    I2S_SWReset(BSP_WM8731_I2S_UNIT, I2S_RST_TYPE_FIFO);
    I2S_SWReset(BSP_WM8731_I2S_UNIT, I2S_RST_TYPE_SW);
    I2S_SetReceiveFIFOLevel(BSP_WM8731_I2S_UNIT, I2S_RECEIVE_LVL4);
    I2S_SetTransFIFOLevel(BSP_WM8731_I2S_UNIT, I2S_TRANS_LVL4);

    m_u8Wm8731StopFlag = 1U;
}

/**
 * @brief  Update the audio frequency.
 * @param  u32AudioFreq:               Audio frequency used to play the audio.
 * @retval None
 */
void BSP_WM8731_SetFreq(uint32_t u32AudioFreq)
{
    (void)WM8731_SetAudioFreq(&m_stcWm8731Config, u32AudioFreq);
    (void)I2S_SetAudioFreq(BSP_WM8731_I2S_UNIT, u32AudioFreq);
}

/**
 * @brief  Set the audio volume level.
 * @param  u8Volume:                   Volume level, Range is 0 to 100.
 * @retval None
 */
void BSP_WM8731_SetVolume(uint8_t u8Volume)
{
    (void)WM8731_SetVolume(&m_stcWm8731Config, u8Volume);
}

/**
 * @brief  De-Initializes the I2S and DMA for the board.
 * @param  None
 * @retval None
 */
__WEAKDEF void BSP_I2S_DeInit(void)
{
    (void)DMA_ChCmd(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SD_CH, DISABLE);
    (void)DMA_ChCmd(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SDIN_CH, DISABLE);
    I2S_FuncCmd(BSP_WM8731_I2S_UNIT, I2S_FUNC_RX | I2S_FUNC_TX, DISABLE);
    I2S_SWReset(BSP_WM8731_I2S_UNIT, I2S_RST_TYPE_ALL);
}

/**
 * @brief  Initializes the I2S and DMA for the board.
 * @param  [in] u32AudioFreq            The audio frequency
 * @retval int32_t:
 *           - LL_OK: Initializes success
 *           - LL_ERR_UNINIT: Initializes DMA failed
 *           - LL_ERR_INVD_PARAM: I2S Invalid parameter
 *           - LL_ERR: I2S Set frequency failed
 */
__WEAKDEF int32_t BSP_I2S_Init(uint32_t u32AudioFreq)
{
    stc_dma_init_t stcDmaInit;
    stc_dma_llp_init_t stcDmaLlpInit;
    stc_i2s_init_t stcI2sInit;

    /* I2S pins configuration */
    GPIO_SetFunc(BSP_WM8731_I2S_CK_PORT,   BSP_WM8731_I2S_CK_PIN,   BSP_WM8731_I2S_CK_FUNC);
    GPIO_SetFunc(BSP_WM8731_I2S_WS_PORT,   BSP_WM8731_I2S_WS_PIN,   BSP_WM8731_I2S_WS_FUNC);
    GPIO_SetFunc(BSP_WM8731_I2S_SD_PORT,   BSP_WM8731_I2S_SD_PIN,   BSP_WM8731_I2S_SD_FUNC);
    GPIO_SetFunc(BSP_WM8731_I2S_SDIN_PORT, BSP_WM8731_I2S_SDIN_PIN, BSP_WM8731_I2S_SDIN_FUNC);
    GPIO_SetFunc(BSP_WM8731_I2S_EXCK_PORT, BSP_WM8731_I2S_EXCK_PIN, BSP_WM8731_I2S_EXCK_FUNC);

    /* I2S DMA configuration */
    FCG_Fcg0PeriphClockCmd(BSP_WM8731_DMA_CLK, ENABLE);
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn      = DMA_INT_DISABLE;
    stcDmaInit.u32BlockSize  = BSP_WM8731_DMA_BLK_SIZE;
    stcDmaInit.u32TransCount = BSP_WM8731_DMA_BLK_LEN;
    stcDmaInit.u32DataWidth  = DMA_DATAWIDTH_32BIT;
    (void)DMA_LlpStructInit(&stcDmaLlpInit);
    stcDmaLlpInit.u32State = DMA_LLP_ENABLE;
    stcDmaLlpInit.u32Mode  = DMA_LLP_WAIT;

    /* Configure I2S DMA transfer */
    stcDmaInit.u32DestAddr   = (uint32_t)(&BSP_WM8731_I2S_UNIT->TXBUF);
    stcDmaInit.u32SrcAddr    = (uint32_t)(&m_u32AudioDataBuf[0U][0U]);
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_FIX;
    if (LL_OK != DMA_Init(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SD_CH, &stcDmaInit)) {
        return LL_ERR_UNINIT;
    }
    m_stcTxLlpDesc[0U].SARx   = (uint32_t)(&m_u32AudioDataBuf[1U][0U]);
    m_stcTxLlpDesc[0U].DARx   = (uint32_t)(&BSP_WM8731_I2S_UNIT->TXBUF);
    m_stcTxLlpDesc[0U].DTCTLx = (BSP_WM8731_DMA_BLK_LEN << DMA_DTCTL_CNT_POS) | (BSP_WM8731_DMA_BLK_SIZE << DMA_DTCTL_BLKSIZE_POS);
    m_stcTxLlpDesc[0U].LLPx   = (uint32_t)&m_stcTxLlpDesc[1U];
    m_stcTxLlpDesc[0U].CHCTLx = DMA_SRC_ADDR_INC | DMA_DEST_ADDR_FIX | DMA_DATAWIDTH_32BIT |  \
                                DMA_LLP_ENABLE   | DMA_LLP_WAIT;
    m_stcTxLlpDesc[1U].SARx   = (uint32_t)(&m_u32AudioDataBuf[0U][0U]);
    m_stcTxLlpDesc[1U].DARx   = (uint32_t)(&BSP_WM8731_I2S_UNIT->TXBUF);
    m_stcTxLlpDesc[1U].DTCTLx = (BSP_WM8731_DMA_BLK_LEN << DMA_DTCTL_CNT_POS) | (BSP_WM8731_DMA_BLK_SIZE << DMA_DTCTL_BLKSIZE_POS);
    m_stcTxLlpDesc[1U].LLPx   = (uint32_t)&m_stcTxLlpDesc[0U];
    m_stcTxLlpDesc[1U].CHCTLx = DMA_SRC_ADDR_INC | DMA_DEST_ADDR_FIX | DMA_DATAWIDTH_32BIT |  \
                                DMA_LLP_ENABLE   | DMA_LLP_WAIT;
    stcDmaLlpInit.u32Addr  = (uint32_t)&m_stcTxLlpDesc[0U];
    if (LL_OK != DMA_LlpInit(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SD_CH, &stcDmaLlpInit)) {
        return LL_ERR_UNINIT;
    }
    AOS_SetTriggerEventSrc(BSP_WM8731_DMA_SD_TRIG_CH, BSP_WM8731_I2S_SD_EVT_SRC);

    /* Configure I2S DMA receive */
    stcDmaInit.u32DestAddr   = (uint32_t)(&m_u32AudioDataBuf[1U][0U]);
    stcDmaInit.u32SrcAddr    = (uint32_t)(&BSP_WM8731_I2S_UNIT->RXBUF);
    stcDmaInit.u32SrcAddrInc = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestAddrInc = DMA_DEST_ADDR_INC;
    if (LL_OK != DMA_Init(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SDIN_CH, &stcDmaInit)) {
        return LL_ERR_UNINIT;
    }
    m_stcRxLlpDesc[0U].SARx   = (uint32_t)(&BSP_WM8731_I2S_UNIT->RXBUF);
    m_stcRxLlpDesc[0U].DARx   = (uint32_t)(&m_u32AudioDataBuf[0U][0U]);
    m_stcRxLlpDesc[0U].DTCTLx = (BSP_WM8731_DMA_BLK_LEN << DMA_DTCTL_CNT_POS) | (BSP_WM8731_DMA_BLK_SIZE << DMA_DTCTL_BLKSIZE_POS);
    m_stcRxLlpDesc[0U].LLPx   = (uint32_t)&m_stcRxLlpDesc[1U];
    m_stcRxLlpDesc[0U].CHCTLx = DMA_SRC_ADDR_FIX | DMA_DEST_ADDR_INC | DMA_DATAWIDTH_32BIT |  \
                                DMA_LLP_ENABLE   | DMA_LLP_WAIT;
    m_stcRxLlpDesc[1U].SARx   = (uint32_t)(&BSP_WM8731_I2S_UNIT->RXBUF);
    m_stcRxLlpDesc[1U].DARx   = (uint32_t)(&m_u32AudioDataBuf[1U][0U]);
    m_stcRxLlpDesc[1U].DTCTLx = (BSP_WM8731_DMA_BLK_LEN << DMA_DTCTL_CNT_POS) | (BSP_WM8731_DMA_BLK_SIZE << DMA_DTCTL_BLKSIZE_POS);
    m_stcRxLlpDesc[1U].LLPx   = (uint32_t)&m_stcRxLlpDesc[0U];
    m_stcRxLlpDesc[1U].CHCTLx = DMA_SRC_ADDR_FIX | DMA_DEST_ADDR_INC | DMA_DATAWIDTH_32BIT |  \
                                DMA_LLP_ENABLE   | DMA_LLP_WAIT;
    stcDmaLlpInit.u32Addr  = (uint32_t)&m_stcRxLlpDesc[0U];
    if (LL_OK != DMA_LlpInit(BSP_WM8731_DMA_UNIT, BSP_WM8731_DMA_SDIN_CH, &stcDmaLlpInit)) {
        return LL_ERR_UNINIT;
    }
    AOS_SetTriggerEventSrc(BSP_WM8731_DMA_SDIN_TRIG_CH, BSP_WM8731_I2S_SDIN_EVT_SRC);

    /* Enable DMA unit */
    DMA_Cmd(BSP_WM8731_DMA_UNIT, ENABLE);

    /* Set stop flag */
    m_u8Wm8731StopFlag = 1U;

    /* I2S configuration */
    FCG_Fcg1PeriphClockCmd(BSP_WM8731_I2S_CLK, ENABLE);
    (void)I2S_StructInit(&stcI2sInit);
    stcI2sInit.u32ClockSrc         = I2S_CLK_SRC_EXT;
    stcI2sInit.u32Mode             = I2S_MD_MASTER;
    stcI2sInit.u32Protocol         = I2S_PROTOCOL_PHILLIPS;
    stcI2sInit.u32TransMode        = I2S_TRANS_MD_FULL_DUPLEX;
    stcI2sInit.u32AudioFreq        = u32AudioFreq;
    stcI2sInit.u32ChWidth          = I2S_CH_LEN_32BIT;
    stcI2sInit.u32DataWidth        = I2S_DATA_LEN_32BIT;
    stcI2sInit.u32TransFIFOLevel   = I2S_TRANS_LVL4;
    stcI2sInit.u32ReceiveFIFOLevel = I2S_RECEIVE_LVL4;
    return I2S_Init(BSP_WM8731_I2S_UNIT, &stcI2sInit);
}

/**
 * @}
 */

#endif /* (BSP_WM8731_ENABLE && BSP_EV_HC32F4A0_LQFP176) */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/******************************************************************************
 * EOF (not truncated)
 *****************************************************************************/
