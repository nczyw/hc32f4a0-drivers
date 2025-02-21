/**
 *******************************************************************************
 * @file  ev_hc32f4a0_lqfp176_rtl8201.c
 * @brief This file provides configure functions for rtl8201 of the board
 *        EV_HC32F4A0_LQFP176.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2023-09-30       CDT             First version
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
#include "ev_hc32f4a0_lqfp176_rtl8201.h"
#include "ev_hc32f4a0_lqfp176_tca9539.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @addtogroup EV_HC32F4A0_LQFP176
 * @{
 */

/**
 * @defgroup EV_HC32F4A0_LQFP176_RTL8201 EV_HC32F4A0_LQFP176 RTL8201
 * @{
 */

#if ((BSP_RTL8201_ENABLE == DDL_ON) && (BSP_EV_HC32F4A0_LQFP176 == BSP_EV_HC32F4XX))

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
static int32_t BSP_RTL8201_SMI_Init(void);
static int32_t BSP_RTL8201_SMI_ReadReg(uint16_t u16Reg, uint16_t *pu16Value);
static int32_t BSP_RTL8201_SMI_WriteReg(uint16_t u16Reg, uint16_t u16Value);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/
/**
 * @defgroup EV_HC32F4A0_LQFP176_RTL8201_Local_Variables EV_HC32F4A0_LQFP176 RTL8201 Local Variables
 * @{
 */
const static stc_rtl8201_ll_t m_stcRtl8201Config = {
    .Init = BSP_RTL8201_SMI_Init,
    .ReadReg = BSP_RTL8201_SMI_ReadReg,
    .WriteReg = BSP_RTL8201_SMI_WriteReg,
    .GetTick = SysTick_GetTick,
};

/**
 * @}
 */

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @defgroup EV_HC32F4A0_LQFP176_RTL8201_Local_Functions EV_HC32F4A0_LQFP176 RTL8201 Local Functions
 * @{
 */
/**
 * @brief  Reset RTL8201 in hardware mode.
 * @param  None
 * @retval None
 */
static void RTL8201_HW_Reset(void)
{
    BSP_IO_ConfigPortPin(ETH_RST_PORT, ETH_RST_PIN, EIO_DIR_OUT);
    BSP_IO_WritePortPin(ETH_RST_PORT, ETH_RST_PIN, EIO_PIN_RESET);
    SysTick_Delay(RTL8201_HW_RST_DELAY);
    BSP_IO_WritePortPin(ETH_RST_PORT, ETH_RST_PIN, EIO_PIN_SET);
    SysTick_Delay(RTL8201_HW_RST_DELAY);
}

/**
 * @brief  Initializes SMI port.
 * @param  None
 * @retval None
 */
static void BSP_RTL8201_SMI_PortInit(void)
{
    stc_gpio_init_t stcGpioInit;

    (void)GPIO_StructInit(&stcGpioInit);
    stcGpioInit.u16PinDrv = PIN_HIGH_DRV;
    (void)GPIO_Init(BSP_RTL8201_SMI_MDIO_PORT, BSP_RTL8201_SMI_MDIO_PIN, &stcGpioInit);
    (void)GPIO_Init(BSP_RTL8201_SMI_MDC_PORT, BSP_RTL8201_SMI_MDC_PIN, &stcGpioInit);
    GPIO_SetFunc(BSP_RTL8201_SMI_MDIO_PORT, BSP_RTL8201_SMI_MDIO_PIN, BSP_RTL8201_SMI_FUNC);
    GPIO_SetFunc(BSP_RTL8201_SMI_MDC_PORT, BSP_RTL8201_SMI_MDC_PIN, BSP_RTL8201_SMI_FUNC);
}

/**
 * @brief  Initializes the BSP RTL8201.
 * @param  None
 * @retval int32_t:
 *           - LL_OK: Init success
 */
static int32_t BSP_RTL8201_SMI_Init(void)
{
    int32_t i32Ret = LL_OK;

    /* Configure SMI */
    BSP_RTL8201_SMI_PortInit();
    /* Configure the ETH MDC Clock */
    ETH_MAC_SetMdcClock();

    return i32Ret;
}

/**
 * @brief  Read RTL8201 register.
 * @param  [in] u16Reg                  PHY register address
 * @param  [out] pu16Value              Pointer to PHY register value
 * @retval int32_t:
 *           - LL_OK: Read register success
 *           - LL_ERR_TIMEOUT: Read timeout
 */
static int32_t BSP_RTL8201_SMI_ReadReg(uint16_t u16Reg, uint16_t *pu16Value)
{
    return ETH_PHY_ReadReg(BSP_RTL8201_DEV_ADDR, u16Reg, pu16Value);
}

/**
 * @brief  Write RTL8201 register.
 * @param  [in] u16Reg                  PHY register address
 * @param  [in] u16Value                PHY register value to write
 * @retval int32_t:
 *           - LL_OK: Write register success
 *           - LL_ERR_TIMEOUT: Write timeout
 */
static int32_t BSP_RTL8201_SMI_WriteReg(uint16_t u16Reg, uint16_t u16Value)
{
    return ETH_PHY_WriteReg(BSP_RTL8201_DEV_ADDR, u16Reg, u16Value);
}
/**
 * @}
 */

/**
 * @defgroup EV_HC32F4A0_LQFP176_RTL8201_Global_Functions EV_HC32F4A0_LQFP176 RTL8201 Global Functions
 * @{
 */
/**
 * @brief  Initializes the BSP RTL8201.
 * @param  None
 * @retval int32_t:
 *           - LL_OK: Init success
 *           - LL_ERR: Init error
 */
int32_t BSP_RTL8201_Init(void)
{
    int32_t i32Ret;

    RTL8201_HW_Reset();
    i32Ret = RTL8201_Init(&m_stcRtl8201Config);
    if (LL_OK == i32Ret) {
        i32Ret = RTL8201_SW_Reset(&m_stcRtl8201Config);
    }

    return i32Ret;
}

/**
 * @brief  Reset the RTL8201 by specified mode.
 * @param  [in] u16Mode                 RTL8201 reset mode
 *         This parameter can be one of the following values:
 *           @arg BSP_RTL8201_RST_MD_SW:    Half duplex mode
 *           @arg BSP_RTL8201_RST_MD_HW:    Full duplex mode
 * @retval int32_t:
 *           - LL_OK: Reset success
 *           - LL_ERR: Reset error
 */
int32_t BSP_RTL8201_Reset(uint16_t u16Mode)
{
    int32_t i32Ret = LL_OK;

    if (BSP_RTL8201_RST_MD_SW == u16Mode) {
        i32Ret = RTL8201_SW_Reset(&m_stcRtl8201Config);
    } else {
        RTL8201_HW_Reset();
    }

    return i32Ret;
}

/**
 * @brief  Enable or disable auto negotiation of RTL8201.
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval int32_t:
 *           - LL_OK: Auto negotiation success
 *           - LL_ERR_TIMEOUT: auto negotiation timeout
 *           - LL_ERR: Communication error
 */
int32_t BSP_RTL8201_AutoNegoCmd(en_functional_state_t enNewState)
{
    return RTL8201_AutoNegoCmd(&m_stcRtl8201Config, enNewState);
}

/**
 * @brief  Set link mode of RTL8201.
 * @param  [in] u32Mode                 Link mode
 *         This parameter can be one of the following values:
 *           @arg RTL8201_LINK_100M_FULL_DUPLEX
 *           @arg RTL8201_LINK_100M_HALF_DUPLEX
 *           @arg RTL8201_LINK_10M_FULL_DUPLEX
 *           @arg RTL8201_LINK_10M_HALF_DUPLEX
 * @retval int32_t:
 *           - LL_OK: Set link mode success
 *           - LL_ERR_TIMEOUT: Set link mode timeout
 *           - LL_ERR: Communication error
 */
int32_t BSP_RTL8201_SetLinkMode(uint32_t u32Mode)
{
    return RTL8201_SetLinkMode(&m_stcRtl8201Config, (uint16_t)u32Mode);
}

/**
 * @brief  Set led mode of RTL8201.
 * @param  [in] u32Mode                 Traditional led mode
 *         This parameter can be one of the following values:
 *           @arg RTL8201_LED0_ACTALL_LDE1_LINK100
 *           @arg RTL8201_LED0_LINKALL_ACTALL_LED1_LINK100
 *           @arg RTL8201_LED0_LINK10_ACTALL_LED1_LINK100
 *           @arg RTL8201_LED0_LINK10_ACT10_LED1_LINK100_ACT100
 * @retval int32_t:
 *           - LL_OK: Set led mode success
 *           - LL_ERR_TIMEOUT: Set led mode timeout
 *           - LL_ERR: Communication error
 */
int32_t BSP_RTL8201_SetLedMode(uint32_t u32Mode)
{
    int32_t i32Ret;

    __disable_irq();
    i32Ret = RTL8201_SelectPage(&m_stcRtl8201Config, RTL8201_PAGE_ADDR_7);
    if (LL_OK == i32Ret) {
        i32Ret = RTL8201_SetTradLedMode(&m_stcRtl8201Config, u32Mode);
    }
    (void)RTL8201_SelectPage(&m_stcRtl8201Config, RTL8201_PAGE_ADDR_0);
    __enable_irq();

    return i32Ret;
}

/**
 * @brief  Enable or disable power save mode of RTL8201.
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval int32_t:
 *           - LL_OK: Enable or disable power save mode success
 *           - LL_ERR: Communication error
 */
int32_t BSP_RTL8201_PowerSaveModeCmd(en_functional_state_t enNewState)
{
    return RTL8201_PowerSaveModeCmd(&m_stcRtl8201Config, enNewState);
}

/**
 * @brief  Enable or disable RTL8201 interrupt.
 * @param  [in] u16IntType              RTL8201 interrupt type
 *         This parameter can be one of the following values:
 *           @arg RTL8201_INT_AUT_NEGO_ERR
 *           @arg RTL8201_INT_DUPLEX_MD_CHANGE
 *           @arg RTL8201_INT_LINK_STAT_CHANGE
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval int32_t:
 *           - LL_OK: Enable or disable interrupt success
 *           - LL_ERR: Communication error
 */
int32_t BSP_RTL8201_IntCmd(uint16_t u16IntType, en_functional_state_t enNewState)
{
    int32_t i32Ret;

    __disable_irq();
    i32Ret = RTL8201_SelectPage(&m_stcRtl8201Config, RTL8201_PAGE_ADDR_7);
    if (LL_OK == i32Ret) {
        i32Ret = RTL8201_IntCmd(&m_stcRtl8201Config, u16IntType, enNewState);
    }
    (void)RTL8201_SelectPage(&m_stcRtl8201Config, RTL8201_PAGE_ADDR_0);
    __enable_irq();

    return i32Ret;
}

/**
 * @brief  Enable or disable RTL8201 loopback.
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval int32_t:
 *           - LL_OK: Enable or disable loopback success
 *           - LL_ERR: Communication error
 */
int32_t BSP_RTL8201_LoopBackCmd(en_functional_state_t enNewState)
{
    return RTL8201_LoopBackCmd(&m_stcRtl8201Config, enNewState);
}

/**
 * @brief  Get link status of RTL8201.
 * @param  [out] pu16LinkStatus         The link status
 *         This parameter can be one of the following values:
 *           @arg RTL8201_LINK_DOWN
 *           @arg RTL8201_LINK_AUTO_NEGO_NOT_CPLT
 *           @arg RTL8201_LINK_100M_FULL_DUPLEX
 *           @arg RTL8201_LINK_100M_HALF_DUPLEX
 *           @arg RTL8201_LINK_10M_FULL_DUPLEX
 *           @arg RTL8201_LINK_10M_HALF_DUPLEX
 * @retval int32_t:
 *           - LL_OK: Get link status success
 *           - LL_ERR: Communication error
 */
int32_t BSP_RTL8201_GetLinkStatus(uint16_t *pu16LinkStatus)
{
    return RTL8201_GetLinkStatus(&m_stcRtl8201Config, pu16LinkStatus);
}

/**
 * @brief  Get interrupt status of RTL8201.
 * @param  [in] u16IntType              Interrupt type
 *         This parameter can be one of the following values:
 *           @arg RTL8201_INT_AUT_NEGO_ERR
 *           @arg RTL8201_INT_SPEED_MD_CHANGE
 *           @arg RTL8201_INT_DUPLEX_MD_CHANGE
 *           @arg RTL8201_INT_LINK_STAT_CHANGE
 * @param  [out] penStatus              An @ref en_flag_status_t enumeration value
 * @retval int32_t:
 *           - LL_OK: Get interrupt status success
 *           - LL_ERR: Communication error
 */
int32_t BSP_RTL8201_GetIntStatus(uint16_t u16IntType, en_flag_status_t *penStatus)
{
    return RTL8201_GetIntStatus(&m_stcRtl8201Config, u16IntType, penStatus);
}

/**
 * @}
 */

#endif /* (BSP_RTL8201_ENABLE && BSP_EV_HC32F4A0_LQFP176) */

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
