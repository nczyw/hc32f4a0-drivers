/**
 *******************************************************************************
 * @file  rtl8201.c
 * @brief This midware file provides firmware functions to manage the phy
 *        component library for rtl8201.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2024-08-31       CDT             First version
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
#include "rtl8201.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @addtogroup Components
 * @{
 */

/**
 * @defgroup RTL8201 PHY RTL8201
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup RTL8201_Local_Macros RTL8201 Local Macros
 * @{
 */
#define RTL8201_R0_MSK                  (0xFFC0U)
#define RTL8201_R0_DEFAULT_VALUE        (0x3100U)
#define RTL8201_R1_MSK                  (0xF87FU)
#define RTL8201_R1_DEFAULT_VALUE        (0x7849U)

/**
 * @}
 */
/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/


/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/
/**
 * @defgroup RTL8201_Local_Functions RTL8201 Local Functions
 * @{
 */
/**
 * @brief  Switch basic control register value to macro define.
 * @param  [in] u16Value             The basic control register value only bit8 & bit13
 * @retval uint16_t:
 *           - RTL8201_LINK_100M_FULL_DUPLEX
 *           - RTL8201_LINK_100M_HALF_DUPLEX
 *           - RTL8201_LINK_10M_FULL_DUPLEX
 *           - RTL8201_LINK_10M_HALF_DUPLEX
 */
static uint16_t RTL8201_LinkStatus(uint16_t u16Value)
{
    uint16_t u16LinkStatus = 0U;
    switch (u16Value) {
        case RTL8201_FULL_DUPLEX_100M:
            u16LinkStatus = RTL8201_LINK_100M_FULL_DUPLEX;
            break;
        case RTL8201_HALF_DUPLEX_100M:
            u16LinkStatus = RTL8201_LINK_100M_HALF_DUPLEX;
            break;
        case RTL8201_FULL_DUPLEX_10M:
            u16LinkStatus = RTL8201_LINK_10M_FULL_DUPLEX;
            break;
        case RTL8201_HALF_DUPLEX_10M:
            u16LinkStatus = RTL8201_LINK_10M_HALF_DUPLEX;
            break;
        default:
            break;
    }
    return u16LinkStatus;
}

/**
 * @}
 */

/**
 * @defgroup RTL8201_Global_Functions RTL8201 Global Functions
 * @{
 */
/**
 * @brief  Init RTL8201.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @retval int32_t:
 *           - LL_OK: Init success
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is NULL
 *           - LL_ERR: Init error
 */
int32_t RTL8201_Init(const stc_rtl8201_ll_t *pstcRtl8201LL)
{
    int32_t i32Ret;
    if (NULL == pstcRtl8201LL) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->Init();
    }

    return i32Ret;
}


/**
 * @brief  SOftware reset RTL8201.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @retval int32_t:
 *           - LL_OK: Reset success
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is NULL
 *           - LL_ERR: Reset error
 */
int32_t RTL8201_SW_Reset(const stc_rtl8201_ll_t *pstcRtl8201LL)
{
    int32_t i32Ret;
    uint16_t u16Reg0Value = 0U;
    uint16_t u16Reg1Value = 0U;
    uint32_t u32TickStart;

    if (NULL == pstcRtl8201LL) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->WriteReg(RTL8201_R0_BCR, RTL8201_SOFT_RESET);
        if (LL_OK == i32Ret) {
            /* To ensure reset complete */
            u32TickStart = pstcRtl8201LL->GetTick();
            while ((RTL8201_R0_DEFAULT_VALUE != (u16Reg0Value & RTL8201_R0_MSK)) && \
                   (RTL8201_R1_DEFAULT_VALUE != (u16Reg1Value & RTL8201_R1_MSK))) {
                /* Read R0 */
                (void)pstcRtl8201LL->ReadReg(RTL8201_R0_BCR, &u16Reg0Value);
                /* Read R1 */
                (void)pstcRtl8201LL->ReadReg(RTL8201_R1_BSR, &u16Reg1Value);
                if ((pstcRtl8201LL->GetTick() - u32TickStart) > RTL8201_SW_RST_DELAY) {
                    i32Ret = LL_ERR;
                    break;
                }
            }
        }
    }

    return i32Ret;
}

/**
 * @brief  Get RTL8201 link mode.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @param  [out] pu16LinkStatus         The link status
 *   @arg  RTL8201_LINK_DOWN
 *   @arg  RTL8201_LINK_AUTO_NEGO_NOT_CPLT
 *   @arg  RTL8201_LINK_100M_FULL_DUPLEX
 *   @arg  RTL8201_LINK_100M_HALF_DUPLEX
 *   @arg  RTL8201_LINK_10M_FULL_DUPLEX
 *   @arg  RTL8201_LINK_10M_HALF_DUPLEX
 * @retval uint32_t:
 *           - LL_OK: Get link mode success
 *           - LL_ERR: Communication error
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is null
 */
int32_t RTL8201_GetLinkStatus(const stc_rtl8201_ll_t *pstcRtl8201LL, uint16_t *pu16LinkStatus)
{
    int32_t i32Ret;
    uint16_t u16Value;

    if ((NULL == pstcRtl8201LL) || (NULL == pu16LinkStatus)) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->ReadReg(RTL8201_R1_BSR, &u16Value);
        if (LL_OK == i32Ret) {
            /* To judge whether establish valid link */
            if (RTL8201_LINK_STAT != (u16Value & RTL8201_LINK_STAT)) {
                /* No valid link */
                *pu16LinkStatus = RTL8201_LINK_DOWN;
            } else {
                /* Valid link */
                if (LL_OK == pstcRtl8201LL->ReadReg(RTL8201_R0_BCR, &u16Value)) {
                    /* To judge whether enable auto-negotiation */
                    if (RTL8201_AUTO_NEGO == (u16Value & RTL8201_AUTO_NEGO)) {
                        /* Auto negotiation */
                        if (LL_OK == pstcRtl8201LL->ReadReg(RTL8201_R1_BSR, &u16Value)) {
                            /* To judge whether auto-negotiation complete */
                            if (RTL8201_AUTO_NEGO_CPLT != (u16Value & RTL8201_AUTO_NEGO_CPLT)) {
                                /* auto-negotiation not complete*/
                                *pu16LinkStatus = RTL8201_LINK_AUTO_NEGO_NOT_CPLT;
                            } else {
                                /* auto-negotiation complete*/
                                if (LL_OK == pstcRtl8201LL->ReadReg(RTL8201_R0_BCR, &u16Value)) {
                                    *pu16LinkStatus = RTL8201_LinkStatus(u16Value & RTL8201_FULL_DUPLEX_100M);
                                } else {
                                    i32Ret = LL_ERR;
                                }
                            }
                        } else {
                            i32Ret = LL_ERR;
                        }
                    } else {
                        /* Not auto negotiation */
                        *pu16LinkStatus = RTL8201_LinkStatus(u16Value & RTL8201_FULL_DUPLEX_100M);
                    }
                } else {
                    i32Ret = LL_ERR;
                }
                if (0U == *pu16LinkStatus) {
                    i32Ret = LL_ERR;
                }
            }
        }
    }

    return i32Ret;
}

/**
 * @brief  Set RTL8201 link mode.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @param  [in] u16Mode                 specifies the link mode
 *   @arg  RTL8201_LINK_100M_FULL_DUPLEX
 *   @arg  RTL8201_LINK_100M_HALF_DUPLEX
 *   @arg  RTL8201_LINK_10M_FULL_DUPLEX
 *   @arg  RTL8201_LINK_10M_HALF_DUPLEX
 * @retval int32_t:
 *           - LL_OK: Set link mode success
 *           - LL_ERR: Communication error
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is null
 */
int32_t RTL8201_SetLinkMode(const stc_rtl8201_ll_t *pstcRtl8201LL, uint16_t u16Mode)
{
    int32_t i32Ret;
    uint16_t u16Value;

    if (NULL == pstcRtl8201LL) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->ReadReg(RTL8201_R0_BCR, &u16Value);
        if (LL_OK == i32Ret) {
            /* Set the register value */
            switch (u16Mode) {
                case RTL8201_LINK_100M_FULL_DUPLEX:
                    SET_REG16_BIT(u16Value, RTL8201_FULL_DUPLEX_100M);
                    break;
                case RTL8201_LINK_100M_HALF_DUPLEX:
                    MODIFY_REG16(u16Value, RTL8201_FULL_DUPLEX_100M, RTL8201_HALF_DUPLEX_100M);
                    break;
                case RTL8201_LINK_10M_FULL_DUPLEX:
                    MODIFY_REG16(u16Value, RTL8201_FULL_DUPLEX_100M, RTL8201_FULL_DUPLEX_10M);
                    break;
                case RTL8201_LINK_10M_HALF_DUPLEX:
                    CLR_REG16_BIT(u16Value, RTL8201_FULL_DUPLEX_100M);
                    break;
                default:
                    i32Ret = LL_ERR_INVD_PARAM;
                    break;
            }
            if (LL_OK == i32Ret) {
                /* set link mode */
                if (LL_OK != pstcRtl8201LL->WriteReg(RTL8201_R0_BCR, u16Value)) {
                    i32Ret = LL_ERR;
                }
            }
        }
    }

    return i32Ret;
}

/**
 * @brief  Set RTL8201 traditional led mode.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @param  [in] u32Mode                 specifies the traditional led mode
 *   @arg  RTL8201_LED0_ACTALL_LDE1_LINK100
 *   @arg  RTL8201_LED0_LINKALL_ACTALL_LED1_LINK100
 *   @arg  RTL8201_LED0_LINK10_ACTALL_LED1_LINK100
 *   @arg  RTL8201_LED0_LINK10_ACT10_LED1_LINK100_ACT100
 * @retval int32_t:
 *           - LL_OK: Set traditional led mode success
 *           - LL_ERR: Communication error
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is null
 */
int32_t RTL8201_SetTradLedMode(const stc_rtl8201_ll_t *pstcRtl8201LL, uint32_t u32Mode)
{
    int32_t i32Ret;
    uint16_t u16Value;

    if (NULL == pstcRtl8201LL) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->ReadReg(RTL8201_R19_P7_IWLFR, &u16Value);
        if (LL_OK == i32Ret) {
            MODIFY_REG16(u16Value, RTL8201_LED_SEL, u32Mode);
            if (LL_OK != pstcRtl8201LL->WriteReg(RTL8201_R19_P7_IWLFR, u16Value)) {
                i32Ret = LL_ERR;
            }
        }
    }

    return i32Ret;
}

/**
 * @brief  Select RTL8201 page.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @param  [in] u8Page                  Specifies the page to selected (0~0xFF)
 * @retval int32_t:
 *           - LL_OK: Select page success
 *           - LL_ERR: Communication error
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is null
 */
int32_t RTL8201_SelectPage(const stc_rtl8201_ll_t *pstcRtl8201LL, uint8_t u8Page)
{
    int32_t i32Ret;
    if (NULL == pstcRtl8201LL) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->WriteReg(RTL8201_R31_PSR, (uint16_t)u8Page);
        if (LL_OK != i32Ret) {
            i32Ret = LL_ERR;
        }
    }

    return i32Ret;
}

/**
 * @brief  Enable or disable RTL8201 auto negotiation.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval int32_t:
 *           - LL_OK: Auto negotiation success
 *           - LL_ERR_TIMEOUT: auto negotiation timeout
 *           - LL_ERR: Communication error
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is null
 */
int32_t RTL8201_AutoNegoCmd(const stc_rtl8201_ll_t *pstcRtl8201LL, en_functional_state_t enNewState)
{
    int32_t i32Ret;
    uint16_t u16Value;
    uint32_t u32TickStart;

    if (NULL == pstcRtl8201LL) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->ReadReg(RTL8201_R0_BCR, &u16Value);
        if (LL_OK == i32Ret) {
            if (ENABLE == enNewState) {
                SET_REG16_BIT(u16Value, RTL8201_AUTO_NEGO);
            } else {
                CLR_REG16_BIT(u16Value, RTL8201_AUTO_NEGO);
            }
            if (LL_OK != pstcRtl8201LL->WriteReg(RTL8201_R0_BCR, u16Value)) {
                i32Ret = LL_ERR;
            } else {
                if (ENABLE == enNewState) {
                    /* Wait until the auto-negotiation will be completed */
                    u32TickStart = pstcRtl8201LL->GetTick();
                    u16Value = 0U;
                    while (RTL8201_AUTO_NEGO_CPLT != (u16Value & RTL8201_AUTO_NEGO_CPLT)) {
                        (void)pstcRtl8201LL->ReadReg(RTL8201_R1_BSR, &u16Value);
                        /* Check for the Timeout (3s) */
                        if ((pstcRtl8201LL->GetTick() - u32TickStart) > 3000U) {
                            i32Ret = LL_ERR_TIMEOUT;
                            break;
                        }
                    }
                }
            }
        }
    }

    return i32Ret;
}

/**
 * @brief  Enable or disable RTL8201 loopback.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval int32_t:
 *           - LL_OK: Enable or disable loopback success
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is null
 *           - LL_ERR: Communication error
 */
int32_t RTL8201_LoopBackCmd(const stc_rtl8201_ll_t *pstcRtl8201LL, en_functional_state_t enNewState)
{
    int32_t i32Ret;
    uint16_t u16Value;

    if (NULL == pstcRtl8201LL) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->ReadReg(RTL8201_R0_BCR, &u16Value);
        if (LL_OK == i32Ret) {
            if (ENABLE == enNewState) {
                SET_REG16_BIT(u16Value, RTL8201_LOOPBACK);
            } else {
                CLR_REG16_BIT(u16Value, RTL8201_LOOPBACK);
            }
            if (LL_OK != pstcRtl8201LL->WriteReg(RTL8201_R0_BCR, u16Value)) {
                i32Ret = LL_ERR;
            }
        }
    }

    return i32Ret;
}

/**
 * @brief  Enable or disable RTL8201 power save mode.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval int32_t:
 *           - LL_OK: Enable or disable power save mode success
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is null
 *           - LL_ERR: Communication error
 */
int32_t RTL8201_PowerSaveModeCmd(const stc_rtl8201_ll_t *pstcRtl8201LL, en_functional_state_t enNewState)
{
    int32_t i32Ret;

    if (NULL == pstcRtl8201LL) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->WriteReg(RTL8201_R24_PSMR, ((uint16_t)enNewState << 15U));
    }

    return i32Ret;
}

/**
 * @brief  Enable or disable RTL8201 power down mode.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value.
 * @retval int32_t:
 *           - LL_OK: Enable or disable power down mode success
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is null
 *           - LL_ERR: Communication error
 */
int32_t RTL8201_PowerDownModeCmd(const stc_rtl8201_ll_t *pstcRtl8201LL, en_functional_state_t enNewState)
{
    int32_t i32Ret;
    uint16_t u16Value;

    if (NULL == pstcRtl8201LL) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->ReadReg(RTL8201_R0_BCR, &u16Value);
        if (LL_OK == i32Ret) {
            if (ENABLE == enNewState) {
                SET_REG16_BIT(u16Value, RTL8201_POWER_DOWN);
            } else {
                CLR_REG16_BIT(u16Value, RTL8201_POWER_DOWN);
            }
            if (LL_OK != pstcRtl8201LL->WriteReg(RTL8201_R0_BCR, u16Value)) {
                i32Ret = LL_ERR;
            }
        }
    }

    return i32Ret;
}

/**
 * @brief  Enable or disable the specifies interrupt.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @param  [in] u16IntType              A @ref RTL8201_Interrupt value
 *   @arg  RTL8201_INT_AUT_NEGO_ERR
 *   @arg  RTL8201_INT_DUPLEX_MD_CHANGE
 *   @arg  RTL8201_INT_LINK_STAT_CHANGE
 * @param  [in] enNewState              An @ref en_functional_state_t enumeration value
 * @retval int32_t:
 *           - LL_OK: Enable or disable the interrupt success
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is null
 *           - LL_ERR: Communication error
 */
int32_t RTL8201_IntCmd(const stc_rtl8201_ll_t *pstcRtl8201LL, uint16_t u16IntType, en_functional_state_t enNewState)
{
    int32_t i32Ret;
    uint16_t u16Value;
    uint16_t u16IntValue = 0U;

    if (NULL == pstcRtl8201LL) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        if (RTL8201_INT_LINK_STAT_CHANGE == (u16IntType & RTL8201_INT_LINK_STAT_CHANGE)) {
            u16IntValue |= RTL8201_INT_MSK_LINK_STAT_CHANGE;
        }
        if (RTL8201_INT_DUPLEX_MD_CHANGE == (u16IntType & RTL8201_INT_DUPLEX_MD_CHANGE)) {
            u16IntValue |= RTL8201_INT_MSK_DUPLEX_MD_CHANGE;
        }
        if (RTL8201_INT_AUT_NEGO_ERR == (u16IntType & RTL8201_INT_AUT_NEGO_ERR)) {
            u16IntValue |= RTL8201_INT_MSK_NWAY_ERR;
        }
        if (0U != u16IntValue) {
            i32Ret = pstcRtl8201LL->ReadReg(RTL8201_R19_P7_IWLFR, &u16Value);
            if (LL_OK == i32Ret) {
                if (ENABLE == enNewState) {
                    SET_REG16_BIT(u16Value, u16IntValue);
                } else {
                    CLR_REG16_BIT(u16Value, u16IntValue);
                }
                if (LL_OK != pstcRtl8201LL->WriteReg(RTL8201_R19_P7_IWLFR, u16Value)) {
                    i32Ret = LL_ERR;
                }
            }
        } else {
            i32Ret = LL_ERR_INVD_PARAM;
        }
    }

    return i32Ret;
}

/**
 * @brief  Get the specifies interrupt status.
 * @param  [in] pstcRtl8201LL           Pointer to a @ref stc_rtl8201_ll_t structure
 * @param  [in] u16IntType              A @ref RTL8201_Interrupt value
 *   @arg  RTL8201_INT_AUT_NEGO_ERR
 *   @arg  RTL8201_INT_SPEED_MD_CHANGE
 *   @arg  RTL8201_INT_DUPLEX_MD_CHANGE
 *   @arg  RTL8201_INT_LINK_STAT_CHANGE
 * @param  [out] penStatus              An @ref en_flag_status_t enumeration value
 * @retval int32_t:
 *           - LL_OK: Get the interrupt status success
 *           - LL_ERR_INVD_PARAM: pstcRtl8201LL is null
 *           - LL_ERR: Communication error
 */
int32_t RTL8201_GetIntStatus(const stc_rtl8201_ll_t *pstcRtl8201LL, uint16_t u16IntType, en_flag_status_t *penStatus)
{
    int32_t i32Ret;
    uint16_t u16Value;

    if ((NULL == pstcRtl8201LL) || (NULL == penStatus)) {
        i32Ret = LL_ERR_INVD_PARAM;
    } else {
        i32Ret = pstcRtl8201LL->ReadReg(RTL8201_R30_IISDR, &u16Value);
        if (LL_OK == i32Ret) {
            *penStatus = ((READ_REG16_BIT(u16Value, u16IntType) != 0U) ? SET : RESET);
        }
    }

    return i32Ret;
}

/**
 * @}
 */

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
