/**
 *******************************************************************************
 * @file  rtl8201.h
 * @brief This file contains all the functions prototypes of the phy component
 *        library for rtl8201.
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
#ifndef __RTL8201_H__
#define __RTL8201_H__

/* C binding of definitions if building with C++ compiler */
#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ll_def.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @addtogroup Components
 * @{
 */

/**
 * @addtogroup RTL8201
 * @{
 */

/*******************************************************************************
 * Global type definitions ('typedef')
 ******************************************************************************/
/**
 * @defgroup RTL8201_Global_Types RTL8201 Global Types
 * @{
 */

/**
 * @brief RTL8201 low layer structure definition
 */
typedef struct {
    /* Methods */
    int32_t (*Init)(void);
    int32_t (*ReadReg)(uint16_t u16Reg, uint16_t *u16Value);
    int32_t (*WriteReg)(uint16_t u16Reg, uint16_t u16Value);
    uint32_t (*GetTick)(void);
} stc_rtl8201_ll_t;

/**
 * @}
 */


/*******************************************************************************
 * Global pre-processor symbols/macros ('#define')
 ******************************************************************************/
/**
 * @defgroup RTL8201_Global_Macros RTL8201 Global Macros
 * @{
 */

/**
 * @defgroup RTL8201_Delay RTL8201 Delay
 * @{
 */
#define RTL8201_HW_RST_DELAY                (10U) /*!< For a complete reset, Set PHYRSTB pin to low at least 10ms */
#define RTL8201_SW_RST_DELAY                (20U) /*!< For a software reset, at least 20ms */
#define RTL8201_RD_TIMEOUT                  (10U)
/**
 * @}
 */

/**
 * @defgroup RTL8201_Register RTL8201 Register
 * @{
 */
#define RTL8201_R0_BCR                      (0x00U) /*!< Basic Mode Control Register                        */
#define RTL8201_R1_BSR                      (0x01U) /*!< Basic Mode Status Register                         */
#define RTL8201_R2_PIDT1                    (0x02U) /*!< PHY Identifier Register 1                          */
#define RTL8201_R3_PIDT2                    (0x03U) /*!< PHY Identifier Register 2                          */
#define RTL8201_R4_ANAR                     (0x04U) /*!< Auto-Negotiation Advertisement Register            */
#define RTL8201_R5_ANLPAR                   (0x05U) /*!< Auto-Negotiation Link Partner Ability Register     */
#define RTL8201_R6_ANER                     (0x06U) /*!< Auto-Negotiation Expansion Register                */
#define RTL8201_R13_MACR                    (0x0DU) /*!< MMD Access Control Register                        */
#define RTL8201_R14_MAADR                   (0x0EU) /*!< MMD Access Address Data Register                   */
#define RTL8201_R16_P4_ECER                 (0x10U) /*!< EEE Capability Enable Register                     */
#define RTL8201_R16_P7_RMSR                 (0x10U) /*!< RMII Mode Setting Register                         */
#define RTL8201_R17_P7_CLSR                 (0x11U) /*!< Customized LEDs Setting Register                   */
#define RTL8201_R18_P7_ELER                 (0x12U) /*!< EEE LEDs Enable Register                           */
#define RTL8201_R19_P7_IWLFR                (0x13U) /*!< Interrupt, WOL Enable, and LEDs Function Registers */
#define RTL8201_R20_P7_MTIR                 (0x14U) /*!< MII TX Isolate Register                            */
#define RTL8201_R21_P4_ECR                  (0x15U) /*!< EEE Capability Register                            */
#define RTL8201_R24_P7_SSCR                 (0x18U) /*!< Spread Spectrum Clock Register                     */
#define RTL8201_R24_PSMR                    (0x18U) /*!< Power Saving Mode Register                         */
#define RTL8201_R28_FMLR                    (0x1CU) /*!< Fiber Mode and Loopback Register                   */
#define RTL8201_R30_IISDR                   (0x1EU) /*!< Interrupt Indicators and SNR Display Register      */
#define RTL8201_R31_PSR                     (0x1FU) /*!< Page Select Register                               */
/**
 * @}
 */

/**
 * @defgroup RTL8201_Rage RTL8201 Page Address
 * @{
 */
#define RTL8201_PAGE_ADDR_0                 (0x0000U)   /*!< Page Address 0 (default)               */
#define RTL8201_PAGE_ADDR_4                 (0x0004U)   /*!< Page Address 4                         */
#define RTL8201_PAGE_ADDR_7                 (0x0007U)   /*!< Page Address 7                         */
/**
 * @}
 */

/**
 * @defgroup RTL8201_Control_Register RTL8201 Control Register
 * @{
 */
#define RTL8201_SOFT_RESET                  (0x8000U)   /*!< PHY Soft Reset                         */
#define RTL8201_LOOPBACK                    (0x4000U)   /*!< Select loop-back mode                  */
#define RTL8201_DUPLEX_BIT                  (0x0100U)   /*!< The duplex control bit                 */
#define RTL8201_SPEED_BIT                   (0x2000U)   /*!< The speed control bit                  */
#define RTL8201_FULL_DUPLEX_100M            (0x2100U)   /*!< Set the full-duplex mode at 100 Mb/s   */
#define RTL8201_HALF_DUPLEX_100M            (0x2000U)   /*!< Set the half-duplex mode at 100 Mb/s   */
#define RTL8201_FULL_DUPLEX_10M             (0x0100U)   /*!< Set the full-duplex mode at 10 Mb/s    */
#define RTL8201_HALF_DUPLEX_10M             (0x0000U)   /*!< Set the half-duplex mode at 10 Mb/s    */
#define RTL8201_AUTO_NEGO                   (0x1000U)   /*!< Enable Auto-Negotiation function       */
#define RTL8201_POWER_DOWN                  (0x0800U)   /*!< Select the power down mode             */
#define RTL8201_ISOLATE                     (0x0400U)   /*!< Isolate PHY from MII                   */
#define RTL8201_RESTART_AUTO_NEGO           (0x0200U)   /*!< Restart Auto-Negotiation function      */
#define RTL8201_COLLISION_TEST              (0x0080U)   /*!< Collision test function                */
/**
 * @}
 */

/**
 * @defgroup RTL8201_Status_Register RTL8201 Status Register
 * @{
 */
#define RTL8201_100BASE_T4                  (0x8000U)   /*!< 100Base-T4 support                             */
#define RTL8201_100BASE_TX_FD               (0x4000U)   /*!< 100Base-TX full duplex support                 */
#define RTL8201_100BASE_TX_HD               (0x2000U)   /*!< 100Base-TX half duplex support                 */
#define RTL8201_10BASE_T_FD                 (0x1000U)   /*!< 10Base-T full duplex support                   */
#define RTL8201_10BASE_T_HD                 (0x0800U)   /*!< 10Base-T half duplex support                   */
#define RTL8201_MF_PREAMBLE_SUPPR           (0x0040U)   /*!< Management frames with preamble suppression    */
#define RTL8201_AUTO_NEGO_CPLT              (0x0020U)   /*!< Auto-Negotiation process completed             */
#define RTL8201_LINK_STAT                   (0x0004U)   /*!< Valid link established                         */
#define RTL8201_JABBER_DETECT               (0x0002U)   /*!< Jabber condition detected                      */
/**
 * @}
 */

/**
 * @defgroup RTL8201_Power_Saving_Mode_Register RTL8201 Power Saving Mode Register
 * @{
 */
#define RTL8201_PWR_SAVE_EN                 (0x8000U)   /*!< Enable Power Saving Mode               */
/**
 * @}
 */

/**
 * @defgroup RTL8201_Interrupt RTL8201 Interrupt
 * @{
 */
#define RTL8201_INT_AUT_NEGO_ERR            (0x8000U)   /*!< Auto-Negotiation Error Interrupt       */
#define RTL8201_INT_SPEED_MD_CHANGE         (0x4000U)   /*!< Speed Mode Change Interrupt            */
#define RTL8201_INT_DUPLEX_MD_CHANGE        (0x2000U)   /*!< Duplex Mode Change Interrupt           */
#define RTL8201_INT_LINK_STAT_CHANGE        (0x0800U)   /*!< Link Status Change Interrupt           */
/**
 * @}
 */

/**
 * @defgroup RTL8201_RMII_Mode_Setting_Register RTL8201 RMII Mode Setting Register
 * @{
 */
#define RTL8201_RMII_CLK_DIR                (0x1000U)   /*!< TXC direction in RMII Mode             */
#define RTL8201_RMII_TX_OFFSET              (0x0F00U)   /*!< RMII TX interface timing               */
#define RTL8201_RMII_RX_OFFSET              (0x00F0U)   /*!< RMII RX interface timing               */
#define RTL8201_RMII_MD                     (0x0008U)   /*!< RMII Mode or MII Mode                  */
#define RTL8201_RMII_RXDV_CRSDV             (0x0004U)   /*!< CRS/CRS_DV pin is CRS_DV or RXDV       */
#define RTL8201_RMII_RX_DATA_SEL            (0x0002U)   /*!< RMII data only or data with SSD error  */
/**
 * @}
 */

/**
 * @defgroup RTL8201_Interrupt_Masks RTL8201 Interrupt Masks
 * @note  Set to 0 only mask the choose interrupt event in INTB pin
 * @{
 */
#define RTL8201_INT_MSK_LINK_STAT_CHANGE    (0x2000U)   /*!< Link Status Change Interrupt           */
#define RTL8201_INT_MSK_DUPLEX_MD_CHANGE    (0x1000U)   /*!< Duplex Mode Change Interrupt           */
#define RTL8201_INT_MSK_NWAY_ERR            (0x0800U)   /*!< NWay Error Interrupt                   */
/**
 * @}
 */

/**
 * @defgroup RTL8201_LED_Selection RTL8201 LED Selection
 * @{
 */
#define RTL8201_LED_WOL_SEL                             (0x0400U)   /*!< LED(default) or Wake-On-LAN Function               */
#define RTL8201_LED_SEL                                 (0x0030U)   /*!< Traditional LED Function Selection                 */
#define RTL8201_LED0_ACTALL_LDE1_LINK100                (0x0000U)   /*!< LED0: ACT(all)           LED1: LINK(100)           */
#define RTL8201_LED0_LINKALL_ACTALL_LED1_LINK100        (0x0010U)   /*!< LED0: LINK(ALL)/ACT(all) LED1: LINK(100)           */
#define RTL8201_LED0_LINK10_ACTALL_LED1_LINK100         (0x0020U)   /*!< LED0: LINK(10)/ACT(all)  LED1: LINK(100)           */
#define RTL8201_LED0_LINK10_ACT10_LED1_LINK100_ACT100   (0x0030U)   /*!< LED0: LINK(10)/ACT(10)   LED1: LINK(100)/ACT(100)  */
#define RTL8201_LED_CUSTOMIZE                           (0x0008U)   /*!< Enable customize LED                               */
#define RTL8201_LED_10M_LPI                             (0x0001U)   /*!< Enable 10M LPI LED Function                        */
/**
 * @}
 */

/**
 * @defgroup RTL8201_Link_Statue RTL8201 Link State
 * @{
 */
#define RTL8201_LINK_DOWN                               (0x01U) /*!< No valid link established */
#define RTL8201_LINK_AUTO_NEGO_NOT_CPLT                 (0x02U) /*!< Auto-negotiation process not completed */
#define RTL8201_LINK_100M_FULL_DUPLEX                   (0x03U)
#define RTL8201_LINK_100M_HALF_DUPLEX                   (0x04U)
#define RTL8201_LINK_10M_FULL_DUPLEX                    (0x05U)
#define RTL8201_LINK_10M_HALF_DUPLEX                    (0x06U)
/**
 * @}
 */


/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions ('extern')
 ******************************************************************************/

/*******************************************************************************
  Global function prototypes (definition in C source)
 ******************************************************************************/
/**
 * @addtogroup RTL8201_Global_Functions RTL8201 Global Functions
 * @{
 */
int32_t RTL8201_Init(const stc_rtl8201_ll_t *pstcRtl8201LL);
int32_t RTL8201_SW_Reset(const stc_rtl8201_ll_t *pstcRtl8201LL);
int32_t RTL8201_GetLinkStatus(const stc_rtl8201_ll_t *pstcRtl8201LL, uint16_t *pu16LinkStatus);
int32_t RTL8201_SetLinkMode(const stc_rtl8201_ll_t *pstcRtl8201LL, uint16_t u16Mode);
int32_t RTL8201_SetTradLedMode(const stc_rtl8201_ll_t *pstcRtl8201LL, uint32_t u32Mode);
int32_t RTL8201_SelectPage(const stc_rtl8201_ll_t *pstcRtl8201LL, uint8_t u8Page);

int32_t RTL8201_AutoNegoCmd(const stc_rtl8201_ll_t *pstcRtl8201LL, en_functional_state_t enNewState);
int32_t RTL8201_LoopBackCmd(const stc_rtl8201_ll_t *pstcRtl8201LL, en_functional_state_t enNewState);
int32_t RTL8201_PowerSaveModeCmd(const stc_rtl8201_ll_t *pstcRtl8201LL, en_functional_state_t enNewState);
int32_t RTL8201_PowerDownModeCmd(const stc_rtl8201_ll_t *pstcRtl8201LL, en_functional_state_t enNewState);
int32_t RTL8201_IntCmd(const stc_rtl8201_ll_t *pstcRtl8201LL, uint16_t u16IntType, en_functional_state_t enNewState);
int32_t RTL8201_GetIntStatus(const stc_rtl8201_ll_t *pstcRtl8201LL, uint16_t u16IntType, en_flag_status_t *penStatus);

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

#ifdef __cplusplus
}
#endif

#endif /* __RTL8201_H__ */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
