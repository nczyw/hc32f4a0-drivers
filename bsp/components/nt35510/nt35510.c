/**
 *******************************************************************************
 * @file  nt35510.c
 * @brief This file provides firmware functions for LCD NT35510.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-03-31       CDT             First version
   2022-12-31       CDT             Compliant LCD drive IC: NT35310
   2023-12-15       CDT             Add null pointer check
   2024-05-31       CDT             Optimize function: NT35510_Init/ReadID
                                    Compliant LCD drive IC: NT35510(0x5510), ILI9341, ST7789V, SSD1963
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
#include "nt35510.h"

/**
 * @addtogroup BSP
 * @{
 */

/**
 * @addtogroup Components
 * @{
 */

/**
 * @defgroup NT35510 LCD NT35510
 * @{
 */

/*******************************************************************************
 * Local type definitions ('typedef')
 ******************************************************************************/
/**
 * @defgroup NT35510_Local_Types NT35510 Local Types
 * @{
 */

/**
 * @brief LCD Device Structure Definition
 */
typedef struct {
    /* Properties */
    uint16_t u16ID;         /*!< LCD ID */
    uint16_t u16Width;      /*!< LCD Width */
    uint16_t u16Height;     /*!< LCD Height */
    uint16_t u16WRamCmd;    /*!< Start to write GRAM */
    uint16_t u16SetXCmd;    /*!< Set X axis */
    uint16_t u16SetYCmd;    /*!< Set Y axis */
    uint16_t u16Dir;        /*!< Direction: 0 - Vertical; 1 - Horizontal */
    /* Methods */
    void (*Config)(stc_lcd_controller_t *pstcLCD);
    uint16_t (*ReadID)(stc_lcd_controller_t *pstcLCD);
} stc_lcd_device_t;

/**
 * @}
 */

/*******************************************************************************
 * Local pre-processor symbols/macros ('#define')
 ******************************************************************************/

/**
 * @defgroup NT35510_Local_Macros NT35510 Local Macros
 * @{
 */

/* LCD Scan Direction */
#define LCD_SCAN_DIR                (LCD_SCAN_DIR_L2R_U2D)

/**
 * @defgroup LCD_ID LCD ID
 * @{
 */
#define LCD_ID_INVALID              (0x0000U)
#define LCD_ID_ILI9341              (0x9341U)
#define LCD_ID_NT35310              (0x5310U)
#define LCD_ID_NT35510              (0x5510U)
#define LCD_ID_SSD1963              (0x1963U)
#define LCD_ID_ST7789V              (0x7789U)
/**
 * @}
 */
/**
 * @}
 */

/*******************************************************************************
 * Global variable definitions (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * Local function prototypes ('static')
 ******************************************************************************/
static uint16_t LCD_ILI9341_ReadID(stc_lcd_controller_t *pstcLCD);
static void LCD_ILI9341_Config(stc_lcd_controller_t *pstcLCD);
static uint16_t LCD_NT35310_ReadID(stc_lcd_controller_t *pstcLCD);
static void LCD_NT35310_Config(stc_lcd_controller_t *pstcLCD);
static uint16_t LCD_NT35510_ReadID(stc_lcd_controller_t *pstcLCD);
static void LCD_NT35510_Config(stc_lcd_controller_t *pstcLCD);
static uint16_t LCD_SSD1963_ReadID(stc_lcd_controller_t *pstcLCD);
static void LCD_SSD1963_Config(stc_lcd_controller_t *pstcLCD);
static uint16_t LCD_ST7789V_ReadID(stc_lcd_controller_t *pstcLCD);
static void LCD_ST7789V_Config(stc_lcd_controller_t *pstcLCD);

/*******************************************************************************
 * Local variable definitions ('static')
 ******************************************************************************/

/**
 * @defgroup NT35510_Local_Variables NT35510 Local Variables
 * @{
 */
static stc_lcd_device_t *m_pstcLcdDevice;

static stc_lcd_device_t m_stcLcdCompliantDevice[] = {
    {LCD_ID_SSD1963, 480U, 800U, 0x2CU,   0x2AU,   0x2BU,   0U, LCD_SSD1963_Config, LCD_SSD1963_ReadID},
    {LCD_ID_ST7789V, 240U, 320U, 0x2CU,   0x2AU,   0x2BU,   0U, LCD_ST7789V_Config, LCD_ST7789V_ReadID},
    {LCD_ID_NT35310, 320U, 480U, 0x2CU,   0x2AU,   0x2BU,   0U, LCD_NT35310_Config, LCD_NT35310_ReadID},
    {LCD_ID_NT35510, 480U, 800U, 0x2C00U, 0x2A00U, 0x2B00U, 0U, LCD_NT35510_Config, LCD_NT35510_ReadID},
    {LCD_ID_ILI9341, 240U, 320U, 0x2CU,   0x2AU,   0x2BU,   0U, LCD_ILI9341_Config, LCD_ILI9341_ReadID},
};
/**
 * @}
 */

/*******************************************************************************
 * Function implementation - global ('extern') and local ('static')
 ******************************************************************************/

/**
 * @defgroup NT35510_Local_Functions NT35510 Local Functions
 * @{
 */

/**
 * @brief  LCD delay
 * @param  [in] u32Delay:               Delay
 * @retval None
 */
static void LCD_Delay(uint32_t u32Delay)
{
    volatile uint32_t i;
    const uint32_t u32Cyc = 24000UL;

    while (u32Delay-- > 0UL) {
        i = u32Cyc;
        while (i-- > 0UL) {
            ;
        }
    }
}

/**
 * @brief  Read SSD1963 ID
 * @param  [in] pstcLCD:                LCD controller
 * @retval LCD ID
 */
static uint16_t LCD_SSD1963_ReadID(stc_lcd_controller_t *pstcLCD)
{
    uint8_t u8ID[5];
    uint16_t u16ID;

    NT35510_WriteReg(pstcLCD, 0xA1U);
    u8ID[0] = (uint8_t)NT35510_ReadData(pstcLCD);   /* SSL15~8 */
    u8ID[1] = (uint8_t)NT35510_ReadData(pstcLCD);   /* SSL7~0 */
    u8ID[2] = (uint8_t)NT35510_ReadData(pstcLCD);   /* PROD: 0x61 */
    u8ID[3] = (uint8_t)NT35510_ReadData(pstcLCD);   /* REV */
    u8ID[4] = (uint8_t)NT35510_ReadData(pstcLCD);   /* 0xFF */
    if ((0x61U == u8ID[2]) && (0xFFU == u8ID[4])) {
        u16ID = LCD_ID_SSD1963;                     /* ID convert to: 0x1963 */
    } else {
        u16ID = LCD_ID_INVALID;
    }

    return u16ID;
}

/**
 * @brief  Configure LCD SSD1963
 * @param  [in] pstcLCD:                LCD controller
 * @retval None
 */
static void LCD_SSD1963_Config(stc_lcd_controller_t *pstcLCD)
{
    NT35510_WriteReg(pstcLCD,  0xE2U);
    NT35510_WriteData(pstcLCD, 0x1DU);
    NT35510_WriteData(pstcLCD, 0x02U);
    NT35510_WriteData(pstcLCD, 0x04U);
    LCD_Delay(100UL);
    NT35510_WriteReg(pstcLCD,  0xE0U);
    NT35510_WriteData(pstcLCD, 0x01U);
    LCD_Delay(10UL);
    NT35510_WriteReg(pstcLCD,  0xE0U);
    NT35510_WriteData(pstcLCD, 0x03U);
    LCD_Delay(12);
    NT35510_WriteReg(pstcLCD,  0x01U);
    LCD_Delay(10UL);
    NT35510_WriteReg(pstcLCD,  0xE6U);
    NT35510_WriteData(pstcLCD, 0x2FU);
    NT35510_WriteData(pstcLCD, 0xFFU);
    NT35510_WriteData(pstcLCD, 0xFFU);
    NT35510_WriteReg(pstcLCD,  0xB0U);
    NT35510_WriteData(pstcLCD, 0x20U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x03U);
    NT35510_WriteData(pstcLCD, 0x1FU);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteData(pstcLCD, 0xDFU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0xB4U);
    NT35510_WriteData(pstcLCD, 0x04U);
    NT35510_WriteData(pstcLCD, 0x1FU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x2EU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0xB6U);
    NT35510_WriteData(pstcLCD, 0x02U);
    NT35510_WriteData(pstcLCD, 0x0CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x17U);
    NT35510_WriteData(pstcLCD, 0x16U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0xF0U);
    NT35510_WriteData(pstcLCD, 0x03U);
    NT35510_WriteReg(pstcLCD,  0x29U);
    NT35510_WriteReg(pstcLCD,  0xD0U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0xBEU);
    NT35510_WriteData(pstcLCD, 0x05U);
    NT35510_WriteData(pstcLCD, 0xFEU);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0xB8U);
    NT35510_WriteData(pstcLCD, 0x03U);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteReg(pstcLCD,  0xBAU);
    NT35510_WriteData(pstcLCD, 0X01U);
}

/**
 * @brief  Read ILI9341 ID
 * @param  [in] pstcLCD:                LCD controller
 * @retval LCD ID
 */
static uint16_t LCD_ILI9341_ReadID(stc_lcd_controller_t *pstcLCD)
{
    uint16_t u16ID;

    NT35510_WriteReg(pstcLCD, 0xD3U);
    (void)NT35510_ReadData(pstcLCD);                /* dummy read */
    (void)NT35510_ReadData(pstcLCD);                /* read: 0x00 */
    u16ID  = NT35510_ReadData(pstcLCD) << 8;        /* read: 0x93 */
    u16ID |= NT35510_ReadData(pstcLCD);             /* read: 0x41 */
    if (u16ID != LCD_ID_ILI9341) {
        u16ID = LCD_ID_INVALID;
    }
    return u16ID;
}

/**
 * @brief  Configure LCD ILI9341
 * @param  [in] pstcLCD:                LCD controller
 * @retval None
 */
static void LCD_ILI9341_Config(stc_lcd_controller_t *pstcLCD)
{
    NT35510_WriteReg(pstcLCD,  0xCFU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xC1U);
    NT35510_WriteData(pstcLCD, 0x30U);
    NT35510_WriteReg(pstcLCD,  0xEDU);
    NT35510_WriteData(pstcLCD, 0x64U);
    NT35510_WriteData(pstcLCD, 0x03U);
    NT35510_WriteData(pstcLCD, 0x12U);
    NT35510_WriteData(pstcLCD, 0x81U);
    NT35510_WriteReg(pstcLCD,  0xE8U);
    NT35510_WriteData(pstcLCD, 0x85U);
    NT35510_WriteData(pstcLCD, 0x10U);
    NT35510_WriteData(pstcLCD, 0x7AU);
    NT35510_WriteReg(pstcLCD,  0xCBU);
    NT35510_WriteData(pstcLCD, 0x39U);
    NT35510_WriteData(pstcLCD, 0x2CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x34U);
    NT35510_WriteData(pstcLCD, 0x02U);
    NT35510_WriteReg(pstcLCD,  0xF7U);
    NT35510_WriteData(pstcLCD, 0x20U);
    NT35510_WriteReg(pstcLCD,  0xEAU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0xC0U);
    NT35510_WriteData(pstcLCD, 0x1BU);
    NT35510_WriteReg(pstcLCD,  0xC1U);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteReg(pstcLCD,  0xC5U);
    NT35510_WriteData(pstcLCD, 0x30U);
    NT35510_WriteData(pstcLCD, 0x30U);
    NT35510_WriteReg(pstcLCD,  0xC7U);
    NT35510_WriteData(pstcLCD, 0xB7U);
    NT35510_WriteReg(pstcLCD,  0x36U);
    NT35510_WriteData(pstcLCD, 0x48U);
    NT35510_WriteReg(pstcLCD,  0x3AU);
    NT35510_WriteData(pstcLCD, 0x55U);
    NT35510_WriteReg(pstcLCD,  0xB1U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x1AU);
    NT35510_WriteReg(pstcLCD,  0xB6U);
    NT35510_WriteData(pstcLCD, 0x0AU);
    NT35510_WriteData(pstcLCD, 0xA2U);
    NT35510_WriteReg(pstcLCD,  0xF2U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0x26U);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteReg(pstcLCD,  0xE0U);
    NT35510_WriteData(pstcLCD, 0x0FU);
    NT35510_WriteData(pstcLCD, 0x2AU);
    NT35510_WriteData(pstcLCD, 0x28U);
    NT35510_WriteData(pstcLCD, 0x08U);
    NT35510_WriteData(pstcLCD, 0x0EU);
    NT35510_WriteData(pstcLCD, 0x08U);
    NT35510_WriteData(pstcLCD, 0x54U);
    NT35510_WriteData(pstcLCD, 0xA9U);
    NT35510_WriteData(pstcLCD, 0x43U);
    NT35510_WriteData(pstcLCD, 0x0AU);
    NT35510_WriteData(pstcLCD, 0x0FU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0xE1U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x15U);
    NT35510_WriteData(pstcLCD, 0x17U);
    NT35510_WriteData(pstcLCD, 0x07U);
    NT35510_WriteData(pstcLCD, 0x11U);
    NT35510_WriteData(pstcLCD, 0x06U);
    NT35510_WriteData(pstcLCD, 0x2BU);
    NT35510_WriteData(pstcLCD, 0x56U);
    NT35510_WriteData(pstcLCD, 0x3CU);
    NT35510_WriteData(pstcLCD, 0x05U);
    NT35510_WriteData(pstcLCD, 0x10U);
    NT35510_WriteData(pstcLCD, 0x0FU);
    NT35510_WriteData(pstcLCD, 0x3FU);
    NT35510_WriteData(pstcLCD, 0x3FU);
    NT35510_WriteData(pstcLCD, 0x0FU);
    NT35510_WriteReg(pstcLCD,  0x2BU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteData(pstcLCD, 0x3FU);
    NT35510_WriteReg(pstcLCD,  0x2AU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xEFU);
    NT35510_WriteReg(pstcLCD,  0x11U);
    NT35510_WriteReg(pstcLCD,  0x29U);
}

/**
 * @brief  Read ST7789V ID
 * @param  [in] pstcLCD:                LCD controller
 * @retval LCD ID
 */
static uint16_t LCD_ST7789V_ReadID(stc_lcd_controller_t *pstcLCD)
{
    uint16_t u16ID;

    NT35510_WriteReg(pstcLCD, 0x04U);
    (void)NT35510_ReadData(pstcLCD);            /* dummy read */
    (void)NT35510_ReadData(pstcLCD);            /* read: 0x85 */
    u16ID  = NT35510_ReadData(pstcLCD) << 8;    /* read: 0x85 */
    u16ID |= NT35510_ReadData(pstcLCD);         /* read: 0x41 */
    if (0x8552U == u16ID) {
        u16ID = LCD_ID_ST7789V;                 /* ID convert to: 0x7789 */
    } else {
        u16ID = LCD_ID_INVALID;
    }

    return u16ID;
}

/**
 * @brief  Configure LCD ST7789V
 * @param  [in] pstcLCD:                LCD controller
 * @retval None
 */
static void LCD_ST7789V_Config(stc_lcd_controller_t *pstcLCD)
{
    NT35510_WriteReg(pstcLCD,  0x11U);
    NT35510_WriteReg(pstcLCD,  0x36U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0x3AU);
    NT35510_WriteData(pstcLCD, 0x05U);
    NT35510_WriteReg(pstcLCD,  0xB2U);
    NT35510_WriteData(pstcLCD, 0x0CU);
    NT35510_WriteData(pstcLCD, 0x0CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x33U);
    NT35510_WriteData(pstcLCD, 0x33U);
    NT35510_WriteReg(pstcLCD,  0xB7U);
    NT35510_WriteData(pstcLCD, 0x35U);
    NT35510_WriteReg(pstcLCD,  0xBBU);
    NT35510_WriteData(pstcLCD, 0x32U);
    NT35510_WriteReg(pstcLCD,  0xC0U);
    NT35510_WriteData(pstcLCD, 0x0CU);
    NT35510_WriteReg(pstcLCD,  0xC2U);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteReg(pstcLCD,  0xC3U);
    NT35510_WriteData(pstcLCD, 0x10U);
    NT35510_WriteReg(pstcLCD,  0xC4U);
    NT35510_WriteData(pstcLCD, 0x20U);
    NT35510_WriteReg(pstcLCD,  0xC6U);
    NT35510_WriteData(pstcLCD, 0x0FU);
    NT35510_WriteReg(pstcLCD,  0xD0U);
    NT35510_WriteData(pstcLCD, 0xA4U);
    NT35510_WriteData(pstcLCD, 0xA1U);
    NT35510_WriteReg(pstcLCD,  0xE0U);
    NT35510_WriteData(pstcLCD, 0xD0U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x02U);
    NT35510_WriteData(pstcLCD, 0x07U);
    NT35510_WriteData(pstcLCD, 0x0AU);
    NT35510_WriteData(pstcLCD, 0x28U);
    NT35510_WriteData(pstcLCD, 0x32U);
    NT35510_WriteData(pstcLCD, 0x44U);
    NT35510_WriteData(pstcLCD, 0x42U);
    NT35510_WriteData(pstcLCD, 0x06U);
    NT35510_WriteData(pstcLCD, 0x0EU);
    NT35510_WriteData(pstcLCD, 0x12U);
    NT35510_WriteData(pstcLCD, 0x14U);
    NT35510_WriteData(pstcLCD, 0x17U);
    NT35510_WriteReg(pstcLCD,  0xE1U);
    NT35510_WriteData(pstcLCD, 0xD0U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x02U);
    NT35510_WriteData(pstcLCD, 0x07U);
    NT35510_WriteData(pstcLCD, 0x0AU);
    NT35510_WriteData(pstcLCD, 0x28U);
    NT35510_WriteData(pstcLCD, 0x31U);
    NT35510_WriteData(pstcLCD, 0x54U);
    NT35510_WriteData(pstcLCD, 0x47U);
    NT35510_WriteData(pstcLCD, 0x0EU);
    NT35510_WriteData(pstcLCD, 0x1CU);
    NT35510_WriteData(pstcLCD, 0x17U);
    NT35510_WriteData(pstcLCD, 0x1BU);
    NT35510_WriteData(pstcLCD, 0x1EU);
    NT35510_WriteReg(pstcLCD,  0x2AU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xEFU);
    NT35510_WriteReg(pstcLCD,  0x2BU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteData(pstcLCD, 0x3FU);
    NT35510_WriteReg(pstcLCD,  0x29U);
}

/**
 * @brief  Read NT35310 ID
 * @param  [in] pstcLCD:                LCD controller
 * @retval LCD ID
 */
static uint16_t LCD_NT35310_ReadID(stc_lcd_controller_t *pstcLCD)
{
    uint16_t u16ID;

    NT35510_WriteReg(pstcLCD, 0xD4U);
    (void)NT35510_ReadData(pstcLCD);            /* dummy read */
    (void)NT35510_ReadData(pstcLCD);            /* read: 0x01 */
    u16ID  = NT35510_ReadData(pstcLCD) << 8;    /* read: 0x53 */
    u16ID |= NT35510_ReadData(pstcLCD);         /* read: 0x10 */
    if (u16ID != LCD_ID_NT35310) {
        u16ID = LCD_ID_INVALID;
    }
    return u16ID;
}

/**
 * @brief  Configure LCD NT35310
 * @param  [in] pstcLCD:                LCD controller
 * @retval None
 */
static void LCD_NT35310_Config(stc_lcd_controller_t *pstcLCD)
{
    NT35510_WriteReg(pstcLCD,  0xEDU);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteData(pstcLCD, 0xFEU);

    NT35510_WriteReg(pstcLCD,  0xEEU);
    NT35510_WriteData(pstcLCD, 0xDEU);
    NT35510_WriteData(pstcLCD, 0x21U);

    NT35510_WriteReg(pstcLCD,  0xF1U);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteReg(pstcLCD,  0xDFU);
    NT35510_WriteData(pstcLCD, 0x10U);

    /* VCOM voltage */
    NT35510_WriteReg(pstcLCD,  0xC4U);
    NT35510_WriteData(pstcLCD, 0x8FU);

    NT35510_WriteReg(pstcLCD,  0xC6U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xE2U);
    NT35510_WriteData(pstcLCD, 0xE2U);
    NT35510_WriteData(pstcLCD, 0xE2U);
    NT35510_WriteReg(pstcLCD,  0xBFU);
    NT35510_WriteData(pstcLCD, 0xAAU);

    NT35510_WriteReg(pstcLCD,  0xB0U);
    NT35510_WriteData(pstcLCD, 0x0DU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x0DU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x11U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x19U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x21U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x2DU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x3DU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x5DU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x5DU);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xB1U);
    NT35510_WriteData(pstcLCD, 0x80U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x8BU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x96U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xB2U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x02U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x03U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xB3U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xB4U);
    NT35510_WriteData(pstcLCD, 0x8BU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x96U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xA1U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xB5U);
    NT35510_WriteData(pstcLCD, 0x02U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x03U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x04U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xB6U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xB7U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x3FU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x5EU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x64U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x8CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xACU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xDCU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x70U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x90U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xEBU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xDCU);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xB8U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xBAU);
    NT35510_WriteData(pstcLCD, 0x24U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xC1U);
    NT35510_WriteData(pstcLCD, 0x20U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x54U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xFFU);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xC2U);
    NT35510_WriteData(pstcLCD, 0x0AU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x04U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xC3U);
    NT35510_WriteData(pstcLCD, 0x3CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x3AU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x39U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x37U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x3CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x36U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x32U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x2FU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x2CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x29U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x26U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x24U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x24U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x23U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x3CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x36U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x32U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x2FU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x2CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x29U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x26U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x24U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x24U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x23U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xC4U);
    NT35510_WriteData(pstcLCD, 0x62U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x05U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x84U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xF0U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x18U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xA4U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x18U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x50U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x0CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x17U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x95U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xF3U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xE6U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xC5U);
    NT35510_WriteData(pstcLCD, 0x32U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x44U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x65U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x76U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x88U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xC6U);
    NT35510_WriteData(pstcLCD, 0x20U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x17U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xC7U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xC8U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xC9U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xE0U);
    NT35510_WriteData(pstcLCD, 0x16U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x1CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x21U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x36U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x46U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x52U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x64U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x7AU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x8BU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x99U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xA8U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xB9U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xC4U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xCAU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xD2U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xD9U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xE0U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xF3U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xE1U);
    NT35510_WriteData(pstcLCD, 0x16U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x1CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x22U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x36U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x45U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x52U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x64U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x7AU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x8BU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x99U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xA8U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xB9U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xC4U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xCAU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xD2U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xD8U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xE0U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xF3U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xE2U);
    NT35510_WriteData(pstcLCD, 0x05U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x0BU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x1BU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x34U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x44U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x4FU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x61U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x79U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x88U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x97U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xA6U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xB7U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xC2U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xC7U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xD1U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xD6U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xDDU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xF3U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0xE3U);
    NT35510_WriteData(pstcLCD, 0x05U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x0AU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x1CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x33U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x44U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x50U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x62U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x78U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x88U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x97U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xA6U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xB7U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xC2U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xC7U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xD1U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xD5U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xDDU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xF3U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xE4U);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x01U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x02U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x2AU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x3CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x4BU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x5DU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x74U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x84U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x93U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xA2U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xB3U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xBEU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xC4U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xCDU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xD3U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xDDU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xF3U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteReg(pstcLCD,  0xE5U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x02U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x29U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x3CU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x4BU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x5DU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x74U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x84U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x93U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xA2U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xB3U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xBEU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xC4U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xCDU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xD3U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xDCU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xF3U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xE6U);
    NT35510_WriteData(pstcLCD, 0x11U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x34U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x56U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x76U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x77U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x66U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x88U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x99U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xBBU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x99U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x66U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x55U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x55U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x45U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x43U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x44U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xE7U);
    NT35510_WriteData(pstcLCD, 0x32U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x55U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x76U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x66U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x67U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x67U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x87U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x99U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xBBU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x99U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x77U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x44U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x56U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x23U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x33U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x45U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xE8U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x99U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x87U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x88U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x77U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x66U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x88U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xAAU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0xBBU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x99U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x66U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x55U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x55U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x44U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x44U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x55U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xE9U);
    NT35510_WriteData(pstcLCD, 0xAAU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0x00U);
    NT35510_WriteData(pstcLCD, 0xAAU);

    NT35510_WriteReg(pstcLCD,  0xCFU);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xF0U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x50U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xF3U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0xF9U);
    NT35510_WriteData(pstcLCD, 0x06U);
    NT35510_WriteData(pstcLCD, 0x10U);
    NT35510_WriteData(pstcLCD, 0x29U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0x3AU);
    NT35510_WriteData(pstcLCD, 0x55U);

    NT35510_WriteReg(pstcLCD,  0x11U);
    LCD_Delay(100UL);
    NT35510_WriteReg(pstcLCD,  0x29U);
    NT35510_WriteReg(pstcLCD,  0x35U);
    NT35510_WriteData(pstcLCD, 0x00U);

    NT35510_WriteReg(pstcLCD,  0x51U);
    NT35510_WriteData(pstcLCD, 0xFFU);
    NT35510_WriteReg(pstcLCD,  0x53U);
    NT35510_WriteData(pstcLCD, 0x2CU);
    NT35510_WriteReg(pstcLCD,  0x55U);
    NT35510_WriteData(pstcLCD, 0x82U);
    NT35510_WriteReg(pstcLCD,  0x2CU);
}

/**
 * @brief  Read NT35510 ID
 * @param  [in] pstcLCD:                LCD controller
 * @retval LCD ID
 */
static uint16_t LCD_NT35510_ReadID(stc_lcd_controller_t *pstcLCD)
{
    uint16_t u16ID;

    /* Try to read ID: 0x8000 (NT35510) */
    NT35510_WriteReg(pstcLCD, 0xDA00);
    (void)NT35510_ReadData(pstcLCD);        /* read 0xDA00: 0x0000 */
    NT35510_WriteReg(pstcLCD, 0xDB00U);
    u16ID = NT35510_ReadData(pstcLCD) << 8; /* read 0xDB00: 0x0080 */
    NT35510_WriteReg(pstcLCD, 0xDC00U);
    u16ID |= NT35510_ReadData(pstcLCD);     /* read 0xDC00: 0x0000 */
    /* Read ID: ID=8000H (5510H) */
    if (0x8000U == u16ID) {
        u16ID = LCD_ID_NT35510;             /* ID convert to: 0x5510 */
    } else {
        NT35510_WriteReg(pstcLCD,  0xF000U);
        NT35510_WriteData(pstcLCD, 0x0055U);
        NT35510_WriteReg(pstcLCD,  0xF001U);
        NT35510_WriteData(pstcLCD, 0x00AAU);
        NT35510_WriteReg(pstcLCD,  0xF002U);
        NT35510_WriteData(pstcLCD, 0x0052U);
        NT35510_WriteReg(pstcLCD,  0xF003U);
        NT35510_WriteData(pstcLCD, 0x0008U);
        NT35510_WriteReg(pstcLCD,  0xF004U);
        NT35510_WriteData(pstcLCD, 0x0001U);     /* send key (vendor provided) */

        NT35510_WriteReg(pstcLCD,  0xC500U);
        u16ID = NT35510_ReadData(pstcLCD) << 8; /* read ID high 8 bits */

        NT35510_WriteReg(pstcLCD,  0xC501U);
        u16ID |= NT35510_ReadData(pstcLCD);     /* read ID low 8 bits */
        if (u16ID != LCD_ID_NT35510) {
            u16ID = LCD_ID_INVALID;
        }
    }

    return u16ID;
}

/**
 * @brief  Configure LCD NT35510
 * @param  [in] pstcLCD:                LCD controller
 * @retval None
 */
static void LCD_NT35510_Config(stc_lcd_controller_t *pstcLCD)
{
    /* Config LCD */
    NT35510_WriteRegData(pstcLCD, 0xF000U, 0x55U);
    NT35510_WriteRegData(pstcLCD, 0xF001U, 0xAAU);
    NT35510_WriteRegData(pstcLCD, 0xF002U, 0x52U);
    NT35510_WriteRegData(pstcLCD, 0xF003U, 0x08U);
    NT35510_WriteRegData(pstcLCD, 0xF004U, 0x01U);
    /* AVDD Set AVDD 5.2V */
    NT35510_WriteRegData(pstcLCD, 0xB000U, 0x0DU);
    NT35510_WriteRegData(pstcLCD, 0xB001U, 0x0DU);
    NT35510_WriteRegData(pstcLCD, 0xB002U, 0x0DU);
    /* AVDD ratio */
    NT35510_WriteRegData(pstcLCD, 0xB600U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xB601U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xB602U, 0x34U);
    /* AVEE -5.2V */
    NT35510_WriteRegData(pstcLCD, 0xB100U, 0x0DU);
    NT35510_WriteRegData(pstcLCD, 0xB101U, 0x0DU);
    NT35510_WriteRegData(pstcLCD, 0xB102U, 0x0DU);
    /* AVEE ratio */
    NT35510_WriteRegData(pstcLCD, 0xB700U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xB701U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xB702U, 0x34U);
    /* VCL -2.5V */
    NT35510_WriteRegData(pstcLCD, 0xB200U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xB201U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xB202U, 0x00U);
    /* VCL ratio */
    NT35510_WriteRegData(pstcLCD, 0xB800U, 0x24U);
    NT35510_WriteRegData(pstcLCD, 0xB801U, 0x24U);
    NT35510_WriteRegData(pstcLCD, 0xB802U, 0x24U);
    /* VGH 15V (Free pump) */
    NT35510_WriteRegData(pstcLCD, 0xBF00U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xB300U, 0x0FU);
    NT35510_WriteRegData(pstcLCD, 0xB301U, 0x0FU);
    NT35510_WriteRegData(pstcLCD, 0xB302U, 0x0FU);
    /* VGH ratio */
    NT35510_WriteRegData(pstcLCD, 0xB900U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xB901U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xB902U, 0x34U);
    /* VGL_REG -10V */
    NT35510_WriteRegData(pstcLCD, 0xB500U, 0x08U);
    NT35510_WriteRegData(pstcLCD, 0xB501U, 0x08U);
    NT35510_WriteRegData(pstcLCD, 0xB502U, 0x08U);
    NT35510_WriteRegData(pstcLCD, 0xC200U, 0x03U);
    /* VGLX ratio */
    NT35510_WriteRegData(pstcLCD, 0xBA00U, 0x24U);
    NT35510_WriteRegData(pstcLCD, 0xBA01U, 0x24U);
    NT35510_WriteRegData(pstcLCD, 0xBA02U, 0x24U);
    /* VGMP/VGSP 4.5V/0V */
    NT35510_WriteRegData(pstcLCD, 0xBC00U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xBC01U, 0x78U);
    NT35510_WriteRegData(pstcLCD, 0xBC02U, 0x00U);
    /* VGMN/VGSN -4.5V/0V */
    NT35510_WriteRegData(pstcLCD, 0xBD00U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xBD01U, 0x78U);
    NT35510_WriteRegData(pstcLCD, 0xBD02U, 0x00U);
    /* VCOM */
    NT35510_WriteRegData(pstcLCD, 0xBE00U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xBE01U, 0x64U);
    /* Gamma Setting */
    NT35510_WriteRegData(pstcLCD, 0xD100U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD101U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD102U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD103U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD104U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD105U, 0x3AU);
    NT35510_WriteRegData(pstcLCD, 0xD106U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD107U, 0x4AU);
    NT35510_WriteRegData(pstcLCD, 0xD108U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD109U, 0x5CU);
    NT35510_WriteRegData(pstcLCD, 0xD10AU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD10BU, 0x81U);
    NT35510_WriteRegData(pstcLCD, 0xD10CU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD10DU, 0xA6U);
    NT35510_WriteRegData(pstcLCD, 0xD10EU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD10FU, 0xE5U);
    NT35510_WriteRegData(pstcLCD, 0xD110U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD111U, 0x13U);
    NT35510_WriteRegData(pstcLCD, 0xD112U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD113U, 0x54U);
    NT35510_WriteRegData(pstcLCD, 0xD114U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD115U, 0x82U);
    NT35510_WriteRegData(pstcLCD, 0xD116U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD117U, 0xCAU);
    NT35510_WriteRegData(pstcLCD, 0xD118U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD119U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD11AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD11BU, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD11CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD11DU, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD11EU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD11FU, 0x67U);
    NT35510_WriteRegData(pstcLCD, 0xD120U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD121U, 0x84U);
    NT35510_WriteRegData(pstcLCD, 0xD122U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD123U, 0xA4U);
    NT35510_WriteRegData(pstcLCD, 0xD124U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD125U, 0xB7U);
    NT35510_WriteRegData(pstcLCD, 0xD126U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD127U, 0xCFU);
    NT35510_WriteRegData(pstcLCD, 0xD128U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD129U, 0xDEU);
    NT35510_WriteRegData(pstcLCD, 0xD12AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD12BU, 0xF2U);
    NT35510_WriteRegData(pstcLCD, 0xD12CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD12DU, 0xFEU);
    NT35510_WriteRegData(pstcLCD, 0xD12EU, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD12FU, 0x10U);
    NT35510_WriteRegData(pstcLCD, 0xD130U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD131U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD132U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD133U, 0x6DU);
    NT35510_WriteRegData(pstcLCD, 0xD200U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD201U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD202U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD203U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD204U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD205U, 0x3AU);
    NT35510_WriteRegData(pstcLCD, 0xD206U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD207U, 0x4AU);
    NT35510_WriteRegData(pstcLCD, 0xD208U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD209U, 0x5CU);
    NT35510_WriteRegData(pstcLCD, 0xD20AU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD20BU, 0x81U);
    NT35510_WriteRegData(pstcLCD, 0xD20CU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD20DU, 0xA6U);
    NT35510_WriteRegData(pstcLCD, 0xD20EU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD20FU, 0xE5U);
    NT35510_WriteRegData(pstcLCD, 0xD210U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD211U, 0x13U);
    NT35510_WriteRegData(pstcLCD, 0xD212U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD213U, 0x54U);
    NT35510_WriteRegData(pstcLCD, 0xD214U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD215U, 0x82U);
    NT35510_WriteRegData(pstcLCD, 0xD216U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD217U, 0xCAU);
    NT35510_WriteRegData(pstcLCD, 0xD218U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD219U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD21AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD21BU, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD21CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD21DU, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD21EU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD21FU, 0x67U);
    NT35510_WriteRegData(pstcLCD, 0xD220U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD221U, 0x84U);
    NT35510_WriteRegData(pstcLCD, 0xD222U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD223U, 0xA4U);
    NT35510_WriteRegData(pstcLCD, 0xD224U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD225U, 0xB7U);
    NT35510_WriteRegData(pstcLCD, 0xD226U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD227U, 0xCFU);
    NT35510_WriteRegData(pstcLCD, 0xD228U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD229U, 0xDEU);
    NT35510_WriteRegData(pstcLCD, 0xD22AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD22BU, 0xF2U);
    NT35510_WriteRegData(pstcLCD, 0xD22CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD22DU, 0xFEU);
    NT35510_WriteRegData(pstcLCD, 0xD22EU, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD22FU, 0x10U);
    NT35510_WriteRegData(pstcLCD, 0xD230U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD231U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD232U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD233U, 0x6DU);
    NT35510_WriteRegData(pstcLCD, 0xD300U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD301U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD302U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD303U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD304U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD305U, 0x3AU);
    NT35510_WriteRegData(pstcLCD, 0xD306U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD307U, 0x4AU);
    NT35510_WriteRegData(pstcLCD, 0xD308U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD309U, 0x5CU);
    NT35510_WriteRegData(pstcLCD, 0xD30AU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD30BU, 0x81U);
    NT35510_WriteRegData(pstcLCD, 0xD30CU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD30DU, 0xA6U);
    NT35510_WriteRegData(pstcLCD, 0xD30EU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD30FU, 0xE5U);
    NT35510_WriteRegData(pstcLCD, 0xD310U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD311U, 0x13U);
    NT35510_WriteRegData(pstcLCD, 0xD312U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD313U, 0x54U);
    NT35510_WriteRegData(pstcLCD, 0xD314U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD315U, 0x82U);
    NT35510_WriteRegData(pstcLCD, 0xD316U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD317U, 0xCAU);
    NT35510_WriteRegData(pstcLCD, 0xD318U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD319U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD31AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD31BU, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD31CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD31DU, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD31EU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD31FU, 0x67U);
    NT35510_WriteRegData(pstcLCD, 0xD320U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD321U, 0x84U);
    NT35510_WriteRegData(pstcLCD, 0xD322U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD323U, 0xA4U);
    NT35510_WriteRegData(pstcLCD, 0xD324U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD325U, 0xB7U);
    NT35510_WriteRegData(pstcLCD, 0xD326U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD327U, 0xCFU);
    NT35510_WriteRegData(pstcLCD, 0xD328U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD329U, 0xDEU);
    NT35510_WriteRegData(pstcLCD, 0xD32AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD32BU, 0xF2U);
    NT35510_WriteRegData(pstcLCD, 0xD32CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD32DU, 0xFEU);
    NT35510_WriteRegData(pstcLCD, 0xD32EU, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD32FU, 0x10U);
    NT35510_WriteRegData(pstcLCD, 0xD330U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD331U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD332U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD333U, 0x6DU);
    NT35510_WriteRegData(pstcLCD, 0xD400U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD401U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD402U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD403U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD404U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD405U, 0x3AU);
    NT35510_WriteRegData(pstcLCD, 0xD406U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD407U, 0x4AU);
    NT35510_WriteRegData(pstcLCD, 0xD408U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD409U, 0x5CU);
    NT35510_WriteRegData(pstcLCD, 0xD40AU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD40BU, 0x81U);
    NT35510_WriteRegData(pstcLCD, 0xD40CU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD40DU, 0xA6U);
    NT35510_WriteRegData(pstcLCD, 0xD40EU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD40FU, 0xE5U);
    NT35510_WriteRegData(pstcLCD, 0xD410U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD411U, 0x13U);
    NT35510_WriteRegData(pstcLCD, 0xD412U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD413U, 0x54U);
    NT35510_WriteRegData(pstcLCD, 0xD414U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD415U, 0x82U);
    NT35510_WriteRegData(pstcLCD, 0xD416U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD417U, 0xCAU);
    NT35510_WriteRegData(pstcLCD, 0xD418U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD419U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD41AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD41BU, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD41CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD41DU, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD41EU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD41FU, 0x67U);
    NT35510_WriteRegData(pstcLCD, 0xD420U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD421U, 0x84U);
    NT35510_WriteRegData(pstcLCD, 0xD422U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD423U, 0xA4U);
    NT35510_WriteRegData(pstcLCD, 0xD424U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD425U, 0xB7U);
    NT35510_WriteRegData(pstcLCD, 0xD426U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD427U, 0xCFU);
    NT35510_WriteRegData(pstcLCD, 0xD428U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD429U, 0xDEU);
    NT35510_WriteRegData(pstcLCD, 0xD42AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD42BU, 0xF2U);
    NT35510_WriteRegData(pstcLCD, 0xD42CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD42DU, 0xFEU);
    NT35510_WriteRegData(pstcLCD, 0xD42EU, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD42FU, 0x10U);
    NT35510_WriteRegData(pstcLCD, 0xD430U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD431U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD432U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD433U, 0x6DU);
    NT35510_WriteRegData(pstcLCD, 0xD500U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD501U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD502U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD503U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD504U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD505U, 0x3AU);
    NT35510_WriteRegData(pstcLCD, 0xD506U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD507U, 0x4AU);
    NT35510_WriteRegData(pstcLCD, 0xD508U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD509U, 0x5CU);
    NT35510_WriteRegData(pstcLCD, 0xD50AU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD50BU, 0x81U);
    NT35510_WriteRegData(pstcLCD, 0xD50CU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD50DU, 0xA6U);
    NT35510_WriteRegData(pstcLCD, 0xD50EU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD50FU, 0xE5U);
    NT35510_WriteRegData(pstcLCD, 0xD510U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD511U, 0x13U);
    NT35510_WriteRegData(pstcLCD, 0xD512U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD513U, 0x54U);
    NT35510_WriteRegData(pstcLCD, 0xD514U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD515U, 0x82U);
    NT35510_WriteRegData(pstcLCD, 0xD516U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD517U, 0xCAU);
    NT35510_WriteRegData(pstcLCD, 0xD518U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD519U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD51AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD51BU, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD51CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD51DU, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD51EU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD51FU, 0x67U);
    NT35510_WriteRegData(pstcLCD, 0xD520U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD521U, 0x84U);
    NT35510_WriteRegData(pstcLCD, 0xD522U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD523U, 0xA4U);
    NT35510_WriteRegData(pstcLCD, 0xD524U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD525U, 0xB7U);
    NT35510_WriteRegData(pstcLCD, 0xD526U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD527U, 0xCFU);
    NT35510_WriteRegData(pstcLCD, 0xD528U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD529U, 0xDEU);
    NT35510_WriteRegData(pstcLCD, 0xD52AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD52BU, 0xF2U);
    NT35510_WriteRegData(pstcLCD, 0xD52CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD52DU, 0xFEU);
    NT35510_WriteRegData(pstcLCD, 0xD52EU, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD52FU, 0x10U);
    NT35510_WriteRegData(pstcLCD, 0xD530U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD531U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD532U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD533U, 0x6DU);
    NT35510_WriteRegData(pstcLCD, 0xD600U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD601U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD602U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD603U, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD604U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD605U, 0x3AU);
    NT35510_WriteRegData(pstcLCD, 0xD606U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD607U, 0x4AU);
    NT35510_WriteRegData(pstcLCD, 0xD608U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD609U, 0x5CU);
    NT35510_WriteRegData(pstcLCD, 0xD60AU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD60BU, 0x81U);
    NT35510_WriteRegData(pstcLCD, 0xD60CU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD60DU, 0xA6U);
    NT35510_WriteRegData(pstcLCD, 0xD60EU, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD60FU, 0xE5U);
    NT35510_WriteRegData(pstcLCD, 0xD610U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD611U, 0x13U);
    NT35510_WriteRegData(pstcLCD, 0xD612U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD613U, 0x54U);
    NT35510_WriteRegData(pstcLCD, 0xD614U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD615U, 0x82U);
    NT35510_WriteRegData(pstcLCD, 0xD616U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD617U, 0xCAU);
    NT35510_WriteRegData(pstcLCD, 0xD618U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD619U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xD61AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD61BU, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xD61CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD61DU, 0x34U);
    NT35510_WriteRegData(pstcLCD, 0xD61EU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD61FU, 0x67U);
    NT35510_WriteRegData(pstcLCD, 0xD620U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD621U, 0x84U);
    NT35510_WriteRegData(pstcLCD, 0xD622U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD623U, 0xA4U);
    NT35510_WriteRegData(pstcLCD, 0xD624U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD625U, 0xB7U);
    NT35510_WriteRegData(pstcLCD, 0xD626U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD627U, 0xCFU);
    NT35510_WriteRegData(pstcLCD, 0xD628U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD629U, 0xDEU);
    NT35510_WriteRegData(pstcLCD, 0xD62AU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD62BU, 0xF2U);
    NT35510_WriteRegData(pstcLCD, 0xD62CU, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xD62DU, 0xFEU);
    NT35510_WriteRegData(pstcLCD, 0xD62EU, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD62FU, 0x10U);
    NT35510_WriteRegData(pstcLCD, 0xD630U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD631U, 0x33U);
    NT35510_WriteRegData(pstcLCD, 0xD632U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xD633U, 0x6DU);
    /* LV2 Page 0 enable */
    NT35510_WriteRegData(pstcLCD, 0xF000U, 0x55U);
    NT35510_WriteRegData(pstcLCD, 0xF001U, 0xAAU);
    NT35510_WriteRegData(pstcLCD, 0xF002U, 0x52U);
    NT35510_WriteRegData(pstcLCD, 0xF003U, 0x08U);
    NT35510_WriteRegData(pstcLCD, 0xF004U, 0x00U);
    /* Display control */
    NT35510_WriteRegData(pstcLCD, 0xB100U, 0xCCU);
    NT35510_WriteRegData(pstcLCD, 0xB101U, 0x00U);
    /* Source hold time */
    NT35510_WriteRegData(pstcLCD, 0xB600U, 0x05U);
    /* Gate EQ control */
    NT35510_WriteRegData(pstcLCD, 0xB700U, 0x70U);
    NT35510_WriteRegData(pstcLCD, 0xB701U, 0x70U);
    /* Source EQ control (Mode 2) */
    NT35510_WriteRegData(pstcLCD, 0xB800U, 0x01U);
    NT35510_WriteRegData(pstcLCD, 0xB801U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xB802U, 0x03U);
    NT35510_WriteRegData(pstcLCD, 0xB803U, 0x03U);
    /* Inversion mode (2-dot) */
    NT35510_WriteRegData(pstcLCD, 0xBC00U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xBC01U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0xBC02U, 0x00U);
    /* Timing control 4H w/ 4-delay */
    NT35510_WriteRegData(pstcLCD, 0xC900U, 0xD0U);
    NT35510_WriteRegData(pstcLCD, 0xC901U, 0x02U);
    NT35510_WriteRegData(pstcLCD, 0xC902U, 0x50U);
    NT35510_WriteRegData(pstcLCD, 0xC903U, 0x50U);
    NT35510_WriteRegData(pstcLCD, 0xC904U, 0x50U);
    NT35510_WriteRegData(pstcLCD, 0x3500U, 0x00U);
    NT35510_WriteRegData(pstcLCD, 0x3A00U, 0x55U);  /* 16-bit/pixel */
    NT35510_WriteReg(pstcLCD, 0x1100U);
    LCD_Delay(120UL);
    NT35510_WriteReg(pstcLCD, 0x2900U);
}

/**
 * @brief  Search the compliant LCD device.
 * @param  [in] pstcLCD:                LCD controller
 * @retval None
 */
static stc_lcd_device_t *LCD_SearchDevice(stc_lcd_controller_t *pstcLCD)
{
    uint32_t i;
    uint16_t u16ID;
    stc_lcd_device_t *pstcLcdDevice = NULL;
    uint32_t u32DevNum = sizeof(m_stcLcdCompliantDevice) / sizeof(m_stcLcdCompliantDevice[0]);

    for (i = 0UL; i < u32DevNum; i++) {
        u16ID = m_stcLcdCompliantDevice[i].ReadID(pstcLCD);
        if (m_stcLcdCompliantDevice[i].u16ID == u16ID) {
            pstcLcdDevice = &m_stcLcdCompliantDevice[i];
            break;
        }
    }

    m_pstcLcdDevice = pstcLcdDevice;
    return pstcLcdDevice;
}

/**
 * @brief  Configure the LCD device.
 * @param  [in] pstcLCD:                LCD controller
 * @param  [in] pstcDevice:             LCD device
 * @retval None
 */
static void LCD_ConfigDevice(stc_lcd_controller_t *pstcLCD, stc_lcd_device_t *pstcDevice)
{
    pstcDevice->Config(pstcLCD);
}

/**
 * @}
 */

/**
 * @defgroup NT35510_Global_Functions NT35510 Global Functions
 * @{
 */

/**
 * @brief  Initialize LCD device.
 * @param  [in] pstcLCD:                LCD controller
 * @retval None
 */
void NT35510_Init(stc_lcd_controller_t *pstcLCD)
{
    stc_lcd_device_t *pstcLcdDevice;

    if (NULL != pstcLCD) {
        /* NOP */
        NT35510_WriteRegData(pstcLCD, 0x0000U, 0x00U);

        /* Search LCD device */
        pstcLcdDevice = LCD_SearchDevice(pstcLCD);
        if (pstcLcdDevice != NULL) {
            LCD_ConfigDevice(pstcLCD, pstcLcdDevice);

            /* Set direction */
            NT35510_SetDisplayDir(pstcLCD, LCD_DISPLAY_VERTICAL);

            /* Set cursor */
            NT35510_SetCursor(pstcLCD, 0U, 0U);

            /* Prepare to write to LCD RAM */
            NT35510_PrepareWriteRAM(pstcLCD);
        }
    }
}

/**
 * @brief  Read LCD ID.
 * @param  [in] pstcLCD:                LCD controller @ref stc_lcd_controller_t structure.
 * @retval LCD Register Value.
 */
uint16_t NT35510_ReadID(stc_lcd_controller_t *pstcLCD)
{
    (void)pstcLCD;
    return m_pstcLcdDevice->u16ID;
}

/**
 * @brief  Enable the Display.
 * @param  [in] pstcLCD:                LCD controller
 * @retval None
 */
void NT35510_DisplayOn(stc_lcd_controller_t *pstcLCD)
{
    if (NULL != pstcLCD) {
        if (LCD_ID_NT35510 == m_pstcLcdDevice->u16ID) {
            NT35510_WriteReg(pstcLCD, 0x2900U);     /* 5510 */
        } else {
            NT35510_WriteReg(pstcLCD, 0x29U);       /* 9341/5310/1963/7789 */
        }
    }
}

/**
 * @brief  Disable the Display.
 * @param  [in] pstcLCD:                LCD controller
 * @retval None
 */
void NT35510_DisplayOff(stc_lcd_controller_t *pstcLCD)
{
    if (NULL != pstcLCD) {
        if (LCD_ID_NT35510 == m_pstcLcdDevice->u16ID) {
            NT35510_WriteReg(pstcLCD, 0x2800U);     /* 5510 */
        } else {
            NT35510_WriteReg(pstcLCD, 0x28U);       /* 9341/5310/1963/7789 */
        }
    }
}

/**
 * @brief  Get LCD PIXEL WIDTH.
 * @param  None
 * @retval LCD PIXEL WIDTH.
 */
uint16_t NT35510_GetPixelWidth(void)
{
    return (NULL != m_pstcLcdDevice) ? m_pstcLcdDevice->u16Width : 0U;
}

/**
 * @brief  Get LCD PIXEL HEIGHT.
 * @param  None
 * @retval LCD PIXEL HEIGHT.
 */
uint16_t NT35510_GetPixelHeight(void)
{
    return (NULL != m_pstcLcdDevice) ? m_pstcLcdDevice->u16Height : 0U;
}

/**
 * @brief  Set scan direction.
 * @param  [in] pstcLCD:                LCD controller
 * @param  [in] u16Dir:                 Scan direction
 *         This parameter can be one of the following values:
 *           @arg LCD_SCAN_DIR_L2R_U2D: From left to right && from up to down
 *           @arg LCD_SCAN_DIR_L2R_D2U: From left to right && from down to up
 *           @arg LCD_SCAN_DIR_R2L_U2D: From right to left && from up to down
 *           @arg LCD_SCAN_DIR_R2L_D2U: From right to left && from down to up
 *           @arg LCD_SCAN_DIR_U2D_L2R: From up to down && from left to right
 *           @arg LCD_SCAN_DIR_U2D_R2L: From up to down && from right to left
 *           @arg LCD_SCAN_DIR_D2U_L2R: From down to up && from left to right
 *           @arg LCD_SCAN_DIR_D2U_R2L: From down to up && from right to left
 * @retval None
 */
void NT35510_SetScanDir(stc_lcd_controller_t *pstcLCD, uint16_t u16Dir)
{
    uint16_t u16Temp;
    uint16_t dirreg;
    uint16_t regval = 0U;

    if (NULL != pstcLCD) {
        /* when display dir is VERTICAL, 1963 IC change scan-direction, other IC don't change
           when display dir is HORIZONTAL, 1963 IC don't change scan-direction, other IC change */
        if (((0U == m_pstcLcdDevice->u16Dir) && (m_pstcLcdDevice->u16ID == LCD_ID_SSD1963)) || \
            ((1U == m_pstcLcdDevice->u16Dir) && (m_pstcLcdDevice->u16ID != LCD_ID_SSD1963))) {
            if (0U == u16Dir) {
                u16Dir = 6U;
            } else if (1U == u16Dir) {
                u16Dir = 7U;
            } else if (2U == u16Dir) {
                u16Dir = 4U;
            } else if (3UL == u16Dir) {
                u16Dir = 5U;
            } else if (4U == u16Dir) {
                u16Dir = 1U;
            } else if (5U == u16Dir) {
                u16Dir = 0U;
            } else if (6U == u16Dir) {
                u16Dir = 3U;
            } else if (7U == u16Dir) {
                u16Dir = 2U;
            } else {
                u16Dir = 6U;
            }
        }

        switch (u16Dir) {
            case LCD_SCAN_DIR_L2R_U2D:
                regval |= ((0U << 7) | (0U << 6) | (0U << 5));
                break;
            case LCD_SCAN_DIR_L2R_D2U:
                regval |= ((1U << 7) | (0U << 6) | (0U << 5));
                break;
            case LCD_SCAN_DIR_R2L_U2D:
                regval |= ((0U << 7) | (1U << 6) | (0U << 5));
                break;
            case LCD_SCAN_DIR_R2L_D2U:
                regval |= ((1U << 7) | (1U << 6) | (0U << 5));
                break;
            case LCD_SCAN_DIR_U2D_L2R:
                regval |= ((0U << 7) | (0U << 6) | (1U << 5));
                break;
            case LCD_SCAN_DIR_U2D_R2L:
                regval |= ((0U << 7) | (1U << 6) | (1U << 5));
                break;
            case LCD_SCAN_DIR_D2U_L2R:
                regval |= ((1U << 7) | (0U << 6) | (1U << 5));
                break;
            case LCD_SCAN_DIR_D2U_R2L:
                regval |= ((1U << 7) | (1U << 6) | (1U << 5));
                break;
            default:
                break;
        }

        if (LCD_ID_NT35510 == m_pstcLcdDevice->u16ID) {
            dirreg = 0x3600U;
        } else {
            dirreg = 0x36U;
        }

        /* 0x9341 & 0x7789 set BGR bit */
        if ((LCD_ID_ILI9341 == m_pstcLcdDevice->u16ID) || (LCD_ID_ST7789V == m_pstcLcdDevice->u16ID)) {
            regval |= 0x08U;
        }

        NT35510_WriteRegData(pstcLCD, dirreg, regval);

        /* 1963 don't handle coordinate */
        if (m_pstcLcdDevice->u16ID != LCD_ID_SSD1963) {
            if ((regval & 0x20U) > 0U) {
                /* swap X,Y */
                if (m_pstcLcdDevice->u16Width < m_pstcLcdDevice->u16Height) {
                    u16Temp = m_pstcLcdDevice->u16Width;
                    m_pstcLcdDevice->u16Width = m_pstcLcdDevice->u16Height;
                    m_pstcLcdDevice->u16Height = u16Temp;
                }
            } else {
                /* swap X,Y */
                if (m_pstcLcdDevice->u16Width > m_pstcLcdDevice->u16Height) {
                    u16Temp = m_pstcLcdDevice->u16Width;
                    m_pstcLcdDevice->u16Width = m_pstcLcdDevice->u16Height;
                    m_pstcLcdDevice->u16Height = u16Temp;
                }
            }
        }

        /* Set display window size */
        if (LCD_ID_NT35510 == m_pstcLcdDevice->u16ID) {
            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetXCmd);
            NT35510_WriteData(pstcLCD, 0U);

            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetXCmd + 1U);
            NT35510_WriteData(pstcLCD, 0U);

            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetXCmd + 2U);
            NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Width - 1U) >> 8U);

            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetXCmd + 3U);
            NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Width - 1U) & 0xFFU);

            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetYCmd);
            NT35510_WriteData(pstcLCD, 0U);

            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetYCmd + 1U);
            NT35510_WriteData(pstcLCD, 0U);

            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetYCmd + 2U);
            NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Height - 1U) >> 8U);

            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetYCmd + 3U);
            NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Height - 1U) & 0xFFU);
        } else {
            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetYCmd);
            NT35510_WriteData(pstcLCD, 0U);
            NT35510_WriteData(pstcLCD, 0U);
            NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Width - 1U) >> 8);
            NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Width - 1U) & 0xFFU);
            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetYCmd);
            NT35510_WriteData(pstcLCD, 0U);
            NT35510_WriteData(pstcLCD, 0U);
            NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Height - 1U) >> 8);
            NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Height - 1U) & 0xFFU);
        }
    }
}

/**
 * @brief  Set screen direction.
 * @param  [in] pstcLCD:                LCD controller
 * @param  [in] u16Dir:                 Screen direction
 *         This parameter can be one of the following values:
 *           @arg LCD_DISPLAY_VERTICAL:   LCD vertical display
 *           @arg LCD_DISPLAY_HORIZONTAL: LCD horizontal display
 * @retval None
 */
void NT35510_SetDisplayDir(stc_lcd_controller_t *pstcLCD, uint16_t u16Dir)
{
    if (NULL != pstcLCD) {
        if (LCD_DISPLAY_VERTICAL == u16Dir) { /* Vertical */
            if (LCD_ID_SSD1963 == m_pstcLcdDevice->u16ID) {
                m_pstcLcdDevice->u16WRamCmd = 0x2CU;
                m_pstcLcdDevice->u16SetXCmd = 0x2BU;
                m_pstcLcdDevice->u16SetYCmd = 0x2AU;
                m_pstcLcdDevice->u16Width  = 480U;
                m_pstcLcdDevice->u16Height = 800U;
            } else if (LCD_ID_NT35510 == m_pstcLcdDevice->u16ID) {
                /* NT35510 */
                m_pstcLcdDevice->u16WRamCmd = 0x2C00U;
                m_pstcLcdDevice->u16SetXCmd = 0x2A00U;
                m_pstcLcdDevice->u16SetYCmd = 0x2B00U;
                m_pstcLcdDevice->u16Width  = 480U;
                m_pstcLcdDevice->u16Height = 800U;
            } else {
                /* NT35310 / 9341 / 5310 / 7789 etc */
                m_pstcLcdDevice->u16WRamCmd = 0x2CU;
                m_pstcLcdDevice->u16SetXCmd = 0x2AU;
                m_pstcLcdDevice->u16SetYCmd = 0x2BU;
                if (LCD_ID_NT35310 == m_pstcLcdDevice->u16ID) {
                    /* NT35310 */
                    m_pstcLcdDevice->u16Width  = 320U;
                    m_pstcLcdDevice->u16Height = 480U;
                } else {
                    m_pstcLcdDevice->u16Width  = 240U;
                    m_pstcLcdDevice->u16Height = 320U;
                }
            }
        } else { /* Horizontal */
            if (LCD_ID_SSD1963 == m_pstcLcdDevice->u16ID) {
                m_pstcLcdDevice->u16WRamCmd = 0x2CU;
                m_pstcLcdDevice->u16SetXCmd = 0x2AU;
                m_pstcLcdDevice->u16SetYCmd = 0x2BU;
                m_pstcLcdDevice->u16Width  = 800U;
                m_pstcLcdDevice->u16Height = 480U;
            } else if (LCD_ID_NT35510 == m_pstcLcdDevice->u16ID) {
                /* NT35510 */
                m_pstcLcdDevice->u16WRamCmd = 0x2C00U;
                m_pstcLcdDevice->u16SetXCmd = 0x2A00U;
                m_pstcLcdDevice->u16SetYCmd = 0x2B00U;
                m_pstcLcdDevice->u16Width  = 800U;
                m_pstcLcdDevice->u16Height = 480U;
            } else {
                /* NT35310 / 9341 / 5310 / 7789 etc */
                m_pstcLcdDevice->u16WRamCmd = 0x2CU;
                m_pstcLcdDevice->u16SetXCmd = 0x2AU;
                m_pstcLcdDevice->u16SetYCmd = 0x2BU;
                if (LCD_ID_NT35310 == m_pstcLcdDevice->u16ID) {
                    /* NT35310 */
                    m_pstcLcdDevice->u16Width  = 480U;
                    m_pstcLcdDevice->u16Height = 320U;
                } else {
                    m_pstcLcdDevice->u16Width  = 320U;
                    m_pstcLcdDevice->u16Height = 240U;
                }
            }
        }

        m_pstcLcdDevice->u16Dir = u16Dir;
        NT35510_SetScanDir(pstcLCD, LCD_SCAN_DIR);
    }
}

/**
 * @brief  Prepare to write LCD RAM.
 * @param  [in] pstcLCD:                LCD controller
 * @retval None
 */
void NT35510_PrepareWriteRAM(stc_lcd_controller_t *pstcLCD)
{
    if (NULL != pstcLCD) {
        NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16WRamCmd);
    }
}

/**
 * @brief  Set screen backlight.
 * @param  [in] pstcLCD:                LCD controller
 * @param  [in] u8PWM:                  PWM level
           This parameter can be a value between Min_Data = 0 and Max_Data = 100
 * @retval None
 */
void NT35510_SetBackLight(stc_lcd_controller_t *pstcLCD, uint8_t u8PWM)
{
    float32_t f32PWM = ((float32_t)u8PWM * 2.55F);

    if (NULL != pstcLCD) {
        NT35510_WriteReg(pstcLCD, 0xBEU);
        NT35510_WriteData(pstcLCD, 0x05U);
        NT35510_WriteData(pstcLCD, (uint16_t)f32PWM);
        NT35510_WriteData(pstcLCD, 0x01U);
        NT35510_WriteData(pstcLCD, 0xFFU);
        NT35510_WriteData(pstcLCD, 0x00U);
        NT35510_WriteData(pstcLCD, 0x00U);
    }
}

/**
 * @brief  Set Cursor position.
 * @param  [in] pstcLCD:                LCD controller
 * @param  u16Xpos:                     Specifies the X position.
 * @param  u16Ypos:                     Specifies the Y position.
 * @retval None
 */
void NT35510_SetCursor(stc_lcd_controller_t *pstcLCD, uint16_t u16Xpos, uint16_t u16Ypos)
{
    if (NULL != pstcLCD) {
        if (LCD_ID_SSD1963 == m_pstcLcdDevice->u16ID) {
            /* Convert X coordinate */
            if (m_pstcLcdDevice->u16Dir == 0U) {
                u16Xpos = m_pstcLcdDevice->u16Width - 1U - u16Xpos;
                NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetXCmd);
                NT35510_WriteData(pstcLCD, 0U);
                NT35510_WriteData(pstcLCD, 0U);
                NT35510_WriteData(pstcLCD, u16Xpos >> 8);
                NT35510_WriteData(pstcLCD, u16Xpos & 0xFFU);
            } else {
                NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetXCmd);
                NT35510_WriteData(pstcLCD, u16Xpos >> 8);
                NT35510_WriteData(pstcLCD, u16Xpos & 0xFFU);
                NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Width - 1U) >> 8);
                NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Width - 1U) & 0xFFU);
            }

            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetYCmd);
            NT35510_WriteData(pstcLCD, u16Ypos >> 8);
            NT35510_WriteData(pstcLCD, u16Ypos & 0xFFU);
            NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Height - 1U) >> 8);
            NT35510_WriteData(pstcLCD, (m_pstcLcdDevice->u16Height - 1U) & 0xFFU);
        } else if (LCD_ID_NT35510 == m_pstcLcdDevice->u16ID) {
            NT35510_WriteRegData(pstcLCD, m_pstcLcdDevice->u16SetXCmd, (u16Xpos >> 8U));
            NT35510_WriteRegData(pstcLCD, (m_pstcLcdDevice->u16SetXCmd + 1U), (u16Xpos & 0xFFU));
            NT35510_WriteRegData(pstcLCD, m_pstcLcdDevice->u16SetYCmd, (u16Ypos >> 8U));
            NT35510_WriteRegData(pstcLCD, (m_pstcLcdDevice->u16SetYCmd + 1U), (u16Ypos & 0xFFU));
        } else {    /* NT35310 / 9341 / 5310 / 7789 etc */
            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetXCmd);
            NT35510_WriteData(pstcLCD, (u16Xpos >> 8));
            NT35510_WriteData(pstcLCD, (u16Xpos & 0xFFU));
            NT35510_WriteReg(pstcLCD, m_pstcLcdDevice->u16SetYCmd);
            NT35510_WriteData(pstcLCD, (u16Ypos >> 8));
            NT35510_WriteData(pstcLCD, (u16Ypos & 0xFFU));
        }
    }
}

/**
 * @brief  Write pixel.
 * @param  [in] pstcLCD:                LCD controller
 * @param  u16Xpos:                     Specifies the X position.
 * @param  u16Ypos:                     Specifies the Y position.
 * @param  u16RGBCode:                  The RGB pixel color in RGB565 format
 * @retval None
 */
void NT35510_WritePixel(stc_lcd_controller_t *pstcLCD, uint16_t u16Xpos, uint16_t u16Ypos, uint16_t u16RGBCode)
{
    if (NULL != pstcLCD) {
        /* Set cursor */
        NT35510_SetCursor(pstcLCD, u16Xpos, u16Ypos);

        /* Prepare to write to LCD RAM */
        NT35510_PrepareWriteRAM(pstcLCD);

        NT35510_WriteData(pstcLCD, u16RGBCode);
    }
}

/**
 * @brief  Draw line.
 * @param  [in] pstcLCD:                LCD controller
 * @param  u16X1:                       Specifies the X position 1.
 * @param  u16X2:                       Specifies the X position 2.
 * @param  u16Y1:                       Specifies the Y position 1.
 * @param  u16Y2:                       Specifies the Y position 2.
 * @param  u16RGBCode:                  The RGB pixel color in RGB565 format
 * @retval None
 */
void NT35510_DrawLine(stc_lcd_controller_t *pstcLCD, uint16_t u16X1, uint16_t u16Y1,
                      uint16_t u16X2, uint16_t u16Y2, uint16_t u16RGBCode)
{
    int16_t t;
    int16_t xerr = 0;
    int16_t yerr = 0;
    int16_t delta_x;
    int16_t delta_y;
    int16_t distance;
    int16_t incx;
    int16_t incy;
    int16_t Row;
    int16_t Col;

    if (NULL != pstcLCD) {
        Row = (int16_t)u16X1;
        Col = (int16_t)u16Y1;
        delta_x = ((int16_t)u16X2 - (int16_t)u16X1);      /* calc delta X, Y*/
        delta_y = ((int16_t)u16Y2 - (int16_t)u16Y1);

        if (delta_x > 0) {
            incx = 1;           /* forward u8Direction */
        } else if (delta_x == 0) {
            incx = 0;           /* vertical line */
        } else {
            incx = -1;          /* reverse direction */
            delta_x = -delta_x;
        }

        if (delta_y > 0) {
            incy = 1;             /* downward direction */
        } else if (delta_y == 0) {
            incy = 0;             /* horizontal line */
        } else {
            incy = -1;            /* upward direction */
            delta_y = -delta_y;
        }

        if (delta_x > delta_y) {
            distance = delta_x; /* set axis */
        } else {
            distance = delta_y;
        }

        for (t = 0; t <= (distance + 1); t++) {
            NT35510_WritePixel(pstcLCD, (uint16_t)Row, (uint16_t)Col, u16RGBCode);   /* draw pixel */

            xerr += delta_x ;
            yerr += delta_y ;

            if (xerr > distance) {
                xerr -= distance;
                Row += incx;
            }

            if (yerr > distance) {
                yerr -= distance;
                Col += incy;
            }
        }
    }
}

/**
 * @brief  Draw a circle.
 * @param  [in] pstcLCD:                LCD controller
 * @param  [in]                         u16Xpos: X position
 * @param  [in]                         u16Ypos: Y position
 * @param  [in]                         u16Radius: Circle radius
 * @param  u16RGBCode:                  The RGB pixel color in RGB565 format
 * @retval None
 */
void NT35510_DrawCircle(stc_lcd_controller_t *pstcLCD, uint16_t u16Xpos, uint16_t u16Ypos,
                        uint16_t u16Radius, uint16_t u16RGBCode)
{
    int32_t  decision;       /* Decision Variable */
    uint32_t current_x;      /* Current X Value */
    uint32_t current_y;      /* Current Y Value */

    if (NULL != pstcLCD) {
        decision = 3 - ((int32_t)u16Radius * 2);
        current_x = 0U;
        current_y = u16Radius;

        while (current_x <= current_y) {
            NT35510_WritePixel(pstcLCD, (u16Xpos + (uint16_t)current_x), (u16Ypos - (uint16_t)current_y), u16RGBCode);
            NT35510_WritePixel(pstcLCD, (u16Xpos + (uint16_t)current_y), (u16Ypos - (uint16_t)current_x), u16RGBCode);
            NT35510_WritePixel(pstcLCD, (u16Xpos + (uint16_t)current_y), (u16Ypos + (uint16_t)current_x), u16RGBCode);
            NT35510_WritePixel(pstcLCD, (u16Xpos + (uint16_t)current_x), (u16Ypos + (uint16_t)current_y), u16RGBCode);
            NT35510_WritePixel(pstcLCD, (u16Xpos - (uint16_t)current_x), (u16Ypos + (uint16_t)current_y), u16RGBCode);
            NT35510_WritePixel(pstcLCD, (u16Xpos - (uint16_t)current_y), (u16Ypos + (uint16_t)current_x), u16RGBCode);
            NT35510_WritePixel(pstcLCD, (u16Xpos - (uint16_t)current_x), (u16Ypos - (uint16_t)current_y), u16RGBCode);
            NT35510_WritePixel(pstcLCD, (u16Xpos - (uint16_t)current_y), (u16Ypos - (uint16_t)current_x), u16RGBCode);
            current_x++;
            /* Bresenham algorithm */
            if (decision < 0) {
                decision += ((4 * (int32_t)current_x) + 6);
            } else {
                decision += (10 + (4 * ((int32_t)current_x - (int32_t)current_y)));
                current_y--;
            }
        }
    }
}

/**
 * @brief  Fill a triangle (between 3 points).
 * @param  [in] pstcLCD:                LCD controller
 * @param  [in] u16X1:                  Point 1 X position
 * @param  [in] u16Y1:                  Point 1 Y position
 * @param  [in] u16X2:                  Point 2 X position
 * @param  [in] u16Y2:                  Point 2 Y position
 * @param  [in] u16X3:                  Point 3 X position
 * @param  [in] u16Y3:                  Point 3 Y position
 * @param  u16RGBCode:                  The RGB pixel color in RGB565 format
 * @retval None
 */
void NT35510_FillTriangle(stc_lcd_controller_t *pstcLCD, uint16_t u16X1, uint16_t u16Y1,
                          uint16_t u16X2, uint16_t u16Y2, uint16_t u16X3, uint16_t u16Y3, uint16_t u16RGBCode)
{
    uint16_t deltax;
    uint16_t deltay;
    int16_t xoff;
    int16_t yoff;
    int16_t xinc1;
    int16_t xinc2;
    int16_t yinc1;
    int16_t yinc2;
    uint16_t den;
    uint16_t num;
    uint16_t numadd;
    uint16_t numpixels;
    uint16_t curpixel;

    if (NULL != pstcLCD) {
        xoff = (int16_t)u16X1;                  /* Start x off at the first pixel */
        yoff = (int16_t)u16Y1;                  /* Start y off at the first pixel */

        /* The difference between the x's */
        if (u16X2 > u16X1) {
            deltax = (u16X2 - u16X1);
        } else {
            deltax = (u16X1 - u16X2);
        }

        /* The difference between the y's */
        if (u16Y2 > u16Y1) {
            deltay = (u16Y2 - u16Y1);
        } else {
            deltay = (u16Y1 - u16Y2);
        }

        if (u16X2 >= u16X1) {
            /* The x-values are increasing */
            xinc1 = 1;
            xinc2 = 1;
        } else {
            /* The x-values are decreasing */
            xinc1 = -1;
            xinc2 = -1;
        }

        if (u16Y2 >= u16Y1) {
            /* The y-values are increasing */
            yinc1 = 1;
            yinc2 = 1;
        } else {
            /* The y-values are decreasing */
            yinc1 = -1;
            yinc2 = -1;
        }

        /* There is at least one x-value for every y-value */
        if (deltax >= deltay) {
            xinc1 = 0;                  /* Don't change the x when numerator >= denominator */
            yinc2 = 0;                  /* Don't change the y for every iteration */
            den = deltax;
            num = (deltax / 2U);
            numadd = deltay;
            numpixels = deltax;         /* There are more x-values than y-values */
        } else {
            /* There is at least one y-value for every x-value */
            xinc2 = 0;                  /* Don't change the x for every iteration */
            yinc1 = 0;                  /* Don't change the y when numerator >= denominator */
            den = deltay;
            num = (deltay / 2U);
            numadd = deltax;
            numpixels = deltay;         /* There are more y-values than x-values */
        }

        for (curpixel = 0U; curpixel <= numpixels; curpixel++) {
            NT35510_DrawLine(pstcLCD, (uint16_t)xoff, (uint16_t)yoff, u16X3, u16Y3, u16RGBCode);

            num += numadd;              /* Increase the numerator by the top of the fraction */

            /* Check if numerator >= denominator */
            if (num >= den) {
                num -= den;             /* Calculate the new numerator value */
                xoff += xinc1;          /* Change the x as appropriate */
                yoff += yinc1;          /* Change the y as appropriate */
            }
            xoff += xinc2;              /* Change the x as appropriate */
            yoff += yinc2;              /* Change the y as appropriate */
        }
    }
}

/**
 * @brief  Draw rectangle.
 * @param  [in] pstcLCD:                LCD controller
 * @param  [in] u16X1:                  Point 1 X position
 * @param  [in] u16Y1:                  Point 1 Y position
 * @param  [in] u16X2:                  Point 2 X position
 * @param  [in] u16Y2:                  Point 2 Y position
 * @param  u16RGBCode:                  The RGB pixel color in RGB565 format
 * @retval None
 */
void NT35510_DrawRectangle(stc_lcd_controller_t *pstcLCD, uint16_t u16X1, uint16_t u16Y1,
                           uint16_t u16X2, uint16_t u16Y2, uint16_t u16RGBCode)
{
    if (NULL != pstcLCD) {
        NT35510_DrawLine(pstcLCD, u16X1, u16Y1, u16X2, u16Y1, u16RGBCode);
        NT35510_DrawLine(pstcLCD, u16X1, u16Y1, u16X1, u16Y2, u16RGBCode);
        NT35510_DrawLine(pstcLCD, u16X1, u16Y2, u16X2, u16Y2, u16RGBCode);
        NT35510_DrawLine(pstcLCD, u16X2, u16Y1, u16X2, u16Y2, u16RGBCode);
    }
}

/**
 * @brief  Clear screen.
 * @param  [in] pstcLCD:                LCD controller
 * @param  u16RGBCode:                  The RGB pixel color in RGB565 format
 * @retval None
 */
void NT35510_Clear(stc_lcd_controller_t *pstcLCD, uint16_t u16RGBCode)
{
    uint32_t i;
    uint32_t u32TotalPoint;

    if (NULL != pstcLCD) {
        /* Set cursor */
        NT35510_SetCursor(pstcLCD, 0U, 0U);
        /* Prepare to write to LCD RAM */
        NT35510_PrepareWriteRAM(pstcLCD);

        u32TotalPoint = (uint32_t)m_pstcLcdDevice->u16Width * (uint32_t)m_pstcLcdDevice->u16Height;
        for (i = 0UL; i < u32TotalPoint; i++) {
            NT35510_WriteData(pstcLCD, u16RGBCode);
        }
    }
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
