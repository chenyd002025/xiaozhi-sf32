/*
 * SPDX-FileCopyrightText: 2019-2022 SiFli Technologies(Nanjing) Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <rtthread.h>
#include "string.h"
#include "board.h"
#include "drv_io.h"
#include "drv_lcd.h"

#include "log.h"

#ifdef ROW_OFFSET_PLUS
    #define ROW_OFFSET (ROW_OFFSET_PLUS)
#else
    #define ROW_OFFSET (0)  // GC9A01通常不需要行偏移
#endif

/**
 * @brief GC9A01 chip IDs
 */
#define THE_LCD_ID 0x9320

/**
 * @brief  GC9A01 Size
 */
#define  THE_LCD_PIXEL_WIDTH    ((uint16_t)240)
#define  THE_LCD_PIXEL_HEIGHT   ((uint16_t)240)

/**
 * @brief  GC9A01 Registers
 */
#define REG_SLEEP_IN 0x10
#define REG_SLEEP_OUT 0x11
#define REG_DISPLAY_OFF 0x28
#define REG_DISPLAY_ON 0x29
#define REG_WRITE_RAM 0x2C
#define REG_READ_RAM 0x2E
#define REG_CASET 0x2A
#define REG_RASET 0x2B
#define REG_TEARING_EFFECT 0x35
#define REG_MADCTL 0x36  // Memory Access Control
#define REG_COLMOD 0x3A  // Color Mode
#define REG_FRAME_RATE 0xB1  // Frame Rate Control
#define REG_DISPLAY_CTRL 0xB6  // Display Function Control
#define REG_POWER_CTRL1 0xC0  // Power Control 1
#define REG_POWER_CTRL2 0xC1  // Power Control 2
#define REG_VCOM_CTRL1 0xC5  // VCOM Control 1
#define REG_VCOM_CTRL2 0xC7  // VCOM Control 2
#define REG_POSITIVE_GAMMA 0xE0  // Positive Gamma Correction
#define REG_NEGATIVE_GAMMA 0xE1  // Negative Gamma Correction

// #define DEBUG

#ifdef DEBUG
    #define DEBUG_PRINTF(...) LOG_I(__VA_ARGS__)
#else
    #define DEBUG_PRINTF(...)
#endif
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg,
                         uint8_t *Parameters, uint32_t NbParameters);
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue,
                             uint8_t ReadSize);

static LCDC_InitTypeDef lcdc_int_cfg = {
    .lcd_itf = LCDC_INTF_SPI_DCX_1DATA,
    .freq = 24000000,
    .color_mode = LCDC_PIXEL_FORMAT_RGB565,
    .cfg =
        {
            .spi =
                {

                    .dummy_clock = 1,
                    .syn_mode = HAL_LCDC_SYNC_DISABLE,
                    .vsyn_polarity = 0,
                    .vsyn_delay_us = 1000,
                    .hsyn_num = 0,
                },
        },

};

/**
 * @brief  spi read/write mode
 * @param  enable: false - write spi mode |  true - read spi mode
 * @retval None
 */
static void LCD_ReadMode(LCDC_HandleTypeDef *hlcdc, bool enable)
{
    if (HAL_LCDC_IS_SPI_IF(lcdc_int_cfg.lcd_itf))
    {
        if (enable)
        {
            HAL_LCDC_SetFreq(hlcdc, 4000000); // read mode min cycle 300ns
        }
        else
        {
            HAL_LCDC_SetFreq(hlcdc,
                             lcdc_int_cfg.freq); // Restore normal frequency
        }
    }
}

#define LCD_CMD_MV_BIT                                                         \
    (1 << 5) // Row/Column order, 0: normal mode, 1: reverse mode
#define LCD_CMD_MX_BIT                                                         \
    (1 << 6) // Column address order, 0: left to right, 1: right to left
#define LCD_CMD_MY_BIT                                                         \
    (1 << 7) // Row address order, 0: top to bottom, 1: bottom to top
/**
 * @brief  Power on the LCD.
 * @param  None
 * @retval None
 */
static void LCD_Init(LCDC_HandleTypeDef *hlcdc)
{
    uint8_t parameter[15];

    rt_kprintf("DBG:%s %d", __FUNCTION__, __LINE__);
    /* Initialize GC9A01 low level bus layer
     * ----------------------------------*/
    memcpy(&hlcdc->Init, &lcdc_int_cfg, sizeof(LCDC_InitTypeDef));
    HAL_LCDC_Init(hlcdc);

    BSP_LCD_Reset(0); // Reset LCD
    HAL_Delay_us(10);
    BSP_LCD_Reset(1);
    HAL_Delay_us(120);

    /* Software Reset */
    LCD_WriteReg(hlcdc, 0x01, (uint8_t *)NULL, 0);
    HAL_Delay_us(120);

    /* Sleep Out */
    LCD_WriteReg(hlcdc, REG_SLEEP_OUT, (uint8_t *)NULL, 0);
    HAL_Delay_us(120);

    /* Normal Display Mode On */
    LCD_WriteReg(hlcdc, 0x13, (uint8_t *)NULL, 0);
    HAL_Delay_us(10);

    /* Memory Access Control */
    parameter[0] = 0x08;  // MX, MY, RGB mode
    LCD_WriteReg(hlcdc, REG_MADCTL, parameter, 1);

    /* Interface Pixel Format - 16bits/pixel */
    parameter[0] = 0x55;  // RGB565
    LCD_WriteReg(hlcdc, REG_COLMOD, parameter, 1);

    /* Frame Rate Control */
    parameter[0] = 0x00;
    parameter[1] = 0x0F;
    LCD_WriteReg(hlcdc, REG_FRAME_RATE, parameter, 2);

    /* Display Function Control */
    parameter[0] = 0x0A;
    parameter[1] = 0x82;
    parameter[2] = 0x27;
    parameter[3] = 0x00;
    LCD_WriteReg(hlcdc, REG_DISPLAY_CTRL, parameter, 4);

    /* Power Control 1 */
    parameter[0] = 0x19;
    parameter[1] = 0x1A;
    LCD_WriteReg(hlcdc, REG_POWER_CTRL1, parameter, 2);

    /* Power Control 2 */
    parameter[0] = 0x45;
    parameter[1] = 0x45;
    LCD_WriteReg(hlcdc, REG_POWER_CTRL2, parameter, 2);

    /* VCOM Control 1 */
    parameter[0] = 0x90;
    LCD_WriteReg(hlcdc, REG_VCOM_CTRL1, parameter, 1);

    /* VCOM Control 2 */
    parameter[0] = 0x86;
    LCD_WriteReg(hlcdc, REG_VCOM_CTRL2, parameter, 1);

    /* Positive Gamma Correction */
    parameter[0] = 0xD0;
    parameter[1] = 0x00;
    parameter[2] = 0x02;
    parameter[3] = 0x07;
    parameter[4] = 0x0A;
    parameter[5] = 0x28;
    parameter[6] = 0x32;
    parameter[7] = 0x44;
    parameter[8] = 0x42;
    parameter[9] = 0x06;
    parameter[10] = 0x0E;
    parameter[11] = 0x12;
    parameter[12] = 0x14;
    parameter[13] = 0x17;
    LCD_WriteReg(hlcdc, REG_POSITIVE_GAMMA, parameter, 14);

    /* Negative Gamma Correction */
    parameter[0] = 0xD0;
    parameter[1] = 0x00;
    parameter[2] = 0x02;
    parameter[3] = 0x07;
    parameter[4] = 0x0A;
    parameter[5] = 0x28;
    parameter[6] = 0x31;
    parameter[7] = 0x54;
    parameter[8] = 0x47;
    parameter[9] = 0x0E;
    parameter[10] = 0x1C;
    parameter[11] = 0x17;
    parameter[12] = 0x1B;
    parameter[13] = 0x1E;
    LCD_WriteReg(hlcdc, REG_NEGATIVE_GAMMA, parameter, 14);

    /* Set Column Address */
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x00;
    parameter[3] = THE_LCD_PIXEL_WIDTH - 1;
    LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);

    /* Set Row Address */
    parameter[0] = 0x00;
    parameter[1] = 0x00;
    parameter[2] = 0x00;
    parameter[3] = THE_LCD_PIXEL_HEIGHT - 1;
    LCD_WriteReg(hlcdc, REG_RASET, parameter, 4);

    /* Display ON */
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
    HAL_Delay_us(120);
}

/**
 * @brief  Disables the Display.
 * @param  None
 * @retval LCD Register Value.
 */
static uint32_t LCD_ReadID(LCDC_HandleTypeDef *hlcdc)
{
    uint32_t data = 0;

    data = LCD_ReadData(hlcdc, REG_LCD_ID, 3);
    rt_kprintf("DBG:%s %d ID:%0x", __FUNCTION__, __LINE__, data);
    data = 0x8181b3; // 不同厂商的ID可能不一样，因此这里使用固定的ID
    return data;
}

/**
 * @brief  Enables the Display.
 * @param  None
 * @retval None
 */
static void LCD_DisplayOn(LCDC_HandleTypeDef *hlcdc)
{
    rt_kprintf("DBG:%s %d", __FUNCTION__, __LINE__);
    /* Display On */
    LCD_WriteReg(hlcdc, REG_DISPLAY_ON, (uint8_t *)NULL, 0);
}

/**
 * @brief  Disables the Display.
 * @param  None
 * @retval None
 */
static void LCD_DisplayOff(LCDC_HandleTypeDef *hlcdc)
{
    rt_kprintf("DBG:%s %d", __FUNCTION__, __LINE__);
    /* Display Off */
    LCD_WriteReg(hlcdc, REG_DISPLAY_OFF, (uint8_t *)NULL, 0);
}

/**
 * @brief Set LCD & LCDC clip area
 * @param hlcdc LCD controller handle
 * @param Xpos0 - Clip area left coordinate, base on LCD top-left, same as
 * below.
 * @param Ypos0 - Clip area top coordinate
 * @param Xpos1 - Clip area right coordinate
 * @param Ypos1 - Clip area bottom coordinate
 */
static void LCD_SetRegion(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos0,
                          uint16_t Ypos0, uint16_t Xpos1, uint16_t Ypos1)
{
    uint8_t parameter[4];

    // Set LCDC clip area
    HAL_LCDC_SetROIArea(hlcdc, Xpos0, Ypos0, Xpos1, Ypos1);

    // Set LCD clip area
    Xpos0 += ROW_OFFSET;
    Xpos1 += ROW_OFFSET;

    parameter[0] = (Xpos0) >> 8;
    parameter[1] = (Xpos0) & 0xFF;
    parameter[2] = (Xpos1) >> 8;
    parameter[3] = (Xpos1) & 0xFF;
    LCD_WriteReg(hlcdc, REG_CASET, parameter, 4);

    parameter[0] = (Ypos0) >> 8;
    parameter[1] = (Ypos0) & 0xFF;
    parameter[2] = (Ypos1) >> 8;
    parameter[3] = (Ypos1) & 0xFF;
    LCD_WriteReg(hlcdc, REG_RASET, parameter, 4);
}

/**
 * @brief  Writes pixel.
 * @param  Xpos: specifies the X position.
 * @param  Ypos: specifies the Y position.
 * @param  RGBCode: the RGB pixel color
 * @retval None
 */
static void LCD_WritePixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos,
                           uint16_t Ypos, const uint8_t *RGBCode)
{
    uint8_t data = 0;
    /* Set Cursor */
    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);
    LCD_WriteReg(hlcdc, REG_WRITE_RAM, (uint8_t *)RGBCode, 2);
}

/**
 * @brief Send layer data to LCD
 * @param hlcdc LCD controller handle
 * @param RGBCode - Pointer to layer data
 * @param Xpos0 - Layer data left coordinate, base on LCD top-left, same as
 * below.
 * @param Ypos0 - Layer data top coordinate
 * @param Xpos1 - Layer data right coordinate
 * @param Ypos1 - Layer data bottom coordinate
 */
static void LCD_WriteMultiplePixels(LCDC_HandleTypeDef *hlcdc,
                                    const uint8_t *RGBCode, uint16_t Xpos0,
                                    uint16_t Ypos0, uint16_t Xpos1,
                                    uint16_t Ypos1)
{
    uint32_t size;

    // Set default layer data
    HAL_LCDC_LayerSetData(hlcdc, HAL_LCDC_LAYER_DEFAULT, (uint8_t *)RGBCode,
                          Xpos0, Ypos0, Xpos1, Ypos1);
    // Write datas to LCD register
    HAL_LCDC_SendLayerData2Reg_IT(hlcdc, REG_WRITE_RAM, 1);
}

/**
 * @brief  Writes  to the selected LCD register.
 * @param  LCD_Reg: address of the selected register.
 * @retval None
 */
static void LCD_WriteReg(LCDC_HandleTypeDef *hlcdc, uint16_t LCD_Reg,
                         uint8_t *Parameters, uint32_t NbParameters)
{
    HAL_LCDC_WriteU8Reg(hlcdc, LCD_Reg, Parameters, NbParameters);
}

/**
 * @brief  Reads the selected LCD Register.
 * @param  RegValue: Address of the register to read
 * @param  ReadSize: Number of bytes to read
 * @retval LCD Register Value.
 */
static uint32_t LCD_ReadData(LCDC_HandleTypeDef *hlcdc, uint16_t RegValue,
                             uint8_t ReadSize)
{
    uint32_t rd_data = 0;

    LCD_ReadMode(hlcdc, true);

    HAL_LCDC_ReadU8Reg(hlcdc, RegValue, (uint8_t *)&rd_data, ReadSize);

    LCD_ReadMode(hlcdc, false);

    return rd_data;
}

static uint32_t LCD_ReadPixel(LCDC_HandleTypeDef *hlcdc, uint16_t Xpos,
                              uint16_t Ypos)
{
    uint8_t parameter[2];
    uint32_t c;
    uint32_t ret_v;

    parameter[0] = 0x66;
    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    LCD_SetRegion(hlcdc, Xpos, Ypos, Xpos, Ypos);

    /*
        read ram need 8 dummy cycle, and it's result is 24bit color which format
       is:

        6bit red + 2bit dummy + 6bit green + 2bit dummy + 6bit blue + 2bit dummy

    */
    c = LCD_ReadData(hlcdc, REG_READ_RAM, 4);
    c >>= lcdc_int_cfg.cfg.spi.dummy_clock; // revert fixed dummy cycle

    switch (lcdc_int_cfg.color_mode)
    {
    case LCDC_PIXEL_FORMAT_RGB565:
        parameter[0] = 0x55;
        ret_v = (uint32_t)(((c >> 8) & 0xF800) | ((c >> 5) & 0x7E0) |
                           ((c >> 3) & 0X1F));
        break;

    case LCDC_PIXEL_FORMAT_RGB666:
        parameter[0] = 0x66;
        ret_v = (uint32_t)(((c >> 6) & 0x3F000) | ((c >> 4) & 0xFC0) |
                           ((c >> 2) & 0X3F));
        break;

    case LCDC_PIXEL_FORMAT_RGB888:
        /*
           pretend that st7789v can support RGB888,

           treated as RGB666 actually(6bit R + 2bit dummy + 6bit G + 2bit dummy
           + 6bit B + 2bit dummy )

           lcdc NOT support RGB666
        */
        /*
            st7789 NOT support RGB888：

            fill 2bit dummy bit with 2bit MSB of color
        */
        parameter[0] = 0x66;
        ret_v = (uint32_t)((c & 0xFCFCFC) | ((c >> 6) & 0x030303));
        break;

    default:
        RT_ASSERT(0);
        break;
    }

    rt_kprintf("ST7789_ReadPixel %x -> %x\n", c, ret_v);

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);

    return ret_v;
}

static void LCD_SetColorMode(LCDC_HandleTypeDef *hlcdc, uint16_t color_mode)
{
    uint8_t parameter[2];

    /*

    Control interface color format
    ‘011’ = 12bit/pixel ‘101’ = 16bit/pixel ‘110’ = 18bit/pixel ‘111’ = 16M
    truncated

    */
    switch (color_mode)
    {
    case RTGRAPHIC_PIXEL_FORMAT_RGB565:
        /* Color mode 16bits/pixel */
        parameter[0] = 0x55;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB565;
        break;

    /*
       pretend that st7789v can support RGB888,

       treated as RGB666 actually(6bit R + 2bit dummy + 6bit G + 2bit dummy +
       6bit B + 2bit dummy )

       lcdc NOT support RGB666
    */
    case RTGRAPHIC_PIXEL_FORMAT_RGB888:
        /* Color mode 18bits/pixel */
        parameter[0] = 0x66;
        lcdc_int_cfg.color_mode = LCDC_PIXEL_FORMAT_RGB888;
        break;

    default:
        RT_ASSERT(0);
        return; // unsupport
        break;
    }

    LCD_WriteReg(hlcdc, REG_COLOR_MODE, parameter, 1);
    HAL_LCDC_SetOutFormat(hlcdc, lcdc_int_cfg.color_mode);
}

#define ST7789_BRIGHTNESS_MAX 127

static void LCD_SetBrightness(LCDC_HandleTypeDef *hlcdc, uint8_t br)
{
    uint8_t bright = (uint8_t)((int)ST7789_BRIGHTNESS_MAX * br / 100);
    LCD_WriteReg(hlcdc, REG_WBRIGHT, &bright, 1);
}

static const LCD_DrvOpsDef GC9A01_drv = {LCD_Init,
                                         LCD_ReadID,
                                         LCD_DisplayOn,
                                         LCD_DisplayOff,
                                         LCD_SetRegion,
                                         LCD_WritePixel,
                                         LCD_WriteMultiplePixels,
                                         LCD_ReadPixel,
                                         LCD_SetColorMode,
                                         LCD_SetBrightness,
                                         NULL,
                                         NULL};

LCD_DRIVER_EXPORT2(gc9a01, 0x9320, &lcdc_int_cfg, &GC9A01_drv, 1);
