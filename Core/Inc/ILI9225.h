/**
 * @author  Ibrahim Ibrahim Nizar
 * @email   ibrncfe@gmail.com
 * @website https://github.com/ibrncfe
 * @link    
 * @version 5.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   ILI9225 TFT LCD Library- HAL API STM Compatible 
 *	
\verbatim
   ----------------------------------------------------------------------
    Copyright (c) 2019 Ibrahim Ibrahim Nizar

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without restriction,
    including without limitation the rights to use, copy, modify, merge,
    publish, distribute, sublicense, and/or sell copies of the Software, 
    and to permit persons to whom the Software is furnished to do so, 
    subject to the following conditions:

    The above copyright notice and this permission notice shall be
    included in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
    AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.
   ----------------------------------------------------------------------
\endverbatim
 */
#ifndef ILI9225_H
#define ILI9225_H

/* C++ detection */
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup OLED ILI9225 LIBRARY
 * @{
 */

/**
 * @defgroup ILI9225
 * @brief    Library description here
 * @{
 *
 * \par Changelog
 *
\verbatim
 Version 1.0
  - First release
\endverbatim
 *
 * \par Dependencies
 *
\verbatim
 - STM32Fxxx HAL
 - defines.h
\endverbatim
 */

#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "spi.h"
#include "string.h"
#include "math.h"
#include <stdio.h>
//#include "fonts.h"


/**
 * @defgroup TM_LIB_Macros
 * @brief    Library defines
 * @{
 */
#define STRING const char *

/* ILI9225 screen size */
#define ILI9225_LCD_WIDTH  220
#define ILI9225_LCD_HEIGHT 176

static uint32_t HEIGHT    = ILI9225_LCD_HEIGHT; ///< Screen height
static uint32_t WIDTH     = ILI9225_LCD_WIDTH; ///< Screen width
#define ILI9225_PIXEL_COUNT	ILI9225_LCD_WIDTH * ILI9225_LCD_HEIGHT

enum autoIncMode_t { R2L_BottomUp, BottomUp_R2L, L2R_BottomUp, BottomUp_L2R, R2L_TopDown, TopDown_R2L, L2R_TopDown, TopDown_L2R };


// Functions defines Macros
#define swap(a, b) { int16_t t = a; a = b; b = t; }
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define min(a,b) (((a)<(b))?(a):(b))

// Functions Control Signal of LCD RS (DC), CS, RST
#define CS_RESET HAL_GPIO_WritePin(ILI9225_CS_GPIO_Port, ILI9225_CS_Pin, GPIO_PIN_RESET)
#define CS_SET HAL_GPIO_WritePin(ILI9225_CS_GPIO_Port, ILI9225_CS_Pin, GPIO_PIN_SET)
 
#define RS_CONTROL_RESET HAL_GPIO_WritePin(ILI9225_RS_GPIO_Port, ILI9225_RS_Pin, GPIO_PIN_RESET)
#define RS_DATA_SET HAL_GPIO_WritePin(ILI9225_RS_GPIO_Port, ILI9225_RS_Pin, GPIO_PIN_SET)

#define ILI9225_RST HAL_GPIO_WritePin(ILI9225_RST_GPIO_Port, ILI9225_RST_Pin, GPIO_PIN_RESET)
#define ILI9225_NOTRST HAL_GPIO_WritePin(ILI9225_RST_GPIO_Port, ILI9225_RST_Pin, GPIO_PIN_SET)

/* Font Properties */
//uint16_t _FONT_DIRECTION_;
//uint16_t _FONT_FILL_;
//uint16_t _FONT_FILL_COLOR_;
//uint16_t _FONT_UNDER_LINE_;
//uint16_t _FONT_UNDER_LINE_COLOR_;

/* SET Font Directions */
#define DIRECTION0      0
#define DIRECTION90     1
#define DIRECTION180    2
#define DIRECTION270    3

#define TRUE	1
#define	FALSE	0

#define false	FALSE
#define true	TRUE



/* Font structure and properties */
struct currentFont
{
    uint8_t* font;
    uint8_t width;
    uint8_t height;
    uint8_t offset;
    uint8_t numchars;
    uint8_t nbrows;
    _Bool    monoSp;
};

extern uint8_t Terminal6x8[];
extern uint8_t Terminal11x16[];
extern uint8_t Terminal12x16[];
extern uint8_t Trebuchet_MS16x21[];



#define readFontByte(addr) (*(const unsigned char *)(addr))
#define FONT_HEADER_SIZE 4 // 1: pixel width of 1 font character, 2: pixel height, 



/* end of font structure */

/* RGB 16-bit color table definition (RG565) */
#define COLOR_BLACK          0x0000      /*   0,   0,   0 */
#define COLOR_WHITE          0xFFFF      /* 255, 255, 255 */
#define COLOR_BLUE           0x001F      /*   0,   0, 255 */
#define COLOR_GREEN          0x07E0      /*   0, 255,   0 */
#define COLOR_RED            0xF800      /* 255,   0,   0 */
#define COLOR_NAVY           0x000F      /*   0,   0, 128 */
#define COLOR_DARKBLUE       0x0011      /*   0,   0, 139 */
#define COLOR_DARKGREEN      0x03E0      /*   0, 128,   0 */
#define COLOR_DARKCYAN       0x03EF      /*   0, 128, 128 */
#define COLOR_CYAN           0x07FF      /*   0, 255, 255 */
#define COLOR_TURQUOISE      0x471A      /*  64, 224, 208 */
#define COLOR_INDIGO         0x4810      /*  75,   0, 130 */
#define COLOR_DARKRED        0x8000      /* 128,   0,   0 */
#define COLOR_OLIVE          0x7BE0      /* 128, 128,   0 */
#define COLOR_GREY           0x8410      /* 128, 128, 128 */
#define COLOR_SKYBLUE        0x867D      /* 135, 206, 235 */
#define COLOR_BLUEVIOLET     0x895C      /* 138,  43, 226 */
#define COLOR_LIGHTGREEN     0x9772      /* 144, 238, 144 */
#define COLOR_DARKVIOLET     0x901A      /* 148,   0, 211 */
#define COLOR_YELLOWGREEN    0x9E66      /* 154, 205,  50 */
#define COLOR_BROWN          0xA145      /* 165,  42,  42 */
#define COLOR_DARKGRAY       0x7BEF      /* 128, 128, 128 */
#define COLOR_SIENNA         0xA285      /* 160,  82,  45 */
#define COLOR_LIGHTBLUE      0xAEDC      /* 172, 216, 230 */
#define COLOR_GREENYELLOW    0xAFE5      /* 173, 255,  47 */
#define COLOR_SILVER         0xC618      /* 192, 192, 192 */
#define COLOR_LIGHTGREY      0xC618      /* 192, 192, 192 */
#define COLOR_LIGHTCYAN      0xE7FF      /* 224, 255, 255 */
#define COLOR_VIOLET         0xEC1D      /* 238, 130, 238 */
#define COLOR_AZUR           0xF7FF      /* 240, 255, 255 */
#define COLOR_BEIGE          0xF7BB      /* 245, 245, 220 */
#define COLOR_MAGENTA        0xF81F      /* 255,   0, 255 */
#define COLOR_TOMATO         0xFB08      /* 255,  99,  71 */
#define COLOR_GOLD           0xFEA0      /* 255, 215,   0 */
#define COLOR_ORANGE         0xFD20      /* 255, 165,   0 */
#define COLOR_SNOW           0xFFDF      /* 255, 250, 250 */
#define COLOR_YELLOW         0xFFE0      /* 255, 255,   0 */


/* ILI9225 LCD Registers */
#define ILI9225_DRIVER_OUTPUT_CTRL      (0x01u)  // Driver Output Control
#define ILI9225_LCD_AC_DRIVING_CTRL     (0x02u)  // LCD AC Driving Control
#define ILI9225_ENTRY_MODE              (0x03u)  // Entry Mode
#define ILI9225_DISP_CTRL1              (0x07u)  // Display Control 1
#define ILI9225_BLANK_PERIOD_CTRL1      (0x08u)  // Blank Period Control
#define ILI9225_FRAME_CYCLE_CTRL        (0x0Bu)  // Frame Cycle Control
#define ILI9225_INTERFACE_CTRL          (0x0Cu)  // Interface Control
#define ILI9225_OSC_CTRL                (0x0Fu)  // Osc Control
#define ILI9225_POWER_CTRL1             (0x10u)  // Power Control 1
#define ILI9225_POWER_CTRL2             (0x11u)  // Power Control 2
#define ILI9225_POWER_CTRL3             (0x12u)  // Power Control 3
#define ILI9225_POWER_CTRL4             (0x13u)  // Power Control 4
#define ILI9225_POWER_CTRL5             (0x14u)  // Power Control 5
#define ILI9225_VCI_RECYCLING           (0x15u)  // VCI Recycling
#define ILI9225_RAM_ADDR_SET1           (0x20u)  // Horizontal GRAM Address Set
#define ILI9225_RAM_ADDR_SET2           (0x21u)  // Vertical GRAM Address Set
#define ILI9225_GRAM_DATA_REG           (0x22u)  // GRAM Data Register
#define ILI9225_GATE_SCAN_CTRL          (0x30u)  // Gate Scan Control Register
#define ILI9225_VERTICAL_SCROLL_CTRL1   (0x31u)  // Vertical Scroll Control 1 Register
#define ILI9225_VERTICAL_SCROLL_CTRL2   (0x32u)  // Vertical Scroll Control 2 Register
#define ILI9225_VERTICAL_SCROLL_CTRL3   (0x33u)  // Vertical Scroll Control 3 Register
#define ILI9225_PARTIAL_DRIVING_POS1    (0x34u)  // Partial Driving Position 1 Register
#define ILI9225_PARTIAL_DRIVING_POS2    (0x35u)  // Partial Driving Position 2 Register
#define ILI9225_HORIZONTAL_WINDOW_ADDR1 (0x36u)  // Horizontal Address Start Position
#define ILI9225_HORIZONTAL_WINDOW_ADDR2 (0x37u)  // Horizontal Address End Position
#define ILI9225_VERTICAL_WINDOW_ADDR1   (0x38u)  // Vertical Address Start Position
#define ILI9225_VERTICAL_WINDOW_ADDR2   (0x39u)  // Vertical Address End Position
#define ILI9225_GAMMA_CTRL1             (0x50u)  // Gamma Control 1
#define ILI9225_GAMMA_CTRL2             (0x51u)  // Gamma Control 2
#define ILI9225_GAMMA_CTRL3             (0x52u)  // Gamma Control 3
#define ILI9225_GAMMA_CTRL4             (0x53u)  // Gamma Control 4
#define ILI9225_GAMMA_CTRL5             (0x54u)  // Gamma Control 5
#define ILI9225_GAMMA_CTRL6             (0x55u)  // Gamma Control 6
#define ILI9225_GAMMA_CTRL7             (0x56u)  // Gamma Control 7
#define ILI9225_GAMMA_CTRL8             (0x57u)  // Gamma Control 8
#define ILI9225_GAMMA_CTRL9             (0x58u)  // Gamma Control 9
#define ILI9225_GAMMA_CTRL10            (0x59u)  // Gamma Control 10

#define ILI9225C_INVOFF  0x20
#define ILI9225C_INVON   0x21

/**
 * @defgroup TM_LIB_Typedefs
 * @brief    Library Typedefs
 * @{
 */
/* Typedefs here */
/**
 * @}
 */
extern SPI_HandleTypeDef hspi5; //ILI9225 Connecting

/**
 * @defgroup TM_LIB_Functions
 * @brief    Library Functions
 * @{
 */
void ILI9225_ini(void);
void ILI9225_WriteCommand(uint8_t reg);
void ILI9225_WriteData(uint8_t reg);
void ILI9225_WriteData16(uint16_t reg);
uint8_t ILI9225_WriteCommand16(uint8_t add, uint16_t reg);

/// Switch display on
/// @param     no
void ILI9225_lcdDisplayOn(void);

/// Switch display off
/// @param     no
void ILI9225_lcdDisplayOff(void);

/// Clear the screen
void ILI9225_clear(void);

/// Clear the screen with color
/// @param     color 16-bit color
void ILI9225_LCD_Clear(uint16_t j);

/// Invert screen
/// @param     flag true to invert, false for normal screen
void ILI9225_invert(uint8_t flag);

/// Set background color
/// @param    color background color, default=black
void ILI9225_setBackgroundColor(uint16_t color);  

/// Set orientation
/// @param     orientation orientation, 0=portrait, 1=right rotated landscape, 2=reverse portrait, 3=left rotated landscape
void ILI9225_setOrientation(uint8_t ori);

void ILI9225_orientCoordinates(uint16_t *x1, uint16_t *y1 );

void ILI9225_setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, enum autoIncMode_t mode);
/// Draw pixel
/// @param    x1 point coordinate, x-axis
/// @param    y1 point coordinate, y-axis
/// @param    color 16-bit color
void ILI9225_lcdDrawPixel(uint16_t x, uint16_t y, uint16_t color);

/// Draw line, rectangle coordinates
/// @param    x1 start point coordinate, x-axis
/// @param    y1 start point coordinate, y-axis
/// @param    x2 end point coordinate, x-axis
/// @param    y2 end point coordinate, y-axis
/// @param    color 16-bit color
void ILI9225_drawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color); 

/// Draw rectangle, rectangle coordinates
/// @param    x1 top left coordinate, x-axis
/// @param    y1 top left coordinate, y-axis
/// @param    x2 bottom right coordinate, x-axis
/// @param    y2 bottom right coordinate, y-axis
/// @param    color 16-bit color
void ILI9225_drawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color); 

/// Draw solid rectangle, rectangle coordinates
/// @param    x1 top left coordinate, x-axis
/// @param    y1 top left coordinate, y-axis
/// @param    x2 bottom right coordinate, x-axis
/// @param    y2 bottom right coordinate, y-axis
/// @param    color 16-bit color
void ILI9225_fillRectangle(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
/// Draw circle
/// @param    x0 center, point coordinate, x-axis
/// @param    y0 center, point coordinate, y-axis
/// @param    radius radius
/// @param    color 16-bit color
void ILI9225_drawCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);

/// Draw solid circle
/// @param    x0 center, point coordinate, x-axis
/// @param    y0 center, point coordinate, y-axis
/// @param    radius radius
/// @param    color 16-bit color
void ILI9225_lcdDrawFillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);

/// Draw ASCII Text (pixel coordinates)
/// @param    x point coordinate, x-axis
/// @param    y point coordinate, y-axis
/// @param    s text string
/// @param    color 16-bit color, default=white
/// @return   x-position behind text
uint16_t ILI9225_drawText(uint16_t x, uint16_t y, STRING s, uint16_t color, uint16_t bgcolor);

/// width of an ASCII Text (pixel )
/// @param    s text string
uint16_t ILI9225_getTextWidth( STRING s ) ;

/// Calculate 16-bit color from 8-bit Red-Green-Blue components
/// @param    red red component, 0x00..0xff
/// @param    green green component, 0x00..0xff
/// @param    blue blue component, 0x00..0xff
/// @return   16-bit color
uint16_t ILI9225_setColor(uint8_t red, uint8_t green, uint8_t blue);

/// Draw triangle, triangle coordinates
/// @param    x1 corner 1 coordinate, x-axis
/// @param    y1 corner 1 coordinate, y-axis
/// @param    x2 corner 2 coordinate, x-axis
/// @param    y2 corner 2 coordinate, y-axis
/// @param    x3 corner 3 coordinate, x-axis
/// @param    y3 corner 3 coordinate, y-axis
/// @param    color 16-bit color
void ILI9225_drawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color); 

/// Draw solid triangle, triangle coordinates
/// @param    x1 corner 1 coordinate, x-axis
/// @param    y1 corner 1 coordinate, y-axis
/// @param    x2 corner 2 coordinate, x-axis
/// @param    y2 corner 2 coordinate, y-axis
/// @param    x3 corner 3 coordinate, x-axis
/// @param    y3 corner 3 coordinate, y-axis
/// @param    color 16-bit color
void ILI9225_fillTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color);

/// Set current font
/// @param    font Font name
void setFont(uint8_t* font, uint8_t monoSp); // default = proportional

/// Get current font
//_currentFont getFont();

/// Draw single character (pixel coordinates)
/// @param    x point coordinate, x-axis
/// @param    y point coordinate, y-axis
/// @param    ch ASCII character
/// @param    color 16-bit color, default=white
/// @return   width of character in display pixels
uint16_t ILI9225_drawChar(int16_t Xpos, int16_t Ypos, unsigned char ch, uint16_t color, uint16_t bg);
/// width of an ASCII character (pixel )
/// @param    ch ASCII character
uint16_t ILI9225_getCharWidth( uint16_t ch ) ;

/**
  * @brief  Draws a bitmap picture loaded in the internal Flash in ARGB888 format (32 bits per pixel).
  * @param  Xpos: Bmp X position in the LCD
  * @param  Ypos: Bmp Y position in the LCD
  * @param  pbmp: Pointer to Bmp picture address in the internal Flash
  * @retval None
  */
void BSP_LCD_DrawBitmap(uint32_t Xpos, uint32_t Ypos, uint16_t *pbmp);
	
	
/// Draw arrow of filling
/// @param x1:Start X coordinate
/// @param y1:Start Y coordinate
/// @param x2:End   X coordinate
/// @param y2:End   Y coordinate
/// @param w:Width of the botom
/// @param color:color
void lcdDrawFillArrow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color);


/// Draw arrow
/// @param x1:Start X coordinate
/// @param y1:Start Y coordinate
/// @param x2:End   X coordinate
/// @param y2:End   Y coordinate
/// @param w :Width of the botom
/// @param color:color
/// Thanks http://k-hiura.cocolog-nifty.com/blog/2010/11/post-2a62.html
void lcdDrawArrow(uint16_t x0,uint16_t y0,uint16_t x1,uint16_t y1,uint16_t w,uint16_t color);


void ILI9225_lcdDrawFillCircle(uint16_t x0, uint16_t y0, uint16_t r, uint16_t color);
void ILI9225_Address_set(unsigned int x1,unsigned int y1,unsigned int x2,unsigned int y2);


/**
 * @}
 */
 
/**
 * @}
 */
 
/**
 * @}
 */

/* C++ detection */
#ifdef __cplusplus
}
#endif

#endif
