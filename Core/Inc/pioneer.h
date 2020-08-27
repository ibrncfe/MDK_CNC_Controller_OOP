/**
 * @author  Ibrahim Ibrahim Nizar
 * @email   ibrncfe@gmail.com
 * @website https://github.com/ibrncfe
 * @link
 * @version 5.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   PIONEER MACHINE 2ND VERSION LIBRARY
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

#ifndef __PIONEER_H
#define __PIONEER_H

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"
#include "x_nucleo_plc01a1.h"
#include "math.h"
#include "ILI9225.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

#define BACK GPIO_PIN_SET
#define FORW GPIO_PIN_RESET

#define RIG GPIO_PIN_SET
#define LEF GPIO_PIN_RESET

#define CHANGE 1
#define NOCHANGE 0

//Defenition part for internal temperature sensor
#define TEMP_SENSOR_AVG_SLOPE_MV_PER_CELSIUS                        2.5f
#define TEMP_SENSOR_VOLTAGE_MV_AT_25                                760.0f
#define ADC_REFERENCE_VOLTAGE_MV                                    3300.0f
#define ADC_MAX_OUTPUT_VALUE                                        4095.0f
#define TEMP110_CAL_VALUE                                           ((uint16_t*)((uint32_t)0x1FFF7A2E))
#define TEMP30_CAL_VALUE                                            ((uint16_t*)((uint32_t)0x1FFF7A2C))
#define adcCalTemp30C 		(float)(*TEMP30_CAL_VALUE) * (REFERENCE_VOLTAGE/CALIBRATION_REFERENCE_VOLTAGE)
#define adcCalTemp110C		(float)(*TEMP110_CAL_VALUE) * (REFERENCE_VOLTAGE/CALIBRATION_REFERENCE_VOLTAGE)
#define TEMP110                                                     110.0f
#define TEMP30

//FEED MOTOR STATE
typedef enum
{
	FORWARD=1U,
	BACKWARD=2U,
	FEED_STOP=0U
}FEED_DIRECTION;

//SERVO MOTOR STATE
typedef enum
{
	RIGHT=1U,
	LEFT=2U,
	MOTION_STOP=0U
}MOTION_DIRECTION;

//MACHINE ERROR STATE
typedef enum
{
  STATE_NORMAL_NOCHANGE=5U,
	STATE_NORMAL_CHANGE=4U,
	STATE_ERROR=3U,
	STATE_ALARM=2U,
	STATE_OK=1U,
	STATE_STOP=0U
}PIONEER_STATE;

//state output by printing msg using bluetooth or usart
typedef enum
{
	USART_OK=1U,
	USART_ERROR=0U
}USART_STATE;

//ALARM STATE
typedef enum
{
	ALARM=1U,
	NOALARM=0U
}ALARM_STATE;

//STOP BUTTON STATE
typedef enum
{
	STOP=0U,
	NOSTOP=1U
}STOP_STATE;

//MODES OF THE MACHINE PROCESS
typedef enum
{
	initial=1U,
	normal=2U,
	error=3U
}modes;

//TYPE OF TARGET IN WHICH MESSAGE SEND
typedef enum
{
	DEBUG=1U,
	BLUETOOTH=2U
}TARGET;

//STRUCTURE STATE OF THE MACHINE
typedef struct
{
	ALARM_STATE EUALARM;			//ALARM
	STOP_STATE	EUSTOP;				//START-STOP
	MOTION_DIRECTION EUMDIR;	//FEED DIRECTION
	FEED_DIRECTION EUFDIR;		//SERVO DIRECTION
	uint32_t UIFRATE;					//FEED RATE
	uint32_t UIMRATE;					//SERVO RATE
}state;


//DECLARATIONS OF THE PIONEER FUNCTIONS
STOP_STATE CHECKSTOP(void);
ALARM_STATE CHECKSERALARM(void);
PIONEER_STATE GO_FEED (uint8_t DIR, uint32_t feed_rate);
PIONEER_STATE GO_MOTION (uint8_t DIR, uint32_t arg_motionrate);
PIONEER_STATE STOP_FEED (void);
PIONEER_STATE STOP_MOTION (void);
PIONEER_STATE basic (void);
FEED_DIRECTION CHECKFDIR(void);
MOTION_DIRECTION CHECKMDIR(void);
PIONEER_STATE CHECKSPD(void);
PIONEER_STATE SETLEDALARM(void);
PIONEER_STATE CLRLEDALARM(void);
PIONEER_STATE SETLEDREADY(void);
PIONEER_STATE CLRLEDREADY(void);
static uint8_t CheckChange(void);
static PIONEER_STATE GetCommands(void);
PIONEER_STATE init (void);
static void printStatus (void);
USART_STATE SEND_DATA(TARGET tDevice, unsigned char* sMessage, uint8_t len);

//FUNCTION TO CONVERT LINEAR SPEED OF TOOL BY FEED RATE
static uint32_t FEEDRATECALC (uint8_t linearspeed);		//based on step MM/RPM
static uint8_t DISTANCECALC (uint32_t feedrate);
static uint32_t LIMITFEED (uint32_t feedrate);
static uint32_t LIMITMOTION (uint32_t motionrate);
static void Ready_LED_Pulses(void);
static void ALARM_LED_Pulses(void);








#endif
