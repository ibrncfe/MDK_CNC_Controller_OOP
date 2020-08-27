/**
 * @author  Ibrahim Ibrahim Nizar
 * @email   ibrncfe@gmail.com
 * @website https://github.com/ibrncfe
 * @link
 * @version 5.0
 * @ide     Keil uVision
 * @license MIT
 * @brief   PIONEER MACHINE RNK 2nd VERSION LIBRARY
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

#ifndef __RNK_H
#define __RNK_H

#include "pioneer.h"

static modes Steps = initial;


typedef enum
{
	RNOCHANGE = 0U,
	RCHANGE = 1U
}RNK_INPUTS_CHANGE;

//FEED MOTOR STATE
typedef enum
{
	RNK_FEED_FORWARD=1U,
	RNK_FEED_BACKWARD=2U,
	RNK_FEED_STOP=0U
}RNK_FEED_DIRECTION;

//SERVO MOTOR ROTATION STATE
typedef enum
{
	RNK_ROTSR_RIGHT=1U,
	RNK_ROTSR_LEFT=2U,
	RNK_ROTSR_STOP=0U
}RNK_ROTSR_DIRECTION;

//STEPPER MOTOR ROTATION STATE
typedef enum
{
	RNK_ROTST_RIGHT=1U,
	RNK_ROTST_LEFT=2U,
	RNK_ROTST_STOP=0U
}RNK_ROTST_DIRECTION;

//STEPPER MOTOR ROTATION STATE
typedef enum
{
	RNK_BUTTON_PRSD_RIGHT=1U,
	RNK_BUTTON_PRSD_LEFT=2U,
	RNK_BUTTON_PRSD_STOP=0U
}RNK_BUTTON_DIRECTION;

//state output by printing msg using bluetooth or usart
typedef enum
{
	RNK_USART_OK=1U,
	RNK_USART_ERROR=0U
}RNK_USART_STATE;

//ALARM STATE
typedef enum
{
	RNK_ALARM=1U,
	RNK_NOALARM=0U
}RNK_ALARM_STATE;

//STOP BUTTON STATE
typedef enum
{
	RNK_STOPPED=0U,
	RNK_NOSTOPPED=1U
}RNK_STOPPED_STATE;

//MACHINE ERROR STATE
typedef enum
{
	RNK_STATE_ERROR=3U,
	RNK_STATE_ALARM=2U,
	RNK_STATE_OK=1U,
	RNK_STATE_STOPPED=0U
}RNK_STATE;

typedef enum
{
	RNK_MODE_NORMAL_WELDING_III = 2U,
  RNK_MODE_NORMAL_SURFACING_WELDING_II = 1U,
	RNK_MODE_NORMAL_BORING_I = 0U,
}RNK_INPUT_MODES;

typedef struct{
	RNK_FEED_DIRECTION EUFDIR; //FEED Direction
	RNK_ROTST_DIRECTION EURSTDIR;
	RNK_ROTSR_DIRECTION EURSRDIR;
	RNK_ALARM_STATE EUALARM;
	RNK_STOPPED_STATE EUSTOPPED;
	RNK_INPUT_MODES Mode;
	RNK_BUTTON_DIRECTION UpperButtonDIR;
	RNK_BUTTON_DIRECTION LowerButtonDIR;
	uint32_t potrateUpper;
	uint32_t potrateLower;	
	uint32_t UIFRATE;
	uint32_t UIRSRRATE;
	uint32_t UIRSTRATE;
	RNK_STATE STATE;
	int32_t temperature;
}RNK_MACHINE;


RNK_STATE RNK_Basic (void);
RNK_STATE RNK_Initialization (void);

static RNK_STATE RNK_GetCommands(void);
static RNK_INPUTS_CHANGE RNK_CheckChange(void);
RNK_STATE RNK_CHECKSPD(void);
static uint32_t Nonlinear_FEEDRATE_POT(uint8_t feedrate_nonlin_in_manual);
static uint32_t Nonlinear_MOTIONRATE_POT(uint8_t motionrate_nonlin_in_manual);
static uint32_t RNK_LIMITFEED (uint32_t feedrate);
static uint32_t RNK_LIMITMOTION (uint32_t motionrate);
RNK_BUTTON_DIRECTION RNK_CHECK_UPPER_BUTTON(void);
RNK_BUTTON_DIRECTION RNK_CHECK_LOWER_BUTTON(void);
RNK_INPUT_MODES RNK_CHECK_MODE_TOGGLE(void);
RNK_ALARM_STATE RNK_CHECKSERALARM(void);
RNK_STOPPED_STATE RNK_CHECKSTOP(void);
RNK_STATE RNK_GO_FEED(RNK_FEED_DIRECTION DIR, uint32_t arg_feedrate);
RNK_STATE RNK_STOP_FEED (void);
RNK_STATE RNK_GO_SERVO (RNK_ROTSR_DIRECTION DIR, uint32_t arg_motionrate);
RNK_STATE RNK_STOP_SERVO (void);
RNK_STATE RNK_GO_STROT (RNK_ROTST_DIRECTION DIR, uint32_t strot_rate);
RNK_STATE RNK_STOP_STROT (void);


#endif
