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

#include "pioneer.h"
#include "snow_tiger.h"


#define VARISTOR_CONT
//#define BT_CONT

#define abs(x) ((x)<0 ? (-x) : (x))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b)	((a) < (b) ? (a) : (b))

#define MINFEED	1300000
#define MINMOTION 65535
#define FEEDRESOLUTION 2000
#define MOTIONRESOLUTION 100
#define MaxFilterAverage 500
#define FIFO_SIZE 100
#define MAX_ITERATION FIFO_SIZE

volatile static uint32_t ADC_Data[2];
uint32_t uADC_DATA[3];

static uint8_t* Input_Data;//Input channels state
static uint8_t output = 0x00;
static uint8_t* foutput = 0;

//static ALARM_STATE ServoAlarm = NOALARM;
//static STOP_STATE StopAlarm = NOSTOP;
static modes Steps = initial;

static uint32_t feedrate = MINFEED;			//MAXIMUM ANGULAR SPEED STEPPER MOTOR
volatile static uint8_t feedrate_average = 0;
volatile static uint32_t feedrate_sum = 0;
uint8_t feedrateReceived = 0;

static uint32_t DIFFERENCE_FEEDRATE = 0;


static uint32_t motionrate = MINMOTION;	//MAXIMUM ANGULAR SPEED SERVO MOTOR
volatile static uint8_t motionrate_average = 0;
volatile static uint32_t motionrate_sum = 0;
uint8_t motionrateReceived = 0;

static uint32_t DIFFERENCE_MOTIONRATE = 0;

extern TIM_HandleTypeDef htim5;

//uint16_t I = 0;

static uint8_t iteration_pulses_leds=0;
static uint8_t* RStatus;//Output relay status
static uint32_t InitialTime = 4;//Time in seconds for system to be ready
static int32_t temperature=0;
static state PIONEER;
static state PIONEER_Past;
static PIONEER_STATE eStPioneer = STATE_OK;
uint8_t *feedSpd = 0;
uint8_t *motionSpd = 0;

uint8_t txPio_buffer[2]={0};
uint8_t rxPio_buffer[5]={0};
uint8_t rxBT[100]={2};

uint8_t dmaRxReady = false;
uint8_t RData = 0x00;
uint8_t ik = 0;
uint16_t i = 0;
uint16_t k = 0;

char ReceivedCommands[5]={0};
char tempStr_5[5];
char tempStr_4[4];
char tempStr_3[3];
uint64_t fdata = 0;


#ifdef BT_CONT
static PIONEER_STATE eSPCheckChange = STATE_OK;

#endif

//#ifdef VARISTOR_CONT
volatile uint8_t iteration =0;
volatile static uint32_t motionrate_fifo_100[FIFO_SIZE];
volatile static uint32_t feedrate_fifo_100[FIFO_SIZE];
uint8_t feedrate_nonlin_in_manual = 0;
uint8_t motionrate_nonlin_in_manual = 0;
//#endif


PIONEER_STATE init (void)
{
	//INITIAL STATE PIONEER 2
	PIONEER.EUSTOP = NOSTOP;
	PIONEER.EUALARM = NOALARM;
	PIONEER.EUMDIR = MOTION_STOP;
	PIONEER.EUFDIR = FEED_STOP;
	PIONEER.UIFRATE = MINFEED;
	PIONEER.UIMRATE = MINMOTION;

	PIONEER_Past.EUSTOP = NOSTOP;
	PIONEER_Past.EUALARM = NOALARM;
	PIONEER_Past.EUMDIR = MOTION_STOP;
	PIONEER_Past.EUFDIR = FEED_STOP;

	/*Initialize the OLED with controller ILI9225*/
	ILI9225_ini();
	ILI9225_LCD_Clear(COLOR_WHITE);
	ILI9225_lcdDisplayOn();
	BSP_LCD_DrawBitmap(5, 0, (uint16_t *)snow_tiger);
	
	//START TIMER 1 FOR LEDS
	HAL_TIM_Base_Start_IT(&htim1);

	//STATE MACHINE- SET INITIAL STATE
	Steps = initial;

	//check the bluetooth module parameters
	HAL_Delay(200);
	SEND_DATA(BLUETOOTH,(uint8_t *) "AT+VERSION", 10);
	SEND_DATA(DEBUG, (uint8_t*)"AT+VERSION\r\n", 12);
	HAL_Delay(1000);
	HAL_UART_Receive(&huart6, (uint8_t*) rxBT, 10, 100);
	SEND_DATA(DEBUG, (uint8_t*) rxBT, 10);	

	SEND_DATA(BLUETOOTH,(uint8_t *) "AT+NAMEPI-2-SARMAT", 21);
  SEND_DATA(DEBUG,(uint8_t *) "AT+NAMEPI-2-SARMAT\r\n", 20);	
	HAL_UART_Receive(&huart6, (uint8_t*) rxBT, 30, 100);
	SEND_DATA(DEBUG, (uint8_t*) rxBT, 30);

//	SEND_DATA(BLUETOOTH,(uint8_t *) "AT+ROLE=S", 9);
//	SEND_DATA(DEBUG, (uint8_t*)"AT+ROLE=S\r\n", 10);
//	HAL_UART_Receive(&huart6, (uint8_t*) rxBT, 20, 100);
//	SEND_DATA(DEBUG, (uint8_t*)rxBT, 20);	SEND_DATA(DEBUG, (uint8_t*)"\r\n", 2);
//	
//	SEND_DATA(BLUETOOTH,(uint8_t *) "AT+CONT=0", 9);
//	SEND_DATA(DEBUG, (uint8_t*)"AT+CONT=1\r\n", 10);
//	HAL_UART_Receive(&huart6, (uint8_t*) rxBT, 20, 100);
//	SEND_DATA(DEBUG, (uint8_t*)rxBT, 20);	SEND_DATA(DEBUG, (uint8_t*)"\r\n", 2);	
	
//	SEND_DATA(BLUETOOTH,(uint8_t *) "AT+RX", 5);
//	SEND_DATA(DEBUG, (uint8_t*)"AT+RX\r\n", 6);
//	//__disable_irq();
//	HAL_UART_Receive(&huart6, (uint8_t*) rxBT, 100, 100);
//	//__enable_irq();
//	SEND_DATA(DEBUG, (uint8_t*)rxBT, 100);
//	SEND_DATA(DEBUG, (uint8_t*)"\r\n", 2);
	
//	SEND_DATA(BLUETOOTH,(uint8_t *) "AT+ADDR=1234567890AB", 20);
//	SEND_DATA(DEBUG, (uint8_t*)"AT+ADDR=1234567890AB", 20);
//	HAL_UART_Receive(&huart6, (uint8_t*) rxBT, 20, 100);
//	SEND_DATA(DEBUG, (uint8_t*)rxBT, 20);	SEND_DATA(DEBUG, (uint8_t*)"\r\n", 2);
		
		//SEND_DATA(BLUETOOTH, (uint8_t*)"HELLO", 5);

	output = 0x00;
	return STATE_OK;
}



PIONEER_STATE basic (void)
{
	//STATE 1 INITIAL MODE
	if (Steps==initial)
	{
		HAL_TIM_Base_Start_IT(&htim10);
	}

	//STATE 2 NORMAL MODE
	if(Steps == normal)
	{
		
		eStPioneer = GetCommands();
		if (eStPioneer == STATE_ALARM)
		{
			return STATE_ERROR;
		}
		else if(eStPioneer == STATE_STOP)
		{
			return STATE_ERROR;
		}
		else if(eStPioneer == STATE_ERROR)
		{
			return STATE_ERROR;
		}
		else if(eStPioneer == STATE_OK)
		{
			return STATE_OK;
		}		
	}
	return STATE_OK;
}


#ifdef VARISTOR_CONT

/*check the change between states- Manual Mode*/
static uint8_t CheckChange(void)
{
	/* clear output */
	output = 0x00;
	
	//check stop button on the station
	if (CHECKSTOP()== STOP)
	{
		PIONEER.EUSTOP = STOP;
		/*Stop the motors*/
		STOP_FEED();
		STOP_MOTION();
		/*change the state to off*/
		PIONEER.EUFDIR = FEED_STOP;
		PIONEER.EUMDIR = MOTION_STOP;
		
		Input_Data = CURRENT_LIMITER_Handler();

		output |= 0x01;
		foutput=&output;

		/* Handler for output relay */
		RStatus = BSP_RELAY_SetOutputs(&output);

		if (BSP_GetRelayStatus(RStatus) != RELAY_OK)
		{
		/* Set output error code here */
		}		
		
		return 0;
	}
		
	
	if (CHECKSERALARM() == ALARM)
	{
		/*change the state to alarm*/
		PIONEER.EUALARM = ALARM;
		/*Stop the motors*/
		STOP_FEED();
		STOP_MOTION();
		/*change the state to off*/
		PIONEER.EUFDIR = FEED_STOP;
		PIONEER.EUMDIR = MOTION_STOP;		
		/*turn on the alarm led*/
//		//HAL_TIM_OC_Start_IT(&htim10, TIM_CHANNEL_1);

		Input_Data = CURRENT_LIMITER_Handler();

		output |= 0x02;
		foutput=&output;

		/* Handler for output relay */
		RStatus = BSP_RELAY_SetOutputs(&output);

		if (BSP_GetRelayStatus(RStatus) != RELAY_OK)
		{
		/* Set output error code here */
		}

//		SETLEDALARM();
//		/*send alarm signal to the remote control*/
//		SEND_DATA(BLUETOOTH,(uint8_t *) "ATL+ALARMON", 11);
//		SEND_DATA(DEBUG, (uint8_t *) "ATL+ALARMON\r\n", 12);
		return 0;
	}
	

	/* clear output */
	output = 0x00;
	
	/* set ready led*/
	SETLEDREADY();	
	
	/* AVERAGE FILTER USING FIFO */ 
	motionrate_sum = 0;
	feedrate_sum = 0;
	//READING THE ADC (Motors' speed rates)
	for(int j=0 ; j < MAX_ITERATION  ; j++)
	{
		motionrate_sum += motionrate_fifo_100[j];
		feedrate_sum += feedrate_fifo_100[j];
	}

	motionrate_average = (uint8_t)((float) motionrate_sum / MAX_ITERATION);
	feedrate_average = (uint8_t)((float) feedrate_sum / MAX_ITERATION);
	/* AVERAGE FILTER END*/
	
	//CALCULATE FOR SERVO AND STEPPER SCALES
	motionrate_nonlin_in_manual = (uint8_t)((double)motionrate_average * 2.55 );
	feedrate_nonlin_in_manual = (uint8_t)((double)feedrate_average * 2.55 );

	//NONLINEAR FUNCTION
	//CALCULATE FOR SERVO AND STEPPER SCALES
	if ( feedrate_nonlin_in_manual <= 10 )
	{
		feedrate = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 10) && (feedrate_nonlin_in_manual <= 30) )
	{
		feedrate = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 30) && (feedrate_nonlin_in_manual <= 50) )
	{
		feedrate = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 50) && (feedrate_nonlin_in_manual <= 90) )
	{
		feedrate = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 90) && (feedrate_nonlin_in_manual <= 120) )
	{
		feedrate = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 120) && (feedrate_nonlin_in_manual <= 160) )
	{
		feedrate = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 160) && (feedrate_nonlin_in_manual <= 180) )
	{
		feedrate = (feedrate_nonlin_in_manual * 20 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 180) && (feedrate_nonlin_in_manual <= 200) )
	{
		feedrate = (feedrate_nonlin_in_manual * 40 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 200) && (feedrate_nonlin_in_manual <= 220) )
	{
		feedrate = (feedrate_nonlin_in_manual * 50 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 220) && (feedrate_nonlin_in_manual <= 240) )
	{
		feedrate = (feedrate_nonlin_in_manual * 70 ) + 1000;
	}
	else if( feedrate_nonlin_in_manual > 240 )
	{
		feedrate = (feedrate_nonlin_in_manual * 120 ) + 1000;
	}
	else
	{
		feedrate = MINFEED;
	}

	//
	
	if (motionrate_nonlin_in_manual <= 50)
	{
		motionrate = (motionrate_nonlin_in_manual * 1 ) + 219;
	}
	else if ( (motionrate_nonlin_in_manual <= 100) & (motionrate_nonlin_in_manual > 50) )
	{
		motionrate = (motionrate_nonlin_in_manual * 1.3 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 130) & (motionrate_nonlin_in_manual > 100) )
	{
		motionrate = (motionrate_nonlin_in_manual * 1.32 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 150) & (motionrate_nonlin_in_manual > 130) )
	{
		motionrate = (motionrate_nonlin_in_manual * 1.38 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 150) & (motionrate_nonlin_in_manual > 140) ) //140-150
	{
		motionrate = (motionrate_nonlin_in_manual * 1.4 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 160) & (motionrate_nonlin_in_manual > 150) ) //150-160
	{
		motionrate = (motionrate_nonlin_in_manual * 1.45 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 170) & (motionrate_nonlin_in_manual > 160) ) //160-170
	{
		motionrate = (motionrate_nonlin_in_manual * 1.5 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 180) & (motionrate_nonlin_in_manual > 170) ) //170-180
	{
		motionrate = (motionrate_nonlin_in_manual * 1.6 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 190) & (motionrate_nonlin_in_manual > 180) ) //180-190
	{
		motionrate = (motionrate_nonlin_in_manual * 1.69 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 200) & (motionrate_nonlin_in_manual > 190) ) //190-200
	{
		motionrate = (motionrate_nonlin_in_manual * 2 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 210) & (motionrate_nonlin_in_manual > 200) ) //200-210
	{
		motionrate = (motionrate_nonlin_in_manual * 2.2 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 220) & (motionrate_nonlin_in_manual > 210) ) //210-220
	{
		motionrate = (motionrate_nonlin_in_manual * 2.5 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 230) & (motionrate_nonlin_in_manual > 220) ) //220-230
	{
		motionrate = (motionrate_nonlin_in_manual * 3 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 240) & (motionrate_nonlin_in_manual > 230) ) //230-240
	{
		motionrate = (motionrate_nonlin_in_manual * 5 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 250) & (motionrate_nonlin_in_manual > 240) ) //240-250
	{
		motionrate = (motionrate_nonlin_in_manual * 7 ) + 220;
	}
	else
	{
		motionrate = (motionrate_nonlin_in_manual * 20 ) + 220;
	}

	//PUT LIMITERS HERE
	motionrate = LIMITMOTION(motionrate);
	feedrate = LIMITFEED(feedrate);

	//UPDATING THE STATE OF PIONEER
	PIONEER.EUFDIR = CHECKFDIR();
	PIONEER.EUMDIR = CHECKMDIR();
	PIONEER.UIFRATE = feedrate;
	PIONEER.UIMRATE = motionrate;

	DIFFERENCE_FEEDRATE = MAX(PIONEER.UIFRATE,  PIONEER_Past.UIFRATE) - MIN(PIONEER.UIFRATE,  PIONEER_Past.UIFRATE);
	DIFFERENCE_MOTIONRATE = MAX(PIONEER.UIMRATE, PIONEER_Past.UIMRATE) - MIN(PIONEER.UIMRATE, PIONEER_Past.UIMRATE);

	//CHECK THE CHANGE
	if ( (PIONEER.EUFDIR == PIONEER_Past.EUFDIR) & (PIONEER.EUMDIR == PIONEER_Past.EUMDIR) &
		( DIFFERENCE_FEEDRATE <= FEEDRESOLUTION )	& (DIFFERENCE_MOTIONRATE <= MOTIONRESOLUTION) )
	{
			return 0; //Nochange
	}
	else
	{
		//IDENTIFY THE TYPE OF CHANGE IN FEED STPPER DRIVER
		if ( (PIONEER.EUFDIR != PIONEER_Past.EUFDIR) | (PIONEER.EUMDIR != PIONEER_Past.EUMDIR) )		//check the condition -1-
		{
			if ((PIONEER.EUFDIR != PIONEER_Past.EUFDIR))
			{
				PIONEER_Past.EUFDIR = PIONEER.EUFDIR;
				HAL_Delay(300);
				STOP_FEED();
				HAL_Delay(200);
			}
			else
			{
				PIONEER_Past.EUMDIR = PIONEER.EUMDIR;
				HAL_Delay(300);
				STOP_MOTION();
				HAL_Delay(300);
			}
		}
		else if ( DIFFERENCE_FEEDRATE > FEEDRESOLUTION )
		{
			PIONEER_Past.UIFRATE = PIONEER.UIFRATE;
		}
		else if (DIFFERENCE_MOTIONRATE > MOTIONRESOLUTION)
		{
			PIONEER_Past.UIMRATE = PIONEER.UIMRATE;
		}

		return 1; //Change
	}
}

#endif


#ifdef BT_CONT
/*check the change between states- Bluetooth mode*/
static PIONEER_STATE CheckChangeBT(void)
{
	//checking alarm
	if (CHECKSERALARM() == ALARM)
	{
//		/*change the state to alarm*/
//		PIONEER.EUALARM = ALARM;
//		/*Stop the motors*/
//		STOP_FEED();
//		STOP_MOTION();
//		/*change the state to off*/
//		PIONEER.EUFDIR = FEED_STOP;
//		PIONEER.EUMDIR = MOTION_STOP;		
//		/*turn on the alarm led*/
//		//HAL_TIM_OC_Start_IT(&htim10, TIM_CHANNEL_1);
//		SETLEDALARM();
//		/*send alarm signal to the remote control*/
//		SEND_DATA(BLUETOOTH,(uint8_t *) "ATL+ALARMON", 11);
//		SEND_DATA(DEBUG, (uint8_t *) "ATL+ALARMON\r\n", 12);
//		return STATE_ALARM;
	}

	//checking stop button
	if (CHECKSTOP()== STOP)
	{
//		PIONEER.EUSTOP = STOP;
//		/*Stop the motors*/
//		STOP_FEED();
//		STOP_MOTION();
//		/*change the state to off*/
//		PIONEER.EUFDIR = FEED_STOP;
//		PIONEER.EUMDIR = MOTION_STOP;
//		/*send alarm signal to the remote control*/
//		SEND_DATA(BLUETOOTH,(uint8_t *) "ATL+STATEOF", 11);
//		SEND_DATA(DEBUG, (uint8_t *) "ATL+STATEOF\r\n", 12);		
//		return STATE_STOP;
	}
			
	HAL_UART_Receive_IT(&huart6, rxPio_buffer, 5);

	//CHECK THE CHANGE
	if ( (PIONEER.EUFDIR == PIONEER_Past.EUFDIR) & (PIONEER.EUMDIR == PIONEER_Past.EUMDIR) )
	{
		//if no change return 0
		return STATE_NORMAL_NOCHANGE;
	}
	else
	{
		//IDENTIFY THE TYPE OF CHANGE IN FEED STPPER DRIVER
		if ( (PIONEER.EUFDIR != PIONEER_Past.EUFDIR) | (PIONEER.EUMDIR != PIONEER_Past.EUMDIR) )		//check the condition -1-
		{
			if ((PIONEER.EUFDIR != PIONEER_Past.EUFDIR))
			{
				PIONEER_Past.EUFDIR = PIONEER.EUFDIR;
				HAL_Delay(300);
				STOP_FEED();
				HAL_Delay(200);
			}
			else
			{
				PIONEER_Past.EUMDIR = PIONEER.EUMDIR;
				HAL_Delay(300);
				STOP_MOTION();
				HAL_Delay(300);
			}
		}
		else if ( DIFFERENCE_FEEDRATE > FEEDRESOLUTION )
		{
			//PIONEER_Past.UIFRATE = PIONEER.UIFRATE;
		}
		else if (DIFFERENCE_MOTIONRATE > MOTIONRESOLUTION)
		{
			//PIONEER_Past.UIMRATE = PIONEER.UIMRATE;
		}

		return STATE_NORMAL_CHANGE;
	}
	
}

#endif

/*check the inputs from serial or REMOTE CONTROL*/
static PIONEER_STATE GetCommands(void)
{
/*Set control using the wired remote control*/
#ifdef VARISTOR_CONT

	//RUN THE ADC
	HAL_ADC_Start_DMA(&hadc1, uADC_DATA, 3);

	while(1)
	{
		while (CheckChange() == 0);

		if (PIONEER.EUFDIR == FORWARD)
		{
			GO_FEED(FORW, PIONEER.UIFRATE);
		}
		else if (PIONEER.EUFDIR == BACKWARD)
		{
			GO_FEED(BACK, PIONEER.UIFRATE);
		}
		else
		{
			STOP_FEED();
		}

		if (RIGHT == CHECKMDIR())
		{
			GO_MOTION(LEF, PIONEER.UIMRATE);
		}
		else if (LEFT == CHECKMDIR())
		{
			GO_MOTION(RIG, PIONEER.UIMRATE);
		}
		else
		{
			STOP_MOTION();
		}
	}
#endif


/*Set control using the wireless remote control*/
#ifdef BT_CONT

//	motionrate_average = 0;
//	feedrate_average = 0;
	//HAL_TIM_Base_Stop_IT(&htim10);

	eSPCheckChange = CheckChangeBT();
	
	while(eSPCheckChange == STATE_NORMAL_NOCHANGE)
	{
		/*LAUNCH START LED*/
		SETLEDREADY();
		
		/*MONITOR DATA RECEIVED */
		eSPCheckChange = CheckChangeBT();

	}
	
	if (eSPCheckChange == STATE_ALARM)
	{
		return STATE_ALARM;
	}
	else if (eSPCheckChange == STATE_STOP)
	{
		return STATE_STOP;
	}
	else
	{
		/*STOP THE ALARM BLINKING*/
		//HAL_TIM_OC_Stop_IT(&htim10, TIM_CHANNEL_1);
		
		if (PIONEER.EUSTOP == STOP)
		{
			STOP_FEED();
			STOP_MOTION();
			PIONEER.EUFDIR = FEED_STOP;
			PIONEER.EUMDIR = MOTION_STOP;
		}

		if (PIONEER.EUFDIR == FORWARD)
		{
			GO_FEED(FORW, PIONEER.UIFRATE);
		}
		else if (PIONEER.EUFDIR == BACKWARD)
		{
			GO_FEED(BACK, PIONEER.UIFRATE);
		}
		else
		{
			STOP_FEED();
		}
		
		if (PIONEER.EUMDIR == RIGHT)
		{
			if (PIONEER.UIMRATE != MINMOTION)
			{
			GO_MOTION(RIG, PIONEER.UIMRATE);
			}
		}
		else if (PIONEER.EUMDIR == LEFT)
		{
			GO_MOTION(LEF, PIONEER.UIMRATE);
		}
		else
		{
			STOP_MOTION();
		}
	}
		
#endif

	return STATE_OK;
}

static uint32_t LIMITMOTION (uint32_t motionrate)
{
	if (motionrate <= 220)
	{
		return 220;
	}
	else if (motionrate > 65535)
	{
		return 65535;
	}
	else
	{
		return motionrate;
	}
}

static uint32_t LIMITFEED (uint32_t feedrate)
{
	if (feedrate < 1300)
		return 1350;
	else if (feedrate > MINFEED)
		return MINFEED;
	else
		return feedrate;
}

FEED_DIRECTION CHECKFDIR(void)
{
	static uint8_t* clData = NULL;
  static uint8_t* Current_Limiter_Status;//Input current limiter status
  clData = BSP_CURRENT_LIMITER_Read();

  Current_Limiter_Status = clData;

  if (BSP_GetCurrentLimiterStatus(Current_Limiter_Status) != CURRENT_LIMITER_OK)
  {
    /* Set input error code here */
  }

	if ((*(clData+1)) & 0x04)
	{
		return FORWARD;
	}
	else if ((*(clData+1)) & 0x08)
	{
		return BACKWARD;
	}
	else
		return FEED_STOP;
}

MOTION_DIRECTION CHECKMDIR(void)
{
	static uint8_t* clData = NULL;
  static uint8_t* Current_Limiter_Status;//Input current limiter status
  clData = BSP_CURRENT_LIMITER_Read();

  Current_Limiter_Status = clData;

  if (BSP_GetCurrentLimiterStatus(Current_Limiter_Status) != CURRENT_LIMITER_OK)
  {
    /* Set input error code here */
  }
	if ((*(clData+1)) & 0x10)
	{
		return RIGHT;
	}
	else if ((*(clData+1)) & 0x20)
	{
		return LEFT;
	}
	else
		return MOTION_STOP;
}



PIONEER_STATE SETLEDALARM(void)
{
	Input_Data = CURRENT_LIMITER_Handler();

	output |= 0x02;
	foutput=&output;//(*output) | 0x06;

	/* Handler for output relay */
  RStatus = BSP_RELAY_SetOutputs(&output);

  if (BSP_GetRelayStatus(RStatus) != RELAY_OK)
  {
    /* Set output error code here */
  }

return STATE_OK;
}

PIONEER_STATE CLRLEDALARM(void)
{
	Input_Data = CURRENT_LIMITER_Handler();

	output &= 0xFE;
	foutput = &output;

	/* Handler for output relay */
  RStatus = BSP_RELAY_SetOutputs(&output);

  if (BSP_GetRelayStatus(RStatus) != RELAY_OK)
  {
    /* Set output error code here */
  }

return STATE_OK;
}

PIONEER_STATE SETLEDREADY(void)
{
	Input_Data = CURRENT_LIMITER_Handler();

	output |= 0x02;
	foutput=&output;

  RStatus = BSP_RELAY_SetOutputs(&output);
	//BSP_Output_ON(output);
  if (BSP_GetRelayStatus(RStatus) != RELAY_OK)
  {
    /* Set output error code here */
  }
return STATE_OK;
}

PIONEER_STATE CLRLEDREADY(void)
{
	Input_Data = CURRENT_LIMITER_Handler();

	output &= 0xFD;
	foutput = &output;
    /* Handler for output relay */
  RStatus = BSP_RELAY_SetOutputs(&output);

  if (BSP_GetRelayStatus(RStatus) != RELAY_OK)
  {
    /* Set output error code here */
  }

return STATE_OK;
}



//ALARM (it should be checked again)
ALARM_STATE CHECKSERALARM(void)
{
	static uint8_t* clData = NULL;
  static uint8_t* Current_Limiter_Status;//Input current limiter status
  clData = BSP_CURRENT_LIMITER_Read();

  Current_Limiter_Status = clData;

  if (BSP_GetCurrentLimiterStatus(Current_Limiter_Status) != CURRENT_LIMITER_OK)
  {
    /* Set input error code here */
		//ERROR CODE 11
  }

	//alarm (clData+1);
	if ((*(clData+1)) & 0x02)
	{
		ILI9225_setWindow(0, 200, 175, 219, TopDown_L2R);
		ILI9225_lcdDrawFillCircle(146+15, 198+13, 8, COLOR_WHITE);
		return NOALARM;
	}
	else
	{
		ILI9225_setWindow(0, 200, 175, 219, TopDown_L2R);
		ILI9225_lcdDrawFillCircle(146+15, 198+13, 8, COLOR_RED);
		return ALARM;
	}
}



//AVARIA (emergency button)
STOP_STATE CHECKSTOP(void)
{
	static uint8_t* clData = NULL;
  static uint8_t* Current_Limiter_Status;//Input current limiter status
  clData = BSP_CURRENT_LIMITER_Read();

  Current_Limiter_Status = clData;

  if (BSP_GetCurrentLimiterStatus(Current_Limiter_Status) != CURRENT_LIMITER_OK)
  {
    /* Set input error code here */
  }

	if ((*(clData+1)) & 0x01)
	{
		return NOSTOP;
	}
	else
	{
		return STOP;
	}
}

//FEED MOTOR CONTROL
PIONEER_STATE GO_FEED (uint8_t DIR, uint32_t feedrate)
{

	//CHANGE SETTING TIMER 5 CHANNEL 2
	__HAL_TIM_SET_PRESCALER	(&htim5, 1);
	__HAL_TIM_SET_AUTORELOAD(&htim5, feedrate);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, feedrate/2);

	//enable
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(1);

	//dir
	if (DIR==FORW)
	{
		HAL_GPIO_WritePin(FDIR_GPIO_Port, FDIR_Pin, FORW);
	}
	else
	{
		HAL_GPIO_WritePin(FDIR_GPIO_Port, FDIR_Pin, BACK);
	}

	HAL_Delay(1);

	//pulse
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_2);

	return STATE_OK;
}


PIONEER_STATE STOP_FEED (void)
{
	HAL_TIM_PWM_Stop_IT(&htim5, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

	return STATE_OK;
}



//MOTION MOTOR CONTROL
PIONEER_STATE GO_MOTION (uint8_t DIR, uint32_t arg_motionrate)
{
	//CHANGE SETTING TIMER 3 CHANNEL 1
	__HAL_TIM_SET_PRESCALER	(&htim3, 1);
	__HAL_TIM_SET_AUTORELOAD(&htim3, arg_motionrate);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arg_motionrate/2);

	//dir
	if (DIR==RIG)
	{
		HAL_GPIO_WritePin(SIGN_GPIO_Port, SIGN_Pin, RIG);
	}
	else
	{
		HAL_GPIO_WritePin(SIGN_GPIO_Port, SIGN_Pin, LEF);
	
	}

	HAL_Delay(1);

	//pulse
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

	return STATE_OK;
}

PIONEER_STATE STOP_MOTION (void)
{
	HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	return STATE_OK;
}


//READ ADC + TEMPERATURE
PIONEER_STATE CHECKSPD(void)
{
	/*Reading Analog Inputs*/
	HAL_ADC_Start_DMA(&hadc1, uADC_DATA, 3);

	ADC_Data[0] = uADC_DATA[0] * 0.0244;
	ADC_Data[1] = uADC_DATA[1] * 0.0244;

	temperature = (uint32_t)(((0.754 * 1000.0 - (double)uADC_DATA[2] * 0.8) / 2.5) + 25.0);

return STATE_OK;
}


//PRINTING STATUS SCREEN ON OLED ILI9225
static void printStatus (void)
{
			/*					*/
			ILI9225_LCD_Clear(COLOR_BLACK);

			ILI9225_drawRectangle(5, 5, ILI9225_LCD_HEIGHT-5, 40, COLOR_WHITE);

			/*					*/
			ILI9225_drawChar(10, 10, 'C', COLOR_WHITE, COLOR_BLACK);
			ILI9225_drawChar(25, 10, 'A', COLOR_RED, 	 COLOR_BLACK);
			ILI9225_drawChar(40, 10, 'P', COLOR_WHITE, COLOR_BLACK);
			ILI9225_drawChar(58, 10, 'M', COLOR_WHITE, COLOR_BLACK);
			ILI9225_drawChar(78, 10, 'A', COLOR_WHITE, COLOR_BLACK);
			ILI9225_drawChar(93, 10, 'T', COLOR_WHITE, COLOR_BLACK);

			/*					*/
			ILI9225_drawChar(10+5+5, 42, 'P', COLOR_WHITE, COLOR_BLACK);
			ILI9225_drawChar(25+5+5, 42, 'I', COLOR_WHITE, COLOR_BLACK);
			ILI9225_drawChar(40+5+5, 42, 'O', COLOR_WHITE, COLOR_BLACK);
			ILI9225_drawChar(58+5+5, 42, 'N', COLOR_WHITE, COLOR_BLACK);
			ILI9225_drawChar(78+2+5, 42, 'E', COLOR_WHITE, COLOR_BLACK);
			ILI9225_drawChar(93+5+5, 42, 'R', COLOR_WHITE, COLOR_BLACK);
			ILI9225_drawChar(120+5+5 , 42, '2', COLOR_WHITE, COLOR_BLACK);

			/*					*/

			/* Draw two Progress Bar*/
			ILI9225_setWindow(0, 62, 0+50, 62+30, TopDown_L2R);
			ILI9225_drawChar(0, 63+5, 'M', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_drawChar(15+5, 63+5, '1', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_fillRectangle(70, 62+5+5, 175, 92-5+5, COLOR_WHITE);
			ILI9225_fillRectangle(70, 62+5+5, 70, 92-5+5, COLOR_BLUE);

			ILI9225_setWindow(0, 92, 0+50, 92+30, TopDown_L2R);
			ILI9225_drawChar(0, 93+5, 'M', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_drawChar(17+5, 93+5, '2', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_fillRectangle(70, 92+5+5, 175, 122-5+5, COLOR_WHITE);
			ILI9225_fillRectangle(70, 92+5+5, 70, 122-5+5, COLOR_GREEN);

			/* Draw directions*/
			ILI9225_setWindow(0, 97, 175, 180, TopDown_L2R);
			
			//print LABELS M1D &  M2D
			ILI9225_drawChar(10, 130+30, 'M', COLOR_LIGHTGREY, COLOR_AZUR);
			ILI9225_drawChar(10+20, 130+30, '1', COLOR_LIGHTGREY, COLOR_AZUR);
			ILI9225_drawChar(10+20+17, 130+30, 'D', COLOR_ORANGE, COLOR_AZUR);

			ILI9225_drawChar(110, 130+30, 'M', COLOR_LIGHTGREY, COLOR_RED);
			ILI9225_drawChar(110+20, 130+30, '2', COLOR_LIGHTGREY, COLOR_RED);
			ILI9225_drawChar(110+20+17, 130+30, 'D', COLOR_BLACK, COLOR_RED);

			//print the values of the directions
			ILI9225_setWindow(0, 160, 175, 200, TopDown_L2R);
			
			ILI9225_drawChar(10, 150+30, '-', COLOR_RED, COLOR_AZUR);
			ILI9225_drawChar(10+20, 150+30, 'S', COLOR_WHITE, COLOR_RED);

			ILI9225_drawChar(110, 150+30, '-', COLOR_RED, COLOR_AZUR);
			ILI9225_drawChar(110+20, 150+30, 'S', COLOR_WHITE, COLOR_RED);

			/* Draw Two Boolean Circle*/
			ILI9225_setWindow(0, 200, 175, 219, TopDown_L2R);
			
			ILI9225_drawChar(0, 199, 'R', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_drawChar(0+15, 199, 'D', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_drawChar(18+17, 199, 'Y', COLOR_YELLOW, COLOR_BLACK);

			ILI9225_drawChar(89, 199, 'A', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_drawChar(88+15, 199, 'R', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_drawChar(107+15, 199, 'M', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_lcdDrawFillCircle(58+15, 198+13, 8, COLOR_RED);
			ILI9225_lcdDrawFillCircle(146+15, 198+13, 8, COLOR_RED);


			/* Draw Error Code Serial Number station*/
}

//PRINTING TEXT OVER BLUETOOTH OR DEBUG USING SERIAL USART
USART_STATE SEND_DATA(TARGET tDevice, uint8_t* sMessage, uint8_t len)
{ 
  if (tDevice == DEBUG)
	{
//		__disable_irq();
		if (HAL_UART_Transmit(&huart2, sMessage, len, 10) != HAL_OK)
		{
//			__enable_irq();
			return USART_ERROR;
		}
	}
	if (tDevice == BLUETOOTH)
	{
//		__disable_irq();
		if (HAL_UART_Transmit(&huart6, sMessage, len, 10) != HAL_OK)
		{
//			__enable_irq();
			return USART_ERROR;
		}
	}
	__enable_irq();
	return USART_OK;
}
 

//TOGGLEING H2 READY LED
static void Ready_LED_Pulses(void)
{
		if (iteration_pulses_leds==1)
		{
			iteration_pulses_leds=0;
			if (output & 0xFD)
			{
				//CLRLEDREADY();
			}
			else
			{
				SETLEDREADY();
			}
		}
		iteration_pulses_leds++;
}


//TOGGLEING H3 ALARM LED
static void ALARM_LED_Pulses(void)
{
			if (iteration_pulses_leds==1)
		{
			iteration_pulses_leds=0;
			if (output & 0xFE)
			{
				//CLRLEDALARM();
			}
			else
			{
				SETLEDALARM();
			}
		}
		iteration_pulses_leds++;
}

/*
Communication between control panel and station
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
}
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
}

/*Interrupt routine of receiving frames through bluetooth*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
if (huart->Instance == USART6)
{
	if (rxPio_buffer[0] == 0x50)
	{
	SEND_DATA(DEBUG, (uint8_t*)rxPio_buffer, 5);
	}
	/* Receive commands from computer terminal (prototyping test)*/
	//	HAL_UART_Receive(&huart2, (uint8_t*)rxPio_buffer, 5, 100); //used above

	/* GET COMMANDS FILL DATA */
	if (rxPio_buffer[0] == 0x50 & rxPio_buffer[1] == 0x49 & rxPio_buffer[4] == 0x00)
	{
		if ( (rxPio_buffer[2] == 0x00) & (rxPio_buffer[3] == 0x00))	//Update command
		{
			SEND_DATA(BLUETOOTH,(uint8_t *) "UPDT", 4);
			SEND_DATA(DEBUG,(uint8_t *) "UPDT", 4);			
		}
		else if (rxPio_buffer[2] == 0x01)	//address STEPPER_SPEED
		{
			feedrateReceived = rxPio_buffer[3];

			/*Print speed progress bar on OLED Screen*/
			ILI9225_setWindow(0, 92, 0+50, 92+30, TopDown_L2R);
			ILI9225_drawChar(0, 93+5, 'M', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_drawChar(17+15, 93+5, '2', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_fillRectangle(70, 92+5+5, 175, 122-5+5, COLOR_WHITE);
			ILI9225_fillRectangle(70, 92+5+5, 70+ (105-(feedrateReceived/2.428)), 122-5+5, COLOR_GREEN);

			/*
			it contains a condition of feedrateReceived == 0 
			for interference problem in sending and receiving frames
			*/
			if (0)
			{
				SEND_DATA(BLUETOOTH,(uint8_t *) "ERR3", 4);
				SEND_DATA(DEBUG,(uint8_t *) "ERR3", 4);
				//HAL_UART_Transmit(&huart2,  (uint8_t *) "ERR3", 4, 10);
			}
			else
			{
				//CALCULATE FOR SERVO AND STEPPER SCALES
				//NONLINEAR FUNCTION
				if ( feedrateReceived <= 10 )
				{
					feedrate = (feedrateReceived * 15 ) + 1000;
				}
				else if( (feedrateReceived > 10) && (feedrateReceived <= 30) )
				{
					feedrate = (feedrateReceived * 15 ) + 1000;
				}
				else if( (feedrateReceived > 30) && (feedrateReceived <= 50) )
				{
					feedrate = (feedrateReceived * 15 ) + 1000;
				}
				else if( (feedrateReceived > 50) && (feedrateReceived <= 90) )
				{
					feedrate = (feedrateReceived * 15 ) + 1000;
				}
				else if( (feedrateReceived > 90) && (feedrateReceived <= 120) )
				{
					feedrate = (feedrateReceived * 15 ) + 1000;
				}
				else if( (feedrateReceived > 120) && (feedrateReceived <= 160) )
				{
					feedrate = (feedrateReceived * 15 ) + 1000;
				}
				else if( (feedrateReceived > 160) && (feedrateReceived <= 180) )
				{
					feedrate = (feedrateReceived * 20 ) + 1000;
				}
				else if( (feedrateReceived > 180) && (feedrateReceived <= 200) )
				{
					feedrate = (feedrateReceived * 40 ) + 1000;
				}
				else if( (feedrateReceived > 200) && (feedrateReceived <= 220) )
				{
					feedrate = (feedrateReceived * 50 ) + 1000;
				}
				else if( (feedrateReceived > 220) && (feedrateReceived <= 240) )
				{
					feedrate = (feedrateReceived * 70 ) + 1000;
				}
				else if( feedrateReceived > 240 )
				{
					feedrate = (feedrateReceived * 120 ) + 1000;
				}
				else
				{
					feedrate = MINFEED;
				}

			/*limit the speed */
			feedrate = LIMITFEED(feedrate);
				
			/*save the speed */	
			PIONEER.UIFRATE = feedrate;
				
			/*prepare the reply message and send it*/
			if (feedrateReceived > 100)
			{
				sprintf(tempStr_5, "%s%d", "M", feedrateReceived);
				SEND_DATA(BLUETOOTH, (uint8_t *) tempStr_5, 4);
				SEND_DATA(DEBUG, (uint8_t *) tempStr_5, 4);
			}
			else if ( (feedrateReceived > 10) && (feedrateReceived <100) )
			{
				sprintf(tempStr_4, "%s%d", "M", feedrateReceived);
				SEND_DATA(BLUETOOTH, (uint8_t *) tempStr_4, 3);
				SEND_DATA(DEBUG, (uint8_t *) tempStr_4, 3);
			}
			else
			{
				sprintf(tempStr_3, "%s%d", "M", feedrateReceived);
				SEND_DATA(BLUETOOTH, (uint8_t *) tempStr_3, 2);
				SEND_DATA(DEBUG, (uint8_t *) tempStr_3, 2);	
			}
			}

		}
		else if (rxPio_buffer[2] == 0x02)	//address STEPPER_DIR
		{
			if (PIONEER.UIFRATE != MINFEED)
			{
				if (rxPio_buffer[3] == 0x00)
				{
					PIONEER.EUFDIR = FORWARD;
					SEND_DATA(BLUETOOTH,(uint8_t *) "MFWD", 4);
					SEND_DATA(DEBUG,(uint8_t *) "MFWD", 4);										
				}
				else if (rxPio_buffer[3] == 0xFF)
				{
					PIONEER.EUFDIR = BACKWARD;
					SEND_DATA(BLUETOOTH,(uint8_t *) "MBCK", 4);
					SEND_DATA(DEBUG,(uint8_t *) "MBCK", 4);
				}
				else 
				{
					//error
				}
			}
		}
		else if (rxPio_buffer[2] == 0x03)	//address STEPPER_EN
		{
			if (rxPio_buffer[3] == 0x00)
			{
				PIONEER.EUFDIR = FEED_STOP;
				PIONEER.UIFRATE = MINFEED;
				SEND_DATA(BLUETOOTH,(uint8_t *) "MSTP", 4);
				SEND_DATA(DEBUG,(uint8_t *) "MSTP", 4);
			}
			else if (rxPio_buffer[3] == 0xFF)
			{
				__NOP;
			}
			else
			{
				//error
			}				
			
		}
		else if (rxPio_buffer[2] == 0x04)	//address SERVO_SPEED
		{
			motionrateReceived = rxPio_buffer[3];

			ILI9225_setWindow(0, 62, 0+50, 62+30, TopDown_L2R);
			ILI9225_drawChar(0, 63+5, 'M', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_drawChar(15+15, 63+5, '1', COLOR_YELLOW, COLOR_BLACK);
			ILI9225_fillRectangle(70, 62+5+5, 175, 92-5+5, COLOR_WHITE);
			ILI9225_fillRectangle(70, 62+5+5, 70+(105-(motionrateReceived/2.4285)), 92-5+5, COLOR_BLUE);

			/*
			it contains a condition of motionrateReceived == 0 
			for interference problem in sending and receiving frames
			*/
			if (0)
			{
					SEND_DATA(BLUETOOTH,(uint8_t *) "ERR4", 4);
					HAL_UART_Transmit(&huart2,  (uint8_t *) "ERR4", 4, 10);
			}
			else
			{
				if (motionrateReceived <= 50)
				{
					motionrate = (motionrateReceived * 1 ) + 219;
				}
				else if ( (motionrateReceived <= 100) & (motionrateReceived > 50) )
				{
					motionrate = (motionrateReceived * 1.3 ) + 220;
				}
				else if ( (motionrateReceived <= 130) & (motionrateReceived > 100) )
				{
					motionrate = (motionrateReceived * 1.32 ) + 220;
				}
				else if ( (motionrateReceived <= 150) & (motionrateReceived > 130) )
				{
					motionrate = (motionrateReceived * 1.38 ) + 220;
				}
				else if ( (motionrateReceived <= 150) & (motionrateReceived > 140) ) //140-150
				{
					motionrate = (motionrateReceived * 1.4 ) + 220;
				}
				else if ( (motionrateReceived <= 160) & (motionrateReceived > 150) ) //150-160
				{
					motionrate = (motionrateReceived * 1.45 ) + 220;
				}
				else if ( (motionrateReceived <= 170) & (motionrateReceived > 160) ) //160-170
				{
					motionrate = (motionrateReceived * 1.5 ) + 220;
				}
				else if ( (motionrateReceived <= 180) & (motionrateReceived > 170) ) //170-180
				{
					motionrate = (motionrateReceived * 1.6 ) + 220;
				}
				else if ( (motionrateReceived <= 190) & (motionrateReceived > 180) ) //180-190
				{
					motionrate = (motionrateReceived * 1.69 ) + 220;
				}
				else if ( (motionrateReceived <= 200) & (motionrateReceived > 190) ) //190-200
				{
					motionrate = (motionrateReceived * 2 ) + 220;
				}
				else if ( (motionrateReceived <= 210) & (motionrateReceived > 200) ) //200-210
				{
					motionrate = (motionrateReceived * 2.2 ) + 220;
				}
				else if ( (motionrateReceived <= 220) & (motionrateReceived > 210) ) //210-220
				{
					motionrate = (motionrateReceived * 2.5 ) + 220;
				}
				else if ( (motionrateReceived <= 230) & (motionrateReceived > 220) ) //220-230
				{
					motionrate = (motionrateReceived * 3 ) + 220;
				}
				else if ( (motionrateReceived <= 240) & (motionrateReceived > 230) ) //230-240
				{
					motionrate = (motionrateReceived * 5 ) + 220;
				}
				else if ( (motionrateReceived <= 250) & (motionrateReceived > 240) ) //240-250
				{
					motionrate = (motionrateReceived * 7 ) + 220;
				}
				else
				{
					motionrate = (motionrateReceived * 20 ) + 220;
				}

				motionrate = LIMITMOTION(motionrate);
				PIONEER.UIMRATE = motionrate;

				/*prepare the reply message and send it*/
				if (motionrateReceived > 100)
				{
					sprintf(tempStr_5, "%s%d", "R", motionrateReceived);	
					SEND_DATA(BLUETOOTH, (uint8_t *) tempStr_5, 4);
					SEND_DATA(DEBUG, (uint8_t *) tempStr_5, 4);
				}
				else if ( (motionrateReceived > 10) && (motionrateReceived <100) )
				{
					sprintf(tempStr_4, "%s%d", "R", motionrateReceived);	
					SEND_DATA(BLUETOOTH, (uint8_t *) tempStr_4, 3);
					SEND_DATA(DEBUG, (uint8_t *) tempStr_4, 3);
				}
				else
				{
					sprintf(tempStr_3, "%s%d", "R", motionrateReceived);	
					SEND_DATA(BLUETOOTH, (uint8_t *) tempStr_3, 2);
					SEND_DATA(DEBUG, (uint8_t *) tempStr_3, 2);	
				}				
					
			}

		}
		else if (rxPio_buffer[2] == 0x05)	//address	SERVO_DIR
		{
			if (PIONEER.UIMRATE != MINMOTION)
			{
				if (rxPio_buffer[3] == 0x00)
				{
					PIONEER.EUMDIR = RIGHT;
					HAL_UART_Transmit(&huart2, (uint8_t *) "RRIT", 4, 10);
					HAL_UART_Transmit_IT(&huart6, (uint8_t *) "RRIT", 4);
				}
				else if (rxPio_buffer[3] == 0xFF)
				{
					PIONEER.EUMDIR = LEFT;
					HAL_UART_Transmit(&huart2, (uint8_t *) "RLFT", 4, 10);
					HAL_UART_Transmit_IT(&huart6, (uint8_t *) "RLFT", 4);
				}
				else
				{
					//error
				}
			}
		}
		else if (rxPio_buffer[2] == 0x06)	//address SERVO_EN
		{
			if (rxPio_buffer[3] == 0x00)
			{
				PIONEER.EUMDIR = MOTION_STOP;
				PIONEER.UIMRATE = MINMOTION;
				HAL_UART_Transmit(&huart2, (uint8_t *) "RTSP", 4, 10);
				HAL_UART_Transmit_IT(&huart6, (uint8_t *) "RSTP", 4);
			}	
			else if (rxPio_buffer[3] == 0xFF)
			{
				__NOP;
			}
			else
			{
				//error
			}
		}
		else if (rxPio_buffer[2] == 0x07)	//address	LAMP_READY
		{
			RData = rxPio_buffer[3];
		}
		else if (rxPio_buffer[2] == 0x08)	//address LAMP_ALARM
		{
			RData = rxPio_buffer[3];
		}
		else if (rxPio_buffer[2] == 0x09)	//address DISTANCE
		{
			RData = rxPio_buffer[3];
		}
		else if (rxPio_buffer[2] == 0x0A)	//address ALARM_BULT
		{
			if (rxPio_buffer[3] == 0x00)
			{
				PIONEER.EUSTOP = STOP;
				SEND_DATA(BLUETOOTH,(uint8_t *) "PLAY", 4);
				SEND_DATA(DEBUG,(uint8_t *) "PLAY", 4);
			}
			else if (rxPio_buffer[3] == 0xFF)
			{
				PIONEER.EUSTOP = NOSTOP;
				SEND_DATA(BLUETOOTH,(uint8_t *) "HALT", 4);
				SEND_DATA(DEBUG,(uint8_t *) "HALT", 4);
			}
			else
			{
				//error
			}
		}
	}
	/* RESEST BUFFER UART */
	ik = 0;
	while (ik<5)
	{
		rxPio_buffer[ik] = 0x00;
		ik++;
	}

}

}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
//error 9


}
