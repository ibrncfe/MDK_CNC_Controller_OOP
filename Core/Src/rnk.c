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

#include "rnk.h"

#define abs(x) ((x)<0 ? (-x) : (x))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b)	((a) < (b) ? (a) : (b))

#define MINFEED	1300000
#define MINMOTION 80000
#define STEPRESOLUTION 1
#define MOTIONRESOLUTION 1
#define MaxFilterAverage 500
#define FIFO_SIZE 100
#define MAX_ITERATION FIFO_SIZE

static uint8_t* Input_Data;//Input channels state
static uint8_t output = 0x00;
static uint8_t* foutput = 0;

static RNK_MACHINE RNK_ST;
static RNK_MACHINE RNK_ST_PAST;

//static RNK_MACHINE* eSTRNK;
static uint8_t* RStatus;//Output relay status
static uint32_t RNK_InitialTime = 4;//Time in seconds for system to be ready

/* ADC Variables */
volatile static uint32_t RNK_ADC_Data[2];
uint32_t RNK_uADC_DATA[3];

/* Motors Variables */
static uint32_t rotaterate = MINFEED;	//MINIMUM ANGULAR SPEED ROTATE MOTOR
static uint32_t motionrate = MINMOTION;	//MINIMUM ANGULAR SPEED SERVO MOTOR
static uint32_t feedrate = MINFEED;			//MINIMUM ANGULAR SPEED STEPPER MOTOR

//static uint32_t potrateUpper = 0;
//static uint32_t potrateLower = 0;

/* Pot Variables */
volatile static uint8_t potrateUpper_average = 0;
volatile static uint32_t potrateUpper_sum = 0;
uint8_t potrateUpperReceived = 0;
static uint32_t DIFFERENCE_POTRATEUPPER = 0;

volatile static uint8_t potrateLower_average = 0;
volatile static uint32_t potrateLower_sum = 0;
uint8_t potrateLowerReceived = 0;
static uint32_t DIFFERENCE_POTRATELOWER = 0;

volatile uint8_t rnk_iteration =0;
volatile static uint32_t potrateLower_fifo_100[FIFO_SIZE];
volatile static uint32_t potrateUpper_fifo_100[FIFO_SIZE];
uint8_t potrateUpper_nonlin_in_manual = 0;
uint8_t potrateLower_nonlin_in_manual = 0;


/*				*/
RNK_STATE RNK_Initialization (void)
{
	//INITIAL STATE RNK
	RNK_ST.EUALARM = RNK_NOALARM;
	RNK_ST.EUFDIR = RNK_FEED_STOP;
	RNK_ST.EURSRDIR = RNK_ROTSR_STOP;
	RNK_ST.EURSTDIR = RNK_ROTST_STOP;
	RNK_ST.EUSTOPPED = RNK_STOPPED;
	RNK_ST.LowerButtonDIR  =RNK_BUTTON_PRSD_STOP;
	RNK_ST.Mode = RNK_MODE_NORMAL_BORING_I;
	RNK_ST.UpperButtonDIR = RNK_BUTTON_PRSD_STOP;
	RNK_ST.STATE = RNK_STATE_STOPPED;
	RNK_ST.UIFRATE = MINFEED;
	RNK_ST.UIRSTRATE = MINFEED;
	RNK_ST.UIRSRRATE = MINMOTION;
	RNK_ST.potrateLower = RNK_ST.potrateUpper = 0;

	//START TIMER 1 FOR LEDS
	HAL_TIM_Base_Start_IT(&htim1);
		
	HAL_TIM_Base_Start_IT(&htim10);

	//STATE MACHINE- SET INITIAL STATE
	Steps = initial;

	return RNK_STATE_OK;
}

RNK_STATE RNK_Basic (void)
{
	//STATE 1 INITIAL MODE
	if (Steps==initial)
	{
	}

	//STATE 2 NORMAL MODE
	if(Steps == normal)
	{
		RNK_ST.STATE = RNK_GetCommands();

		if (RNK_ST.STATE == STATE_ALARM)
		{
			return RNK_STATE_ERROR;
		}
		else if(RNK_ST.STATE == STATE_STOP)
		{
			return RNK_STATE_ERROR;
		}
		else if(RNK_ST.STATE == STATE_ERROR)
		{
			return RNK_STATE_ERROR;
		}
		else if(RNK_ST.STATE == STATE_OK)
		{
			return RNK_STATE_OK;
		}
	}
	return RNK_STATE_OK;
}

static uint32_t Nonlinear_FEEDRATE_POT(uint8_t feedrate_nonlin_in_manual)
{
	uint32_t steprate_nonlin_out_manual = MINFEED;
	if ( feedrate_nonlin_in_manual <= 10 )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 10) && (feedrate_nonlin_in_manual <= 30) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 30) && (feedrate_nonlin_in_manual <= 50) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 50) && (feedrate_nonlin_in_manual <= 90) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 90) && (feedrate_nonlin_in_manual <= 120) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 120) && (feedrate_nonlin_in_manual <= 160) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 15 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 160) && (feedrate_nonlin_in_manual <= 180) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 20 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 180) && (feedrate_nonlin_in_manual <= 200) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 40 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 200) && (feedrate_nonlin_in_manual <= 220) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 50 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 220) && (feedrate_nonlin_in_manual <= 230) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 70 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 230) && ( feedrate_nonlin_in_manual <= 238) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 120 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 238) && ( feedrate_nonlin_in_manual <= 242) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 250 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 242) && ( feedrate_nonlin_in_manual <= 246) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 500 ) + 1000;
	}
	else if( (feedrate_nonlin_in_manual > 246) && ( feedrate_nonlin_in_manual <= 250) )
	{
		steprate_nonlin_out_manual = (feedrate_nonlin_in_manual * 1000 ) + 1000;
	}
	else if( feedrate_nonlin_in_manual > 250 )
	{
		RNK_STOP_FEED();
	}	
	else
	{
		steprate_nonlin_out_manual = MINFEED;
	}
	return steprate_nonlin_out_manual;
}

static uint32_t Nonlinear_MOTIONRATE_POT(uint8_t motionrate_nonlin_in_manual)
{
	uint32_t motionrate_nonlin_out_manual = MINFEED;

		if (motionrate_nonlin_in_manual <= 50)
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 1 ) + 219;
	}
	else if ( (motionrate_nonlin_in_manual <= 100) && (motionrate_nonlin_in_manual > 50) )
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 1.3 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 130) && (motionrate_nonlin_in_manual > 100) )
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 1.32 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 150) && (motionrate_nonlin_in_manual > 130) )
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 1.38 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 150) && (motionrate_nonlin_in_manual > 140) ) //140-150
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 1.4 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 160) && (motionrate_nonlin_in_manual > 150) ) //150-160
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 1.45 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 170) && (motionrate_nonlin_in_manual > 160) ) //160-170
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 1.5 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 180) && (motionrate_nonlin_in_manual > 170) ) //170-180
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 1.6 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 190) && (motionrate_nonlin_in_manual > 180) ) //180-190
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 1.69 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 200) && (motionrate_nonlin_in_manual > 190) ) //190-200
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 2 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 210) && (motionrate_nonlin_in_manual > 200) ) //200-210
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 2.2 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 220) && (motionrate_nonlin_in_manual > 210) ) //210-220
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 2.5 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 230) && (motionrate_nonlin_in_manual > 220) ) //220-230
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 3 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 238) && (motionrate_nonlin_in_manual > 230) ) //230-240
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 7 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 242) && (motionrate_nonlin_in_manual > 238) ) //240-250
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 10 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 248) && (motionrate_nonlin_in_manual > 242) ) //240-250
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 20 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 250) && (motionrate_nonlin_in_manual > 248) ) //240-250
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 30 ) + 220;
	}
	else if ( (motionrate_nonlin_in_manual <= 253) && (motionrate_nonlin_in_manual > 250) ) //240-250
	{
		motionrate_nonlin_out_manual = (motionrate_nonlin_in_manual * 40 ) + 220;
	}	
	else if (motionrate_nonlin_in_manual > 253)  //250-inf
	{
		RNK_STOP_SERVO();
	}	
	else
	{
		RNK_STOP_SERVO();
	}
return motionrate_nonlin_out_manual;
}

static uint32_t RNK_LIMITMOTION (uint32_t mr)
{
	if (mr <= 220)
	{
		return 220;
	}
	else if (mr > 65535)
	{
		return 65535;
	}
	else
	{
		return mr;
	}
}

static uint32_t RNK_LIMITFEED (uint32_t sr)
{
	if (sr < 1300)
		return 1350;
	else if (sr > MINFEED)
		return MINFEED;
	else
		return sr;
}

static RNK_INPUTS_CHANGE RNK_CheckChange(void)
{
		/* clear output */
//	output = 0x00;

	//check stop button on the station

	if (RNK_CHECKSTOP()== STOP)
	{
		RNK_ST.EUSTOPPED = RNK_STOPPED;
		/*Stop the motors*/
		RNK_STOP_STROT();
		RNK_STOP_FEED();
		RNK_STOP_SERVO();
		/*change the state to off*/
		RNK_ST.EUFDIR = RNK_FEED_STOP;
		RNK_ST.EURSRDIR = RNK_ROTSR_STOP;
		RNK_ST.EURSTDIR = RNK_ROTST_STOP;

		Input_Data = CURRENT_LIMITER_Handler();

		output = 0x00;
		foutput=&output;

		/* Handler for output relay */
		RStatus = BSP_RELAY_SetOutputs(&output);

		if (BSP_GetRelayStatus(RStatus) != RELAY_OK)
		{
		/* Set output error code here */
			//X-PLC Error
		}

		return RNOCHANGE; //output error
	}


	/* clear output */
//	output = 0x00;

	/* set ready led*/
	Input_Data = CURRENT_LIMITER_Handler();

	output |= 0x01;
	foutput=&output;

  RStatus = BSP_RELAY_SetOutputs(&output);
	//BSP_Output_ON(output);
  if (BSP_GetRelayStatus(RStatus) != RELAY_OK)
  {
    /* Set output error code here */
  }


	/* AVERAGE FILTER USING FIFO */
	potrateLower_sum = 0;
	potrateUpper_sum = 0;
	
	//READING THE ADC (Motors' speed rates)
	for(int j=0 ; j < MAX_ITERATION  ; j++)
	{
		potrateLower_sum += potrateLower_fifo_100[j];
		potrateUpper_sum += potrateUpper_fifo_100[j];
	}

	potrateLower_average = (uint8_t)((float) potrateLower_sum / MAX_ITERATION);
	potrateUpper_average = (uint8_t)((float) potrateUpper_sum / MAX_ITERATION);
	/* AVERAGE FILTER END*/

	//CALCULATE FOR SERVO AND STEPPER SCALES
	potrateLower_nonlin_in_manual = (uint8_t)((double)potrateLower_average * 2.55 );
	potrateUpper_nonlin_in_manual = (uint8_t)((double)potrateUpper_average * 2.55 );

	
	//UPDATING THE STATE OF PIONEER
	RNK_ST .UpperButtonDIR = RNK_CHECK_UPPER_BUTTON();
	RNK_ST .LowerButtonDIR = RNK_CHECK_LOWER_BUTTON();
	RNK_ST .potrateUpper = potrateUpper_nonlin_in_manual;
	RNK_ST .potrateLower = potrateLower_nonlin_in_manual;

	RNK_ST.Mode = RNK_CHECK_MODE_TOGGLE();
	
	DIFFERENCE_POTRATEUPPER = MAX(RNK_ST .potrateUpper,  RNK_ST_PAST .potrateUpper) - MIN(RNK_ST .potrateUpper,  RNK_ST_PAST .potrateUpper);
	DIFFERENCE_POTRATELOWER = MAX(RNK_ST .potrateLower, RNK_ST_PAST .potrateLower) - MIN(RNK_ST .potrateLower, RNK_ST_PAST .potrateLower);

	//CHECK THE CHANGE
	if ( (RNK_ST .UpperButtonDIR == RNK_ST_PAST .UpperButtonDIR) & (RNK_ST .LowerButtonDIR == RNK_ST_PAST .LowerButtonDIR) &
		( DIFFERENCE_POTRATEUPPER <= STEPRESOLUTION )	& (DIFFERENCE_POTRATELOWER <= MOTIONRESOLUTION) )
	{
			return RNOCHANGE; //Nochange
	}
	else
	{
		//IDENTIFY THE TYPE OF CHANGE IN FEED STPPER DRIVER
		if ( (RNK_ST .UpperButtonDIR != RNK_ST_PAST .UpperButtonDIR) | (RNK_ST .LowerButtonDIR != RNK_ST_PAST .LowerButtonDIR) )		//check the condition -1-
		{
			if ((RNK_ST .UpperButtonDIR != RNK_ST_PAST .UpperButtonDIR))
			{
				RNK_ST_PAST .UpperButtonDIR = RNK_ST .UpperButtonDIR;

			}
			else
			{
				RNK_ST_PAST .LowerButtonDIR = RNK_ST .LowerButtonDIR;

			}
		}
		else if ( DIFFERENCE_POTRATEUPPER > STEPRESOLUTION )
		{
			RNK_ST_PAST .potrateUpper = RNK_ST .potrateUpper;
		}
		else if (DIFFERENCE_POTRATELOWER > MOTIONRESOLUTION)
		{
			RNK_ST_PAST .potrateLower = RNK_ST .potrateLower;
		}

		return RCHANGE; //Change
	}
return RNOCHANGE;
}

static RNK_STATE RNK_GetCommands(void)
{
	//RUN THE ADC
	HAL_ADC_Start_DMA(&hadc1, RNK_uADC_DATA, 3);
	
	
	while(1)
	{

		while (RNK_CheckChange() == RNOCHANGE);
		
		
		//////ALGORITHM/////
		if (RNK_ST .Mode == RNK_MODE_NORMAL_BORING_I)
		{
			//NONLINEAR & LIMITERS FUNCTIONS
			RNK_ST.UIFRATE = RNK_LIMITFEED(Nonlinear_FEEDRATE_POT(RNK_ST.potrateUpper));
			RNK_ST.UIRSRRATE = RNK_LIMITMOTION(Nonlinear_MOTIONRATE_POT(RNK_ST .potrateLower));
			feedrate = RNK_ST.UIFRATE;
			motionrate = RNK_ST.UIRSRRATE;
			
			// Control Feed Motor
			if (RNK_ST.UpperButtonDIR == RNK_BUTTON_PRSD_STOP)
			{
				RNK_STOP_FEED();
			}
			else if (RNK_ST.UpperButtonDIR == RNK_BUTTON_PRSD_LEFT)
			{	
				RNK_GO_FEED (RNK_FEED_FORWARD, RNK_ST.UIFRATE);
			}
			else //
			{
				RNK_GO_FEED (RNK_FEED_BACKWARD, RNK_ST.UIFRATE);
			}
			
			// Control Servo Motor
			if ((RNK_ST.LowerButtonDIR == RNK_BUTTON_PRSD_STOP) || (RNK_ST.UIRSRRATE > 65533) )
			{
				RNK_STOP_SERVO();
			}
			else if ( (RNK_ST.LowerButtonDIR == RNK_BUTTON_PRSD_LEFT) && (RNK_ST.UIRSRRATE <= 65533) )
			{	
				RNK_GO_SERVO (RNK_ROTSR_LEFT, RNK_ST.UIRSRRATE);
			}
			else //
			{
				RNK_GO_SERVO (RNK_ROTSR_RIGHT, RNK_ST.UIRSRRATE);
			}
			////////
		}
		else if (RNK_ST .Mode == RNK_MODE_NORMAL_SURFACING_WELDING_II)
		{
			RNK_ST.UIFRATE = RNK_LIMITFEED(Nonlinear_FEEDRATE_POT(RNK_ST.potrateLower));

			/*
			equation 2.7-3.1MM
			*/
			
			if (RNK_ST.UIFRATE <2000)
			{
					RNK_ST.UIFRATE = 2000;
			}
			
			feedrate = RNK_ST.UIFRATE;
			RNK_ST.UIRSTRATE = 0.58*feedrate;
			rotaterate = 0.58*feedrate;
			
			// Control Feed and Rotate Motors
			if (RNK_ST.LowerButtonDIR == RNK_BUTTON_PRSD_STOP)
			{
				RNK_STOP_FEED();
				RNK_STOP_STROT();
			}
			else if (RNK_ST.LowerButtonDIR == RNK_BUTTON_PRSD_LEFT)
			{	
				RNK_GO_FEED (RNK_FEED_BACKWARD, RNK_ST.UIFRATE);
				RNK_GO_STROT (RNK_ROTST_LEFT, RNK_ST.UIRSTRATE);
			}
			else //
			{
				RNK_GO_FEED (RNK_FEED_FORWARD, RNK_ST.UIFRATE);
				RNK_GO_STROT (RNK_ROTST_RIGHT, RNK_ST.UIRSTRATE);
			}			
			///////////
		}
		else //RNK_MODE_NORMAL_WELDING_III
		{

			RNK_ST.UIFRATE = RNK_LIMITFEED(Nonlinear_FEEDRATE_POT(RNK_ST.potrateUpper));
			feedrate = RNK_ST.UIFRATE;
			RNK_ST.UIRSTRATE = RNK_LIMITFEED(Nonlinear_FEEDRATE_POT(RNK_ST.potrateLower));
			rotaterate = RNK_ST.UIRSTRATE;
			
			// Control Feed Motor
			if (RNK_ST.UpperButtonDIR == RNK_BUTTON_PRSD_STOP)
			{
				RNK_STOP_FEED();
			}
			else if (RNK_ST.UpperButtonDIR == RNK_BUTTON_PRSD_LEFT)
			{	
				RNK_GO_FEED (RNK_FEED_FORWARD, RNK_ST.UIFRATE);
			}
			else //Right
			{
				RNK_GO_FEED (RNK_FEED_BACKWARD, RNK_ST.UIFRATE);
			}
			
			// Control Rotate Motor
			if (RNK_ST.LowerButtonDIR == RNK_BUTTON_PRSD_STOP)
			{
				RNK_STOP_STROT();
			}
			else if (RNK_ST.LowerButtonDIR == RNK_BUTTON_PRSD_LEFT)
			{	
				RNK_GO_STROT (RNK_ROTST_LEFT, RNK_ST.UIRSTRATE);
			}
			else //Right
			{
				RNK_GO_STROT (RNK_ROTST_RIGHT, RNK_ST.UIRSTRATE);
			}			
		}
	}

	//////error , break to here and output alarms
	return RNK_STATE_OK;
}

//READ ADC + TEMPERATURE
RNK_STATE RNK_CHECKSPD(void)
{
	/*Reading Analog Inputs*/
	HAL_ADC_Start_DMA(&hadc1, RNK_uADC_DATA, 3);

	RNK_ADC_Data[0] = RNK_uADC_DATA[0] * 0.0244;
	RNK_ADC_Data[1] = RNK_uADC_DATA[1] * 0.0244;

	RNK_ST.temperature = (uint32_t)(((0.754 * 1000.0 - (double)RNK_uADC_DATA[2] * 0.8) / 2.5) + 25.0);

return RNK_STATE_OK;
}

//TIMERS INTERRUPT SERVICE ROUTINE
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
	if (htim->Instance == TIM10)
	{
		RNK_CHECKSPD();
		potrateUpper_fifo_100[rnk_iteration] = 100-RNK_ADC_Data[0];
		potrateLower_fifo_100[rnk_iteration] = 100-RNK_ADC_Data[1];
		rnk_iteration++;
		if (rnk_iteration % MAX_ITERATION == 0)
		{
			rnk_iteration = 0;
		}
	}

	if (htim->Instance == TIM1)
	{
		if (RNK_InitialTime==0)
		{
			Steps=normal;
			HAL_TIM_Base_Stop_IT(&htim1);

			/* Draw status machine*/
			//printStatus();
		}
		else
		{
			RNK_InitialTime--;
			//Ready_LED_Pulses();
		}
	}
}

//CHECK STEPPER DIRECTION
RNK_INPUT_MODES RNK_CHECK_MODE_TOGGLE(void)
{
	static uint8_t* clData = NULL;
  static uint8_t* Current_Limiter_Status;//Input current limiter status
  clData = BSP_CURRENT_LIMITER_Read();

  Current_Limiter_Status = clData;

  if (BSP_GetCurrentLimiterStatus(Current_Limiter_Status) != CURRENT_LIMITER_OK)
  {
    /* Set input error code here */
  }

	if ((*(clData+1)) & 0x40)
	{
		return RNK_MODE_NORMAL_BORING_I;
	}
	else if ((*(clData+1)) & 0x80)
	{
		return RNK_MODE_NORMAL_WELDING_III;
	}
	else 
		return RNK_MODE_NORMAL_SURFACING_WELDING_II;
}

//CHECK STEPPER DIRECTION
RNK_BUTTON_DIRECTION RNK_CHECK_UPPER_BUTTON(void)
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
		return RNK_BUTTON_PRSD_RIGHT;
	}
	else if ((*(clData+1)) & 0x20)
	{
		return RNK_BUTTON_PRSD_LEFT;
	}
	else
		return RNK_BUTTON_PRSD_STOP;
}

RNK_BUTTON_DIRECTION RNK_CHECK_LOWER_BUTTON(void)
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
		return RNK_BUTTON_PRSD_RIGHT;
	}
	else if ((*(clData+1)) & 0x08)
	{
		return RNK_BUTTON_PRSD_LEFT;
	}
	else
		return RNK_BUTTON_PRSD_STOP;
}


//ALARM (it should be checked again)
RNK_ALARM_STATE RNK_CHECKSERALARM(void)
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
		return RNK_NOALARM;
	}
	else
	{
		return RNK_ALARM;
	}
}

//AVARIA (emergency button)
RNK_STOPPED_STATE RNK_CHECKSTOP(void)
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
		return RNK_NOSTOPPED;
	}
	else
	{
		return RNK_STOPPED;
	}
}





/* CONTROL MOTORS*/
//PWM INTERRUPT SERVICE ROUTINE CONTROL
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM5)
	{
		__HAL_TIM_SET_PRESCALER(&htim5, 1);
		__HAL_TIM_SET_AUTORELOAD(&htim5, feedrate);
		__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, feedrate/2);
	}

	if (htim->Instance == TIM2)
	{
		__HAL_TIM_SET_PRESCALER(&htim2, 1);
		__HAL_TIM_SET_AUTORELOAD(&htim2, rotaterate);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, rotaterate/2);
	}

	if (htim->Instance == TIM3)
	{
		__HAL_TIM_SET_PRESCALER(&htim3, 1);
		__HAL_TIM_SET_AUTORELOAD(&htim3, motionrate);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, motionrate/2);
	}
}
//FEED MOTOR CONTROL
RNK_STATE RNK_GO_FEED(RNK_FEED_DIRECTION DIR, uint32_t arg_feedrate)
{

	//CHANGE SETTING TIMER 5 CHANNEL 2
	__HAL_TIM_SET_PRESCALER	(&htim5, 1);
	__HAL_TIM_SET_AUTORELOAD(&htim5, arg_feedrate);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, arg_feedrate/2);

	//enable
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_Delay(1);

	//dir
	if (DIR==RNK_FEED_FORWARD)
	{
		HAL_GPIO_WritePin(FDIR_GPIO_Port, FDIR_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(FDIR_GPIO_Port, FDIR_Pin, GPIO_PIN_SET);
	}

	HAL_Delay(1);

	//pulse
	HAL_TIM_PWM_Start_IT(&htim5, TIM_CHANNEL_2);

	return RNK_STATE_OK;
}


RNK_STATE RNK_STOP_FEED(void)
{
	HAL_TIM_PWM_Stop_IT(&htim5, TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

	return RNK_STATE_OK;
}

//MOTION MOTOR CONTROL
RNK_STATE RNK_GO_SERVO (RNK_ROTSR_DIRECTION DIR, uint32_t arg_motionrate)
{
	//CHANGE SETTING TIMER 3 CHANNEL 1
	__HAL_TIM_SET_PRESCALER	(&htim3, 1);
	__HAL_TIM_SET_AUTORELOAD(&htim3, arg_motionrate);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, arg_motionrate/2);

	//dir
	if (DIR==RNK_ROTSR_RIGHT)
	{
		HAL_GPIO_WritePin(SIGN_GPIO_Port, SIGN_Pin, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(SIGN_GPIO_Port, SIGN_Pin, GPIO_PIN_RESET);
	}

	HAL_Delay(1);

	//pulse
	HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1);

	return RNK_STATE_OK;
}

RNK_STATE RNK_STOP_SERVO(void)
{
	HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	return RNK_STATE_OK;
}
//FEED MOTOR CONTROL
RNK_STATE RNK_GO_STROT (RNK_ROTST_DIRECTION DIR, uint32_t strot_rate)
{

	//CHANGE SETTING TIMER 2 CHANNEL 1
	__HAL_TIM_SET_PRESCALER	(&htim2, 1);
	__HAL_TIM_SET_AUTORELOAD(&htim2, strot_rate);
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, strot_rate/2);

	//enable
	HAL_GPIO_WritePin(ROTSTENA_GPIO_Port, ROTSTENA_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);

	//dir
	if (DIR==RNK_ROTST_RIGHT)
	{
		HAL_GPIO_WritePin(ROTSTDIR_GPIO_Port, ROTSTDIR_Pin, FORW);
	}
	else
	{
		HAL_GPIO_WritePin(ROTSTDIR_GPIO_Port, ROTSTDIR_Pin, BACK);
	}

	HAL_Delay(1);

	//pulse
	HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1);

	return RNK_STATE_OK;
}


RNK_STATE RNK_STOP_STROT (void)
{
	HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
	HAL_GPIO_WritePin(ROTSTENA_GPIO_Port, ROTSTENA_Pin, GPIO_PIN_RESET);

	return RNK_STATE_OK;
}
