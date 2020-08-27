#include <Pioneer.hpp>
#include <stdio.h>
#include "main.h"

using namespace PIONEER_MACHINES;

TIM_HandleTypeDef htim5;


StepperMotor* FeedMotor;
	
StepperMotor* RotationMotor;
	
ServoMotor* BundleMotor;



void setup(void)
{
	//setting of feed motor
	FeedMotor->setting(0x01, "Feed Motor", NOT_RUNNING, 0, 5000);
	FeedMotor->HWSetting->htim = htim5;
	FeedMotor->HWSetting->TIMCHANNEL = TIM_CHANNEL_2;
	FeedMotor->HWSetting->GPIO_EN = GPIOA;
	FeedMotor->HWSetting->GPIO_PinEN = GPIO_PIN_0;
	FeedMotor->HWSetting->GPIO_DIR = GPIOA;
	FeedMotor->HWSetting->GPIO_PinDIR = GPIO_PIN_4;

	//setting of rotational motor
	RotationMotor->setting(0x02, "Rotation Motor", NOT_RUNNING, 0, 5000);
	
	FeedMotor->run(RSTOP,0,0);
	printf("Description %s", FeedMotor->getDescription());
	printf("ID %d", FeedMotor->getID());
	
	

}




