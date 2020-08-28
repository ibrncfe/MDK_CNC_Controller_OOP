#include <Pioneer.hpp>
#include <stdio.h>
#include "main.h"


// mikrolib has turned off;

using namespace PIONEER_MACHINES;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

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
	FeedMotor->run(RSTOP,0,0);
	
	//setting of rotational motor
	RotationMotor->setting(0x02, "Rotation Motor", NOT_RUNNING, 0, 5000);
	RotationMotor->HWSetting->htim = htim2;
	RotationMotor->HWSetting->TIMCHANNEL = TIM_CHANNEL_1;
	RotationMotor->HWSetting->GPIO_EN = GPIOA;
	RotationMotor->HWSetting->GPIO_PinEN = GPIO_PIN_8;
	RotationMotor->HWSetting->GPIO_DIR = GPIOB;
	RotationMotor->HWSetting->GPIO_PinDIR = GPIO_PIN_9;
	
	//setting of Servo motor
	BundleMotor->setting(0x03, "Servo Motor", NOT_RUNNING, 0);
	BundleMotor->HWSetting->htim = htim2;
	BundleMotor->HWSetting->TIMCHANNEL = TIM_CHANNEL_1;
	BundleMotor->HWSetting->GPIO_EN = GPIOA;
	BundleMotor->HWSetting->GPIO_PinEN = GPIO_PIN_8;
	BundleMotor->HWSetting->GPIO_DIR = GPIOB;
	BundleMotor->HWSetting->GPIO_PinDIR = GPIO_PIN_9;
	
//	printf("Description %s", FeedMotor->getDescription());
//	printf("ID %d", FeedMotor->getID());
}






