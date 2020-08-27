#ifndef MOTORS_HPP
#define MOTORS_HPP

#include <vector>
#include <main.h>

using namespace std;

//MOTOR RUNNING STATE
typedef enum {
	RUNNING = 1U,
	NOT_RUNNING = 0U
}RUNSTATE;


//ROTATIONAL DIRECTION STATE
typedef enum
{
	RIGHT = 1U,
	LEFT = 2U,
	RSTOP = 0U
}ROT_DIRECTION;

typedef struct 
{
	TIM_HandleTypeDef htim;
	unsigned int TIMCHANNEL;
	GPIO_TypeDef* GPIO_EN;
	uint16_t GPIO_PinEN;
	GPIO_TypeDef* GPIO_PULSE;
	uint16_t GPIO_PinPULSE;
	GPIO_TypeDef* GPIO_DIR;
	uint16_t GPIO_PinDIR;
}HW_PWM_SETTING;

class Motor;
class ServoMotor;
class StepperMotor;



class Motor
{
protected:
	unsigned int id;
	const char *description;
	RUNSTATE running;
	unsigned int velocity;
	ROT_DIRECTION direction;

public:
	Motor(unsigned int a_id = 0, const char* a_description = 0, RUNSTATE a_running = NOT_RUNNING, unsigned int a_velocity = 0)
		:id(a_id), description(a_description), running(a_running), velocity(a_velocity)
	{}

	~Motor();
	
	//virtual void setting(unsigned int a_id = 0, const char* a_description = 0, RUNSTATE a_running = NOT_RUNNING, unsigned int a_velocity = 0)
	//{	this->id = a_id; this->description = a_description; this->running = a_running; this->velocity = a_velocity;	}

	//void setDescription(const char* text) { description = text; }
	//virtual void setDescription(const char* text) { description = text; }

	virtual unsigned int getID(void) {	return id;	}
	virtual const char* getDescription(void) { return description; }
	virtual RUNSTATE isRunning(void) const { return running; }
	unsigned int getVelicity(void) const { return velocity; }
	void setVelocity(unsigned int velocity) { velocity = velocity; }
	void setDirection(ROT_DIRECTION direction) { direction = direction; }

};


/* Stepper Motor Class */
class StepperMotor : public Motor {
public:
	unsigned int required_pos;                    // symmetric threshold in degrees
	unsigned int current_pos;                       // current position in degrees
	unsigned int setting_pulse_revolution;
	HW_PWM_SETTING* HWSetting; 
	
	StepperMotor();
	~StepperMotor();

	void setting(unsigned int a_id = 0, const char* a_description = 0, RUNSTATE a_running = NOT_RUNNING, unsigned int a_velocity = 0, unsigned int spr = 0)
	{	
		id = a_id;
		description = a_description;
		running = a_running;
		velocity = a_velocity;
		setting_pulse_revolution = spr;
	}

	unsigned int getID(void) { return id; }
	const char* getDescription(void) { return description; }
	RUNSTATE isRunning(void) const { return running; }

	void setRequiredPosition(unsigned int required_pos);
	void setCurrentPosition(unsigned int current_position);
	void setSettingPR(unsigned int pulse_revolution);

	unsigned getRequiredPosition() const { return required_pos; }
	int getCurrentPosition() const { return current_pos; }

	void run(ROT_DIRECTION direction, unsigned angle, unsigned speed);
	void stop(void);

	void wait(unsigned milliseconds) const;

	void checkServo(ServoMotor& S);

	//Give accessbility for RunServo getting parameters of stepper class's members
	//friend void ServoMotor::checkStepper(StepperMotor& S);
	friend class ServoMotor;
};

/* Servo Motor Class */
class ServoMotor : Motor
{
public:
	unsigned int req_pos;                    // symmetric threshold in degrees
	unsigned int cur_pos;                       // current position in degrees

	void RunServo(int direction, unsigned angle, unsigned speed);

	void checkStepper(StepperMotor& S);


	friend void StepperMotor::checkServo(ServoMotor& S);
};



#endif
