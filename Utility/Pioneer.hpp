#ifndef PIONEER_HPP
#define PIONEER_HPP

#include <stdint.h>
#include "main.h"
#include <Motors.hpp>

namespace PIONEER_MACHINES
{
	
//STEPPER MOTOR ROTATION STATE
typedef enum
{
	RNK_BUTTON_PRSD_RIGHT=1U,
	RNK_BUTTON_PRSD_LEFT=2U,
	RNK_BUTTON_PRSD_STOP=0U
}RNK_BUTTON_DIRECTION;
//ALARM STATE
typedef enum
{
	RNK_ALARM=1U,
	RNK_NOALARM=0U
}RNK_SERVO_ALRM_STATE;

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

typedef enum
{
	RNK_MODE_NORMAL_WELDING_III = 2U,
  RNK_MODE_NORMAL_SURFACING_WELDING_II = 1U,
	RNK_MODE_NORMAL_BORING_I = 0U,
}RNK_INPUT_MODES;


typedef struct 
{
	unsigned int CODE;
	RNK_STATE STATE;
}MACHINE_STATE;

/**
 * @class Presenter Presenter.hpp mvp/Presenter.hpp
 *
 * @brief The Presenter base class that all application-specific presenters should derive from.
 *
 *        The Presenter base class that all application-specific presenters should derive from.
 *        Only contains activate and deactivate virtual functions which are called
 *        automatically during screen transition.
 */
class Pioneer
{
	private:
		RNK_FEED_DIRECTION EUFDIR; 
		RNK_ROTST_DIRECTION EURSTDIR;
		RNK_ROTSR_DIRECTION EURSRDIR;
		RNK_SERVO_ALRM_STATE EUALARM;
		RNK_STOPPED_STATE EUSTOPPED;
		RNK_INPUT_MODES Mode;
		RNK_BUTTON_DIRECTION UpperButtonDIR;
		RNK_BUTTON_DIRECTION LowerButtonDIR;
		uint32_t potrateUpper;
		uint32_t potrateLower;	
		uint32_t UIFRATE;
		uint32_t UIRSRRATE;
		uint32_t UIRSTRATE;
		MACHINE_STATE STATE;
		int32_t temperature;
	
	public:

    /**
     * @fn virtual void Presenter::activate()
     *
     * @brief Place initialization code for the Presenter here.
     *
     *        The activate function is called automatically when a screen transition causes
     *        this Presenter to become active. Place initialization code for the Presenter here.
     */
    virtual void activate()
    {
    }

		
		void setup(void);
		
		/* functions for setting the private parameters */
		void potrate_push_stack(unsigned int upper, unsigned int lower);
		void potrate_check_values(void);
		void buttons_check(void);
		void alarm_check(void);
		void stop_check(void);
		void input_mode_check(void);
		
		/* function for processing */
		void Pioneer_take_process(void);
		
		/* function for checking the state of machine and set error code and warning*/
		void check_pioneer_state();
		
		/* function for printing the status */
		void print_state(void);
		
    /**
     * @fn virtual void Presenter::deactivate()
     *
     * @brief Place cleanup code for the Presenter here.
     *
     *        The deactivate function is called automatically when a screen transition causes
     *        this Presenter to become inactive. Place cleanup code for the Presenter here.
     */
    virtual void deactivate()
    {
    }

    /**
     * @fn virtual Presenter::~Presenter()
     *
     * @brief Destructor.
     *
     *        Destructor.
     */
    virtual ~Pioneer()
    {
    }
		
		/**
     * @fn Presenter::Presenter()
     *
     * @brief Default constructor.
     *
     *        Default constructor.
     */
    Pioneer()
    {
    }

};

} // namespace PIONEER_2

void setup(void);
#endif // PIONEER_HPP
