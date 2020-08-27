#ifndef PIONEER_HPP
#define PIONEER_HPP

#include <stdint.h>
#include "main.h"
#include <Motors.hpp>

namespace PIONEER_MACHINES
{
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
	protected:
		
	
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
