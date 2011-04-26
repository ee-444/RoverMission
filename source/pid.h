
#ifndef PID_H
#define PID_H

#define WINDUP_GUARD_GAIN	100

//! @defgroup PID-API A Generic templated PID class
/** Generic PID algorithm
 * @author sgrove (inspired by Tim Wescott)
 * @see http://dpm1480.pbworks.com/f/PID%20without%20a%20PhD.pdf
 * @see PID-API 
 *
 * @image __ "Caption for the image"
 *
 * Example:
 * @code
 * #include "ArduinoRoverLib.h"
 *
 * // as used with a magnometer
 * PID <float>pid(5, 0.1, 50); 
 *
 * int main() {
 *		// register the desired setpoint
 *		pid.setTarget(100);
 *		// wait a while and pass the sensor measurement into the error function
 *		while (1){
 *			wait(10);
 *			// see the progress (max change is a parameter if needed, otherwise default to 0)
 *			float result = pid.getError(heading, 360.0);
 *		}    
 * }
 * @endcode
 */

template <typename T>
class PID
{
private:
	// protected data
	T _target, _last_target;
	T _istate;
	// The caluclated terms
	struct{
		float pterm;
		float iterm;
		float dterm;
	}_pid_term;
	// The stored gains
	struct{
		T pgain;
		T igain;
		T dgain;
	}_pid_gain;

public:
	//! PID constructor
    /**
     * Takes the gain for each term, P, I, D and assignes 1 to any non-parameterized terms.
     *
     * @ingroup PID-API
     */
	PID(T p=1, T i=1, T d=1){
		_target = 0;
		_last_target = 0;
		_pid_gain.pgain = p;
		_pid_gain.igain = i;
		_pid_gain.dgain = d;
	} 

	//! Run time access to change the module gain
    /**
     * @param float gain - of the new value
     *
     * @ingroup PID-API
     */
	void setPGain(float gain)	{ _pid_gain.pgain = gain; }
	
	//! Run time access to change the module gain
    /**
     * @param float gain - of the new value
     *
     * @ingroup PID-API
     */
	void setIGain(float gain)	{ _pid_gain.igain = gain; }
	
	//! Run time access to change the module gain
    /**
     * @param float gain - of the new value
     *
     * @ingroup PID-API
     */
	void setDGain(float gain)	{ _pid_gain.dgain = gain; }

	//! Return of the currently implemented gain
    /**
     * @return the current gain
     *
     * @ingroup PID-API
     */
	float getPGain(void)		{ return _pid_gain.pgain; }
	
	//! Return of the currently implemented gain
    /**
     * @return the current gain
     *
     * @ingroup PID-API
     */
	float getIGain(void)		{ return _pid_gain.igain; }
	
	//! Return of the currently implemented gain
    /**
     * @return the current gain
     *
     * @ingroup PID-API
     */
	float getDGain(void)		{ return _pid_gain.dgain; }

	//! Change the setpoint for the module calculation
    /**
     * @param T input - the desired setting for the device (templated to the same type as the object)
     *
     * @ingroup PID-API
     */
	void setTarget(T input)		{ _target = input; }

	//! Get the modules current setpoint
    /**
     * @ingroup PID-API
	 *
	 * @return - the current setpoint
     */
	T getTarget(void)			{ return _target; }
	
	//! Find out how far the system is from the setpoint
    /**
     * @param T input - the current reading of the measurement device(templated to the same type as the object)
     * @param T max_error = 0 - A limitation on the amount of acceptable error (passing a parameter will overwrite)
	 *
     * @ingroup PID-API
     */
	T getError(T input, T max_error = 0){
		// get the current error
		T current_error = _target - input;
		// look out for max error (if implemented)
		if (current_error > max_error)
			current_error -= (max_error * 2);
		else if (current_error < -max_error)
			current_error += (max_error * 2);
		
		// Create the pTerm - present error
		_pid_term.pterm = _pid_gain.pgain * current_error;
		// Create the iTerm - accumulated error
		_istate += current_error;
		// create windup protection
		T windup_protection = static_cast<T>(WINDUP_GUARD_GAIN) / _pid_gain.igain;
		//protects against excessive positive and negative error
		if(_istate > windup_protection)
			_istate = windup_protection;
		else if(_istate < -windup_protection)
			_istate = -windup_protection;

		// Update the iTerm
		_pid_term.iterm = _istate * _pid_gain.igain;
		// Update the dTerm - The projected change
		_pid_term.dterm = (_pid_gain.dgain * (_target - _last_target));
		// store the current target as the last target
		_last_target = _target;

		// return the feedback
		return _pid_term.pterm + _pid_term.iterm - _pid_term.dterm;
	}

};


#endif


