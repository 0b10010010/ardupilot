// controller.cpp  class function definiion file
// This project is an implementation of the Fixed-wing 4D autopilot simulation. 
// It is implemented in a local NED coordinate system using Euler angles.
// The code is copyrighted by 4D Avionic Systems, LLC April 15, 2020, jgt

//#include "stdafx.h"
#include "controller.h"



controller::controller(double p, double i, double d, double r, double uimax, double ucmax, double ucmin, double Trim)
{
	Kp = p;
	Ki = i;
	Kd = d;
	Kr = r;
	ui_max = uimax;
	uc_max = ucmax;
	uc_min = ucmin;
	error_previous = 0.0;
	ui = 0.0;
	trim = Trim;
	h = 0.01; // numerical integration step size (from stdafx.h)
}		// end of constructor

controller::~controller()
{
}		// destructor

double controller::pidr(double input, double command, double measure, double rate_command, double rate_measure)
{
	error = input + command - measure;			// Input Error
	up = Kp*error;								// Proportional Control Action
	ui += Ki*error*h;							// Integral Control Action
	if (ui > ui_max) ui = ui_max;				// Integral Control Anti-windup
	else if (ui < -ui_max) ui = -ui_max;
	ud = Kd*(error - error_previous) / h;		// Derivative Control Action
	error_previous = error;
	rate_error = rate_command - rate_measure;	// Rate Error
	ur = Kr*rate_error;							// Rate Control Action
	uc = up + ui + ud + ur + trim;				// Control Output
	if (uc > uc_max) uc = uc_max;				// Controller Output Limiter
	else if (uc < uc_min) uc = uc_min;

	return uc;									// return the Controller Output
}

