// controller.h  header file for controller class
// The code is copyrighted by 4D Avionic Systems, LLC April 15, 2020, jgt


//#include "stdafx.h"

class controller {
private:
	double	Kp, Ki, Kd, Kr;		// Proportional, Integral, Derivative and Rate Controller Gains
	double	error, up, ui, ud, ur, error_previous;
	double	ui_max, uc_max, uc_min, uc, rate_error, trim;
	double  h;

public:
	// Variables passed to the Constructor:  Proportional Gain (Kp), Integral Gain (Ki), Derivative Gain (Kd), Rate Gain (Kr), Integral Windeup limit (ui_max), 
		// Output Limiter Max (uc_max), Output Limiter Min (uc_min), Output Trim value (trim).

	controller(double p, double i, double d, double r, double uimax, double ucmax, double ucmin, double Trim);		// constructor
	~controller();		// destructor

	// Variables passed to the pidr function:  Input, Command, Measured, Rate_Command, Rate_Measured.

	double	pidr(double input, double command, double measure, double rate_command, double rate_measure);

};
