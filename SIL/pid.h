/*
 * pid.h
 *
 *  Created on: Jul 25, 2022
 *      Author: tjbli
 */

#ifndef PID_H_
#define PID_H_

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class PIDController
{
public:
	PIDController(double P, double I, double D, double ramp, double limit, double Ts);
	~PIDController() = default;

	double operator() (double error);

private:
	double P;
	double I;
	double D;
	double output_ramp;
	double limit;
	double Ts;

	double error_prev;
	double output_prev;
	double integral_prev;
};



#endif /* PID_H_ */
