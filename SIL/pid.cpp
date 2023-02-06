/*
 * pid.cpp
 *
 *  Created on: Jul 25, 2022
 *      Author: RockyLiu
 */

#include "pid.h"

PIDController::PIDController(double P, double I, double D, double ramp, double limit, double Ts)
	: P(P), I(I), D(D), output_ramp(ramp), limit(limit), Ts(Ts), error_prev(0.0),
	  output_prev(0.0), integral_prev(0.0)
{

}

double PIDController::operator() (double error)
{
	double proportional = P * error;

	double integral = integral_prev + I * Ts * 0.5 * (error + error_prev);
	integral = _constrain(integral, -limit, limit);

	double derivative = D * (error - error_prev) / Ts;

	double output = proportional + integral + derivative;
	output = _constrain(output, -limit, limit);

	if (output_ramp > 0) {
		double output_rate = (output - output_prev) / Ts;

		if (output_rate > output_ramp)
			output = output_prev + output_ramp * Ts;
		else if (output_rate < -output_ramp)
			output = output_prev - output_ramp * Ts;
	}

	integral_prev = integral;
	output_prev = output;
	error_prev = error;

	return output;
}




