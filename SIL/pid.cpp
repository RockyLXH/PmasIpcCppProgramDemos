/*
 * pid.cpp
 *
 * Created on: Jul 25, 2022
 * Author: RockyLiu
 */

#include "pid.h"

PIDController::PIDController(double kp, double ki, double kd, double ramp,
		double limit, double ts = 0.00025) :
		_kp(kp), _ki(ki), _kd(kd), _outputRamp(ramp), _limit(limit), _ts(ts)
{

}

double PIDController::operator()(double error)
{
	double proportional = _kp * error;

	double integral = _integralPrev + _ki * _ts * 0.5f * (error + _errorPrev);
	integral = _constrain(integral, -_limit, _limit);

	double derivative = _kd * (error - _errorPrev) / _ts;

	double output = proportional + integral + derivative;
	output = _constrain(output, -_limit, _limit);

	if (_outputRamp > 0)
	{
		double output_rate = (output - _outputPrev) / _ts;

		if (output_rate > _outputRamp)
			output = _outputPrev + _outputRamp * _ts;
		else if (output_rate < -_outputRamp)
			output = _outputPrev - _outputRamp * _ts;
	}

	_integralPrev = integral;
	_outputPrev = output;
	_errorPrev = error;

	return output;
}

