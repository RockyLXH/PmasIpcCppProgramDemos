/*
 * pid.h
 *
 * Created on: Jul 25, 2022
 * Author: RockyLiu
 */

#pragma once

#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

class PIDController
{
	public:
		explicit PIDController(double P, double I, double D, double ramp, double limit, double Ts);
		~PIDController() = default;

		PIDController(const PIDController&) = delete;
		PIDController& operator=(const PIDController&) = delete;

		double operator()(double error);

		const double& GetKP() const;
		void setKP(const double);

		const double& GetKI() const;
		void setKI(const double);

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
