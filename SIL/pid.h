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
		explicit PIDController(double, double, double, double, double, double);
		~PIDController() = default;

		PIDController(const PIDController&) = delete;
		PIDController& operator=(const PIDController&) = delete;

		double operator()(double error);

		const double& GetKp() const;
		void SetKp(const double);

		const double& GetKi() const;
		void SetKi(const double);

	private:
		double kp_;
		double ki_;
		double kd_;
		double output_ramp_;
		double limit_;
		double ts_;

		double error_prev_ = 0.;
		double output_prev_ = 0.;
		double integral_prev_ = 0.;
};
