#pragma once

namespace Pid
{
	#define _constrain(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

	class PIDController
	{
	public:
		explicit
		PIDController(double, double, double, double, double, double);
		~PIDController() = default;

		PIDController(const PIDController&) = delete;
		PIDController&
		operator=(const PIDController&) = delete;

		double
		operator()(double error);

		const double&
		GetKp() const
		{
			return _kp;
		}
		void SetKp(const double kp)
		{
			_kp = kp;
		}

		const double&
		GetKi() const
		{
			return _ki;
		}
		void SetKi(const double ki)
		{
			_ki = ki;
		}

	private:
		double _kp;
		double _ki;
		double _kd;
		double _outputRamp;
		double _limit;
		double _ts;

		double _errorPrev;
		double _outputPrev;
		double _integralPrev;
	};
}
