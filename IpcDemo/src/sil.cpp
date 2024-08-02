#include "sil.h"
#include "mmcpplib.h"
#include "maestro.h"

namespace Sil
{
	CMMCRTSingleAxis rtAxis[MAX_AXES];

	void DoSinGenForPosLoop(void);

	OPM402 opMode = OPM402_CYCLIC_SYNC_POSITION_MODE;
	MMC_PARAMETER_LIST_ENUM source = MMC_UCUSER607A_SRC;
	auto currentSilFunc = DoSinGenForPosLoop;
	void (CMMCRTSingleAxis::*command)(int) = &CMMCRTSingleAxis::SetUser607A;

	int AxisInit(void)
	{
		MMC_MOTIONPARAMS_SINGLE axisDefaultParam;	// Single axis default data
		// Initialize default parameters. This is not a must. Each parameter may be initialized individually.
		axisDefaultParam.fEndVelocity = 0;
		axisDefaultParam.dbDistance = 100000;
		axisDefaultParam.dbPosition = 0;
		axisDefaultParam.fVelocity = 100000;
		axisDefaultParam.fAcceleration = 1000000;
		axisDefaultParam.fDeceleration = 1000000;
		axisDefaultParam.fJerk = 20000000;
		axisDefaultParam.eDirection = MC_POSITIVE_DIRECTION;
		axisDefaultParam.eBufferMode = MC_BUFFERED_MODE;
		axisDefaultParam.ucExecute = 1;
		//
		// TODO: Update number of necessary axes:
		//

		char axisName[20];

		for (int i = 0; i < MAX_AXES; ++i)
		{
			sprintf(axisName, "a%02d", i + 1);
			rtAxis[i].InitAxisData(axisName, Maestro::connectHandler);
			rtAxis[i].SetDefaultParams(axisDefaultParam);

			if (rtAxis[i].ReadStatus() & NC_AXIS_ERROR_STOP_MASK)
				rtAxis[i].Reset();

			MMC_DestroySYNCTimer(Maestro::connectHandler);

			rtAxis[i].SetBoolParameter(0, // 0 - NC profiler; 2 - User;
									   source,
									   0);
			rtAxis[i].SetOpMode(opMode);
			while (rtAxis[i].GetOpMode() != opMode);
			rtAxis[i].SetBoolParameter(2, source, 0);

			(rtAxis[i].*command)(0.0f);

			usleep(1000);	// wait some time for ensure the mode changes done.

			rtAxis[i].PowerOn();

			while (!(rtAxis[i].ReadStatus() & NC_AXIS_STAND_STILL_MASK))
				usleep(10000);
		}

		return 0;
	}

	void SilRun(void)
	{
		MMC_CreateSYNCTimer(Maestro::connectHandler, []
		{currentSilFunc(); return 0;}, 1); // sync timer 1X

		// set user call-back function to highest priority -> 1.
		// it must be set after CreateSyncTimer func, not before.
		MMC_SetRTUserCallback(Maestro::connectHandler, 1);

		return;
	}

	void DoSinGenForPosLoop(void)
	{
		double rtb_SineWave;
		double SineWave_AccFreqNorm = 0.0;
		double SineWave_Frequency = 1.0;
		int rtb_DataTypeConversion;

		rtb_SineWave = 10000.0f * sin(SineWave_AccFreqNorm);

		SineWave_AccFreqNorm += SineWave_Frequency * 0.0062831853071795866;
		if (SineWave_AccFreqNorm >= 6.2831853071795862)
		{
			SineWave_AccFreqNorm -= 6.2831853071795862;
		}
		else
		{
			if (SineWave_AccFreqNorm < 0.0)
			{
				SineWave_AccFreqNorm += 6.2831853071795862;
			}
		}

		rtb_SineWave = floor(rtb_SineWave);

		rtb_DataTypeConversion =
				rtb_SineWave < 0.0 ?
						-static_cast<int>(static_cast<unsigned int>(-rtb_SineWave)) :
						static_cast<int>(static_cast<unsigned int>(rtb_SineWave));

		rtAxis[0].SetUser607A(rtb_DataTypeConversion);
	}
}



