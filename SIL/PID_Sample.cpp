/*
 ============================================================================
 Name : PID_Sample.cpp
 Author :	Elmo Motion Control
 Version :	1.0
 ============================================================================
 */
#include "mmc_definitions.h"
#include "mmcpplib.h"
#include <iostream>
#include <sys/time.h>			// For time structure
#include <signal.h>				// For Timer mechanism
#include "PID_Sample.h"		// Application header file.
#include "pid.h"
#include <chrono>
#include <syslog.h>			// for system log
#include <math.h>

/**
 * Motion Mode Selection
 */
enum MotionMode
{
	PMode, VMode, TMode
};

MotionMode motionMode = TMode;

/*
 * Algorithms switch in SIL function
 */
#define ANALOG_COMMAND_FOR_VEL_LOOP 	0	// in VMode
#define SIN_GEN_FOR_POS_LOOP 			0	// in PMode
#define VEL_LOOP_PID_CONTROLLER			1	// in TMode

#define USE_MDS3						0

/*
 * Using Timer in function for measuring the time elapsed
 * use macro define 'TIMER()' in function which needs to be measured
 */

struct Timer
{
#if USE_MDS3
		private:
		const char* m_name;
		struct timeval start, end;
		public:
		Timer(const char* name) :m_name(name)
		{
			gettimeofday(&start, NULL);
		}

		~Timer()
		{
			gettimeofday(&end, NULL);
			std::cout << "function: " << m_name << " duration: [" << end.tv_usec - start.tv_usec << " us]\n";
		}
#else
	private:
		std::chrono::time_point<std::chrono::steady_clock> start;
		const char* m_name;
	public:
		Timer(const char* name) :
				m_name(name)
		{
			start = std::chrono::high_resolution_clock::now();
		}

		~Timer()
		{
			auto dur = std::chrono::high_resolution_clock::now() - start;
			std::cout << "function: " << m_name << " took: [" << dur.count() << " us]\n";
		}
#endif
};

#define MEASUREMENT	0
#if	MEASUREMENT
#define TIMER()	Timer timer(__PRETTY_FUNCTION__)
#else
#define TIMER(name)
#endif

/*
 *  Kp = 0.08, Ki = 1, Ts = 1ms for velocity close loop gains in rad/s units
 *  Kp = 0.01, Ki = 1, Ts = 1ms for velocity close loop gains in rpm units
 */
#if VEL_LOOP_PID_CONTROLLER
PIDController PID_velocity { vel_kp, vel_ki, 0.0f, 1000.0, 1.0, 0.001 };
#endif

/*
 ============================================================================
 Function:				main()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				29/07/2022
 Modifications:			N/A

 Description:			Use Modbus address to control the motion,
 HoldingRegister[0] -> terminate the programm.
 HoldingRegister[1] -> set target velocity, unit: rpm.

 The main function of this sample project.
 ============================================================================
 */

int main(int argc, char *argv[])
{
	try {
		/*
		 * open system log file to write.
		 *
		 *  LOG_CONS
		 *    Write directly to system console if there is an error while sending to system logger.
		 *
		 *  LOG_NDELAY
		 *     Open the connection immediately (normally, the connection is opened when the first message is logged).
		 *
		 *  LOG_NOWAIT
		 *     Don¡¯t  wait  for  child processes that may have been created while logging the message.  (The GNU C library does not create a
		 *     child process, so this option has no effect on Linux.)
		 *
		 *  LOG_ODELAY
		 *     The converse of LOG_NDELAY; opening of the connection is delayed until syslog() is called.  (This is the  default,  and  need
		 *     not be specified.)
		 *
		 *  LOG_PERROR
		 *     (Not in SUSv3.) Print to stderr as well.
		 *
		 *  LOG_PID
		 *     Include PID with each message.
		 */
		openlog("SIL Program", LOG_CONS | LOG_PID, 0);

		syslog(LOG_DEBUG, "the program <%s> is working\n", argv[0]);

		// Initialize system, axes and all needed initializations
		MainInit();

		SILInit();

		MainLoop();

		MainClose();

		return 0;

	} catch (CMMCException& e) {
		printf("Exception in function %s, axis ref=%s, err=%d, status=%d, bye\n", e.what(),
				e.axisName(), e.error(), e.status());
		MainClose();
		exit(0);
	} catch (...) {
		std::cerr << "Unknown exception caught\n";
		MainClose();
		exit(0);
	}
}

/**
 * HoldingRegister[0] -> terminate the programm.
 * HoldingRegister[1] -> set target velocity, unit: rpm.
 * HoldingRegister[2] -> velocity loop kp.
 * HoldingRegister[3] -> velocity loop ki.
 */
void ReadMbusInput()
{
	MBus.MbusReadHoldingRegisterTable(0, 4, mbus_read_out);

	giTerminate = mbus_read_out.regArr[0];
	target_velocity = (double) mbus_read_out.regArr[1];
	vel_kp = static_cast<double>(mbus_read_out.regArr[2] / 1000.0);
	vel_ki = static_cast<double>(mbus_read_out.regArr[3] / 1000.0);
}

void UpdatePID()
{
	mbus_write_in.startRef = 2;
	mbus_write_in.refCnt = 2;
	mbus_write_in.regArr[0] = static_cast<short>(vel_kp * 1000.0);
	mbus_write_in.regArr[1] = static_cast<short>(vel_ki * 1000.0);

	MBus.MbusWriteHoldingRegisterTable(mbus_write_in);
}

void MainLoop()
{
	UpdatePID();

	while (!giTerminate) {

		ReadMbusInput();

		if ((PID_velocity.GetKI() != vel_ki) || (PID_velocity.GetKP() != vel_kp)) {
			PID_velocity.setKI(vel_ki);
			PID_velocity.setKP(vel_kp);
		}

		usleep(500000);
	}
}

void SILInit()
{
	static_assert(MAX_AXES >= 1, "configure at least 1 axis by setting 'MAX_AXES' macro!");

	for (int i = 0; i < MAX_AXES; ++i) {
		// 0 - NC profiler
		// 1 - 0;
		// 2 - User
		switch (motionMode) {
			case PMode: {
				cRTaxis[i].SetBoolParameter(0, MMC_UCUSER607A_SRC, 0);
				cRTaxis[i].SetOpMode(OPM402_CYCLIC_SYNC_POSITION_MODE);
				while (cRTaxis[i].GetOpMode() != OPM402_CYCLIC_SYNC_POSITION_MODE)
					;
				break;
			}
			case VMode: {
				cRTaxis[i].SetBoolParameter(0, MMC_UCUSER60FF_SRC, 0);
				cRTaxis[i].SetOpMode(OPM402_CYCLIC_SYNC_VELOCITY_MODE);
				while (cRTaxis[i].GetOpMode() != OPM402_CYCLIC_SYNC_VELOCITY_MODE)
					;
				break;
			}
			case TMode: {
				cRTaxis[i].SetBoolParameter(0, MMC_UCUSER6071_SRC, 0);
				cRTaxis[i].SetOpMode(OPM402_CYCLIC_SYNC_TORQUE_MODE);
				while (cRTaxis[i].GetOpMode() != OPM402_CYCLIC_SYNC_TORQUE_MODE)
					;
				break;
			}
		}

		usleep(1000);	// wait some time for ensure the mode changes done.

		cRTaxis[i].PowerOn();

		while (!(cRTaxis[i].ReadStatus() & NC_AXIS_STAND_STILL_MASK))
			usleep(10000);

//			cRTaxis[0].MoveAbsolute(5000, 10000.0);
//
//			while(!(cRTaxis[0].ReadStatus() & NC_AXIS_STAND_STILL_MASK));
	}

	MMC_DestroySYNCTimer(gConnHndl);

	for (int i = 0; i < MAX_AXES; ++i) {
		switch (motionMode) {
			case PMode: {
				cRTaxis[i].SetUser607A(0.0f);
				cRTaxis[i].SetBoolParameter(2, MMC_UCUSER607A_SRC, 0);
				break;
			}
			case VMode: {
				cRTaxis[i].SetUser60FF(0.0f);
				cRTaxis[i].SetBoolParameter(2, MMC_UCUSER60FF_SRC, 0);
				break;
			}
			case TMode: {
				cRTaxis[i].SetUser6071(0.0f);
				cRTaxis[i].SetBoolParameter(2, MMC_UCUSER6071_SRC, 0);
				break;
			}
		}
	}

	MMC_CreateSYNCTimer(gConnHndl, SILCallBackFun, 1);

	MMC_SetRTUserCallback(gConnHndl, 1); //set user call-back function to highest priority -> 1.

}
/*
 ============================================================================
 Function:				MainInit()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Initilaize the system, including axes, communication, etc.
 ============================================================================
 */
void MainInit()
{
	// InitializeCommunication to the GMAS:
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff, (MMC_MB_CLBK) CallbackFunc);

	MBus.MbusStartServer(gConnHndl, 1);

	// 	Enable throw feature.
	CMMCPPGlobal::Instance()->SetThrowFlag(true, false);

	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);

	// Register the callback function for Emergency:
	cConn.RegisterEventCallback(MMCPP_EMCY, (void*) Emergency_Received);

	// Initialize default parameters. This is not a must. Each parameter may be initialized individually.
	stSingleDefault.fEndVelocity = 0;
	stSingleDefault.dbDistance = 100000;
	stSingleDefault.dbPosition = 0;
	stSingleDefault.fVelocity = 100000;
	stSingleDefault.fAcceleration = 1000000;
	stSingleDefault.fDeceleration = 1000000;
	stSingleDefault.fJerk = 20000000;
	stSingleDefault.eDirection = MC_POSITIVE_DIRECTION;
	stSingleDefault.eBufferMode = MC_BUFFERED_MODE;
	stSingleDefault.ucExecute = 1;
	//
	// TODO: Update number of necessary axes:
	//

	char sAxisName[20];

	for (int i = 0; i < MAX_AXES; ++i) {
		sprintf(sAxisName, "a%02d", i + 1);
		cRTaxis[i].InitAxisData(sAxisName, gConnHndl);
		cRTaxis[i].SetDefaultParams(stSingleDefault);

		if (cRTaxis[i].ReadStatus() & NC_AXIS_ERROR_STOP_MASK)
			cRTaxis[i].Reset();
	}

	return;
}
/*
 ============================================================================
 Function:				MainClose()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Close all that needs to be closed before the application progra, is
 terminated.
 ============================================================================
 */
void MainClose()
{
//
//	Here will come code for all closing processes
//
	MMC_DestroySYNCTimer(gConnHndl);

	for (auto axis : cRTaxis) {
		switch (motionMode) {
			case PMode: {
				axis.SetUser607A(0.0f);
				axis.SetBoolParameter(0, MMC_UCUSER607A_SRC, 0);
				break;
			}
			case VMode: {
				axis.SetUser60FF(0.0f);
				axis.SetBoolParameter(0, MMC_UCUSER60FF_SRC, 0);
				break;
			}
			case TMode: {
				axis.SetUser6071(0.0f);
				axis.SetBoolParameter(0, MMC_UCUSER6071_SRC, 0);
				break;
			}
		}
		axis.PowerOff();

		while (!(axis.ReadStatus() & NC_AXIS_DISABLED_MASK))
			usleep(10000);
	}

	MBus.MbusStopServer();
	MMC_CloseConnection(gConnHndl);

	return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Function name	:	void callback function																		//
//	Created			:	Version 1.00																				//
//	Updated			:	3/12/2010																					//
//	Modifications	:	N/A																							//
//	Purpose			:	interupt function 																			//
//																													//
//	Input			:	N/A																							//
//	Output			:	N/A																							//
//	Return Value	:	int																							//
//	Modifications:	:	N/A																							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize, void* lpsock)
{
	// Which function ID was received ...
	switch (recvBuffer[1]) {
		case EMCY_EVT:
			//
			// Please note - The emergency event was registered.
			// printf("Emergency Event received\r\n") ;
			break;
		case MOTIONENDED_EVT:
			printf("Motion Ended Event received\r\n");
			break;
		case HBEAT_EVT:
			printf("H Beat Fail Event received\r\n");
			break;
		case PDORCV_EVT:
			printf("PDO Received Event received - Updating Inputs\r\n");
			break;
		case DRVERROR_EVT:
			printf("Drive Error Received Event received\r\n");
			break;
		case HOME_ENDED_EVT:
			printf("Home Ended Event received\r\n");
			break;
		case SYSTEMERROR_EVT:
			printf("System Error Event received\r\n");
			break;
			/* This is commented as a specific event was written for this function. Once it occurs
			 * the ModbusWrite_Received will be called
			 case MODBUS_WRITE_EVT:
			 // TODO Update additional data to be read such as function parameters.
			 // TODO Remove return 0 if you want to handle as part of callback.
			 return 0;
			 printf("Modbus Write Event received - Updating Outputs\r\n") ;

			 break ;
			 */
	}

	return 1;
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Callback function in case a Run Time Error was received.
//	Input			:	const char - *msg, unsigned int - uiConnHndl, unsigned short - usAxisRef, short - sErrorID, unsigned short - usStatus.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
int OnRunTimeError(const char *msg, unsigned int uiConnHndl, unsigned short usAxisRef,
		short sErrorID, unsigned short usStatus)
{
	MMC_CloseConnection(uiConnHndl);
	printf("MMCPPExitClbk: Run time Error in function %s, axis ref=%d, err=%d, status=%d, bye\n",
			msg, usAxisRef, sErrorID, usStatus);
	exit(0);
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	void terminate_application(int iSigNum)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Called in case application is terminated, stop modbus, engines, and power off engines.
//	Input			:	int iSigNum - Signal Num.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
void TerminateApplication(int iSigNum)
{
	//
	printf("In Terminate Application ...\n");
	giTerminate = 1;
	sigignore(SIGALRM);
	//
	switch (iSigNum) {
		// Handle ctrl+c.
		case SIGINT:
			// TODO Close what needs to be closed before program termination.
			exit(0);
			break;
		default:
			break;
	}
	return;
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Callback function in case an Emergency event was received.
//	Input			:	unsigned short - usAxisRef, short - sEmcyCode.
//	Output			:	N/A
//	Return Value	:	void
//
//	Modifications:	:	N/A
//////////////////////////////////////////////////////////////////////
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
	printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef, sEmcyCode);

}

//static float sigmoid_curve_val[] ={
//                0.018, 0.029, 0.047, 0.076, 0.119, 0.182,
//
//                0.269, 0.377, 0.500, 0.622, 0.731, 0.8178,
//
//                0.881, 0.924, 0.952, 0.970, 0.982, 0.989,
//
//                0.993, 0.996, 0.997, 1.0};
//static int index = 0;

int SILCallBackFun(void)
{
	TIMER();

//	if (b) {
//		cRTaxis[1].EthercatWritePIVar(3, 0);
//		b = false;
//	} else {
//		cRTaxis[1].EthercatWritePIVar(3, 65536);
//		b = true;
//	}

	/*
	 * Analog inputs commands for velocity loop
	 */

#if ANALOG_COMMAND_FOR_VEL_LOOP
	short aiValue = 0;
	cRTaxis[0].EthercatReadPIVar(6,0,aiValue);

//	std::cout << aiValue << std::endl;
	if ((aiValue >=-1000) && (aiValue <=1000))
	aiValue = 0;

//	std::cout << aiValue << std::endl;
	double Kp = 50000.0f;
	double dOutput1 = Kp * aiValue / 1000.0f;

	cRTaxis[0].SetUser60FF(dOutput1);
	cRTaxis[1].SetUser60FF(dOutput1);
	cRTaxis[2].SetUser60FF(dOutput1);
#endif

	/*
	 * PID controller for Velocity loop
	 */

#if VEL_LOOP_PID_CONTROLLER
	double actual_velocity = cRTaxis[0].GetActualVelocity(); // * 60 / 10000.0f;

	double target_current = PID_velocity(target_velocity - actual_velocity);

	cRTaxis[0].SetUser6071(target_current);
#endif

	/*
	 * Sine Gen for Pos Loop
	 */

#if SIN_GEN_FOR_POS_LOOP
	double rtb_SineWave;
	int rtb_DataTypeConversion;

	// S-Function (sdspsine2): '<Root>/Sine Wave'
	rtb_SineWave = 10000.0f * sin(SineWave_AccFreqNorm);

	// Update accumulated normalized freq value
	// for next sample.  Keep in range [0 2*pi)
	SineWave_AccFreqNorm += SineWave_Frequency * 0.0062831853071795866;
	if (SineWave_AccFreqNorm >= 6.2831853071795862) {
		SineWave_AccFreqNorm -= 6.2831853071795862;
	} else {
		if (SineWave_AccFreqNorm < 0.0) {
			SineWave_AccFreqNorm += 6.2831853071795862;
		}
	}

	// End of S-Function (sdspsine2): '<Root>/Sine Wave'

	// DataTypeConversion: '<Root>/Data Type Conversion'
	rtb_SineWave = floor(rtb_SineWave);
//	  if (rtIsNaN(rtb_SineWave) || rtIsInf(rtb_SineWave)) {
//	    rtb_SineWave = 0.0;
//	  } else {
//	    rtb_SineWave = fmod(rtb_SineWave, 4.294967296E+9);
//	  }

	// DataTypeConversion: '<Root>/Data Type Conversion'
	rtb_DataTypeConversion = rtb_SineWave < 0.0 ? -static_cast<int>(static_cast<unsigned int>(-rtb_SineWave))
	: static_cast<int>(static_cast<unsigned int>(rtb_SineWave));

	// S-Function (E607AWrite): '<Root>/E607AWrite'
//	  Elmo_Write_607A(SineForPos_P.E607AWrite_p1, rtb_DataTypeConversion);
	cRTaxis[0].SetUser607A(rtb_DataTypeConversion);
#endif

	return 0;
}
