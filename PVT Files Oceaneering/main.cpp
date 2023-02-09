/*
============================================================================
 Name : 	test2.cpp
 Author :	Benjamin Spitzer
 Version :	1.00
 Description : The following example supports the following functionalities:

The following features are demonstrated for a Ethercat network (Assuming PDOs were initalized via Ethercat configurator):

- Two separate  state machines for handling parallel motions / sequences.
- Modbus callback registration.
- Emergency callback registration.
- Empty modbus reading area.

 The program works with 2 axes - a01 and a02.

============================================================================
*/
#include "mmc_definitions.h"
#include "mmcpplib.h"
#include "main.h"		// Application header file.
#include <iostream>
#include <sys/time.h>			// For time structure
#include <signal.h>				// For Timer mechanism
/*
============================================================================
 Function:				main()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 The main function of this sample project.
============================================================================
*/

int main()
{
	//
	MainInit();			//	Initialize system, axes and all needed initializations


	LinearProgram();


	MainClose();		//	Close what needs to be closed before program termination
	//
	return 1;		// Terminate the application program back to the Operating System
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
//
//	Here will come initialization code for all system, axes, communication etc.
//
// 	InitializeCommunication to the GMAS:
//
	gConnHndl = cConn.ConnectIPCEx(0x7fffffff,(MMC_MB_CLBK)CallbackFunc) ;
	//
	// Start the Modbus Server:
	cHost.MbusStartServer(gConnHndl,1) ;
	//
	// Register Run Time Error Callback function
	CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);

	// Register the callback function for Modbus and Emergency:
	cConn.RegisterEventCallback(MMCPP_MODBUS_WRITE,(void*)ModbusWrite_Received) ;
	cConn.RegisterEventCallback(MMCPP_EMCY,(void*)Emergency_Received) ;
//
// Initialize default parameters. This is not a must. Each parameter may be initialized individually.
	stSingleDefault.fEndVelocity	= 0 ;
	stSingleDefault.dbDistance 		= 10000 ;
	stSingleDefault.dbPosition 		= 0 ;
	stSingleDefault.fVelocity 		= 10000 ;
	stSingleDefault.fAcceleration 	= 10000000 ;
	stSingleDefault.fDeceleration 	= 10000000 ;
	stSingleDefault.fJerk 			= 200000000 ;
	stSingleDefault.eDirection 		= MC_POSITIVE_DIRECTION ;
	stSingleDefault.eBufferMode 	= MC_BUFFERED_MODE ;
	stSingleDefault.ucExecute 		= 1 ;
//
	a01.InitAxisData("a01",gConnHndl) ;
	a02.InitAxisData("a02",gConnHndl) ;
	a03.InitAxisData("a03",gConnHndl) ;
	//
	// Set default motion parameters.
	a01.SetDefaultParams(stSingleDefault) ;
	a02.SetDefaultParams(stSingleDefault) ;
	a03.SetDefaultParams(stSingleDefault) ;
	//
	giXStatus 	= a01.ReadStatus() ;
	if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a01.Reset() ;
		sleep(1) ;
		giXStatus 	= a01.ReadStatus() ;
		if(giXStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a01 in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	giYStatus 	= a02.ReadStatus() ;
	if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a02.Reset() ;
		sleep(1) ;
		giYStatus 	= a02.ReadStatus() ;
		if(giYStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a02 in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	giZStatus 	= a03.ReadStatus() ;
	if(giZStatus & NC_AXIS_ERROR_STOP_MASK)
	{
		a03.Reset() ;
		sleep(1) ;
		giZStatus 	= a03.ReadStatus() ;
		if(giZStatus & NC_AXIS_ERROR_STOP_MASK)
		{
			cout << "Axis a03 in Error Stop. Aborting." ;
			exit(0) ;
		}
	}
	//
	stVectorDefault.fAcceleration	= 100000000.0;
	stVectorDefault.fDeceleration	= 100000000.0;
	stVectorDefault.fJerk			= 10000000000.0;
	stVectorDefault.fVelocity		= 10000.0;
	stVectorDefault.eBufferMode		= MC_BUFFERED_MODE;
	stVectorDefault.eTransitionMode	= MC_TM_NONE_MODE;
	stVectorDefault.eCoordSystem	= MC_MCS_COORD;
	stVectorDefault.m_uiExecDelayMs = 0;
	stVectorDefault.ucSuperimposed 	= 0;
	stVectorDefault.ucExecute		=1;

	// Initialize Vctor names and default parameters.
	cVector.InitAxisData("v01",gConnHndl) ;
	cVector.SetDefaultParams(stVectorDefault);
	giStatus = cVector.GroupReadStatus();
	if(giStatus & NC_GROUP_ERROR_STOP_MASK)
	{
		cVector.GroupReset();
	}
	//
	// Clear the modbus memory array:
	memset(mbus_write_in.regArr,0x0,250) ;
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
	cHost.MbusStopServer() ;
	MMC_CloseConnection(gConnHndl) ;
	return;
}
/*
============================================================================
 Function:				LinearProgram()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:
 Modifications:			N/A

 Description:
 A linear code example
============================================================================
*/
void LinearProgram()
{
	MC_PATH_REF TableHandler;

	printf("Start Linear Program Example.\n");

	a01.PowerOn(MC_BUFFERED_MODE);
	a02.PowerOn(MC_BUFFERED_MODE);
	a03.PowerOn(MC_BUFFERED_MODE);

	while (!(a01.ReadStatus() & NC_AXIS_STAND_STILL_MASK));
	while (!(a02.ReadStatus() & NC_AXIS_STAND_STILL_MASK));
	while (!(a03.ReadStatus() & NC_AXIS_STAND_STILL_MASK));

	cVector.GroupEnable();

	while (!(cVector.GroupReadStatus() & NC_GROUP_STANDBY_MASK));


	MMC_SETKINTRANSFORM_IN stSetKinTransform;

	stSetKinTransform.iNumAxes=3;

	stSetKinTransform.hNode[0]=a01.GetRef();
	stSetKinTransform.eType[0]=NC_X_AXIS_TYPE;
	stSetKinTransform.iMcsToAcsFuncID[0]=NC_TR_SHIFT_FUNC;
	stSetKinTransform.ulTrCoef[0][NC_BACK_TR_RATIO_COEF]=1000;
	stSetKinTransform.ulTrCoef[0][NC_FRWD_TR_RATIO_COEF]=0.001;
	stSetKinTransform.ulTrCoef[0][NC_BACK_SHIFT_COEF]=0.0;

	stSetKinTransform.hNode[1]=a02.GetRef();
	stSetKinTransform.eType[1]=NC_Y_AXIS_TYPE;
	stSetKinTransform.iMcsToAcsFuncID[1]=NC_TR_SHIFT_FUNC;
	stSetKinTransform.ulTrCoef[1][NC_BACK_TR_RATIO_COEF]=1000;
	stSetKinTransform.ulTrCoef[1][NC_FRWD_TR_RATIO_COEF]=0.001;
	stSetKinTransform.ulTrCoef[1][NC_BACK_SHIFT_COEF]=0.0;

	stSetKinTransform.hNode[2]=a03.GetRef();
	stSetKinTransform.eType[2]=NC_Z_AXIS_TYPE;
	stSetKinTransform.iMcsToAcsFuncID[2]=NC_TR_SHIFT_FUNC;
	stSetKinTransform.ulTrCoef[2][NC_BACK_TR_RATIO_COEF]=1000;
	stSetKinTransform.ulTrCoef[2][NC_FRWD_TR_RATIO_COEF]=0.001;
	stSetKinTransform.ulTrCoef[2][NC_BACK_SHIFT_COEF]=0.0;

	stSetKinTransform.eBufferMode=MC_BUFFERED_MODE;

	cVector.SetKinTransform(stSetKinTransform);

	float fVel = 50;
	double dPos[MAX_AXES];

	dPos[0]=76.2; dPos[1]=76.2; dPos[2]=11.0;
	cVector.MoveLinearAbsolute(fVel, dPos, 1000.0, 1000.0, 10000.0, MC_BUFFERED_MODE);

	while (!(cVector.GroupReadStatus() & NC_GROUP_STANDBY_MASK));


	TableHandler = cVector.LoadPVTTableFromFile("/mnt/jffs/usr/PVT_DEMO1.txt",MC_MCS_COORD);

	cVector.MovePVT(TableHandler,MC_MCS_COORD);


	while (!(cVector.GroupReadStatus() & NC_GROUP_STANDBY_MASK));

	cVector.UnloadPVTTable(TableHandler);

	cVector.GroupDisable();

	a01.PowerOff(MC_BUFFERED_MODE);
	a02.PowerOff(MC_BUFFERED_MODE);
	a03.PowerOff(MC_BUFFERED_MODE);





	printf("Program End.\n");

	return;		// Back to the main() for program termination
}


/*
============================================================================
 Function:				EnableMachineSequencesTimer()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 Enables the main machine sequences timer function, to be executed each
 TIMER_CYCLE ms.
============================================================================
*/
void EnableMachineSequencesTimer(int TimerCycle)
{
	struct itimerval timer;
	struct sigaction stSigAction;

	// Whenever a signal is caught, call TerminateApplication function
	stSigAction.sa_handler = TerminateApplication;

	sigaction(SIGINT, &stSigAction, NULL);
	sigaction(SIGTERM, &stSigAction, NULL);
	sigaction(SIGABRT, &stSigAction, NULL);
	sigaction(SIGQUIT, &stSigAction, NULL);
//
//	Enable the main machine sequences timer function
//
	timer.it_interval.tv_sec 	= 0;
	timer.it_interval.tv_usec 	= TimerCycle * 1000;// From ms to micro seconds
	timer.it_value.tv_sec 		= 0;
	timer.it_value.tv_usec 		= TimerCycle * 1000;// From ms to micro seconds

//	setitimer(ITIMER_REAL, &timer, NULL);			- Temporarily !!!

//	signal(SIGALRM, MachineSequencesTimer); 		// every TIMER_CYCLE ms SIGALRM is received which calls MachineSequencesTimer()

	return;
}
/*
============================================================================
 Function:				MachineSequencesTimer()
 Input arguments:		None.
 Output arguments: 		None.
 Returned value:		None.
 Version:				Version 1.00
 Updated:				10/03/2011
 Modifications:			N/A

 Description:

 A timer function that is called by the OS every TIMER_CYCLE ms.
 It executes the machine sequences states machines and actully controls
 the sequences and behavior of the machine.
============================================================================
*/
void MachineSequencesTimer(int iSig)
{

	return;		// End of the sequences timer function. The function will be triggered again upon the next timer event.
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
int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock)
{
	// Which function ID was received ...
	switch(recvBuffer[1])
	{
	case EMCY_EVT:
		//
		// Please note - The emergency event was registered.
		// printf("Emergency Event received\r\n") ;
		break ;
	case MOTIONENDED_EVT:
		printf("Motion Ended Event received\r\n") ;
		break ;
	case HBEAT_EVT:
		printf("H Beat Fail Event received\r\n") ;
		break ;
	case PDORCV_EVT:
		printf("PDO Received Event received - Updating Inputs\r\n") ;
		break ;
	case DRVERROR_EVT:
		printf("Drive Error Received Event received\r\n") ;
		break ;
	case HOME_ENDED_EVT:
		printf("Home Ended Event received\r\n") ;
		break ;
	case SYSTEMERROR_EVT:
		printf("System Error Event received\r\n") ;
		break ;
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
	//
	return 1 ;
}

int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
{
	MMC_CloseConnection(uiConnHndl);
	printf("MMCPPExitClbk: Run time Error in function %s, axis ref=%d, err=%d, status=%d, bye\n", msg, usAxisRef, sErrorID, usStatus);
	exit(0);
}

///////////////////////////////////////////////////////////////////////
//	Function name	:	void terminate_application(int iSigNum)
//	Created			:	Version 1.00
//	Updated			:	20/05/2010
//	Modifications	:	N/A
//	Purpose			:	Called in case application is terminated, stop modbus, engines, and power off engines
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
//	giTerminate = 1 ;
	sigignore(SIGALRM);
	//
	// Wait till other threads exit properly.
	sleep(1) ;
	return ;
}

//
// Callback Function once a Modbus message is received.
void ModbusWrite_Received()
{
	printf("Modbus Write Received\n") ;
}
//
// Callback Function once an Emergency is received.
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
{
	printf("Emergency Message Received on Axis %d. Code: %x\n",usAxisRef,sEmcyCode) ;
}


int WaitTillStandStill(CMMCSingleAxis cAxis)
{
	while (1)
	{
		if (cAxis.ReadStatus() & NC_AXIS_STAND_STILL_MASK)
			break;
		usleep(10000);
	}
	return 0;
}



int WaitTillDisabled(CMMCSingleAxis cAxis)
{
	while (1)
	{
		if (cAxis.ReadStatus() & NC_AXIS_DISABLED_MASK)
			break;
		usleep(10000);
	}
	return 0;
}

unsigned short Condition(CMMCSingleAxis cAxis, unsigned long ulCondition)
{
	if (cAxis.ReadStatus() & ulCondition)
		return TRUE;
	else
		return FALSE;
}
