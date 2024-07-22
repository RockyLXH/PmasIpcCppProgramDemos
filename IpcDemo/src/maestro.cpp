#include "maestro.h"

namespace Maestro
{
	MMC_CONNECT_HNDL connectHandler;				// Connection Handle

	CMMCHostComm modbusConnect;


	bool terminate = false;		// Flag to request program termination

	void Emergency_Received(unsigned short usAxisRef, short sEmcyCode);
	void TerminateApplication(int iSigNum);
	int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize, void* lpsock);
	int OnRunTimeError(const char *msg, unsigned int uiConnHndl,
				       unsigned short usAxisRef, short sErrorID, unsigned short usStatus);
	void ReadModbus(void);
	void WriteModbus(void);

	void SystemInit(void)
	{
		CMMCConnection connect;
		// ensure there is at least 1 axes to be configured.
		static_assert(MAX_AXES >= 1, "configure at least 1 axis by setting 'MAX_AXES' macro!");

		// InitializeCommunication to the GMAS:
		connectHandler = connect.ConnectIPCEx(0x7fffffff, (MMC_MB_CLBK) CallbackFunc);

		modbusConnect.MbusStartServer(connectHandler, 1);

		// 	Enable throw feature.
		CMMCPPGlobal::Instance()->SetThrowFlag(true, false);

		// Register Run Time Error Callback function
		CMMCPPGlobal::Instance()->RegisterRTE(OnRunTimeError);

		// Register the callback function for Emergency:
		connect.RegisterEventCallback(MMCPP_EMCY, (void*) Emergency_Received);

		// Register Ctrl+c signal handler
		struct sigaction SigAction;
		SigAction.sa_handler = TerminateApplication;
		sigaction(SIGINT, &SigAction, NULL);

		return;
	}

	void SystemLoop(void)
	{
		while (!terminate)
		{
			ReadModbus();

			usleep(100000);		// sleep 100ms

			WriteModbus();
		}

		return;
	}

	void SystemClose(void)
	{
	//
	//	Here will come code for all closing processes
	//
		modbusConnect.MbusStopServer();
		MMC_CloseConnection(connectHandler);

		return;
	}

	int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize, void* lpsock)
	{
		// Which function ID was received ...
		switch (recvBuffer[1])
		{
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

	int OnRunTimeError(const char *msg, unsigned int uiConnHndl,
			unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
	{
		MMC_CloseConnection(uiConnHndl);
		printf(
				"MMCPPExitClbk: Run time Error in function %s, axis ref=%d, err=%d, status=%d, bye\n",
				msg, usAxisRef, sErrorID, usStatus);
		exit(0);
	}

	void Emergency_Received(unsigned short usAxisRef, short sEmcyCode)
	{
		printf("Emergency Message Received on Axis %d. Code: %x\n", usAxisRef,
				sEmcyCode);

	}

	void TerminateApplication(int iSigNum)
	{
		printf("\nTerminating Application ...\n");
		terminate = true;
		sigignore(SIGALRM);

		return;
	}

	/**
	 * HoldingRegister[0] -> terminate the programm.
	 * HoldingRegister[1] -> set target velocity, unit: rpm.
	 * HoldingRegister[2] -> velocity loop kp.
	 * HoldingRegister[3] -> velocity loop ki.
	 */
	void ReadModbus(void)
	{
		MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT modbusReadOut;

		modbusConnect.MbusReadHoldingRegisterTable(0, 4, modbusReadOut);

		terminate = modbusReadOut.regArr[0];
//		targetVelocity = static_cast<double>(modbusReadOut.regArr[1]);
//		vel_kp = static_cast<double>(modbusReadOut.regArr[2] / 10000.0);
//		vel_ki = static_cast<double>(modbusReadOut.regArr[3] / 1000.0);

		return;
	}

	void WriteModbus(void)
	{
		MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN modbusWriteIn;

		modbusWriteIn.startRef = 2;
		modbusWriteIn.refCnt = 2;
//		modbusWriteIn.regArr[0] = static_cast<short>(vel_kp * 10000.0);
//		modbusWriteIn.regArr[1] = static_cast<short>(vel_ki * 1000.0);

		modbusConnect.MbusWriteHoldingRegisterTable(modbusWriteIn);

		return;
	}
}



