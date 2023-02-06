#include <stdio.h>
#include "mmcpplib.h"
#include "ecat_sm_service.h"

extern MMC_CONNECT_HNDL gConnHndl ;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//	Function name	:	void callback function																		//
//	Created			:	Version 1.00																				//
//	Updated			:	3/12/2010																					//
//	Modifications	:	N/A																							//
//	Purpose			:	interuprt function 																			//
//																													//
//	Input			:	N/A																							//
//	Output			:	N/A																							//
//	Return Value	:	int																							//
//	Modifications:	:	N/A																							//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock)
{
	// Whcih function ID was received ...
	switch(recvBuffer[1])
	{
	case ASYNC_REPLY_EVT:
		printf("ASYNC event Reply\r\n") ;
		break ;
	case EMCY_EVT:
		// Please note - The emergency event was registered.
		// printf("Emergency Event received\r\n") ;
		break ;
	case MOTIONENDED_EVT:
		printf("Motion Ended Event received\r\n") ;
		break ;
	case STOP_ON_LIMIT_EVT:
		printf("Stop On Limit Event received\r\n") ;
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
	return 0 ;
}

void MMC_MbusWriteHoldingRegisterTable_wrapper(MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN *mbus_write_in,MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_OUT *mbus_write_out)
{
	if (MMC_MbusWriteHoldingRegisterTable(gConnHndl, mbus_write_in,
			mbus_write_out) != 0) {
		printf("ERROR:%s: MMC_MbusWriteHoldingRegisterTable fail \n",
				__func__);
		exit(-1) ;
	}
}

void MMC_MbusReadHoldingRegisterTable_wrapper(MMC_MODBUSREADHOLDINGREGISTERSTABLE_IN *mbus_read_in, MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT *mbus_read_out)
{
	if (MMC_MbusReadHoldingRegisterTable(gConnHndl, mbus_read_in,
			mbus_read_out) != 0) {
		printf("ERROR:%s: MMC_MbusReadHoldingRegisterTable fail \n",
				__func__);
		exit(1);
	}
}

void MMC_MbusStopServer_wrapper()
{
	MMC_MODBUSSTOPSERVER_IN mbus_stopserver_in;
	MMC_MODBUSSTOPSERVER_OUT mbus_stopserver_out;

	if(MMC_MbusStopServer(gConnHndl, &mbus_stopserver_in, &mbus_stopserver_out)!=0)
	{
		printf("ERROR:%s: MMC_MbusStoptServer fail \n", __func__);
					MMC_CloseConnection(gConnHndl);
					exit(-1);
	}
}

void MMC_MbusStartServer_wrapper()
{
	MMC_MODBUSSTARTSERVER_IN mbus_startserver_in;
	MMC_MODBUSSTARTSERVER_OUT mbus_startserver_out;
	MMC_MODBUSISRUNNING_IN	mbus_IsRunningIn ;
	MMC_MODBUSISRUNNING_OUT	mbus_IsRunningOut ;

	MMC_MbusIsRunning (gConnHndl,&mbus_IsRunningIn,&mbus_IsRunningOut);
	if(!mbus_IsRunningOut.isrunning) {
		mbus_startserver_in.id = 1;

		if(MMC_MbusStartServer(gConnHndl, &mbus_startserver_in, &mbus_startserver_out)) {
			printf("ERROR:%s: MMC_MbusStartServer fail \n", __func__);
					MMC_CloseConnection(gConnHndl);
					exit(-1);
		}
	}
}
void InsertLongVarToModbusShortArr(short* spArr, long lVal)
{
	*spArr 		= (short) (lVal	& 0xFFFF);
	*(spArr + 1)= (short) (lVal >> 16);
}
int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus)
{
	MMC_CloseConnection(uiConnHndl);
	printf("OnRunTimeError: %s,axis ref=%d, err=%d, status=%d, bye\n", msg, usAxisRef, sErrorID, usStatus);
	exit(0);
}
//
// Callback Function once a Modbus message is received.
void ModbusWrite_Received()
{
	printf("Modbus Write Received\n") ;
}
//
// Callback Function once an Emergency is received.
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode, char cErrReg)
{
	printf("Emergency Message Received on Axis %d. Emcy Code: %x  Err Reg: %x\n", usAxisRef, sEmcyCode, cErrReg);
}
