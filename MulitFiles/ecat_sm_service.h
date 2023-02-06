/*
 * ecat_sm_service.h
 *
 *  Created on: Feb 9, 2022
 *      Author: tjbli
 */

#pragma once

int CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock);
int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus);
void ModbusWrite_Received();
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode, char cErrReg);
void MMC_MbusStartServer_wrapper();
void MMC_MbusStopServer_wrapper();
void MMC_MbusWriteHoldingRegisterTable_wrapper(MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN *mbus_write_in,MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_OUT *mbus_write_out);
void MMC_MbusReadHoldingRegisterTable_wrapper(MMC_MODBUSREADHOLDINGREGISTERSTABLE_IN *mbus_read_in, MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT *mbus_read_out);
void InsertLongVarToModbusShortArr(short* spArr, long lVal);
