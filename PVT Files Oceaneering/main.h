/*
============================================================================
 Name : CPP_Template.h
 Author  :		Benjamin Spitzer
 Version :
 Description : 	GMAS C/C++ project header file for Template
============================================================================
*/

/*
============================================================================
 Project general functions prototypes
============================================================================
*/
void MainInit();
void LinearProgram();
void MainClose();
void EnableMachineSequencesTimer(int TimerCycle);
void MachineSequencesTimer(int iSig);
int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus) ;
void TerminateApplication(int iSigNum);
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode) ;
void ModbusWrite_Received() ;
int  CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock);
int WaitTillStandStill(CMMCSingleAxis cAxis);
int WaitTillDisabled(CMMCSingleAxis cAxis) ;
unsigned short Condition(CMMCSingleAxis cAxis, unsigned long ulCondition);

/*
============================================================================
 General constants
============================================================================
*/
#define 	MAX_AXES				3		// number of Physical axes in the system. TODO Update MAX_AXES accordingly
#define 	FALSE					0
#define 	TRUE					1
//
// TODO: Modbus memory map offsets must be updated accordingly.
#define 	MODBUS_READ_OUTPUTS_INDEX	0	// Start of Modbus read address
#define 	MODBUS_READ_CNT				16	// Number of registers to read
#define 	MODBUS_UPDATE_START_INDEX	16	// Start of Modbus write address (update to host)
#define 	MODBUS_UPDATE_CNT			16	// Number of registers to update
/*
============================================================================
 Project constants
============================================================================
*/
#define		SLEEP_TIME				10000	// Sleep time of the backround idle loop, in micro seconds
#define		TIMER_CYCLE				20		// Cycle time of the main sequences timer, in ms

/*
============================================================================
 Application global variables
============================================================================
*/
//
// 	Examples for data read from the GMAS core about the X, Y drives
int		giXPosition;
int		giYPosition;
int		giXTorque;
int		giYTorque;
int		giXInMotion;
int		giYInMotion;
int 	giXStatus ;
int 	giYStatus ;
int 	giZStatus ;
int 	giStatus ;
//
/*
============================================================================
 Global structures for Elmo's Function Blocks
============================================================================
*/
MMC_CONNECT_HNDL gConnHndl ;					// Connection Handle
CMMCConnection cConn ;
CMMCSingleAxis a01,a02,a03 ;							// TODO : Update the names and number of the axes in the system
CMMCGroupAxis  cVector;
CMMCHostComm	cHost ;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN 	mbus_write_in;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_OUT 	mbus_write_out;
MMC_MOTIONPARAMS_SINGLE 	stSingleDefault ;	// Single axis default data
MMC_MOTIONPARAMS_GROUP		stVectorDefault ;
