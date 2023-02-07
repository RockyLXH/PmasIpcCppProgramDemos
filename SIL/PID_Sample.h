/*
============================================================================
 Name : PID_Sample.cpp
 Author  :	Elmo Motion Control
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
void MainClose();
void SILInit();
void MainLoop();
int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus) ;
void TerminateApplication(int iSigNum);
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode) ;;
int  CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock);
int SILCallBackFun(void);

/*
============================================================================
 General constants
============================================================================
*/
#define 	MAX_AXES				1		// number of Physical axes in the system
/*
============================================================================
 Application global variables
============================================================================
*/

bool 	giTerminate = 0;		// Flag to request program termination
double	target_velocity = 0.0f;
double  prev_target_velocity = 0.0f;
double  acc_dec = 50.0f;
double  acc_dec_time = 0.0f;
double SineWave_AccFreqNorm = 0.0f;
double SineWave_Frequency = 1.0f;

/*
============================================================================
 Global structures for Elmo's Function Blocks
============================================================================
*/
MMC_CONNECT_HNDL 			gConnHndl ;					// Connection Handle
CMMCConnection 				cConn ;
CMMCHostComm				MBus;

MMC_MOTIONPARAMS_SINGLE 	stSingleDefault ;	// Single axis default data

CMMCRTSingleAxis 			cRTaxis[MAX_AXES];

MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT		mbus_read_out;
