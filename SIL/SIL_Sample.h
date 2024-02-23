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
void
MainInit(void);
void
MainClose(void);
void
SILInit(void);
void
MainLoop(void);
int
OnRunTimeError(const char *msg, unsigned int uiConnHndl,
		unsigned short usAxisRef, short sErrorID, unsigned short usStatus);
void
TerminateApplication(int iSigNum);
void
Emergency_Received(unsigned short usAxisRef, short sEmcyCode);
;
int
CallbackFunc(unsigned char* recvBuffer, short recvBufferSize, void* lpsock);
int
SILCallBackFun(void);

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

bool giTerminate = false;		// Flag to request program termination
double target_velocity = 0.0;
double prev_target_velocity = 0.0;
double vel_ki = 0.0;
double vel_kp = 0.0001;
/*
 ============================================================================
 Global structures for Elmo's Function Blocks
 ============================================================================
 */
MMC_CONNECT_HNDL gConnHndl;				// Connection Handle
CMMCConnection cConn;
CMMCHostComm MBus;

MMC_MOTIONPARAMS_SINGLE stSingleDefault;	// Single axis default data

CMMCRTSingleAxis cRTaxis[MAX_AXES];

MMC_MODBUSREADHOLDINGREGISTERSTABLE_OUT mbus_read_out;
MMC_MODBUSWRITEHOLDINGREGISTERSTABLE_IN mbus_write_in;
