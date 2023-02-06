/*
============================================================================
 Name : ext_rs485_example.h
 Author :
 Version :
 Description : GMAS C/C++ project header file
============================================================================
*/

/*
============================================================================
 Project general functions prototypes
============================================================================
*/
static void MainInit();
static void MainClose();
static int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus) ;
static void TerminateApplication(int iSigNum);
static void Emergency_Received(unsigned short usAxisRef, short sEmcyCode) ;;
static int  CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock);

/*
============================================================================
 General constants
============================================================================
*/
#define 	MAX_AXES				2		// number of Physical axes in the system. TODO Update MAX_AXES accordingly
/*
============================================================================
 Application global variables
============================================================================
*/
int 	giTerminate;		// Flag to request program termination
// 	Examples for data read from the GMAS core about the X, Y drives
int 	giXStatus ;
int 	giYStatus ;
//
/*
============================================================================
 Global structures for Elmo's Function Blocks
============================================================================
*/
MMC_CONNECT_HNDL gConnHndl ;					// Connection Handle
CMMCConnection cConn ;
CMMCSingleAxis a1,a2 ;							// TODO : Update the names and number of the axes in the system
MMC_MOTIONPARAMS_SINGLE 	stSingleDefault ;	// Single axis default data

