/*
============================================================================
 Name : static_spline.cpp
 Author :	Elmo Motion Control.
 Version :
 Description : 	GMAS C/C++ project header file for Template
============================================================================
*/

/*
============================================================================
 Project general functions prototypes
============================================================================
*/
using Func = std::string&(*)(double, double, double, std::string&);

void MainInit();
void MainClose();
int OnRunTimeError(const char *msg,  unsigned int uiConnHndl, unsigned short usAxisRef, short sErrorID, unsigned short usStatus) ;
void TerminateApplication(int iSigNum);
void Emergency_Received(unsigned short usAxisRef, short sEmcyCode) ;
int  CallbackFunc(unsigned char* recvBuffer, short recvBufferSize,void* lpsock);
void ChangeToRelevantMode();
int TrajectoryFileCreator(const char* fileName, Func func);
std::string& HelixCal(double n, double m, double a, std::string& str);
/*
============================================================================
 General constants
============================================================================
*/
#define 	MAX_AXES				2		// number of Physical axes in the system. TODO Update MAX_AXES accordingly
#define		PI						3.141592653
/*
============================================================================
 Application global variables
============================================================================
*/
int 	giTerminate;		// Flag to request program termination
// 	Examples for data read from the GMAS core about the X, Y drives
int 	giXStatus ;
int 	giYStatus ;
int 	giGroupStatus;
int 	giXOpMode;
int 	giYOpMode;
std::fstream file;
std::string str;



typedef struct
{
	int mode;
	int dimension;
	int numberOfPoints;
	double velocity;
	double acc;
	double dec;
	double jerk;
	double sfAcc;
	double sfDec;
} FileHeaderSt;
//
/*
============================================================================
 Global structures for Elmo's Function Blocks
============================================================================
*/
MMC_CONNECT_HNDL gConnHndl ;					// Connection Handle
CMMCConnection cConn ;
CMMCSingleAxis a1,a2,a3 ;							// TODO : Update the names and number of the axes in the system
CMMCGroupAxis v1;
MMC_MOTIONPARAMS_SINGLE 	stSingleDefault ;	// Single axis default data
MMC_MOTIONPARAMS_GROUP 		stGroupDefault;		// Group axis default data
MC_PATH_REF TableHandler;

