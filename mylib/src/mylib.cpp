//
//	Simple Library Template which can be easily refactored for user's usage.
// 	This Library Includes a Single API:  get_MaestroLibVersion() which
//	uses the Maestro C++ library, by using the CMMCConnection class - returns the Library Version.
//

#include "stdafx.h"
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::ostringstream
#include <map>
#include <mmc_definitions.h>
#include <mmcpplib.h>
#include "mylib.h"

#include <signal.h>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#define nullptr NULL


/*!
*	constructor
*/
Maestro_LibExample::Maestro_LibExample()
{
	fprintf(stderr, "%s: constructor\n", __PRETTY_FUNCTION__);
}

/*!
*	destructor
*/
Maestro_LibExample::~Maestro_LibExample()
{
	fprintf(stderr, "%s, %d: destructor\n", __PRETTY_FUNCTION__, __LINE__);
}


int Maestro_LibExample::get_MaestroLibVersion()
{
	CMMCConnection m_Conn;
	m_Conn.GetLibVersion() ;

	return 0;
}


