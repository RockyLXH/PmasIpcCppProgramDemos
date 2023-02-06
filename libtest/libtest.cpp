/*
============================================================================
 Name : libtest.cpp
 Author :
 Version :
 Description : GMAS C++ project source file
============================================================================
*/

#include "libtest.h"
#include <iostream>
#include "MMC_Definitions.h"
#include "mylib.h"

using namespace std;

int main()
{
	cout << "Hello, world!" << endl;

	Maestro_LibExample ex;

	ex.get_MaestroLibVersion();


	return 0;
}
