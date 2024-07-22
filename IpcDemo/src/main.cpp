#include "maestro.h"
#include "sil.h"
#include <iostream>

int main(int agrc, char** agrv)
{
	std::cout << Maestro::banner;

	Maestro::SystemInit();

	if (Sil::AxisInit() == -1)
	{
		std::cout << "Sil::AxisInit() failed\n";
		return -1;
	}

	Sil::SilRun();

	Maestro::SystemLoop();

	Maestro::SystemClose();

	return 0;
}


