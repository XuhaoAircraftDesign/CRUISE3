#include "propulsion.h"

propulsion::propulsion():FileName("./Trajectory/engine.txt"), outFile(FileName, std::ios::out)
{
	thrust = 0;
	mass = 0;
	Jx = 0;
	Jy = 0;
	Jz = 0;
	enginemode = 0;
}

propulsion::~propulsion()
{
	outFile.close();
}

void propulsion::EngineCalculate(double t)
{

	thrust = thrust0;
	mass = mass0;
	Jx = Jx0;
	Jy = Jy0;
	Jz = Jz0;
	enginemode = 1;
	return;
}

void propulsion::savedata(double t)
{
	outFile << t << ",  " << mass << ",  " << thrust << ",  " << Jx << ",  " << Jy << ",  "
		<< Jz << ",  " << enginemode << std::endl;
	return;
}