/*
 * Header file providing class for micromouse sensor simulator
 * Author: Frederick Wachter
 */

#ifndef __SENSOR__
#define __SENSOR__

#include <fstream>
#include <map>
#include <sstream>
#include <string>

#include "Location.h"
#include "Obstacles.h"
using namespace std;

class Sensor {
	public:
		int initialDirection;
		bool initialRightWallOpen;
		Location initialLocation;

		// Defualt constructor
		Sensor() {}

		// Alternate constructor(s)
		Sensor(const string &fileName);

		// Member Function(s)
		bool loadSensorFile(const string &fileName);

		Obstacles getAvailableDirections(const Location &location);
	private:
		map<Location, Obstacles> sensorMap;

		const string badFile     = "The provided file could not be opened or does not exist";
		const string emptyFile   = "The provided file is empty";
		const string endOfHeader = "-----";
};

#endif


