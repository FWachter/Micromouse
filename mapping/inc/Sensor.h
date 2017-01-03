/*
 * Header file providing class for micromouse sensor simulator
 * Author: Frederick Wachter
 */

#ifndef __SENSOR__
#define __SENSOR__

#include <map>
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
	// Load a sensor file
	bool loadSensorFile(const string &fileName);

	// Convert Obstacles object to an interger
	int convertObstaclesToInt(const Obstacles &obstacles);

	// Convert an integer to an Obstacle object
	Obstacles convertIntToObstacles(int &obstacles);

	// Get the Obstacles object from a specified location
	Obstacles getAvailableDirections(const Location &location);
private:
	map<Location, int> sensorMap;

	const string badFile     = "The provided file could not be opened or does not exist";
	const string emptyFile   = "The provided file is empty";
	const string endOfHeader = "-----";
};

#endif


