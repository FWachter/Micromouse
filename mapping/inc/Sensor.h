/*
 * Header file providing class for micromouse sensor simulator
 * Author: Frederick Wachter
 */

#ifndef __SENSOR__
#define __SENSOR__

#include <map>
#include <string>

#include "Location.h"
#include "Directions.h"
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

	// Convert Directions object to an interger
	int convertDirectionsToInt(const Directions &directions);

	// Convert an integer to an Obstacle object
	Directions convertIntToDirections(int directions);

	// Get the Directions object from a specified location
	Directions getAvailableDirections(const Location &location);
private:
	map<Location, int> sensorMap;

	const string badFile     = "The provided file could not be opened or does not exist";
	const string emptyFile   = "The provided file is empty";
	const string endOfHeader = "-----";
};

#endif


