/*
 * Implementation file for sensor class
 * Author: Frederick Wachter
 */

#include <fstream>
#include <iostream>
#include <sstream>

#include "Sensor.h"
using namespace std;

// Alternate constructor(s)
Sensor::Sensor(const string &fileName) {
	this->loadSensorFile(fileName);
}

// Load a sensor file
bool Sensor::loadSensorFile(const string &fileName) {
	ifstream sensorFile(fileName.c_str());
	if (sensorFile.is_open()) {
		string line;
		if (getline(sensorFile, line)) {
			// Skip past the header
			while (getline(sensorFile, line)) { // remove header components
				if (line.compare(0, 5, endOfHeader) == 0) {
					getline(sensorFile, line);
					break;
				}
			}

			// Get robot state location information
			getline(sensorFile, line);
			size_t lastSpace       = line.find_last_of(" ");
			size_t lastOpenBracket = line.find_last_of("[");
			initialDirection     = stoi(line.substr(1, 1));
			initialRightWallOpen = (stoi(line.substr(5, 1)) != 0);
			initialLocation.x    = stoi(line.substr(lastOpenBracket+1, lastSpace-lastOpenBracket-1));
			initialLocation.y    = stoi(line.substr(lastSpace+1, line.size()-lastSpace-2));

			// Get map information
			int x, y;
			bool north, east, south, west;
			Location location;
			Obstacles obstacles;
			while (getline(sensorFile, line)) { // while there are still new lines in the file
				size_t firstSpace = line.find_first_of(" ");
				size_t endBracket = line.find_first_of("]");

				x = stoi(line.substr(1, firstSpace-1));
				y = stoi(line.substr(firstSpace+1, endBracket-firstSpace-1));
				north = (stoi(line.substr(endBracket+3, 1)) != 0);
				east  = (stoi(line.substr(endBracket+5, 1)) != 0);
				south = (stoi(line.substr(endBracket+7, 1)) != 0);
				west  = (stoi(line.substr(endBracket+9, 1)) != 0);

				location.setLocation(x, y);
				obstacles.setObstacles(north, east, south, west);
				sensorMap.insert(pair<Location, Obstacles>(location, obstacles));
			}
		} else {
			cout << emptyFile << endl;
			return false;
		}
	} else {
		cout << badFile << endl;
		return false;
	}
	sensorFile.close();
	return true;
}

// Get obstacles at specified location
Obstacles Sensor::getAvailableDirections(const Location &location) {
	return sensorMap[location];
}