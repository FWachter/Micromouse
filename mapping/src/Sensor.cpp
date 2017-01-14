/*
 * Implementation file for sensor class
 * Author: Frederick Wachter
 */

#include <fstream>
#include <iostream>
#include <sstream>

#include "Sensor.h"
#include "Utilities.h"
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
			displaySensorFileStart();
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
			size_t firstSpace, endBracket;
			Directions directions;
			while (getline(sensorFile, line)) { // while there are still new lines in the file
				firstSpace = line.find_first_of(" ");
				endBracket = line.find_first_of("]");

				x = stoi(line.substr(1, firstSpace-1));
				y = stoi(line.substr(firstSpace+1, endBracket-firstSpace-1));
				north = (stoi(line.substr(endBracket+3, 1)) != 0);
				east  = (stoi(line.substr(endBracket+5, 1)) != 0);
				south = (stoi(line.substr(endBracket+7, 1)) != 0);
				west  = (stoi(line.substr(endBracket+9, 1)) != 0);
				displaySensorFileData(x, y, north, east, south, west);

				sensorMap.insert(pair<Location, int>(Location(x, y), 
					convertDirectionsToInt(Directions(north, east, south, west))));
			}
			displaySensorFileEnd();
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

// Convert Directions object to an interger
int Sensor::convertDirectionsToInt(const Directions &directions) {
	return ((directions.isAvailable(0)*1000) + (directions.isAvailable(1)*100) +
		(directions.isAvailable(2)*10) + directions.isAvailable(3));
}

// Convert an integer to an Obstacle object
Directions Sensor::convertIntToDirections(int obstacle) {
	bool north = 0, east = 0, south = 0, west = 0;
	if (obstacle >= 1000) { north = 1; obstacle -= 1000; }
	if (obstacle >= 100) { east = 1; obstacle -= 100; }
	if (obstacle >= 10) { south = 1; obstacle -= 10; }
	if (obstacle == 1) { west = 1; }
	return Directions(north, east, south, west);
}

// Get directions at specified location
Directions Sensor::getAvailableDirections(const Location &location) {
	int obstacle = sensorMap[location];
	return convertIntToDirections(obstacle);
}