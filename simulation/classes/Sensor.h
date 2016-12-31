/*
 * Header file providing class for micromouse sensor simulator
 * Author: Frederick Wachter
 */

#ifndef __SENSOR__
#define __SENSOR__

#include <map>
#include <sstream>
#include <string>

#include "Location.h"
#include "Obstacles.h"
using namespace std;

class Sensor {
	public:
		map<Location, Obstacles> sensorMap;

		// Defualt constructor
		Sensor() {}

		// Alternate constructor(s)
		Sensor(const string fileName) {
			this.loadSensorFile(fileName);
		}

		// Member Function(s)
		bool loadSensorFile(const string fileName) {
			// ADD: Change directory to log folder directory
			ifstream sensorFile(fileName.c_str());
			if (sensorFile.is_open()) {
				string line;
				if (getline(sensorFile, line)) {
					while (getline(sensorFile, line)) { // remove header components
						if (line.compare(0, 5, endOfHeader) == 0) {
							getline(sensorFile, line);
							break;
						}
					}

					int x, y;
					bool north, east, south, west;
					Location location;
					Obstacles obstacles;
					while (getline(sensorFile, line)) {
						// ADD: Delimit line into variables
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
		}

		Obstacles getAvailableDirections(const Location location) {
			return sensorMap.at(location);
		}
	private:
		const string badFile     = "The provided file could not be opened or does not exist";
		const string emptyFile   = "The provided file is empty";
		const string endOfHeader = "-----";
};

#endif


