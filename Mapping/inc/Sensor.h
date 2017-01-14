/*
 * Header file providing class for micromouse sensor simulator
 * Author: Frederick Wachter
 */

#ifndef __SENSOR__
#define __SENSOR__

#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "Location.h"
#include "Obstacles.h"
using namespace std;

class Sensor {
	public:
		int initialDirection;
		bool initialRightWallOpen;

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

					getline(sensorFile, line);
					initialDirection = stoi(line.substr(1, 1));
					initialRightWallOpen = (stoi(line.substr(5, 1)) != 0);

					int x, y;
					bool north, east, south, west;
					Location location;
					Obstacles obstacles;
					while (getline(sensorFile, line, ' ')) {
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
		}

		Obstacles getAvailableDirections(const Location location) {
			return sensorMap.at(location);
		}
	private:
		map<Location, Obstacles> sensorMap;

		const string badFile     = "The provided file could not be opened or does not exist";
		const string emptyFile   = "The provided file is empty";
		const string endOfHeader = "-----";
};

#endif


