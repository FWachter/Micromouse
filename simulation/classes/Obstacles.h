/*
 * Header file providing class for obstacles
 * Author: Frederick Wachter
 */

#ifndef __OBSTACLES__
#define __OBSTACLES__

#include <array>
using namespace std;

class Obstacles {
	public:
		bool *directions[4] = { &north, &east, &south, &west };

		// Defualt constructor
		Obstacles() {}

		// Alternate constructor(s)
		Obstacles(const bool north, const bool east, const bool south, const bool west) {
			this->north = north;
			this->east  = east;
			this->south = south;
			this->west  = west;
		}

		// Member function
		void removeObstacle(const int direction) {
			*directions[direction] = 0;
		}
	private:
		bool north, east, south, west;
};

#endif


