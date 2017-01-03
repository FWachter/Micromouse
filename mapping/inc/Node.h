/*
 * Class file for representation of a Node in the map
 * Author: Alexander Nhan
 */

#ifndef __NODE__
#define __NODE__

#include "Location.h"
#include "Obstacles.h"

class Node {
public:
	Node(double x, double y, int stackref, bool north=false, bool east=false, 
		bool south=false, bool west=false) {
		this->location.x = x;
		this->location.y = y;
		this->stackRef = stackRef;
		this->availableDirections.setObstacles(north, east, south, west);
	};

	int getTotalAvailableDirections(void) {
		int sum = 0;
		for (int i = 0; i < MAX_DIRECTIONS; i++) {
			sum += availableDirections.isAvailable(i);
		}

		return sum;
	}

	int stackRef;
	Location location;
	// available directions to visit at each node
	// availableDirections.isAvailable(0): 1 if North is available, 0 else
	// availableDirections.isAvailable(1): 1 if East is available, 0 else
	// availableDirections.isAvailable(2): 1 if South is available, 0 else
	// availableDirections.isAvailable(3): 1 if West is available, 0 else
	Obstacles availableDirections;

private:
	const int MAX_DIRECTIONS = 4;
};

#endif
