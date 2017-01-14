/*
 * Class file for representation of a Node in the map
 * Author: Alexander Nhan
 */

#ifndef __NODE__
#define __NODE__

#include "Location.h"
#include "Obstacles.h"

enum DIRECTION {N, E, S, W};

class Node {
public:
	Node(double x, double y, int stackref, bool north=false, bool east=false, 
		bool south=false, bool west=false) {
		this->location.x = x;
		this->location.y = y;
		this->stackRef = stackRef;
		this->availableDirections[N] = north;
		this->availableDirections[E] = east;
		this->availableDirections[S] = south;
		this->availableDirections[W] = west;
	};

	int getTotalAvailableDirections(void) {
		int sum = 0;
		for (int i = 0; i < MAX_DIRECTIONS; i++) {
			sum += availableDirections.isAvailable(i);
		}

		return sum;
	}

	Location location;
	// available directions to visit at each node
	// availableDirections[N]: 1 if North is available, 0 else
	// availableDirections[E]: 1 if East is available, 0 else
	// availableDirections[S]: 1 if South is available, 0 else
	// availableDirections[W]: 1 if West is available, 0 else
	Obstacles availableDirections;
	int stackRef;
};

#endif
