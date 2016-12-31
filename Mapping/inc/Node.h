/*
 * Class file for representation of a Node in the map
 * Author: Alexander Nhan
 */

#ifndef __NODE__
#define __NODE__

enum DIRECTION {N, E, S, W};

class Node {
public:
	Node(double x, double y, bool north=false, bool east=false, 
		bool south=false, bool west=false) {
		this->x = x;
		this->y = y;
		this->availableDirections[N] = north;
		this->availableDirections[E] = east;
		this->availableDirections[S] = south;
		this->availableDirections[W] = west;
	};
	double x; // x position
	double y; // y position
	// available directions to visit at each node
	// availableDirections[N]: 1 if North is available, 0 else
	// availableDirections[E]: 1 if East is available, 0 else
	// availableDirections[S]: 1 if South is available, 0 else
	// availableDirections[W]: 1 if West is available, 0 else
	bool availableDirections[4];
};

#endif
