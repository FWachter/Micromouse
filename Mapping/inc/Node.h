/*
 * Class file for representation of a Node in the map
 * Author: Alexander Nhan
 */

#ifndef __NODE__
#define __NODE__

class Node {
public:
	Node(double x, double y, unsigned int t) {
		this->x = x;
		this->y = y;
		this->availableTurns = t;
	};
	double x; // x position
	double y; // y position
	// available turns to visit at each node
	// 0: used all available directions
	// 1: left direction available
	// 2: right direction available
	// 3: both directions available
	unsigned int availableTurns;
};

#endif
