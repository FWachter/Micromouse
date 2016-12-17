/*
 * Class file for representation of a Node in the map
 * Author: Alexander Nhan
 */

#ifndef __NODE__
#define __NODE__

enum TURNS {o=0, l=1, r=2, rl=3, lr=3};
enum DIRECTION {N, E, S, W};

class Node {
public:
	Node(double x, double y, DIRECTION d, TURNS t) {
		this->x = x;
		this->y = y;
		this->availableTurns = t;
		this->direction = d;
	};
	double x; // x position
	double y; // y position
	// available turns to visit at each node
	// 0: used all available directions
	// 1: left direction available
	// 2: right direction available
	// 3: both directions available
	unsigned int availableTurns;
	// Direction the robot was facing when creating node
	unsigned int direction;
};

#endif
