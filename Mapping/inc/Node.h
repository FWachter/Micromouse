/*
 * Class file for representation of a Node in the map
 * Author: Alexander Nhan
 */

#ifndef __NODE__
#define __NODE__

class Node {
public:
	Node(double x, double y) {
		this->x = x;
		this->y = y;
	};
	double x; // x position
	double y; // y position
};

#endif
