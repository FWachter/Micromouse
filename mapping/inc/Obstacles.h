/*
 * Header file providing class for obstacles
 * Author: Frederick Wachter
 */

#ifndef __OBSTACLES__
#define __OBSTACLES__

#include <array>
#include <iostream>
using namespace std;

class Obstacles {
public:
	bool *directions[4] = { &north, &east, &south, &west };

	// Defualt constructor
	Obstacles() {}

	// Alternate constructor(s)
	Obstacles(const bool north, const bool east, const bool south, const bool west) { // 0 - obstacle, 1 - free space
		this->north = north;
		this->east  = east;
		this->south = south;
		this->west  = west;
	}

	// Member function
	void setObstacles(const bool north, const bool east, const bool south, const bool west) {
		this->north = north;
		this->east  = east;
		this->south = south;
		this->west  = west;
	}

	void removeDirection(const int direction) const {
		*directions[direction] = 0;
	}

	bool isAvailable(const int direction) const {
		return *directions[direction];
	}

	// Operator overloads
	void operator = (const Obstacles &rhs) {
		this->setObstacles(*rhs.directions[0], *rhs.directions[1], *rhs.directions[2], *rhs.directions[3]);
	}

	friend ostream& operator << (ostream& os, const Obstacles &rhs) {
		os << "[" << rhs.isAvailable(0) << " " << rhs.isAvailable(1) << " " << rhs.isAvailable(2) <<
			" " << rhs.isAvailable(3) << "]";
		return os;
	}
private:
	bool north, east, south, west;
};

#endif


