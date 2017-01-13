/*
 * Header file providing class for directions
 * Author: Frederick Wachter
 */

#ifndef __OBSTACLES__
#define __OBSTACLES__

#include <array>
#include <iostream>
using namespace std;

class Directions {
public:
	bool *directions[4];

	// Defualt constructor
	Directions(void) {
		this->directions[0] = &this->north;
		this->directions[1] = &this->east;
		this->directions[2] = &this->south;
		this->directions[3] = &this->west;
	}

	// Alternate constructor(s)
	Directions(const bool north, const bool east, const bool south, const bool west) { // 0 - obstacle, 1 - free space
		this->north = north;
		this->east  = east;
		this->south = south;
		this->west  = west;
		this->directions[0] = &this->north;
		this->directions[1] = &this->east;
		this->directions[2] = &this->south;
		this->directions[3] = &this->west;
	}

	// Member function
	void setDirections(const bool north, const bool east, const bool south, const bool west) {
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
	void operator = (const Directions &rhs) {
		this->setDirections(*rhs.directions[0], *rhs.directions[1], *rhs.directions[2], *rhs.directions[3]);
	}

	friend ostream& operator << (ostream& os, const Directions &rhs) {
		os << "[" << rhs.isAvailable(0) << " " << rhs.isAvailable(1) << " " << rhs.isAvailable(2) <<
			" " << rhs.isAvailable(3) << "]";
		return os;
	}
private:
	bool north, east, south, west;
};

#endif


