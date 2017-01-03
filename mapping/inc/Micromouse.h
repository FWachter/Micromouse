/*
 * Header file providing class for micromouse robot
 * Author: Frederick Wachter
 */

#ifndef __MICROMOUSE__
#define __MICROMOUSE__

#include <array>
#include <deque>
#include <memory>
#include <string>

#include "Location.h"
#include "Node.h"
using namespace std;

class Micromouse {
public:
	int direction = 0; // 0 is North, 1 is East, 2 is South, and 4 is West
	Obstacles availableDirections;
	Location location;
	Location startLocation;
	deque<int> previousDirections;

	// Defualt constructor
	Micromouse();

	// Alternate constructor(s)
	Micromouse(const int x, const int y);

	Micromouse(const Location location);

	// Member function(s)
	int chooseOpenDirection(void); // choose the only direction that is open

	int chooseRandomDirection(); // choose a random direction of travel

	int getOppositeDirection(void); // get the opposite direction of robot

	int getOppositeDirection(const int inputDirection); // get the opposite direction of input direction

	int getTotalAvailableDirections(void); // get total available directions from robot

	bool isAtGoalLocation(void); // check if robot is at the goal location

	void moveForward(void); // move the robot forward (NOT USED DIRECTLY IN IMPLEMENTATION)

	int reverseDirection(void); // set robot direction to opposite direction of the robot direction

	int setTravelDirection(const Location &input_location); // set the travel direction based on goal location

private:
	const int MAX_DIRECTIONS    = 4;
	const int SUM_OF_DIRECTIONS = 10;
	const string TRAVEL_DIRECTION_GOAL_ERROR = "Goal location is not valid";
};

#endif


