/*
 * Header file providing class for micromouse robot
 * Author: Frederick Wachter
 */

#ifndef __MICROMOUSE__
#define __MICROMOUSE__

#include <array>
#include <memory>
#include <string>

#include "Location.h"
#include "Node.h"
using namespace std;

class Micromouse {
	public:
		int direction = 0; // 0 is North, 1 is East, 2 is South, and 4 is West
		bool *availableDirections;
		Location location(0, 0);
		Location startLocation = location;

		// Defualt constructor
		Micromouse();

		// Member function(s)
		void moveForward(void); // move the robot forward (NOT USED DIRECTLY IN IMPLEMENTATION)

		int reverseDirection(void); // set robot direction to opposite direction of the robot direction

		int setTravelDirection(const int xGoal, const int yGoal); // set the travel direction based on goal location

		int getOppositeDirection(void); // get the opposite direction of robot

		int getTotalAvailableDirections(void); // get total available directions from robot

		int getTotalAvailableDirections(shared_ptr<Node> node); // get total available directions from node

		int chooseOpenDirection(void); // choose the only direction that is open

		int chooseRandomDirection(const int input_availableDirections); // choose a random direction to move based on input open directions

	private:
		const int MAX_DIRECTIONS = 4;
		const string TRAVEL_DIRECTION_GOAL_ERROR = "Goal location is not valid";
};

#endif


