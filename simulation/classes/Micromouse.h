/*
 * Header file providing class for micromouse robot
 * Author: Frederick Wachter
 */

#ifndef __MICROMOUSE__
#define __MICROMOUSE__

#include <string>
using namespace std;

class Micromouse {
	public:
		int direction = 1;
		int location[2] = { 0, 0 };
		int openDirections[4] = { 0, 0, 0, 0 };

		// Defualt constructor
		Micromouse();

		// Helper function(s)
		void moveForward(); // move the robot forward (NOT USED DIRECTLY IN IMPLEMENTATION)

		int reverseDirection(); // set robot direction to opposite direction of the robot direction

		int setTravelDirection(const int goal[2]); // set the travel direction based on goal location

		int getTotalOpenDirections(); // get the total open directions

		int chooseRandomDirection(const int input_openDirections); // choose a random direction to move based on input open directions

	private:
		string TRAVEL_DIRECTION_GOAL_ERROR = "Goal location is not valid";
};

#endif


