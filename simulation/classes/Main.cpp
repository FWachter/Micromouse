/*
 * Script for running node mapping algorithm
 * Author: Frederick Wachter
 */

#include <iostream>

#include "Micromouse.h"
using namespace std;

int main() {

	int wallRightOpen = checkRightWall(); // doesn't exist yet, read right sensor to determine if wall is at original directions

	Micromouse robot();
	Map map(robot.location[0], robot.location[1], true, wallRightOpen, false, false);

	robot.moveForward();
	robot.getOpenDirections(); // doesn't exist yet, only for sim purposes

	while (true) {

		int totalOpenDirections = robot.getTotalOpenDirections();

		if (robot.isAtGoalLocation()) { // doesn't exist yet
			// remove last four elements of stack, set goal location in map
			// get travel direction based on last element of stack
		}

		if (backtrack) {

		} else if (totalOpenDirections == 1) {

		} else if ((robot.openDirections[robot.direction] == 0) || (totalOpenDirections > 2)) {

		}

		robot.moveForward();
		robot.getOpenDirections(); // doesn't exist yet, only for sim purposes

	}

}