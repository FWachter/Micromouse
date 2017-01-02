/*
 * Implementation file for micromouse robot class
 * Author: Frederick Wachter
 */

#include <iostream>
#include <vector>

#include "Micromouse.h"
using namespace std;

// Move the robot forward (NOT USED DIRECTLY IN IMPLEMENTATION)
// Choose the only direction that is open
int Micromouse::chooseOpenDirection(void) {
	for (int i = 0; i < MAX_DIRECTIONS; i++) {
		if (robot.availableDirections.isAvailable(i)) {
			direction = i;
			break;
		}
	}

	return direction;
}

// Choose a random direction to move based on input open directions
int Micromouse::chooseRandomDirection(const int input_availableDirections) {
	vector<int> indexes;

	for (int i = 0; i < MAX_DIRECTIONS; i++) {
		if (input_availableDirections[i] == 1) {
			indexes.push_back(i);
		}
	}

	direction = indexes.at(rand() % indexes.size());
	return direction;
}

// Return the opposite direction of robot travel
int Micromouse::getOppositeDirection(void) {
	return (direction+2) % MAX_DIRECTIONS;
}

// Return the opposite direction of the input direction
int Micromouse::getOppositeDirection(const int inputDirection) {
	return (inputDirection+2) % MAX_DIRECTIONS;
}

// Get total available directions from robot
int Micromouse::getTotalAvailableDirections(void) {
	int sum = 0;
	for (int i = 0; i < MAX_DIRECTIONS; i++) {
		sum += availableDirections.isAvailable(i);
	}

	return sum;
}

// Check if the robot is at the goal location of the map
bool Micromouse::isAtGoalLocation(void) {
	previousDirections.push_front(direction);
	if (previousDirections.size() == MAX_DIRECTIONS) {
		previousDirections.pop_back();
		if (previousDirections[0] == previousDirections[1]) { // attempt to disqualify the robot being at the goal as early as possible
			return false;
		}

		int sum = 0;
		for (int i = 0; i < (MAX_DIRECTIONS-1); i++) { // sum the previous four directions
			sum += previousDirections[i];
			if (previousDirections[i] == robot.getOppositeDirection(previousDirections[i+1])) { // if the robot was backtracking
				return false;
			}
		}
		sum += previousDirections.back();
		if (sum == SUM_OF_DIRECTIONS) { // if the previous four directions are unique and were not caused back a backtrack
			return true;
		} else {
			return false;
		}
	}
}

// Move the robot forward
void Micromouse::moveForward(void) {
	switch (direction) {
		case 0: location.y += 1;
		case 1: location.x += 1;
		case 2: location.y -= 1;
		case 3: location.x -= 1;
	}
}

// Set robot direction to opposite direction of the robot direction
int Micromouse::reverseDirection(void) {
	direction = (direction+2) % MAX_DIRECTIONS;

	return direction;
}

// Get the travel direction from start to goal
int Micromouse::setTravelDirection(const Location &input_location) {
	if ((location.x == input_location.x) && (location.y != input_location.y) {
		if (location.y > input_location.y) {
			direction = 3;
		} else {
			direction = 1;
		}
	} else if ((location.y == input_location.y) && (location.x != input_location.x) {
		if (location.x > input_location.x) {
			direction = 4;
		} else {
			direction = 2;
		}
	} else {
		cout << TRAVEL_DIRECTION_GOAL_ERROR << endl;
		return -1;
	}

	return direction;
}


