/*
 * Implementation file for micromouse robot class
 * Author: Frederick Wachter
 */

#include <iostream>
#include <vector>

#include "Micromouse.h"
#include "Utilities.h"
using namespace std;

// Default construtor
Micromouse::Micromouse() {
	location.setLocation(0, 0);
	startLocation = location;
}

// Alternate constructor supplying x and y location
Micromouse::Micromouse(const int &x, const int &y) {
	location.setLocation(x, y);
	startLocation = location;
}

// Alternate constructor supplying Location object
Micromouse::Micromouse(const Location &location) {
	this->location = location;
	this->startLocation = location;
}

// Alternate constructor supplying Location and Directions objects
Micromouse::Micromouse(const Location &location, const int &direction, const Directions &directions) {
	this->location  = location;
	this->direction = direction;
	this->availableDirections = directions;
	this->startLocation = location;

	displayRobotState(location, direction, directions);
}

// Choose the only direction that is open
int Micromouse::chooseOpenDirection(void) {
	for (int i = 0; i < MAX_DIRECTIONS; i++) {
		if (availableDirections.isAvailable(i)) {
			direction = i;
			break;
		}
	}

	return direction;
}

// Choose a random direction of travel
int Micromouse::chooseRandomDirection(void) {
	vector<int> indexes;
	for (int i = 0; i < MAX_DIRECTIONS; i++) {
		if (availableDirections.isAvailable(i) == 1) {
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
int Micromouse::getOppositeDirection(const int &inputDirection) {
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
		if (previousDirections[0] == previousDirections[1]) { // attempt to disqualify the robot being at the goal as early as possible	
			previousDirections.pop_back();
			return false;
		}

		if (isPreviousDirectionsUnique()) { // if all previous directions are unique
			int sum = 0;
			for (int i = 0; i < (MAX_DIRECTIONS-1); i++) { // sum the previous four directions
				sum += previousDirections[i];
				if (previousDirections[i] == getOppositeDirection(previousDirections[i+1])) { // if the robot was backtracking
					previousDirections.pop_back();
					return false;
				}
			}
			sum += previousDirections.back();
			if (sum == SUM_OF_DIRECTIONS) { // if the previous four directions are unique and were not caused back a backtrack
				return true;
			}
		}

		previousDirections.pop_back();
	}

	return false;
}

// Check if the previous directions of the robot are all unique
bool Micromouse::isPreviousDirectionsUnique(void) {
	for (int i = 0; i < (MAX_DIRECTIONS-1); i++) {
		for (int j = i+1; j < MAX_DIRECTIONS; j++) {
			if (previousDirections[i] == previousDirections[j]) {
				return false;
			}
		}
	}

	return true;
}

// Move the robot forward
void Micromouse::moveForward(void) {
	totalMovements++;
	switch (direction) {
		case 0: location.y += 1; break;
		case 1: location.x += 1; break;
		case 2: location.y -= 1; break;
		case 3: location.x -= 1; break;
	}

	displayRobotMovement();
}

// Set robot direction to opposite direction of the robot direction
int Micromouse::reverseDirection(void) {
	direction = (direction+2) % MAX_DIRECTIONS;

	return direction;
}

// Get the travel direction from start to goal
int Micromouse::setTravelDirection(const Location &input_location) {
	if ((location.x == input_location.x) && (location.y != input_location.y)) {
		if (location.y > input_location.y) {
			direction = 2;
		} else {
			direction = 0;
		}
	} else if ((location.y == input_location.y) && (location.x != input_location.x)) {
		if (location.x > input_location.x) {
			direction = 3;
		} else {
			direction = 1;
		}
	} else {
		cout << TRAVEL_DIRECTION_GOAL_ERROR << endl;
		return -1;
	}

	return direction;
}


