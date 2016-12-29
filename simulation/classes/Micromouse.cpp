/*
 * Implementation file for micromouse robot class
 * Author: Frederick Wachter
 */

#include <iostream>
#include <vector>

#include "Micromouse.h"
using namespace std;

// Move the robot forward (NOT USED DIRECTLY IN IMPLEMENTATION)
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
int Micromouse::setTravelDirection(const int xGoal, const int yGoal) {
	if ((location.x == xGoal) && (location.y != yGoal) {
		if (location.y > yGoal) {
			direction = 3;
		} else {
			direction = 1;
		}
	} else if ((location.y == yGoal) && (location.x != xGoal) {
		if (location.x > xGoal) {
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

// REturn the opposite direction of robot travel
int Micromouse::getOppositeDirection(void) {
	return (direction+2) % MAX_DIRECTIONS;
}

int Micromouse::getTotalAvailableDirections(shared_ptr<Node> node) {
	int sum = 0;

	for (int i = 0; i < MAX_DIRECTIONS; i++) {
		sum += node.availableDirections[i];
	}

	return sum;
}

// Get total available directions from node
int Micromouse::getTotalAvailableDirections(shared_ptr<Node> node) {
	int sum = 0;

	for (int i = 0; i < MAX_DIRECTIONS; i++) {
		sum += node.availableDirections[i];
	}

	return sum;
}

// Choose the only direction that is open
int Micromouse::chooseOpenDirection(void) {
	for (int i = 0; i < MAX_DIRECTIONS; i++) {
		if (robot.availableDirections[i] == 1) {
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


