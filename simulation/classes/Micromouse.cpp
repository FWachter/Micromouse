/*
 * Implementation file for micromouse robot class
 * Author: Frederick Wachter
 */

#include <iostream>
#include <vector>

#include "Micromouse.h"
using namespace std;

// Move the robot forward (NOT USED DIRECTLY IN IMPLEMENTATION)
void Micromouse::moveForward() {
	if (direction == 1) {
		location[1] += 1;
	} else if (direction == 2) {
		location[0] += 1;
	} else if (direction == 3) {
		location[1] -= 1;
	} else {
		location[0] -= 1;
	}
}

// Set robot direction to opposite direction of the robot direction
int Micromouse::reverseDirection() {
	direction = (direction+2) % 4;
	if (direction == 0) {
		direction = 4;
	}

	return direction;
}

// Get the travel direction from start to goal
int Micromouse::setTravelDirection(const int goal[2]) {
	if ((location[0] == goal[0]) && (location[1] != goal[1]) {
		if (location[1] > goal[1]) {
			direction = 3;
		} else {
			direction = 1;
		}
	} else if ((location[1] == goal[1]) && (location[0] != goal[0]) {
		if (location[0] > goal[0]) {
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

int Micromouse::getTotalOpenDirections() {
	int sum = 0;

	for (int i = 0; i < 4; i++) {
		sum += openDirections[i];
	}

	return sum;
}

// Choose a random direction to move based on input open directions
int Micromouse::chooseRandomDirection(const int input_openDirections) {
	vector<int> indexes;

	for (int i = 0; i < 4; i++) {
		if (input_openDirections[i] == 1) {
			indexes.push_back(i);
		}
	}

	direction = indexes.at(rand() % indexes.size());
	return direction;
}


