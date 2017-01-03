/*
 * Implementation file providing functions for display information
 * Author: Frederick Wachter
 */

#include "Utilities.h"
using namespace std;

static bool debug = true;

void displayNodeInformation(const int &x, const int &y, const int &stackRef,
	const bool &north, const bool &east, const bool &south, const bool &west) {
	if (debug) {
		cout << "__________ NODE INFORMATION __________" << endl;
		cout << "Location: [" << x << " " << y << "]" << endl;
		cout << "Stack Reference: " << stackRef << endl;
		cout << "Available Directions: [" << north << " " << east << " " << south
			<< " " << west << "]" << endl;
	}
}

void displayNodeInformation(const Location &location, const int &stackRef, 
	const Obstacles &obstacles) {
	if (debug) {
		cout << "__________ NODE INFORMATION __________" << endl;
		cout << "Location: [" << location.x << " " << location.y << "]" << endl;
		cout << "Stack Reference: " << stackRef << endl;
		cout << "Available Directions: [" << *obstacles.directions[1] << " " << 
			*obstacles.directions[2] << " " << *obstacles.directions[3] << " " << 
			*obstacles.directions[3] << "]" << endl;
	}
}

void displayNodeInformation(shared_ptr<Node> &n) {
	if (debug) {
		cout << "__________ NODE INFORMATION __________" << endl;
		cout << "Location: [" << n->location.x << " " << n->location.y << "]" << endl;
		cout << "Stack Reference: " << n->stackRef << endl;
		cout << "Available Directions: [" << *n->availableDirections.directions[0] << " " 
			<< *n->availableDirections.directions[1] << " " 
			<< *n->availableDirections.directions[2] << " " 
			<< *n->availableDirections.directions[3] << "]" << endl;
	}
}

void displayRobotLocation(const Location &location, const int &direction) {
	if (debug) {
		cout << "__________ ROBOT INFORMATION __________" << endl;
		cout << "Location: [" << location.x << " " << location.y << "]" << endl;
		cout << "Direction: " << direction << endl;
	}
}

void displayRobotState(const Location &location, const int &direction, 
	const Obstacles &obstacles) {
	if (debug) {
		cout << "__________ ROBOT STATE __________" << endl;
		cout << "Location: [" << location.x << " " << location.y << "]" << endl;
		cout << "Direction: " << direction << endl;
		cout << "Available Directions: [" << *obstacles.directions[0] << " " << 
			*obstacles.directions[1] << " " << *obstacles.directions[2] << " " << 
			*obstacles.directions[3] << "]" << endl;
	}
}

void displayRobotState(const Micromouse &robot) {
	if (debug) {
		cout << "__________ ROBOT STATE __________" << endl;
		cout << "Location: [" << robot.location.x << " " << robot.location.y << "]" << endl;
		cout << "Direction: " << robot.direction << endl;
		cout << "Available Directions: [" << *robot.availableDirections.directions[0] << " " 
			<< *robot.availableDirections.directions[1] << " " << 
			*robot.availableDirections.directions[2] << " " << 
			*robot.availableDirections.directions[3] << "]" << endl;
	}
}

void displaySensorFileData(const int &x, const int &y, const bool &north,
	const bool &east, const bool &south, const bool &west) {
	if (debug) {
		cout << "Sensor File Line: [" << x << " " << y << "] [" << north << " " << east <<
			" " << south << " " << west << "]" << endl;
	}
}