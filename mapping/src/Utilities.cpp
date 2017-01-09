/*
 * Implementation file providing functions for display information
 * Author: Frederick Wachter
 */

#include "Utilities.h"
using namespace std;

static bool debug = false;

void displayNodeInformation(const int &x, const int &y, const int &stackRef,
	const bool &north, const bool &east, const bool &south, const bool &west) {
	if (debug) {
		cout << "[ACTION] Node added" << endl;
		cout << "__________ NODE INFORMATION __________" << endl;
		cout << "Location: [" << x << " " << y << "]" << endl;
		cout << "Stack Reference: " << stackRef << endl;
		cout << "Available Directions: [" << north << " " << east << " " << south
			<< " " << west << "]" << endl << endl;
	}
}

void displayNodeInformation(const Location &location, const int &stackRef, 
	const Obstacles &obstacles) {
	if (debug) {
		cout << "[ACTION] Node added" << endl;
		cout << "__________ NODE INFORMATION __________" << endl;
		cout << "Location: [" << location.x << " " << location.y << "]" << endl;
		cout << "Stack Reference: " << stackRef << endl;
		cout << "Available Directions: [" << obstacles.isAvailable(0) << " " << 
			obstacles.isAvailable(1) << " " << obstacles.isAvailable(2) << " " << 
			obstacles.isAvailable(3) << "]" << endl << endl;
	}
}

void displayNodeInformation(shared_ptr<Node> &n) {
	if (debug) {
		cout << "[ACTION] Node added" << endl;
		cout << "__________ NODE INFORMATION __________" << endl;
		cout << "Location: [" << n->location.x << " " << n->location.y << "]" << endl;
		cout << "Stack Reference: " << n->stackRef << endl;
		cout << "Available Directions: [" << n->availableDirections.isAvailable(0) << " " 
			<< n->availableDirections.isAvailable(1) << " " 
			<< n->availableDirections.isAvailable(2) << " " 
			<< n->availableDirections.isAvailable(3) << "]" << endl << endl;
	}
}

void displayRobotLocation(const Location &location, const int &direction) {
	if (debug) {
		cout << "__________ ROBOT LOCATION __________" << endl;
		cout << "Location: [" << location.x << " " << location.y << "]" << endl;
		cout << "Direction: " << direction << endl << endl;
	}
}

void displayRobotState(const Location &location, const int &direction, 
	const Obstacles &obstacles) {
	if (debug) {
		cout << "__________ ROBOT STATE __________" << endl;
		cout << "Location: [" << location.x << " " << location.y << "]" << endl;
		cout << "Direction: " << direction << endl;
		cout << "Available Directions: [" << obstacles.isAvailable(0) << " " << 
			obstacles.isAvailable(1) << " " << obstacles.isAvailable(2) << " " << 
			obstacles.isAvailable(3) << "]" << endl << endl;
	}
}

void displayRobotState(const Micromouse &robot) {
	if (debug) {
		cout << "__________ ROBOT STATE __________" << endl;
		cout << "Location: [" << robot.location.x << " " << robot.location.y << "]" << endl;
		cout << "Direction: " << robot.direction << endl;
		cout << "Available Directions: [" << robot.availableDirections.isAvailable(0) << " " 
			<< robot.availableDirections.isAvailable(1) << " " << 
			robot.availableDirections.isAvailable(2) << " " << 
			robot.availableDirections.isAvailable(3) << "]" << endl << endl;
	}
}

void displaySensorFileStart(void) {
	if (debug) {
		cout << "[INFO] Starting to read from sensor file" << endl;
		cout << "__________ SENSOR FILE CONTENTS __________" << endl;
	}
}

void displaySensorFileEnd(void) {
	if (debug) {
		cout << "[INFO] End of sensor file" << endl << endl;
	}
}

void displaySensorFileData(const int &x, const int &y, const bool &north,
	const bool &east, const bool &south, const bool &west) {
	if (debug) {
		cout << "Sensor File Line: [" << x << " " << y << "] [" << north << " " << east <<
			" " << south << " " << west << "]" << endl;
	}
}

void displayRobotMovement(void) {
	if (debug) {
		cout << "[ACTION] Robot moved forward" << endl;
	}
}

void displayBacktrackRemoveLoops(const int &nodesInLoop) {
	if (debug) {
		cout << "[INFO] Backtracking, attempting to remove loops (" << nodesInLoop << 
			" nodes in loop)" << endl;
	}
}

void displayBacktrackQueue(void) {
	if (debug) {
		cout << "[INFO] Backtracking through queue" << endl;
	}
}

void displayBacktrackPreviousNode(void) {
	if (debug) {
		cout << "[INFO] Backtracking to previous node in stack" << endl;
	}
}

void displayBacktrack(void) {
	if (debug) {
		cout << "[INFO] Backtracking" << endl;
	}
}

void displayDeadEnd(void) {
	if (debug) {
		cout << "[INFO] Hit a dead end" << endl;
	}
}

void displayGoalFound(void) {
	cout << "[INFO] Goal position found, backtracking to remaining open spaces" << endl;
}

void displayAlgorithmFinished(const bool solutionFound, const int &totalMovements) {
	if (solutionFound) {
		cout << "[INFO] Algorithm finished! Took " << totalMovements << " movements" << endl;
	} else {
		cout << "[INFO] Algorithm finished, but the solution was not found" << endl;
	}
}

void displayBeginningAlgorithm(void) {
	if (debug) {
		cout << "[INFO] Beginning algorithm" << endl << endl;
	}
}


