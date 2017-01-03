/*
 * Script for running node mapping algorithm
 * Author: Frederick Wachter
 */

#include <deque>
#include <iostream>
#include <memory>
#include <stack>
#include <string>

#include "Map.h"
#include "Micromouse.h"
#include "Node.h"
#include "Sensor.h"
using namespace std;

// MODIFY: Initialize Map at start location
// QUESTION: Need to modify data from MATLAB due to the way the matrix is stored? I don't think so

int main() {

	// Get sensor file
	string sensorFile = "Sensor_33x33_Map1.txt";

	// Initialize objects
	Sensor sensor(sensorFile);
	Micromouse robot(sensor.initialLocation);
	Map map(robot.location.x, robot.location.y, true, sensor.initialRightWallOpen, false, false);

	// Move robot forward
	robot.direction = sensor.initialDirection;
	robot.moveForward();
	robot.availableDirections = sensor.getAvailableDirections(robot.location);
	// NOTE: line above is only meant for sim purposes

	bool runQueue    = 0;
	bool backTrack   = 0;
	bool goalFound   = 0;
	bool removeLoop  = 0;
	bool removeNodes = 1;
	deque<shared_ptr<Node>> nodeQueue;
	while (true) {

		if ((!goalFound) && (robot.isAtGoalLocation())) { // doesn't exist yet
			for (int i = 0; i < 5; i++) { // remove last five nodes from stack
				map.backTrack();
			}
			map.addNode(robot.location.x, robot.location.y, map.stackSize()+1, 0, 0, 0, true); // set current location as goal location
			map.backTrack();

			robot.setTravelDirection(map.getCurrentNode()->location);
			goalFound   = 1;
			backTrack   = 1;
			removeNodes = 0;
		}

		int totalAvailableDirections = robot.getTotalAvailableDirections();
		if (backTrack) { // if the robot is to backTrack
			cout << "Backtracking" << endl;
			shared_ptr<Node> previousNode = map.getCurrentNode();

			if (runQueue) { // if the robot is to run through the queue generated while trying to remove loops
				if (robot.location == nodeQueue.front()->location) {
					nodeQueue.pop_front();
					if (nodeQueue.empty()) {
						runQueue  = 0;
						backTrack = 0;

						robot.availableDirections = map.getCurrentNode()->availableDirections;
						if (robot.getTotalAvailableDirections() == 1) {
							robot.chooseOpenDirection();
						} else {
							robot.chooseRandomDirection();
						}
					} else {
						robot.setTravelDirection(nodeQueue.front()->location);
					}
				}
			} else if (removeLoop) { // if the robot is to try to remove loops while backtracking
				deque<shared_ptr<Node>> nodes;
				stack<shared_ptr<Node>> innerLoops;
				stack<int> innerLoopIndexes;

				int nodesInLoop = (map.stackSize()+1) - map.getCurrentNode()->stackRef;
				for (int i = 0; i < nodesInLoop; i++) {
					shared_ptr<Node> node = map.getCurrentNode();
					nodes.push_back(node);

					if ((!innerLoops.empty()) && (node->location == innerLoops.top()->location)) { // if back at an inner loop start location
						for (int j = i; j > innerLoopIndexes.top(); j--) { // remove inner loop from deque
							nodes.pop_back();
						}
						innerLoopIndexes.pop();
						innerLoops.pop();
					}

					if (node->stackRef != map.stackSize()) { // if the current location the robot is on has been added to the stack before the current instance
						innerLoopIndexes.push(i);
						innerLoops.push(node);
					} else {
						if (node->getTotalAvailableDirections() != 0) { // if there is another option to try in the potential loop
							runQueue  = 1;
							nodeQueue = nodes;
							robot.setTravelDirection(nodeQueue.front()->location);
							break;
						}
					}
					map.backTrack();
				}

				if (!runQueue) {
					robot.setTravelDirection(map.getCurrentNode()->location);
				}
				removeLoop = 0;
			} else if ((robot.location.x == previousNode->location.x) && (robot.location.y == previousNode->location.y)) { // if the robot is at the previous node location
				totalAvailableDirections = previousNode->getTotalAvailableDirections();

				if (totalAvailableDirections == 0) { // if there are no options for the robot to move
					map.backTrack();
					if (removeNodes) { // if the setting was set to remove nodes (dead end)
						if (robot.getTotalAvailableDirections() > 2) { // if the current node is not contributing a dead end
							removeNodes = 0;
						} else {
							map.removeNode(previousNode);
						}
					}

					if (robot.location == robot.startLocation) { // if the robot arrived back to the start location
						cout << "Solution has been found! Backtracking" << endl;
						break;
					}

					robot.setTravelDirection(previousNode->location);
				} else {
					backTrack = 0;

					robot.availableDirections = previousNode->availableDirections;
					if (totalAvailableDirections == 1) {
						robot.chooseOpenDirection();
					} else {
						robot.chooseRandomDirection();
					}
					previousNode->availableDirections.removeDirection(robot.direction);
				}
			}
		} else if (totalAvailableDirections == 1) { // if the robot hit a dead end
			backTrack   = 1;
			removeNodes = 1;
			robot.reverseDirection();
			cout << "Hit dead end" << endl;
		} else if ((!robot.availableDirections.isAvailable(robot.direction)) || (totalAvailableDirections > 2)) { // if the robot is not able to move forward or there are more than two options available
			bool nodeExists = map.addNode(robot.location.x, robot.location.y, map.stackSize()+1,
										  robot.availableDirections.isAvailable(0), robot.availableDirections.isAvailable(1), 
										  robot.availableDirections.isAvailable(2), robot.availableDirections.isAvailable(3), false);
			shared_ptr<Node> node = map.getCurrentNode();
			node->availableDirections.removeDirection(robot.getOppositeDirection());
			cout << "Node added" << endl;
			// NOTE: The opposite direction might already be removed in implementation for when passed to map.addNode
			
			if (nodeExists) {
				if (node->availableDirections.isAvailable(robot.direction)) { // if the robot can move forward
					node->availableDirections.removeDirection(robot.direction);
				} else {
					backTrack   = 1;
					removeLoop  = 1;
					removeNodes = 0;
					map.backTrack();
				}
			} else {
				if (!node->availableDirections.isAvailable(robot.direction)) {
					robot.availableDirections.removeDirection(robot.getOppositeDirection());
					if (node->getTotalAvailableDirections() == 1) {
						robot.chooseOpenDirection();
					} else {
						robot.chooseRandomDirection();
					}
				}
				node->availableDirections.removeDirection(robot.direction);
			}
		}

		if (!removeLoop) {
			cout << "Robot moved forward" << endl;
			robot.moveForward();
			robot.availableDirections = sensor.getAvailableDirections(robot.location);
			// NOTE: line above is only meant for sim purposes
		}

	}

}


