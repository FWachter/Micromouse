/*
 * Script for running node mapping algorithm
 * Author: Frederick Wachter
 */

#include <iostream>
#include <stack>
#include <string>

#include "Map.h"
#include "Micromouse.h"
#include "Node.h"
using namespace std;

// MODIFY: MATLAB script isAtGoalLocation function to match one in Micromouse class
// ADD: public public property in node that stores size of _vStack when node was initialized - map._vStack.size() -> map.getStackSize()
// ADD: function in Map that return size of stack
// MODIFY: the x and y property of Node should be of class Location
// ADD: funciton to Node class called totalAvailalbleDirection
// MODIFY: availableDirections of Node class should be of type Obstacles

int main() {

	int wallRightOpen = checkRightWall(); // doesn't exist yet, read right sensor to determine if wall is at right of start orientation
	string sensorFile = 'SOME NAME';

	Map map(robot.location.x, robot.location.y, true, wallRightOpen, false, false);
	Micromouse robot();
	Sensor sensor(sensorFile);

	robot.moveForward();
	robot.availableDirections = sensor.getAvailableDirections(robot.location);
	// NOTE: line above is only meant for sim purposes

	bool runQueue    = 0;
	bool backtrack   = 0;
	bool removeNodes = 1;
	deque<shared_ptr<Node>> nodeQueue;
	while (true) {

		if (robot.isAtGoalLocation()) { // doesn't exist yet
			for (int i = 0; i < 4; i++) { // remove last four nodes from stack
				map.backtrack();
			}
			map.addNode(robot.location.x, robot.location.y, 0, 0, 0, true); // set current location as goal location
			map.backtrack();

			robot.setTravelDirection(map.getCurrentNode().location);
			backtrack   = 1;
			removeNodes = 0;
		}

		int totalAvailableDirections = robot.getTotalAvailableDirections();
		if (backtrack) { // if the robot is to backtrack
			shared_ptr<Node> previousNode = map.getCurrentNode();

			if (runQueue) { // if the robot is to run through the queue generated while trying to remove loops
				if (robot.location == nodeQueue.front().location) {
					nodeQueue.pop_front();
					if (nodeQueue.empty()) {
						runQueue  = 0;
						backtrack = 0;

						if (map.getCurrentNode().getTotalAvailableDirections()) == 1) {
							robot.chooseOpenDirection();
						} else {
							robot.chooseRandomDirection();
						}
					} else {
						robot.setTravelDirection(nodeQueue.front().location);
					}
				}
			} else if (removeLoop) { // if the robot is to try to remove loops while backtracking
				deque<shared_ptr<Node>> nodes;
				stack<shared_ptr<Node>> innerLoops;
				stack<int> innerLoopIndexes;

				int nodesInLoop = (map.stackSize()+1) - map.getCurrentNode().stackRef;
				for (int i = 0; i < nodesInLoop; i++) {
					shared_ptr<Node> node = map.getCurrentNode();
					nodes.push_back(node);

					if ((!innerLoops.empty()) && (node.location == innerLoops.top().location)) { // if back at an inner loop start location
						for (int j = i; j > innerLoopIndexes.top(); j--) { // remove inner loop from deque
							nodes.pop_back();
						}
						innerLoopIndex.pop();
						innerLoops.pop();
					}

					if (node.stackRef != map.stackSize()) { // if the current location the robot is on has been added to the stack before the current instance
						innerLoopIndex.push(i);
						innerLoops.push(node);
					} else {
						if (node.getTotalAvailableDirections() != 0) { // if there is another option to try in the potential loop
							runQueue  = 1;
							nodeQueue = nodes;
							robot.setTravelDirection(nodeQueue.front().location);
							break;
						}
					}
					map.backtrack();
				}

				if (!runQueue) {
					robot.setTravelDirection(map.getCurrentNode().location);
				}
				removeLoop = 0;
			} else if ((robot.location.x == previousNode.location.x) && (robot.location.y == previousNode.location.y)) { // if the robot is at the previous node location
				totalAvailableDirections = node.getTotalAvailableDirections();

				if (totalAvailableDirections == 0) { // if there are no options for the robot to move
					map.backtrack();
					if (removeNodes) { // if the setting was set to remove nodes (dead end)
						if (robot.getTotalAvailableDirections() > 2) { // if the current node is not contributing a dead end
							removeNodes = 0;
						} else {
							map.removeNode(previousNode);
						}
					}

					if (robot.location == robot.startLocation) { // if the robot arrived back to the start location
						break;
					}

					robot.setTravelDirection(previousNode.location);
				} else {
					backtrack = 0;

					if (totalAvailableDirections == 1) {
						robot.chooseOpenDirection();
					} else {
						robot.chooseRandomDirection();
					}
					previousNode.availableDirections.isAvailable(robot.direction = 0;
				}
			}
		} else if (totalAvailableDirections == 1) { // if the robot hit a dead end
			backtrack   = 1;
			removeNodes = 1;
			robot.reverseDirection();
		} else if ((!robot.availableDirections.isAvailable(robot.direction)) || (totalAvailableDirections > 2)) { // if the robot is not able to move forward or there are more than two options available
			bool nodeExists = map.addNode(robot.location.x, robot.location.y, robot.availableDirections.isAvailable(0),
						                  robot.availableDirections.isAvailable(1), robot.availableDirections.isAvailable(2), 
						                  robot.availableDirections.isAvailable(3), map.stackSize()+1, false);
			node = map.getCurrentNode();
			node.availableDirections.isAvailable(robot.getOppositeDirection()) = 0;
			// NOTE: The opposite direction might already be removed in implementation for when passed to map.addNode
			
			if (nodeExists) {
				if (node.availableDirections.isAvailable(robot.direction)) { // if the robot can move forward
					node.availableDirections.isAvailable(robot.direction) = 0;
				} else {
					backtrack   = 1;
					removeLoop  = 1;
					removeNodes = 0;
					map.backtrack();
				}
			} else {
				if (!node.availableDirections.isAvailable(robot.direction)) {
					if (node.getTotalAvailableDirections() == 1) {
						robot.chooseOpenDirection();
					} else {
						robot.chooseRandomDirection();
					}
				}
				node.availableDirections.isAvailable(robot.direction) = 0;
			}
		}

		if (!removeLoop) {
			robot.moveForward();
			robot.getAvailableDirections();
			// NOTE: line above is only meant for sim purposes
		}

	}

}


