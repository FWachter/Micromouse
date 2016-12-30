/*
 * Script for running node mapping algorithm
 * Author: Frederick Wachter
 */

#include <iostream>
#include <stack>

#include "Map.h"
#include "Micromouse.h"
#include "Node.h"
using namespace std;

// MODIFY: MATLAB script isAtGoalLocation function to match one in Micromouse class
// ADD: public public property in node that stores size of _vStack when node was initialized - map._vStack.size() -> map.getStackSize()
// MODIFY: the x and y property of Node should be of class Location

int main() {

	int wallRightOpen = checkRightWall(); // doesn't exist yet, read right sensor to determine if wall is at right of start orientation

	Micromouse robot();
	Map map(robot.location.x, robot.location.y, true, wallRightOpen, false, false);

	robot.moveForward();
	robot.getAvailableDirections(); // doesn't exist yet, only for sim purposes

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

			robot.setTravelDirection(map.getCurrentNode().x, map.getCurrentNode().y);
			backtrack   = 1;
			removeNodes = 0;
		}

		int totalAvailableDirections = robot.getTotalAvailableDirections();
		if (backtrack) { // if the robot is to backtrack
			shared_ptr<Node> previousNode = map.getCurrentNode();

			if (runQueue) { // if the robot is to run through the queue generated while trying to remove loops
				Location location(nodeQueue.front().x, nodeQueue.front().y);
				if (robot.location == location) {
					nodeQueue.pop_front();
					if (nodeQueue.empty()) {
						runQueue  = 0;
						backtrack = 0;

						if (robot.getTotalAvailableDirections(map.getCurrentNode()) == 1) {
							robot.chooseOpenDirection();
						} else {
							robot.chooseRandomDirection();
						}
					} else {
						robot.setTravelDirection(nodeQueue.front().x, nodeQueue.front().y);
					}
				}
			} else if (removeLoop) { // if the robot is to try to remove loops while backtracking
				deque<shared_ptr<Node>> nodes;
				stack<shared_ptr<Node>> innerLoops;
				stack<int> innerLoopIndexes;

				int nodesInLoop = (map._vStack.size()+1) - map.getCurrentNode().stackRef;
				for (int i = 0; i < nodesInLoop; i++) {
					shared_ptr<Node> node = map.getCurrentNode();
					nodes.push_back(node);

					if ((!innerLoops.empty()) && (node.location == innerLoops.top().location)) { // if back at an inner loop start location
						for (int j = i; j > innerLoopIndex.top(); j--) { // remove inner loop from deque
							nodes.pop_back();
						}
						innerLoopIndex.pop();
						innerLoops.pop();
					}

					if (node.stackRef != map._vStack.size()) { // if the current location the robot is on has been added to the stack before the current instance
						innerLoopIndex.push(i);
						innerLoops.push(node);
					} else {
						if (robot.getTotalAvailableDirections(node) != 0) { // if there is another option to try in the potential loop
							runQueue  = 1;
							nodeQueue = nodes;
							robot.setTravelDirection(nodeQueue.front().x, nodeQueue.front().y);
							break;
						}
					}
				}

				if (!runQueue) {
					robot.setTravelDirection(map.getCurrentNode().x, map.getCurrentNode().y);
				}
				removeLoop = 0;
			} else if ((robot.location.x == previousNode.x) && (robot.location.y == previousNode.y)) { // if the robot is at the previous node location
				totalAvailableDirections = robot.getTotalAvailableDirections(previousNode);

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

					robot.setTravelDirection(previousNode.x, previousNode.y);
				} else {
					backtrack = 0;

					if (totalAvailableDirections == 1) {
						robot.chooseOpenDirection();
					} else {
						robot.chooseRandomDirection();
					}
					previousNode.availableDirections[robot.direction] = 0;
				}
			}
		} else if (totalAvailableDirections == 1) { // if the robot hit a dead end
			backtrack   = 1;
			removeNodes = 1;
			robot.reverseDirection();
		} else if ((robot.availableDirections[robot.direction] == 0) || (totalAvailableDirections > 2)) { // if the robot is not able to move forward or there are more than two options available
			bool nodeExists = map.addNode(robot.location.x, robot.location.y, robot.availableDirections[0],
						                  robot.availableDirections[1], robot.availableDirections[2], robot.availableDirections[3], false);
			node = map.getCurrentNode();
			node.availableDirections[robot.getOppositeDirection()] = 0;
			// NOTE: The opposite direction might already be removed in implementation for when passed to map.addNode
			
			if (nodeExists) {
				if (node.availableDirections[robot.direction]) { // if the robot can move forward
					node.availableDirections[robot.direction] = 0;
				} else {
					backtrack   = 1;
					removeLoop  = 1;
					removeNodes = 0;
					map.backtrack();
				}
			} else {
				if (node.availableDirections[robot.direction] == 0) {
					if (robot.getTotalAvailableDirections(node) == 1) {
						robot.chooseOpenDirection();
					} else {
						robot.chooseRandomDirection();
					}
				}
				node.availableDirections(robot.direction) = 0;
			}
		}

		if (!removeLoop) {
			robot.moveForward();
			robot.getAvailableDirections(); // doesn't exist yet
			// NOTE: line above is only meant for sim purposes
		}

	}

}


