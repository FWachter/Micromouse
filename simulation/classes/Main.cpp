/*
 * Script for running node mapping algorithm
 * Author: Frederick Wachter
 */

#include <iostream>

#include "Map.h"
#include "Micromouse.h"
#include "Node.h"
using namespace std;

int main() {

	int wallRightOpen = checkRightWall(); // doesn't exist yet, read right sensor to determine if wall is at right of start orientation

	Micromouse robot();
	Map map(robot.location.x, robot.location.y, true, wallRightOpen, false, false);

	robot.moveForward();
	robot.getAvailableDirections(); // doesn't exist yet, only for sim purposes

	bool backtrack   = 0;
	bool removeNodes = 1;
	int totalAvailableDirections;
	while (true) {

		if (robot.isAtGoalLocation()) { // doesn't exist yet
			// ADD: remove last four elements of stack, set goal location in map
			// ADD: get travel direction based on last element of stack
		}

		totalAvailableDirections = robot.getTotalAvailableDirections();
		if (backtrack) { // if the robot is to backtrack
			shared_ptr<Node> previousNode = map.getCurrentNode();

			if (removeLoop) { // if the robot is to try to remove loops while backtracking
				// ADD: remove loop algorithm
			} else if ((robot.location.x == previousNode.x) && (robot.location.y == previousNode.y)) { // if the robot is at the previous node location
				totalAvailableDirections  = robot.getTotalAvailableDirections(previousNode);

				if (totalAvailableDirections == 0) { // if
					if (removeNodes) { // if the setting was set to remove nodes (dead end)
						if (totalAvailableDirections == 0) { // if there are no options to move to that haven't been traveled yet
							map.removeNode(previousNode);
						} else {
							removeNodes = 0;
							map.backtrack();
						}
					} else { // if the robot is pure backtracking without node removals
						map.backtrack();
					}

					if (robot.location == robot.startLocation) { // if the robot arrived back to the start location
						break;
					}

					robot.setTravelDirection(map.getCurrentNode().x, map.getCurrentNode().y);
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
		} else if ((robot.availableDirections[robot.direction] == 0) || (totalAvailableDirections > 2)) {
			bool nodeExists = map.addNode(robot.location.x, robot.location.y, robot.availableDirections[0],
						                  robot.availableDirections[1], robot.availableDirections[2], robot.availableDirections[3]);
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
			robot.getAvailableDirections(); // doesn't exist yet, only for sim purposes
		}

	}

}


