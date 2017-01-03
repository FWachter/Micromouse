/*
 * Header file providing functions for display information
 * Author: Frederick Wachter
 */

#ifndef __UTILITIES__
#define __UTILITIES__

#include <iostream>
#include <memory>

#include "Location.h"
#include "Micromouse.h"
#include "Node.h"
#include "Obstacles.h"

void displayNodeInformation(const int &x, const int &y, const int &stackRef,
	const bool &north, const bool &east, const bool &south, const bool &west);

void displayNodeInformation(const Location &location, const int &stackRef, 
	const Obstacles &obstacles);

void displayNodeInformation(shared_ptr<Node> &n);

void displayRobotLocation(const Location &location, const int &direction);

void displayRobotState(const Location &location, const int &direction, 
	const Obstacles &obstacles);

void displayRobotState(const Micromouse &robot);

void displaySensorFileStart(void);

void displaySensorFileEnd(void);

void displaySensorFileData(const int &x, const int &y, const bool &north,
	const bool &east, const bool &south, const bool &west);

void displayRobotMovement(void);

void displayBacktrackRemoveLoops(const int &nodesInLoop);

void displayBacktrackQueue(void);

void displayBacktrackPreviousNode(void);

void displayBacktrack(void);

void displayDeadEnd(void);

void displaySolutionFound(const int &totalMovements);

void displayBeginningAlgorithm(void);

#endif


