/*
 * Header file providing class for mapping phase
 * Author: Alexander Nhan
 */

#ifndef __MAP__
#define __MAP__

#include "Edges.h"
#include "Node.h"
#include <vector>
#include <stack>
#include <memory>
#include <map>
using namespace std;

class Map {
public:
	// Default Constructor
	Map();

	// Constructor to initialize x,y start
	Map(const double x, const double y);

	// Creates a node as a shared_ptr
	shared_ptr<Node> createNode(const double x0, const double y0);

	// Adds a node to the map
	void addNode(const shared_ptr<Node>& n, const bool goal=0);
	
	// Backtrack by popping off the stack
	void backTrack(void);

	// Returns the current Node
	shared_ptr<Node> getCurrentNode() const;

	// Finds the best path from start node to the goal node
	stack<Node> bestFirstSearch() const;

private:
	vector<shared_ptr<Node> > _vertices; // all vertices created so far
	stack<shared_ptr<Node> > _vStack; // order in which vertices visited
	map<shared_ptr<Node>, bool> _verticesTracker;
	shared_ptr<Node> _goalNode;
	shared_ptr<Node> _startNode;
	Edges _edges;
};

#endif
