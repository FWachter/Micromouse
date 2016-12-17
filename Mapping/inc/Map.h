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
#include <cmath>
using namespace std;


#define MAP_RES 1
#define SAME_NODE(x0, y0, x1, y1) (abs(x0-x1) < MAP_RES \
	and abs(y0-y1) < MAP_RES)

class Map {
public:
	// Default Constructor
	Map();

	// Constructor to initialize x,y start
	Map(const double x, const double y, const DIRECTION d, const TURNS t);

	// Adds a node, specified by (x, y) point to the map
	void addNode(const double x, const double y, DIRECTION d,
		const TURNS t, const bool goal=0);
	
	// Backtrack by popping off the stack
	void backTrack(void);

	// Notifies the Map that the current node results in a dead end
	// This will cause the nodes reachable from this point to be deleted
	void removeNode(const double x, const double y);
	void removeNode(shared_ptr<Node>& n);

	// Returns pointer to current Node so that user can alter turns
	shared_ptr<Node> getCurrentNode() const;

	// Returns pointer to Node referred to by (x,y) position
	shared_ptr<Node> getNode(const double x, const double y);

	// Finds the best path from start node to the goal node
	stack<Node> bestFirstSearch() const;

private:
	vector<shared_ptr<Node> > _vertices; // all vertices created so far
	stack<shared_ptr<Node> > _vStack; // order in which vertices visited
	map<unsigned int, vector<shared_ptr<Node> > > _vt; // duplicate vertex tracker 
	shared_ptr<Node> _goalNode;
	shared_ptr<Node> _startNode;
	Edges _edges;

	// Helper methods
	void addNode(const shared_ptr<Node>& n, const bool goal);
	// computes hash for _vt
	unsigned int _hash(const double x, const double y) { 
		return floor(sqrt(pow(x,2)+pow(y,2)));
	}
};

#endif
