/*
 * Implementation file for the Map class
 * Author: Alexander Nhan
 */

#include "Map.h"
#include "Edges.h"
#include "Node.h"
#include <memory>
#include <float.h>
#include <utility>
#include <vector>
#include <map>
#include <stack>
#include <algorithm>
#include <cmath>
using namespace std;

// Constructor
Map::Map() {
	addNode(0,0,o);
	_startNode = getNode(0,0);
}

// Constructor to intialize (x,y) start
Map::Map(const double x, const double y, const TURNS t) {
	// initialize start node as the cartesian point (x, y)
	addNode(x,y,t);
	_startNode = getNode(x,y);
}

// Backtrack by popping off the top of the stack
void Map::backTrack(void) {
	_vStack.pop();
}

// Returns pointer to the current Node at top of stack
shared_ptr<Node> Map::getCurrentNode() const {
	return _vStack.top();
}

// Returns pointer to Node referred to by (x,y) poisiton
shared_ptr<Node> Map::getNode(const double x, const double y) {
	if(_vt.count(_hash(x,y)) == 0) {
		return nullptr;
	}
	for(auto& v: _vt.at(_hash(x,y))) {
		if(SAME_NODE(v->x, v->y, x, y)) {
			return v;
		}
	}
	return nullptr; // node was not in map
}

// Add a node to the map. Checks for duplicate points
void Map::addNode(const double x, const double y, const TURNS t, 
	const bool goal) {
	// vector didn't exist at this hash value
	if(_vt.count(_hash(x,y)) == 0) {
		_vt.insert(pair<unsigned int, vector<shared_ptr<Node> > >(
			_hash(x,y), vector<shared_ptr<Node> >()));
		shared_ptr<Node> newNode = make_shared<Node>(x,y,t);
		_vt.at(_hash(x,y)).push_back(newNode);
		_vertices.push_back(newNode);
		addNode(newNode, goal);
	}
	else {
		auto& vec = _vt.at(_hash(x,y));
		auto cmp = [x,y](shared_ptr<Node>& n)->bool {
			return SAME_NODE(n->x, n->y, x, y);
		};
		// try to find the Node in the vector
		auto it = find_if(vec.begin(), vec.end(), cmp);
		if(it == vec.end()) {
			// consider the point a new node
			shared_ptr<Node> newNode = make_shared<Node>(x,y,t);
			vec.push_back(newNode);
			_vertices.push_back(newNode);
			addNode(newNode, goal);
		}
		else { // Node already existed
			addNode(*it, goal);
		}
	}
}

// Helper implementation. Parent of Node is always at top of stack
void Map::addNode(const shared_ptr<Node>& n, const bool goal) {
	if(!_vStack.empty()) {
		_edges.addEdge(_vStack.top(), n);
	}
	_vStack.push(n);
	if(goal) {
		_goalNode = n;
	}
}

// Dead end means that the outgoing paths from the Node specified by the
// (x,y) point does not contribute to the goal, therefore we can remove
// those paths to make the goal search faster.
void Map::deadEnd(const double x, const double y) {
	shared_ptr<Node> n = getNode(x,y);
	deadEnd(n);
}

void Map::deadEnd(shared_ptr<Node>& n) {
	;
}

// Finds the best path from start node to goal node and returns the path
stack<Node> Map::bestFirstSearch() const {
	stack<Node> ret;
	// Check if a goal node exists
	if(!_goalNode) {
		return ret;
	}
	// Initialize distances from start node and parent nodes
	map<shared_ptr<Node>, double> dist;
	map<shared_ptr<Node>, shared_ptr<Node> > parent;
	map<shared_ptr<Node>, bool> closedList;
	for(unsigned int i=0; i<_vertices.size(); i++) {
		dist[_vertices[i]] = DBL_MAX;
		parent[_vertices[i]] = nullptr;
		closedList[_vertices[i]] = false;
	}
	dist[_startNode] = 0;
	// Lambda for extracting min, each extract cost O(n) however, the map
	// size should be small enough to not worry about it
	shared_ptr<Node> v;
	auto extract_min = [&closedList]
		(map<shared_ptr<Node>, double>& m)
		-> shared_ptr<Node> {
			double minVal = DBL_MAX;
			shared_ptr<Node> minNode;
			for(auto it=m.begin(); it != m.end(); it++) {
				if(!closedList[it->first] and it->second < minVal) {
					minNode = it->first;
					minVal = it->second;
				}
			}
			return minNode;
		};
	while((v=extract_min(dist)).get() != nullptr) {
		for(auto w: _edges.getAdjacents(v)) {
			double edge_vw = _edges.getWeight(v, w);
			if(dist[v] + edge_vw < dist[w]) {
				dist[w] = dist[v] + edge_vw;
				parent[w] = v;
			}
		}
		closedList[v] = true;
	}
	// Extract path to goal node
	v = _goalNode;
	ret.push(*v);
	while((v=parent[v]) != nullptr) {
		ret.push(*v);
	}
	return ret;
}
