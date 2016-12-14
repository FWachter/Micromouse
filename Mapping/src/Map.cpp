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
using namespace std;

// Constructor
Map::Map() {
	_startNode = make_shared<Node>(0, 0);
	_vertices.push_back(_startNode);
	_vStack.push(_startNode);
}

// Constructor to intialize (x,y) start
Map::Map(double x, double y) {
	// initialize start node as the cartesian point (x, y)
	_startNode = make_shared<Node>(x, y);
	_vertices.push_back(_startNode);
	_vStack.push(_startNode);
}

// Backtrack by popping off the top of the stack
void Map::backTrack(void) {
	_vStack.pop();
}

// Returns the current Node
shared_ptr<Node> Map::getCurrentNode() const {
	return _vStack.top();
}

// Creates a node as a shared_ptr
shared_ptr<Node> Map::createNode(const double x0, const double y0) {
	return make_shared<Node>(x0, y0);
}

// Add a node to the map, the parent of this node is always the top of the
// stack
void Map::addNode(const shared_ptr<Node>& n, const bool goal) {
	if(_verticesTracker.count(n) == 0) {
		_vertices.push_back(n);
		_verticesTracker.insert(pair<shared_ptr<Node>, bool>(n, true));
	}
	_edges.addEdge(_vStack.top(), n);
	_vStack.push(n);
	if(goal) {
		_goalNode = n;
	}
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
