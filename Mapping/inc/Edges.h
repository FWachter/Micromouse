/*
 * Class file to support edges in a graph
 * Author: Alexander Nhan
 */
#ifndef _EDGES_
#define _EDGES_

#include "Edge.h"
#include "Node.h"
#include <map>
#include <vector>
#include <memory>
using namespace std;

class Edges {
public:
	// Default constructor
	Edges() {;};

	// Accessor methods
	size_t numEdges(const shared_ptr<Node>& n) const {
		if(_edges.count(n) > 0)
			return _edges.at(n).size();
		else
			return 0;
	};

	double getWeight(const shared_ptr<Node>& n1, 
		const shared_ptr<Node>& n2) const {
		if(_edges.count(n1) > 0) {
			for(auto &e: _edges.at(n1)) {
				if(e->node2 == n2) {
					return e->weight;
				}
			}
		}
		return -1;
	};

	vector<shared_ptr<Node> > getAdjacents(const shared_ptr<Node>& n) 
		const {
		vector<shared_ptr<Node> > ret;
		if(_edges.count(n) > 0) {
			for(auto &e: _edges.at(n)) {
				ret.push_back(e->node2);
			}
		}
		return ret;
	};

	// Mutator methods
	void addEdge(const shared_ptr<Node>& n1, const shared_ptr<Node>& n2)
	{
		// Edges are undirected
		if(_edges.count(n1) == 0) {
			_edges.insert(
				pair<shared_ptr<Node>, vector<unique_ptr<Edge> > >
				(n1, 
				vector<unique_ptr<Edge> >()));		
		}
		_edges.at(n1).push_back(make_unique<Edge>(n1,n2));
		if(_edges.count(n2) == 0) {
			_edges.insert(
				pair<shared_ptr<Node>, vector<unique_ptr<Edge> > >
				(n2, 
				vector<unique_ptr<Edge> >()));		
		}
		_edges.at(n2).push_back(make_unique<Edge>(n2,n1));
	};

	// removes all edges outgoing and ingoing of Node n
	void removeEdges(const shared_ptr<Node>& n) {
		// remove outgoing edges
		auto it = _edges.find(n);
		if(it != _edges.end()) {
			_edges.erase(it);
		}
		// find all edges that are ingoing to n and remove them
		for(auto& p: _edges) {
			for(auto e = p.second.begin(); e != p.second.end(); e++) {
				if((*e)->node2 == n) {
					e = p.second.erase(e);
					break;
				}
			}
		}
	};

private:
	map<shared_ptr<Node>, vector<unique_ptr<Edge> > > _edges;
};

#endif
