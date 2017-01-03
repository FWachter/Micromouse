/*
 * Class file for representation of an Edge in the map
 * Author: Alexander Nhan
 */

#ifndef __EDGE__
#define __EDGE__

#include "Node.h"
#include <cmath>
#include <memory>
using namespace std;

class Edge {
public:
	Edge(const shared_ptr<Node> n1, const shared_ptr<Node> n2) {
		this->node1 = n1;
		this->node2 = n2;
		weight = sqrt(pow(n2->location.x - n1->location.x, 2) + pow(n2->location.y - n1->location.y, 2)); 
	}
	shared_ptr<Node> node1;
	shared_ptr<Node> node2;
	double weight; // Euclidean distance
};

#endif
