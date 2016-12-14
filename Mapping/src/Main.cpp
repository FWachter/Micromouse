#include "Map.h"
#include "Node.h"
#include <iostream>
#include <memory>
#include <stack>
#include <vector>
using namespace std;

void printPath(stack<Node> path) {
	while(!path.empty()) {
		cout << "(" << path.top().x << ", " << path.top().y << ")" << endl;
		path.pop();
	}
}

int main() {
	Map m;
	vector<shared_ptr<Node> > nodes;
	nodes.push_back(m.createNode(0,0));
	nodes.push_back(m.createNode(0,10));
	nodes.push_back(m.createNode(10,10));
	m.addNode(nodes[1]);
	m.addNode(nodes[2],true);
	m.backTrack();
	m.backTrack();
	m.addNode(nodes[2],true);
	printPath(m.bestFirstSearch());
	return 0;
}
