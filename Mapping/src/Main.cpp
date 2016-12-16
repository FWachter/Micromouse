#include "Map.h"
#include "Node.h"
#include <iostream>
#include <stack>
#include <vector>
#include <memory>
using namespace std;

void printPath(stack<Node> path) {
	while(!path.empty()) {
		cout << "(" << path.top().x << ", " << path.top().y << ")" << endl;
		path.pop();
	}
}

int main() {
	Map m(0,0,r);
	m.addNode(0,10,r);
	m.addNode(10,10,r,true);
	m.addNode(0,0.05,o);
	m.deadEnd(0,10);
	printPath(m.bestFirstSearch());
	shared_ptr<Node> curNode = m.getCurrentNode();
	cout << "CurNode: (" << curNode->x << ", " << curNode->y << ")" << endl;
	return 0;
}
