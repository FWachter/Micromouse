#include "Map.h"
#include "Node.h"
#include <iostream>
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
	m.addNode(0,10);
	m.addNode(10,10,true);
	m.backTrack();
	m.backTrack();
	m.addNode(10,10.05);
	printPath(m.bestFirstSearch());
	return 0;
}
