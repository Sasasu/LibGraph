#include<iostream>
#include "graph.h"
using namespace std;

int main(){
	Graph<int,int>  a;
	a.addEdge(1, 2, 3);
	a.addEdge(2, 3, 1);
	a.addEdge(1, 3, 1);
	a.createAdjacencyList();
	a.createAdjacencyMatrix();
	a.printAdjacencyList();
	a.printAdjacencyMatrix();
	a.bfs(1);
	a.printLevel(1);
	map<int, int> dist = a.shortestPathFrom(1);
	cout<<dist[1]<<' '<<dist[2]<<' '<<dist[3]<<endl;
	cout<<a.hasCycle()<<endl;
}
