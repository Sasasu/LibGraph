#include<iostream>
#include "graph.h"
using namespace std;

int main(){
	/*Graph<int,int>  a;
	a.addEdge(1, 2, 3);
	a.addEdge(2, 3, 1);
	a.addEdge(1, 3, 1);
	a.createAdjacencyList();
	a.createAdjacencyMatrix();
	a.printAdjacencyList();
	a.printAdjacencyMatrix();
	a.bfs(1);
	a.printLevel(1);
	map<int, int> dist = a.getShortestPathsFrom(1);
	cout<<dist[1]<<' '<<dist[2]<<' '<<dist[3]<<endl;
	cout<<a.hasCycle()<<endl;
	map<int, map<int, int> > dists = a.getAllPairShortestPath();
	for(int i=1; i<=3; i++){
		for (int j=1; j<=3; j++)
			cout<<dists[i][j]<<'\t';
		cout<<endl;
	}
	Tree<int,int >t;
	t.addEdge(1,2,100);
	t.addEdge(1,3,100);
	t.createAdjacencyList();
	t.printAdjacencyList();
	t.addEdge(2,3,100);	
	
	t.createAdjacencyList();
	t.printAdjacencyList();
	*/
	Graph<int,int>  b;
	

	b.addEdge(0,1,1);
	b.addEdge(0,2,1);
	b.addEdge(2,0,1);
	b.addEdge(1,2,1);
	b.addEdge(2,3,1);
	b.addEdge(3,3,0);
	
	b.createAdjacencyList();
	//b.printAdjacencyList();

	vector<int> x = b.dfs(0);

	for(int i=0;i<x.size();i++)
		cout<<x[i]<<" ";
	cout<<"\n";
	return 0;
}
