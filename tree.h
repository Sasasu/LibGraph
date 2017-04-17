#ifndef TREE_H
#define TREE_H

#include <stack>
#include "graph.h"

template <class T,class W>
class Tree:public Graph<T, W> {

	stack<T> topologicalSortStack;
	map<T, bool> topologicalSortVisit;
	vector<T> topologicalSortSequence;

	public:
	Tree(){};
	Tree(Tree&); 
	void addEdge(T,T,W);
	bool isValidTree(set<T>, vector< Edge<T, W > >);
	bool checkConnected(set<T>, vector< Edge<T, W > >);
	void explore(T);
	vector<T> topologicalSort();
};

/*
 * Tree class Constructor(Copy Constructor)
 * @param (Tree) Tr
 * 
 */
template <class T,class W>
Tree<T, W>::Tree(Tree &Tr) {
	Graph<T,W>::vertices   = Tr.vertices;       
	Graph<T,W>::edges      = Tr.edges;	        
}

template <class T, class W>
void Tree<T, W>::addEdge(T u, T v, W w) {
	if( ! (Graph<T,W>::edges.size()==0 || ( (Graph<T,W>::vertices.find(u)!=Graph<T,W>::vertices.end() && Graph<T,W>::vertices.find(v)==Graph<T,W>::vertices.end()) || (Graph<T,W>::vertices.find(u)==Graph<T,W>::vertices.end() && Graph<T,W>::vertices.find(v)!=Graph<T,W>::vertices.end())  ) ) )
		throw Exception("Invalid Edge");

	Graph<T,W>::edges.push_back(Edge<T,W>(u, v, w));
	Graph<T,W>::vertices.insert(u);
	Graph<T,W>::vertices.insert(v);
}


template <class T,class W>
void Tree<T,W>::explore(T source) 
{
	topologicalSortVisit[source] = true;
	vector< pair< T, W> > neighbours = Graph<T, W>::AdjList[source];
	for(int i = 0; i < neighbours.size(); i++) {
		int vertex = neighbours[i].first;
		if(topologicalSortVisit[vertex] == false)
			explore(vertex);
	}	
	topologicalSortStack.push(source);
}	


template <class T,class W>
vector<T> Tree<T,W>::topologicalSort() 
{
	//Initialisation
	topologicalSortVisit.clear();
	topologicalSortSequence.clear();

	if(Graph<T,W>::AdjList.size()==0)
		Graph<T, W>::createAdjacencyList();

	typename set< T >::iterator it;
	for(it = Graph<T, W>::vertices.begin(); it!=Graph<T, W>::vertices.end(); it++) {
		topologicalSortVisit.insert(make_pair(*it,false));	
	}

	explore( *(Graph<T, W>::vertices.begin()) );
	for(it = Graph<T, W>::vertices.begin(); it!=Graph<T, W>::vertices.end(); it++) {
		if(topologicalSortVisit[*it] == false)
			explore(*it);
	}

	while(!topologicalSortStack.empty())
	{

		topologicalSortSequence.push(topologicalSortStack.top());
		topologicalSortStack.pop();
	}	
	return topologicalSortSequence;
}

#endif