#ifndef TREE_H
#define TREE_H

#include "graph.h"

template <class T,class W>
class Tree:public Graph<T, W> {
	public:
	Tree(){};
	Tree(Tree&); 
	void addEdge(T,T,W);
	bool isValidTree(set<T>, vector< Edge<T, W > >);
	bool checkConnected(set<T>, vector< Edge<T, W > >);
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

#endif