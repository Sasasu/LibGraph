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
	Graph<T,W>::edges.push_back(Edge<T,W>(u, v, w));
	Graph<T,W>::vertices.insert(u);
	Graph<T,W>::vertices.insert(v);
	if(!isValidTree(Graph<T, W>::vertices,Graph<T, W>::edges))
		throw Exception("Invalid Edge");
}

template <class T, class W>
bool Tree<T, W>::isValidTree(set<T> v, vector< Edge<T, W> > e) {
	bool isAcyclic   = !Graph<T,W>::hasCycle(), 
	     isConnected = checkConnected(v,e);
	if(isAcyclic && isConnected)
		return true;
	return false;
}

template <class T,class W>
bool Tree<T,W>::checkConnected(set<T> v,vector< Edge<T,W > > e){
	Graph<T,W>::bfs(*v.begin());
	typename set<T>::iterator it = v.begin();
	while(it!=v.end()){
		if(!(Graph<T,W>::bfsVisit[*it]) )
			return false;
		it++;
	}
	return true;
}

#endif