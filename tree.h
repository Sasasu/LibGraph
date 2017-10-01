#ifndef TREE_H
#define TREE_H

#include <stack>
#include "graph.h"

template <class T, class W>
class Tree:public Graph<T, W> {

	stack<T> topologicalSortStack;
	map<T, bool> topologicalSortVisit;
	vector<T> topologicalSortSequence;

	public:
	Tree(){};
	Tree(const Tree&); 
	void addEdge(T,T,W);
	bool isValidTree(set<T>, vector< Edge<T, W > >,T);
	bool checkConnected(set<T>, vector< Edge<T, W > >,T);
	void explore(T);
	vector<T> topologicalSort();
};

/*
 * Copy Constructor
 * Clones given tree
 * @param (Tree) Tr
 *     Tree to clone
 */
template <class T,class W>
Tree<T, W>::Tree(const Tree &Tr) {
	Graph<T,W>::vertices   = Tr.vertices;       
	Graph<T,W>::edges      = Tr.edges;	        
}

/* 
 * Function to add Edge to tree
 * throws exception of type Exception if adding edge forms invalid tree
 * @param (T) u
 *    starting vertex
 * @param (T) v
 *    ending vertex
 * @param (W) w
 *    weight of edge(u, v)
 */
template <class T, class W>
void Tree<T, W>::addEdge(T u, T v, W w)
{
	int flagu=0,flagv=0;
	T s ;
	Graph<T,W>::edges.push_back(Edge<T,W>(u, v, w));
	
	if(Graph<T,W>::vertices.find(u)!=Graph<T,W>::vertices.end())
		flagu = 1;
	else
		s = u;
	if(Graph<T,W>::vertices.find(v)!=Graph<T,W>::vertices.end())
		flagv = 1;
	else 
		s = v;
	
	Graph<T,W>::vertices.insert(u);
	Graph<T,W>::vertices.insert(v);

	try
	{
		if( !isValidTree(Graph<T,W>::vertices,Graph<T,W>::edges, u))
			throw Exception("Invalid Edge Added");
	}
	catch(Exception e)
	{
		Graph<T,W>::edges.pop_back();
	
		if(!flagu)
		Graph<T,W>::vertices.erase(u);
		if(!flagv)
		Graph<T,W>::vertices.erase(v);
	
		cout<<"Error : "<<e.getMessage()<<endl;
	}
}

/*
 * Function to check if tree is valid
 */
template <class T,class W>
bool Tree<T,W>::isValidTree(set<T> v, vector< Edge<T,W> > e,T s)
{
	bool isAcyclic=false,isConnected=false;
	Graph<T,W>::createAdjacencyList();
	isAcyclic = !Graph<T,W>::hasCycle();	
	isConnected = checkConnected(v,e,s);
	if(isAcyclic && isConnected)
		return true;
	else
		return false;
}

/*
 * Function to check if tree is connected
 */
template <class T,class W>
bool Tree<T,W>::checkConnected(set<T> v,vector< Edge<T,W > > e,T s)
{
	// T s = *v.begin();
	Graph<T,W>::bfs(s);
	typename set<T>::iterator it = v.begin();

	while(it!=v.end())
	{
		if( !(Graph<T,W>::bfsVisit[*it]) )
			return false;
		it++;
	}
	return true;
}

/*
 * Function to explore source 
 * used by topological sort
 * @param (T) source
 *     source of exploration
 */
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

/*
 * Function to sort tree topologically
 * @returns (vactor<T>)
 *    topological sequence of tree
 */
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
