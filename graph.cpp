#include<iostream>
#include<vector>
#include<set>
#include<cstdlib>
#include<queue>
#include<map>
using namespace std;

#define INF 10000000

template <class T, class W>
class Edge {
	public:
	T u, v;
	W w;
	Edge(){}
	Edge(T _u, T _v, int _w){
		u = _u;
		v = _v;
		w = _w;
	}
	W getWeight(){
		return w;
	}
};

template <class T, class W>
class Graph {
	set   < T >                     vertices;       // Vertex set
	vector< Edge<T,W> >             edges;	        // Edges set
	map< T, vector< pair< T, W> > > AdjList;	// Adjacency List
	map< T, map< T, W> >            AdjMat;	        // Adjacency Matrix
	map< T, int>                    Level;          // BFS output levels
	map< T, bool>                   bfsVisit;       // BFS Visit 
 	public:
	Graph(){}
	Graph(vector<T>, vector< Edge<T,W> >);
	void addEdge(T, T, W);
	void createAdjacencyList();
	void printAdjacencyList();
	void createAdjacencyMatrix();
	void printAdjacencyMatrix();
	map<T, W> dijkstras(T);
	map<T, W> bellmanFord(T);
	map<T, W> shortestPathFrom(T, bool negative = false);
	void printLevel(T);
	void bfs(T);	
	bool hasCycleUtil(T, map<T, bool>&, map<T, bool>&);
	bool hasCycle();
};

/*
 * Graph class Constructor
 * @param (vector<T>) v
 *   vector of vertices
 * @param (vector<EdgeT,W>>) e
 *   vector of edges
 */
template <class T,class W>
Graph<T,W>::Graph(vector<T> v, vector< Edge<T,W> > e){
	vertices = v;
	edges    = e;
}

/*
 * Function to add edge
 * @param (T) u
 *   vertex
 * @param (T) v
 *   vertex
 * @param (W) w 
 *   weight of edge from u to v
 */
template <class T,class W>
void Graph<T,W>::addEdge(T u, T v, W w){
	edges.push_back(Edge<T,W>(u, v, w));
	vertices.insert(u);
	vertices.insert(v);
}

/* 
 * Function to create Adjacency list
 */
template <class T,class W>
void Graph<T,W>::createAdjacencyList(){
	for(int i=0;i<edges.size();i++){
		Edge<T,W> e = edges[i];
		AdjList[e.u].push_back(make_pair(e.v, e.w));
	}
}

/*
 * Function to print Adjacency list
 */
template <class T,class W>
void Graph<T,W>::printAdjacencyList(){
	typename map< T, vector< pair< T, W> > >::iterator u;
	for(u=AdjList.begin(); u!=AdjList.end(); u++){
		cout<<u->first<<" - ";
		for(int i=0; i<(u->second).size(); i++){
			pair< T, W > v = u->second[i];
			cout<<"( "<<v.first<<", "<<v.second<<" ) ";
		}
		cout<<endl;
	}
}

/*
 * Function to create Adjacency Matrix
 */
template <class T,class W>
void Graph<T,W>::createAdjacencyMatrix(){
	for(int i=0;i<edges.size();i++){
		Edge<T,W> e = edges[i];
		AdjMat[e.u][e.v] = e.w;
	}
}

/*
 * Fucnction to print Adjacency Matrix
 */
template <class T,class W>
void Graph<T,W>::printAdjacencyMatrix(){
	typename map< T, map< T, int > >::iterator i;
	for(i=AdjMat.begin(); i!=AdjMat.end(); i++){
		typename map<T, W>::iterator j;
		for(j=(i->second).begin(); j!=(i->second).end(); j++)
			cout<<j->second<<' ';
		cout<<endl;
	}
}

/*
 * Function to calculate Single Source Shortest Paths using Dijkstra's Algorithm
 * For graphs with positive edge weights only
 * For graphs consiting negative edge weights, use bellmanFord(T)
 * @param (T) s
 *   source vertex
 * @returns (map<T,W>) 
 *   mapping of weights of shortest paths
 */
template <class T, class W>
map<T, W> Graph<T, W>::dijkstras(T s){
	if(AdjList.size()==0)
		createAdjacencyList();
	map<T, W> dist;
	typename set<T>::iterator it;
	for(it=vertices.begin(); it!=vertices.end(); it++)
		dist[*it] = INF;
	priority_queue< pair<W, T> > q;
	q.push(make_pair(0, s));
	dist[s] = 0;
	while(!q.empty()){
		pair<W, T> u_p = q.top();
		q.pop();
		T u = u_p.second;
		for(int i=0; i<AdjList[u].size();i++){
			pair< T, W> v_p = AdjList[u][i];
			T v = v_p.first;
			W w = v_p.second;
			if(dist[v] > dist[u]+w){
				dist[v] = dist[u]+w;
				q.push(make_pair(dist[v], v));
			}
		}
	}
	return dist;
}

/*
 * Function to calculate Single Source Shortest Paths using Bellman Ford Cycle
 * For graphs consisting negative edge weights
 * @param (T) s
 *   source vertex
 * @returns (map<T,W>) 
 *   mapping of weights of shortest paths
 */
template <class T,  class W>
map<T, W> Graph<T, W>::bellmanFord(T s){
	map<T, W> dist;
	typename set<T>::iterator it;
	for(it=vertices.begin(); it!=vertices.end(); it++)
		dist[*it] = INF;
	dist[s] = 0;
	typename vector< Edge<T, W> >::iterator e;
	for(int i=0; i<vertices.size()-1;i++){
		for(e=edges.begin(); e!=edges.end(); e++){
			T u = e->u,
			  v = e->v;
			W w = e->w;
			if(dist[u]!=INF && dist[u]+w < dist[v])
				dist[v] = dist[u] + w;
		}
	}
	return dist;
}

/*
 * Function to calculate Single Source Shortest Paths
 * @param (T) s
 *   source vertex
 * @param (bool) negative
 *   true if graph consists negative edge weights, default value false
 * @returns (map<T,W>) 
 *   mapping of weights of shortest paths
 */
template <class T, class W>
map<T, W> Graph<T,W>::shortestPathFrom(T s, bool negative){
	if(negative)
		return bellmanFord(s);
	return dijkstras(s);
}

/* 
 * Function to apply Breadth First Search on graph
 * @param (T) source
 *   source of required BFS
 * @returns (map<T, int>) 
 *   mapping of vertices to their levels in BFS tree
 */
template <class T,class W>
void Graph<T,W>::bfs(T source)
{
	//Initialisation
	Level.clear();
	bfsVisit.clear();
	//Level.insert(pair< T , int>(source,0) );
	Level[source] = 0;
	bfsVisit[source] = true;
	queue <T> q;
	q.push(source);
	while(!q.empty())
	{
		T u = q.front();
		q.pop();
		for(int i=0;i<AdjList[u].size();i++)
		{
			Level[AdjList[u][i].first] = Level[u] + 1;
			bfsVisit[AdjList[u][i].first] = true;
			q.push(AdjList[u][i].first);
		}
	}
}

template <class T,class W>
void Graph<T,W>::printLevel(T source)
{
	bfs(source);
	typename map<T,int>::iterator it;
	it = Level.begin();
	while(it!=Level.end())
	{
		cout<<"Vertex "<<it->first<<" is at Level "<<it->second<<endl;
		it++;
	}
}

/* 
 * Utility function for detecting cycle in graph
 * Used by hasCycle(T, map<T, bool>&, map<T, bool>&)
 * @param (T) v
 *   current vertex
 * @param (vector<T, bool>) visited
 *   vector of visit status of vertex
 * @param (vector T, bool>) racStack
 *   vector of stack status of vertex
 * @returns (bool) 
 *   true if graph has cycle, else false
 */
template <class T, class W>
bool Graph<T, W>::hasCycleUtil(T v, map<T, bool> &visited, map<T, bool>  &recStack){
	if(visited[v] == false) {
		// Mark the current node as visited and part of recursion stack
		visited[v] = true;
		recStack[v] = true;

		// Recur for all the vertices adjacent to this vertex
		for(int i=0; i<AdjList[v].size(); i++){
			T u = AdjList[v][i].first;
			if(!visited[u] && hasCycleUtil(u, visited, recStack))
				return true;
			else if(recStack[u])
				return true;
		}
	}
	recStack[v] = false;  // remove the vertex from recursion stack
	return false;
}

/*
 * Function to detect cycle in graph
 * @returns (bool)
 *   true if graph has cycle, else false
 */
template <class T, class W>
bool Graph<T, W>::hasCycle(){
	map<T, bool> visited, recStack;
	typename set<T>::iterator it;
	for(it=vertices.begin(); it!=vertices.end(); it++)
		if(hasCycleUtil(*it, visited, recStack))
			return true;
	return false;
}
		

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
