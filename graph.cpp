#include<iostream>
#include<vector>
#include<set>
#include<cstdlib>
using namespace std;

#define tmplt template <class T>

tmplt
class Edge {
	public:
	T u, v;
	int w;
	Edge(){}
	Edge(T _u, T _v, int _w){
		u = _u;
		v = _v;
		w = _w;
	}
	int getWeight(){
		return w;
	}
};

tmplt
class Graph {
	set   < T >                      vertices;	// Vertex set
	vector< Edge<T> >                edges;		// Edges set
	vector< vector< pair<T, int> > > AdjList;	// Adjacency List
	vector< vector<int> >            AdjMat;	// Adjacency Matrix
	public:
	Graph(){}
	Graph(vector<T>, vector< Edge<T> >);
	void addEdge(T, T, int);
	void createAdjacencyList();
	void createAdjacencyMatrix();
};

tmplt
Graph<T>::Graph(vector<T> v, vector< Edge<T> > e){
	vertices = v;
	edges    = e;
}

tmplt
void Graph<T>::addEdge(T u, T v, int w){
	edges.push_back(Edge<T>(u, v, w));
	vertices.insert(u);
	vertices.insert(v);
}

tmplt
void Graph<T>::createAdjacencyList(){
	AdjList.assign(vertices.size()+1, vector< pair<T, int> >());
	for(int i=0;i<edges.size();i++){
		Edge<T> e = edges[i];
		AdjList[e.u].push_back(make_pair(e.v, e.w));
	}
}

tmplt
void Graph<T>::createAdjacencyMatrix(){
	AdjMat.assign(vertices.size()+1, vector<int>(vertices.size(), 0));
	for(int i=0;i<edges.size();i++){
		Edge<T> e = edges[i];
		AdjMat[e.u][e.v] = e.w;
	}
}

int main(){
	Graph<int>  a;
	a.addEdge(1, 2, 3);
	a.addEdge(2, 3, 1);
	a.addEdge(1, 3, 1);
	a.createAdjacencyList();
	a.createAdjacencyMatrix();
}
