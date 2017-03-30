#include<iostream>
#include<vector>
#include<set>
#include<cstdlib>
#include<queue>
#include<map>
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
	set   < T >                       vertices;	// Vertex set
	vector< Edge<T> >                 edges;	// Edges set
	map< T, vector< pair< T, int> > > AdjList;	// Adjacency List
	map< T, map< T, int> >            AdjMat;	// Adjacency Matrix
	public:
	Graph(){}
	Graph(vector<T>, vector< Edge<T> >);
	void addEdge(T, T, int);
	void createAdjacencyList();
	void printAdjacencyList();
	void createAdjacencyMatrix();
	void printAdjacencyMatrix();
	map<T, int> shortestPathFrom(T);
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
	for(int i=0;i<edges.size();i++){
		Edge<T> e = edges[i];
		AdjList[e.u].push_back(make_pair(e.v, e.w));
	}
}

tmplt
void Graph<T>::printAdjacencyList(){
	typename map< T, vector< pair< T, int> > >::iterator u;
	for(u=AdjList.begin(); u!=AdjList.end(); u++){
		cout<<u->first<<" - ";
		for(int i=0; i<(u->second).size(); i++){
			pair< T, int > v = u->second[i];
			cout<<"( "<<v.first<<", "<<v.second<<" ) ";
		}
		cout<<endl;
	}
}

tmplt
void Graph<T>::createAdjacencyMatrix(){
	for(int i=0;i<edges.size();i++){
		Edge<T> e = edges[i];
		AdjMat[e.u][e.v] = e.w;
	}
}

tmplt
void Graph<T>::printAdjacencyMatrix(){
	typename map< T, map< T, int > >::iterator i;
	for(i=AdjMat.begin(); i!=AdjMat.end(); i++){
		typename map<T, int>::iterator j;
		for(j=(i->second).begin(); j!=(i->second).end(); j++)
			cout<<j->second<<' ';
		cout<<endl;
	}
}

tmplt 
map<T, int> Graph<T>::shortestPathFrom(T s){
	if(AdjList.size()==0)
		createAdjacencyList();
	map<T, int> dist;
	typename set<T>::iterator it;
	for(it=vertices.begin(); it!=vertices.end(); it++)
		dist[*it] = 1000000;
	priority_queue< pair<int, T> > q;
	q.push(make_pair(0, s));
	dist[s] = 0;
	while(!q.empty()){
		pair<int, T> u_p = q.top();
		q.pop();
		T u = u_p.second;
		for(int i=0; i<AdjList[u].size();i++){
			pair< T, int > v_p = AdjList[u][i];
			T v = v_p.first;
			int w = v_p.second;
			if(dist[v] > dist[u]+w){
				dist[v] = dist[u]+w;
				q.push(make_pair(dist[v], v));
			}
		}
	}
	return dist;
}

int main(){
	Graph<int>  a;
	a.addEdge(1, 2, 3);
	a.addEdge(2, 3, 1);
	a.addEdge(1, 3, 1);
	a.createAdjacencyList();
	a.createAdjacencyMatrix();
	a.printAdjacencyList();
	a.printAdjacencyMatrix();
	map<int, int> dist = a.shortestPathFrom(1);
	cout<<dist[1]<<' '<<dist[2]<<' '<<dist[3]<<endl;

}
