#include "../graph.h"
using namespace std;

int main()
{
	int n,m,x,y;
	cin>>n>>m;

	Graph <int ,int> g;

	for(int i=0;i<m;i++)
	{
		cin>>x>>y;
		g.addEdge(x,y,0);
		g.addEdge(y,x,1);
	}

	map<int,int> mp;
	mp = g.dijkstras(1);

	g.printAdjacencyList();

	cout<<mp[n]<<endl;
	return 0;
}