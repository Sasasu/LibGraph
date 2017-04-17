/* 
 * Function to find Minimum Spanning Tree of a Graph
 * @param (T) source
 *   root of Minimum Spanning Tree
 * @returns Tree(Minimum Weight)
 */

template <class T, class W>
Tree<T, W> minimumSpanningTree(T source)
{
	if(AdjList.size()==0)
		createAdjacencyList();

	Tree<T, W> mst;
	map<T, W> key;
	priority_queue< pair<W, T> > heap;
	T u, v, prev;
	W v_w;

	typename set<T>::iterator it;
	for(it = vertices.begin(); it!=vertices.end(); it++)
	{
		key[*it] = INF;
	}

	key[source] = 0; 
	heap.push( make_pair(0, source) );


	prev = u;
	while(!heap.empty())
	{
		u = heap.top().second;
		heap.pop();

		for(int i=0; i<AdjList[u].size() && u!=source ;i++)
		{
			v = AdjList[u][i].first;
			if(prev == v)
			{
				mst.addEdge(u, v, AdjList[u][i].second);
			}	
		}	

		for(int i=0; i<AdjList[u].size() ;i++)
		{
			v = AdjList[u][i].first;
			v_w = AdjList[u][i].second;
			
			if( key[v] > v_w )
			{
				key[v] = v_w;
				heap.push( make_pair(v_w, v) );
			}	
		} 
		prev = u;
	}	

	return mst;
}
