# LibGraph

LibGraph is Graph library for C++. It can be used to solve various graph problems.

### Usage

+ Clone the repository
```sh
git clone http://github.com/Hrily/LibGraph
```

+ Include header in your code
```c++
#include "graph.h"
```

+ And use...

### Features

+ Graph
  - BFS
  - DFS
  - Cycle Detection
  - Single Source Shortest Path
  - All Pair Shortest Path
  - Minimum Spanning Tree
+ Tree

### Graph

#### Initialization

Graph can be initialized as follows
```c++
Graph<int, int> G;
```

where first `int` is type of vertices and second `int` is type of edge weight

#### Adding edges

Edges are added as follows
```c++
G.addEdge(1, 2, 3);
```
where edge `1->2` has weight `3`

#### BFS

BFS of graph generates a mapping of vertices to their levels
```c++
map<int, int> bfsLevel = G.bfs(1);
```
where `1` is source of bfs tree

#### DFS

DFS of graph generates vector of vertices which are in sequence of their dfs visit
```c++
vector<int> dfsVisitSequence = G.dfs(1);
```
Again `1` here is source

#### Cycle Detection

Cycle can be detected as follows
```c++
if(G.hasCycle())
  cout<<"Graph has cycle\n";
else
  cout<<"Graph does not have any cycle\n";
```

#### Single Source Shortest Path

Shortest paths from a source generates mapping of vertices to shortest distance
```c++
map<int, int> shortestPaths = G.getShortestPathsFrom(1, false);
```
where first parameter is source and second parameter is whether graph has negative edges, `false` by default.

#### All Pair Shortest Path

All Pair Shortest Path generates two dimensional mapping of pair of vertices to their shortest paths
```c++
map<int, map<int, int> > allPairShortestPath = G.getAllPairShortestPath();
```

#### Minimum Spanning Tree

Minimum Spanning Tree returns Object of type `Tree`, which is defined in `tree.h`
```c++
Tree<int, int> mst = G.minimumSpanningTree();
```

### Tree

Tree is a class derived from Graph. It has same functionalities as Graph, but can't have any cycles

#### Usage

Tree is defined in `tree.h`. Use it as follows 
```c++
#include "tree.h"
```
*NOTE*: Graph is already included in tree. So you need not to include graph when you are including tree.

#### Initialization

Tree is initialized as follows
```c++
Tree<int, int> T;
```
Same rules apply here, first `int` is vertex type, while second is edge weight type

#### Adding Edges

Adding edges to Tree is same as Graph, but adding edge here can generate exception (of type Exception) if adding the edge to tree makes it invalid
```c++
try{
  T.addEdge(1, 2, 3);
}catch(Exception e){
  cout<<"Exception: "<<e.getMessage()<<endl;
}
```
