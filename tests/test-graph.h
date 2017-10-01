#ifndef TEST_GRAPH_H
#define TEST_GRAPH_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <exception.h>
#include <graph.h>
#include <tree.h>

class GraphTest : public CppUnit::TestFixture {
	CPPUNIT_TEST_SUITE(GraphTest);
	CPPUNIT_TEST(testaddEdgeNormal);
	CPPUNIT_TEST(testgetEdgeNormal);
	CPPUNIT_TEST(testgetEdgeNotFind);
	CPPUNIT_TEST(testcreateAdjacencyListNormal);
	CPPUNIT_TEST(testgetAdjavencyListNormal);
	CPPUNIT_TEST(testgetAdjavencyListEmpty);
	CPPUNIT_TEST(testcreateAdjacencyMatrixNormal);
	CPPUNIT_TEST(testgetAdjacencyMatrixNormal);
	CPPUNIT_TEST(testgetAdjacencyMatrixEmpty);
	CPPUNIT_TEST(testdijkstrasNormal);
	CPPUNIT_TEST_EXCEPTION(testdijkstrasContainNegativeMargin, Exception);
	CPPUNIT_TEST(testdijkstrasEmpty);
	CPPUNIT_TEST(testbellmanFordNormal);
	CPPUNIT_TEST(testbellmanFordContainNegativeMargin);
	CPPUNIT_TEST(testbellmanFordEmpty);
	// getShortestPathsFrom not need test
	CPPUNIT_TEST(testbfsNormal);
	CPPUNIT_TEST(testbfsEmpty);
	CPPUNIT_TEST(testbfContainSindependentGroup);
	// printLevel not need test
	CPPUNIT_TEST(testhasCycleNormalNoCycle);
	CPPUNIT_TEST(testhasCycleNormalHasCycle);
	CPPUNIT_TEST(testhasCycleOtherHasCycle);
	CPPUNIT_TEST(testgetAllPairShortestPathNormal);
	CPPUNIT_TEST(testdfsNormal);
	CPPUNIT_TEST(testdfsEmpty);
	CPPUNIT_TEST(testminimumSpanningTreeNormal);
	CPPUNIT_TEST_SUITE_END();

	private:
	Graph<int, int> *graph;

	public:
	void setUp() { graph = new Graph<int, int>(); }

	void tearDown() { delete graph; }

	void testaddEdgeNormal() {
		Edge<int, int> tmpEdge(1, 2, 3);
		graph->addEdge(1, 2, 3);
	}

	void testgetEdgeNormal() {
		Edge<int, int> tmpEdge(1, 2, 3);
		graph->addEdge(1, 2, 3);

		set<int> tmpSet = graph->getVertices();
		CPPUNIT_ASSERT(tmpSet.find(1) != tmpSet.end());
		CPPUNIT_ASSERT(tmpSet.find(2) != tmpSet.end());

		vector<Edge<int, int>> tmpEdges = {tmpEdge};
		CPPUNIT_ASSERT(graph->getEdges() == tmpEdges);
	}

	void testgetEdgeNotFind() {
		Edge<int, int> tmpEdge(1, 2, 3);
		graph->addEdge(1, 2, 3);

		set<int> tmpSet = graph->getVertices();
		CPPUNIT_ASSERT(tmpSet.find(3) == tmpSet.end());
	}

	void testcreateAdjacencyListNormal() {
		graph->addEdge(1, 2, 3);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 3, 1);
		graph->createAdjacencyList();
	}

	void testgetAdjavencyListNormal() {
		graph->addEdge(1, 2, 3);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 3, 1);
		map<int, vector<pair<int, int>>> tmpMap = graph->getAdjacencyList();
		vector<vector<pair<int, int>>> tmpVec = {{{2, 3}, {3, 1}}, {{3, 1}}};
		CPPUNIT_ASSERT(tmpMap[1] == tmpVec[0]);
		CPPUNIT_ASSERT(tmpMap[2] == tmpVec[1]);
	}

	void testgetAdjavencyListEmpty() {
		map<int, vector<pair<int, int>>> tmpMap = graph->getAdjacencyList();
		CPPUNIT_ASSERT(tmpMap.empty());
	}

	void testcreateAdjacencyMatrixNormal() {
		graph->addEdge(1, 2, 3);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 3, 1);
		graph->createAdjacencyMatrix();
	}

	void testgetAdjacencyMatrixNormal() {
		graph->addEdge(1, 2, 3);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 3, 1);
		map<int, map<int, int>> tmpMap = graph->getAdjacencyMatrix();
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				if (i == 1 && j == 2)
					CPPUNIT_ASSERT(tmpMap[i][j] == 3);
				else if (i == 2 && j == 3)
					CPPUNIT_ASSERT(tmpMap[i][j] == 1);
				else if (i == 1 && j == 3)
					CPPUNIT_ASSERT(tmpMap[i][j] == 1);
				else
					CPPUNIT_ASSERT((tmpMap.find(i) == tmpMap.end() ||
									tmpMap[i].find(j) == tmpMap[i].end()));
			}
		}
	}

	void testgetAdjacencyMatrixEmpty() {
		map<int, map<int, int>> tmpMap = graph->getAdjacencyMatrix();
		CPPUNIT_ASSERT(tmpMap.empty());
	}

	void testdijkstrasNormal() {
		graph->addEdge(1, 2, 3);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 3, 1);
		map<int, int> tmpMap = graph->dijkstras(1);
		CPPUNIT_ASSERT(tmpMap[1] == 0);
		CPPUNIT_ASSERT(tmpMap[2] == 3);
		CPPUNIT_ASSERT(tmpMap[3] == 1);
		CPPUNIT_ASSERT(tmpMap.size() == 3);
	}

	void testdijkstrasContainNegativeMargin() {
		graph->addEdge(1, 2, -3);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 3, 1);
		map<int, int> tmpMap = graph->dijkstras(1);
	}

	void testdijkstrasEmpty() {
		map<int, int> tmpMap = graph->dijkstras(1);
		CPPUNIT_ASSERT(tmpMap[1] == 0);
		CPPUNIT_ASSERT(tmpMap.size() == 1);
	}

	void testbellmanFordNormal() {
		graph->addEdge(1, 2, 3);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 3, 1);
		map<int, int> tmpMap = graph->bellmanFord(1);
		CPPUNIT_ASSERT(tmpMap[1] == 0);
		CPPUNIT_ASSERT(tmpMap[2] == 3);
		CPPUNIT_ASSERT(tmpMap[3] == 1);
		CPPUNIT_ASSERT(tmpMap.size() == 3);
	}

	void testbellmanFordContainNegativeMargin() {
		graph->addEdge(1, 2, -3);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 3, 1);
		map<int, int> tmpMap = graph->bellmanFord(1);
		CPPUNIT_ASSERT(tmpMap[1] == 0);
		CPPUNIT_ASSERT(tmpMap[2] == -3);
		CPPUNIT_ASSERT(tmpMap[3] == -2);
		CPPUNIT_ASSERT(tmpMap.size() == 3);
	}

	void testbellmanFordEmpty() {
		map<int, int> tmpMap = graph->bellmanFord(1);
		CPPUNIT_ASSERT(tmpMap[1] == 0);
		CPPUNIT_ASSERT(tmpMap.size() == 1);
	}

	void testbfsNormal() {
		graph->addEdge(1, 2, 1);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 4, 1);
		map<int, int> tmpMap = graph->bfs(1);
		CPPUNIT_ASSERT(tmpMap.size() == 4);
		CPPUNIT_ASSERT(tmpMap[1] == 0);
		CPPUNIT_ASSERT(tmpMap[2] == 1);
		CPPUNIT_ASSERT(tmpMap[3] == 2);
		CPPUNIT_ASSERT(tmpMap[4] == 1);
	}

	void testbfsEmpty() {
		map<int, int> tmpMap = graph->bfs(1);
		CPPUNIT_ASSERT(tmpMap.size() == 1);
		CPPUNIT_ASSERT(tmpMap[1] == 0);
	}

	void testbfContainSindependentGroup() {
		graph->addEdge(1, 2, 1);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 4, 1);
		graph->addEdge(5, 6, 1);
		map<int, int> tmpMap = graph->bfs(1);
		CPPUNIT_ASSERT(tmpMap.size() == 4);
		CPPUNIT_ASSERT(tmpMap[1] == 0);
		CPPUNIT_ASSERT(tmpMap[2] == 1);
		CPPUNIT_ASSERT(tmpMap[3] == 2);
		CPPUNIT_ASSERT(tmpMap[4] == 1);
	}

	void testhasCycleNormalNoCycle() {
		graph->addEdge(1, 2, 1);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 4, 1);
		CPPUNIT_ASSERT(graph->hasCycle() == false);
	}

	void testhasCycleNormalHasCycle() {
		graph->addEdge(1, 2, 1);
		graph->addEdge(2, 3, 1);
		graph->addEdge(3, 1, 1);
		CPPUNIT_ASSERT(graph->hasCycle() == true);
	}

	void testhasCycleNormalEmpty() {
		CPPUNIT_ASSERT(graph->hasCycle() == false);
	}

	void testhasCycleOtherHasCycle() {
		Graph<int, int> HeadToTail;
		HeadToTail.addEdge(1, 2, 1);
		HeadToTail.addEdge(2, 3, 1);
		HeadToTail.addEdge(3, 1, 1);
		CPPUNIT_ASSERT(HeadToTail.hasCycle() == true);

		Graph<int, int> SelfToSelf;
		SelfToSelf.addEdge(1, 1, 1);
		CPPUNIT_ASSERT(SelfToSelf.hasCycle() == true);

		Graph<int, int> TwoNode;
		TwoNode.addEdge(1, 2, 1);
		TwoNode.addEdge(2, 1, 1);
		CPPUNIT_ASSERT(TwoNode.hasCycle() == true);
	}

	void testgetAllPairShortestPathNormal() {
		graph->addEdge(1, 2, 1);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 3, 1);
		map<int, map<int, int>> tmpMap = graph->getAllPairShortestPath();
		CPPUNIT_ASSERT(tmpMap[1][1] == 0);
		CPPUNIT_ASSERT(tmpMap[1][2] == 1);
		CPPUNIT_ASSERT(tmpMap[1][3] == 1);
		CPPUNIT_ASSERT(tmpMap[2][1] == -1);
		CPPUNIT_ASSERT(tmpMap[2][2] == 0);
		CPPUNIT_ASSERT(tmpMap[2][3] == 1);
		CPPUNIT_ASSERT(tmpMap[3][1] == -1);
		CPPUNIT_ASSERT(tmpMap[3][2] == -1);
		CPPUNIT_ASSERT(tmpMap[3][3] == 0);
	}

	void testdfsNormal() {
		graph->addEdge(1, 2, 1);
		graph->addEdge(2, 3, 1);
		graph->addEdge(1, 3, 1);
		vector<int> tmpMap = graph->dfs(1);
		vector<int> ans = {1, 2, 3};
		for (int i = 0; i < ans.size(); ++i) {
			CPPUNIT_ASSERT(ans[i] == tmpMap[i]);
		}
	}

	void testdfsEmpty() {
		CPPUNIT_ASSERT(graph->dfs(1).size() == 1);
		CPPUNIT_ASSERT(graph->dfs(1)[0] == 1);
	}

	void testminimumSpanningTreeNormal() {
		graph->addEdge(1, 2, 1);
		graph->addEdge(2, 3, 2);
		graph->addEdge(1, 3, 1);
		Tree<int, int> tmpTree = graph->minimumSpanningTree(1);
		map<int, vector<pair<int, int>>> tmpMap = tmpTree.getAdjacencyList();
		map<int, vector<pair<int, int>>> ansMap = {
			{1, {{2, 1}, {3, 1}}}, {2, {}}, {3, {}}};
		for (auto i : tmpMap) {
			cout << endl << i.first << "{";
			for (auto j : i.second) {
				cout << "[" << j.first << " " << j.second << "]";
			}
			cout << "}";
		}
		CPPUNIT_ASSERT(ansMap == tmpMap);
	}
};

CPPUNIT_TEST_SUITE_REGISTRATION(GraphTest);
#endif
