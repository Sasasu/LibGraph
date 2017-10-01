#ifndef TEST_EDGE_H
#define TEST_EDGE_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <edge.h>

class EdgeTest : public CppUnit::TestFixture {
	CPPUNIT_TEST_SUITE(EdgeTest);
	CPPUNIT_TEST(testgetWeightNormal);
	CPPUNIT_TEST_SUITE_END();

	private:
	Edge<int, int> *edge;

	public:
	void setUp() { edge = new Edge<int, int>(1, 2, 3); }
	void tearDown() { delete edge; }
	void testgetWeightNormal() { CPPUNIT_ASSERT(edge->getWeight() == 3); }
};
CPPUNIT_TEST_SUITE_REGISTRATION(EdgeTest);
#endif
