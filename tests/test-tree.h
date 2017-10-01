#ifndef TEST_TREE_H
#define TEST_TREE_H

#include <cppunit/TestFixture.h>
#include <cppunit/extensions/HelperMacros.h>
#include <exception.h>
#include <graph.h>
#include <tree.h>

class TreeTest : public CppUnit::TestFixture {
	CPPUNIT_TEST_SUITE(TreeTest);
	CPPUNIT_TEST(a); // delet later
	CPPUNIT_TEST_SUITE_END();

	private:
	Tree<int, int> *tree;

	public:
	void setUp() { tree = new Tree<int, int>(); }

	void tearDown() { delete tree; }

	void a() {} // delet later
};

CPPUNIT_TEST_SUITE_REGISTRATION(TreeTest);
#endif
