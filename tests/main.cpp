#include <cppunit/extensions/TestFactoryRegistry.h>
#include <cppunit/ui/text/TestRunner.h>
#include "test-edge.h"
#include "test-graph.h"
#include "test-tree.h"

int main(int argc, char **argv) {
    CppUnit::TextUi::TestRunner runner;
    CppUnit::TestFactoryRegistry &registry =
        CppUnit::TestFactoryRegistry::getRegistry();
    runner.addTest(registry.makeTest());
    return !runner.run("", false);
}
