CXX = g++
INCLUDES = -Iusr/include/
INCLUDES += -I../
CXXFLAGS = -w -g $(INCLUDES)
LINKFLAGS = -lcppunit
LINKFLAGS += -L./

run: main-run

clean: main-clean

bulid: main

main: *.h
	$(CXX) $(CXXFLAGS) $(LINKFLAGS) -o $@ main.cpp
main-clean:
	rm -f main
	rm -f *.o
main-run: bulid
	./main
