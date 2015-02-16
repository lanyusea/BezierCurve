all: planning

planning: main.o
	    gcc -L/usr/local/lib -o planning main.o -lgsl -lgslcblas -lm

main.o: main.c
		gcc -Wall -c main.c

#factorial.o: factorial.cpp
#	    g++ -c factorial.cpp
#
#hello.o: hello.cpp
#	    g++ -c hello.cpp

clean:
	    rm *o planning

