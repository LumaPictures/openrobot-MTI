
ALL: MTI

MTI:  MTComm.o MTI.o
	g++ -o MTI MTI.o MTComm.o -lrt

MTI.o: MTI.cpp MTComm.h
	g++ -o2 -Wall -c MTI.cpp -o MTI.o

MTComm.o: MTComm.cpp MTComm.h
	g++ -o2 -Wall -c MTComm.cpp -o MTComm.o

clean:
	rm -rf *.o MTI

