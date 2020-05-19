CXXFLAGS=-c -Wall -D_LINUX_ -O2 -I../src/
CXX = g++

all: libArduinoRPC.a

libArduinoRPC.a: ArduinoRPC.o ../src/ArduinoRPC.h
	ar -rc libArduinoRPC.a ArduinoRPC.o ;\
	sudo cp libArduinoRPC.a /usr/local/lib ;\
	sudo cp ../src/ArduinoRPC.h /usr/local/include

ArduinoRPC.o: ../src/ArduinoRPC.cpp ../src/ArduinoRPC.h
	$(CXX) $(CXXFLAGS) ../src/ArduinoRPC.cpp

clean:
	rm *.o libArduinoRPC.a

