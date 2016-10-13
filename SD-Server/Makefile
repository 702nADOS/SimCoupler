CC=g++
CFLAGS=-Wall -std=c++11
LDLIBS=-lcommon -ltraci -ltcpip -lprotobuf

CFLAGS += -DDEBUG -g -Wall

server: server.cpp
	$(CC) -I$(SUMO_HOME)/src/ -o server server.cpp ssd2.pb.cc $(CFLAGS) -L$(SUMO_HOME)/src/utils/traci -L $(SUMO_HOME)/src/foreign/tcpip -L$(SUMO_HOME)/src/utils/common $(LDLIBS)

clean:
	rm server
