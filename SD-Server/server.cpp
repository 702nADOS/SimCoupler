#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <utils/traci/TraCIAPI.h>
#include "json.hpp"

// SUMO
class TraCIClient : public TraCIAPI {
public:
  TraCIClient();
};

TraCIClient::TraCIClient() {
  try {
    connect("localhost", 2001);
  } catch(tcpip::SocketException &e) {
    printf("tcpip: %s\n", e.what());
  }
}

// for convenience
using json = nlohmann::json;

#define PORT 2000
#define BUFFER_SIZE 1024
#define MAX_ROUTE_ELEMENTS 5

void sigHandler(int);

int serverSocket;

std::vector<std::string> edgeList;

std::vector<std::string> newRoute(std::string edge, bool backwards) {
  // create new route list
  std::vector<std::string> route;

  // find index of current edge
  std::vector<std::string>::iterator index = std::find(edgeList.begin(), edgeList.end(), edge);

  // add elements to route
  if(backwards) { //backwards
    for(std::vector<std::string>::iterator it = index; route.size() < MAX_ROUTE_ELEMENTS; --it) {
      route.push_back(*it);
      printf("Pushed %s to route.\n", (*it).c_str());
      if(it == edgeList.begin()) {
	it = edgeList.end();
      }
    }
  } else { //forwards
    for(std::vector<std::string>::iterator it = index; route.size() < MAX_ROUTE_ELEMENTS; ++it) {
      route.push_back(*it);
      printf("Pushed %s to route.\n", (*it).c_str());
      if(it == edgeList.end()-1 && route.size() < MAX_ROUTE_ELEMENTS) {
	it = edgeList.begin();
	route.push_back(*it);
	printf("Pushed %s to route.\n", (*it).c_str());
      }
    }
  }

  // return new route
  printf("X -> ");
  for(std::vector<std::string>::iterator it = route.begin(); it != route.end(); ++it) {
    printf("%s -> ", (*it).c_str());
  }
  printf("X\n");
  return route;
}

int main(int argc , char *argv[]) {
  TraCIClient traciclnt;
  typedef std::tuple<std::string, double> lane;

  try {
    edgeList = traciclnt.route.getEdges("route0");
  } catch(tcpip::SocketException &e) {
    printf("[Error] tcpip: %s\n", e.what());
  }
	
  std::vector<lane> llist(edgeList.size());
  for(std::vector<std::string>::iterator it = edgeList.begin(); it != edgeList.end(); ++it) {
    printf("Edge: %s\n", (*it).c_str());
    llist.push_back(std::make_tuple(*it + "_0", traciclnt.lane.getLength((*it + "_0"))));
  }

  double length = 0;
  for(std::vector<lane>::iterator it = llist.begin(); it != llist.end(); ++it) {
    printf("Edge: %s with %f meter.\n", std::get<0>(*it).c_str(), std::get<1>(*it));
    length += std::get<1>(*it);
  };
  //printf("Amount of edges: %d\n", list.size());
  printf("Total length: %f\nBy %d lanes\n", length, llist.size());

  traciclnt.simulationStep(1001);

  signal(SIGINT, sigHandler);

  int serverSocket, c, clientSocket;
  char buffer[BUFFER_SIZE];
  struct sockaddr_in serverAddr, clientAddr;

  serverSocket = socket(PF_INET, SOCK_STREAM, 0);

  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(PORT);
  serverAddr.sin_addr.s_addr = inet_addr("0.0.0.0");
  memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);

  if(bind(serverSocket, (struct sockaddr *) &serverAddr, sizeof(serverAddr)) == 0) {
    printf("Bound to port %d \n", ntohs(serverAddr.sin_port));
  } else {
    printf("Bind failed!\n");
    return 1;
  }

  if(listen(serverSocket, 5) == 0)
    printf("Listening\n");
  else
    printf("Error\n");

  c = sizeof(struct sockaddr_in);

  clientSocket = accept(serverSocket, (struct sockaddr *) &clientAddr, (socklen_t *) &c);
  puts("Connection accepted");

  json oldData = NULL;

  while( recv(clientSocket, buffer, BUFFER_SIZE, 0) ) {
    //printf("recv: %s\n", buffer);
    json data = json::parse(buffer);
    if (oldData == NULL) oldData = data;
    /*for (json::iterator it = data.begin(); it != data.end(); ++it) {
      std::cout << it.key() << " : " << it.value() << "\n";
      }*/

    double tmplength = 0;
    for (std::vector<lane>::iterator it = llist.begin(); it != llist.end(); ++it) {
      //traciclnt.simulationStep(0);
      tmplength += std::get<1>(*it);
      if (tmplength >= data["veh0"]["pos"] && data["veh0"]["pos"] >= 0) {
	/*printf("Lane: %s", std::get<0>(*it).c_str());
	  printf(", Pos: %f\n", (double)data["veh0"]["pos"] - (tmplength - std::get<1>(*it)));*/
	try {
	  std::string currentEdge = traciclnt.vehicle.getRoadID("veh0");
	  printf("Current edge: %s\n", currentEdge.c_str());
	  
	  //if ((double)data["veh0"]["speed"] < 0) {
	  if ((double)data["veh0"]["pos"] < (double)oldData["veh0"]["pos"]) {
	    traciclnt.vehicle.changeRoute("veh0", newRoute(currentEdge, true));
	  } else {
	    traciclnt.vehicle.changeRoute("veh0", newRoute(currentEdge, false));
	  }
	  traciclnt.vehicle.moveTo("veh0", (std::get<0>(*it)).c_str(), (double)data["veh0"]["pos"] - (tmplength - std::get<1>(*it)));
	  traciclnt.simulationStep(0);
	} catch(tcpip::SocketException &e) {
	  printf("tcpip: %s\n", e.what());
	}
	break;
      }
    }

    oldData = data;

    //traciclnt.vehicle.moveTo("veh0", "route0", data["veh0"]["pos"]);
    memset(buffer, 0, sizeof(buffer));
  }

  return 0;
}

void sigHandler(int signum) {
  printf("Caught signal %d...\n", signum);
  close(serverSocket);
  exit(1);
}
