#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <utils/traci/TraCIAPI.h>
#include "json.hpp"

#define SUMO_PORT 2001
#define SUMO_IP "localhost"

// SUMO
class TraCIClient : public TraCIAPI {
public:
  TraCIClient();
};

TraCIClient::TraCIClient() {
  try {
    connect(SUMO_IP, SUMO_PORT);
  } catch(tcpip::SocketException &e) {
    printf("tcpip: %s\n", e.what());
  }
}

// for convenience
using json = nlohmann::json;

#define PORT 2000
#define BUFFER_SIZE 1024
#define MAX_ROUTE_ELEMENTS 10

void sigHandler(int);

int serverSocket;

/**
 * Builds a new route for SUMO
 * The number of elements in the route is defined by MAX_ROUTE_ELEMENTS
 *
 * @param name of the current edge the vehicle is standing on
 * @param false if the vehicle is driving forwards
 **/
std::vector<std::string> newRoute(std::vector<std::string> edgeList, std::string edge, bool backwards) {
  // create new route list
  std::vector<std::string> route;

  // find index of current edge
  std::vector<std::string>::iterator index = std::find(edgeList.begin(), edgeList.end(), edge);

  // add elements to route
  if(backwards) { //backwards
    for(std::vector<std::string>::iterator it = index; route.size() < MAX_ROUTE_ELEMENTS; --it) {
      route.push_back(*it);
      std::cout << "Pushed " << *it << " to route." << std::endl;
      if(it == edgeList.begin()) {
	it = edgeList.end();
      }
    }
  } else { //forwards
    for(std::vector<std::string>::iterator it = index; route.size() < MAX_ROUTE_ELEMENTS; ++it) {
      route.push_back(*it);
      std::cout << "Pushed " << *it << " to route." << std::endl;
      if(it == edgeList.end()-1 && route.size() < MAX_ROUTE_ELEMENTS) {
	it = edgeList.begin();
	route.push_back(*it);
	std::cout << "Pushed " << *it << " to route." << std::endl;
      }
    }
  }

  // return new route
  std::cout << "X -> ";
  for(std::vector<std::string>::iterator it = route.begin(); it != route.end(); ++it) {
    std::cout << *it << " -> ";
  }
  std::cout << "X" << std::endl;
  return route;
}

int main(int argc , char *argv[]) {
  TraCIClient traciclnt;

  /* querying routes. */
  std::vector<std::string> routeList = traciclnt.route.getIDList();
  /* querying edges, assuming routeList only contains one route
   * with all edges as a circuit.
   * TODO proper handling
   */
  std::vector<std::string> edgeList = traciclnt.route.getEdges(routeList[0]);
  /* querying lanes. */
  std::vector<std::string> laneList = traciclnt.lane.getIDList();


  /* determine length by iterating over the lanes */
  double _length = 0;
  for(std::vector<std::string>::iterator it = laneList.begin(); it != laneList.end(); ++it) {
    /* "remove" filler lanes */
    if ((*it)[0] == ':')
      continue;

    std::cout << "Current lane: " << *it << std::endl;

    std::string edgeID = traciclnt.lane.getEdgeID(*it);
    double laneLength = traciclnt.lane.getLength(*it);
    std::cout << *it << " from edge " << edgeID << " with " << laneLength << "m length." << std::endl;
    _length += laneLength;
  };

  std::cout << "Complete length: " << _length << std::endl;

  typedef std::tuple<std::string, double> lane;

  /*
   * legacy stuff here
   */
  try {
    edgeList = traciclnt.route.getEdges("route0");
  } catch(tcpip::SocketException &e) {
    printf("[Error] tcpip: %s\n", e.what());
  }

  std::vector<lane> llist(0);
  for(std::vector<std::string>::iterator it = edgeList.begin(); it != edgeList.end(); ++it) {
    printf("Edge: %s\n", (*it).c_str());
    llist.push_back(std::make_tuple(*it + "_0", traciclnt.lane.getLength((*it + "_0"))));
  }

  double length = 0;
  for(std::vector<lane>::iterator it = llist.begin(); it != llist.end(); ++it) {
    printf("Edge: %s with %f meter.\n", std::get<0>(*it).c_str(), std::get<1>(*it));
    length += std::get<1>(*it);
  };
  std::cout << "Total length: " << length << "\nBy " << llist.size() << " lanes\n";

  traciclnt.simulationStep(0);

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
  double sd2_length = 0;

  int count = 1;
  std::string _route = "route" + std::to_string(count);
  
  while( recv(clientSocket, buffer, BUFFER_SIZE, 0) ) {
    json data = json::parse(buffer);
    sd2_length = (double)data["veh0"]["trackLength"];
    if (oldData == NULL) oldData = data;

    double tmplength = 0;
    printf("Current position: %f\n", (double)data["veh0"]["pos"]);
    for (std::vector<lane>::iterator it = llist.begin(); it != llist.end(); ++it) {
      tmplength += std::get<1>(*it);
      printf("Position: %f\n", fmod((double)data["veh0"]["pos"], length));
      if (tmplength > fmod((double)data["veh0"]["pos"], length) && data["veh0"]["pos"] >= 0) {
	//printf("Lane: %s, Pos: %f\n", std::get<0>(*it).c_str(), (double)data["veh0"]["pos"] - (tmplength - std::get<1>(*it)));
	try {
	  std::string currentEdge = traciclnt.vehicle.getRoadID(traciclnt.vehicle.getIDList()[0]);
	  std::cout << "Current edge: " << currentEdge << std::endl;

	  std::cout << "route: " << _route << std::endl;
	  
	  if ((double)data["veh0"]["pos"] < (double)oldData["veh0"]["pos"]) {
	    //traciclnt.vehicle.changeRoute(traciclnt.vehicle.getIDList()[0], newRoute(edgeList, currentEdge, true));
	    traciclnt.route.add(_route, newRoute(edgeList, currentEdge, true));
	  } else {
	    //traciclnt.vehicle.changeRoute(traciclnt.vehicle.getIDList()[0], newRoute(edgeList, currentEdge, false));
	    traciclnt.route.add(_route, newRoute(edgeList, currentEdge, false));
	  }
	  //traciclnt.vehicle.changeRouteID("veh0", _route.c_str());
	  //traciclnt.vehicle.reroute("veh0");
	  //traciclnt.vehicle.changeRouteID("veh0", "route0");
	  //std::cout << "ls vehicle: " << traciclnt.vehicle.getIDList()[0] << std::endl;
	  traciclnt.vehicle.remove(traciclnt.vehicle.getIDList()[0], 0);
	  traciclnt.vehicle.add("veh0", "Car", _route, -2, -2, 0, 0);
	  traciclnt.vehicle.moveTo("veh0", std::get<0>(*it), (double)data["veh0"]["pos"] - (tmplength - std::get<1>(*it)));
	  traciclnt.simulationStep(0);
	  count++;
	  _route = "route" + std::to_string(count);
	  std::cout << "route: " << _route << std::endl;

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
