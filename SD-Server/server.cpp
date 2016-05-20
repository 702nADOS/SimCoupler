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
  /* starting simulation */
  traciclnt.simulationStep(0);

  signal(SIGINT, sigHandler);

  /* prepare network stuff */
  int serverSocket, c, clientSocket;
  char buffer[BUFFER_SIZE];
  struct sockaddr_in serverAddr, clientAddr;

  serverSocket = socket(PF_INET, SOCK_STREAM, 0);

  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(PORT);
  serverAddr.sin_addr.s_addr = inet_addr("127.0.0.1");
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

  /* get starting lane for vehicle */
  std::string laneID = traciclnt.vehicle.getLaneID("veh0");
  std::cout << "[SUMO] veh0 starts on lane: " << laneID << std::endl;

  /* get successor lanes of current lane
   * and insert them into a laneList
   *
   * TODO
   * we assume there is only one lane per edge
   * and therefore only one successor
   */
  typedef std::tuple<std::string, double> lane;
  std::vector<lane> _laneList;
  _laneList.push_back(lane(laneID, traciclnt.lane.getLength(laneID)));

  std::vector<std::string> links = traciclnt.lane.getLinks(laneID);
  while(links[0] != laneID) {
    _laneList.push_back(lane(links[0], traciclnt.lane.getLength(links[0])));
    links = traciclnt.lane.getLinks(links[0]);
  }

  /* print _laneList and determine length*/
  double length = 0.0;
  
  std::cout << "Elements of _laneList:" << std::endl;
  for(std::vector<lane>::iterator it = _laneList.begin(); it != _laneList.end(); ++it) {
    length += std::get<1>(*it);
    std::cout << std::get<0>(*it) << " with length " << std::get<1>(*it) << " m" << std::endl;
  }
  std::cout << "Total length: " << length << std::endl;

  /* oldData buffer for the last simulation step */
  json oldData = NULL;

  while( recv(clientSocket, buffer, BUFFER_SIZE, 0) ) {
    json data = json::parse(buffer);
    /* there is no oldData for the first simulation step */
    if (oldData == NULL) oldData = data;

    std::cout << "[SD2] Current (absolute) position on track: " << (double)data["veh0"]["pos"] << std::endl;
    std::cout << "[SD2] Car angle: " << (double)data["veh0"]["angle"] << std::endl;

    if((double)data["veh0"]["pos"] < 0) {
      continue;
    }

    /* find equivalent lane for absolute SD2 position in lanelist of SUMO */
    double tmplength = 0;
    for(std::vector<lane>::iterator it = _laneList.begin(); it != _laneList.end(); ++it) {
      tmplength += std::get<1>(*it);
      if(tmplength > fmod((double)data["veh0"]["pos"], length)) {
	try {
	  traciclnt.vehicle.remove("veh0", 0);
	  traciclnt.vehicle.add("veh0", "Car", "route0", -2, -2, 0, 0);
	  traciclnt.vehicle.moveTo("veh0", std::get<0>(*it), fmod((double)data["veh0"]["pos"], length) - (tmplength - std::get<1>(*it)));
	  break;
	}
	catch(tcpip::SocketException &e) {
	  std::cout << "[tcpip] " << e.what() << std::endl;
	}
      }
    }

    /* replace old data */
    oldData = data;

    /* jump to next simulation step */
    traciclnt.simulationStep(0);

    /* clear buffer */
    memset(buffer, 0, sizeof(buffer));
  }

  return 0;
}

void sigHandler(int signum) {
  printf("Caught signal %d...\n", signum);
  close(serverSocket);
  exit(1);
}
