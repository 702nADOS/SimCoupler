#include <stdlib.h>
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <utils/traci/TraCIAPI.h>
#include "json.hpp"
#include <cmath>

#define SUMO_PORT 2002
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

// TODO currently only one vehicle is considered
#define VEH_NAME "veh0"

void sigHandler(int);

int serverSocket;

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
  std::string laneID = traciclnt.vehicle.getLaneID(VEH_NAME);
  TraCIAPI::TraCIPosition blubb = traciclnt.vehicle.getPosition(VEH_NAME);
  std::cout << "[SUMO] VEH_NAME position: " << blubb.x << "," << blubb.y << std::endl;
  std::cout << "[SUMO] VEH_NAME starts on lane: " << laneID << std::endl;

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
    if (oldData == NULL) {
      oldData = data;
    }

    std::cout << "[SD2] Current (absolute) position on track: " << (double)data[VEH_NAME]["pos"] << std::endl;
    std::cout << "[SD2] Car angle: " << (double)data[VEH_NAME]["angle"] << std::endl;

    if((double)data[VEH_NAME]["pos"] < 0) {
      continue;
    }

    /* find equivalent lane for absolute SD2 position in lanelist of SUMO */
    double tmplength = 0;
    for(std::vector<lane>::iterator it = _laneList.begin(); it != _laneList.end(); ++it) {
      tmplength += std::get<1>(*it);
      if(tmplength > fmod((double)data[VEH_NAME]["pos"], length)) {
        try {
          /* remove vehicle, reason teleportation
           * 0 = NOTIFICATION_TELEPORT
           *
           * http://sumo.dlr.de/wiki/TraCI/Change_Vehicle_State#remove_.280x81.29
           */
          traciclnt.vehicle.remove(VEH_NAME, 0);

          /* add vehicle */
          traciclnt.vehicle.add(VEH_NAME, "route0", "Car", std::to_string(traciclnt.simulation.getCurrentTime()));

          /* gather information for moveToXY
           *
           * edgeID: ID of the edge, the vehicle will be placed on
           * pos: position of the vehicle along the current lane
           * laneID: ID of the lane (currently always 0)
           * angle: Angle as given by SD2
           */
          std::string edgeID = traciclnt.lane.getEdgeID(std::get<0>(*it));
          double pos = fmod((double)data[VEH_NAME]["pos"], length) - (tmplength - std::get<1>(*it));
          int laneID = 0;
          double angle = (double)data[VEH_NAME]["angle"];

          /* angle conversion from SD2 ---> SUMO
           *
           *         90°                           0°
           *
           * 180°           0°/360° ---> -90°            90°
           *
           *        270°                       180°/-180°
           */
          double sangle;
          if (angle > 270)
            sangle = 450 - angle;
          else
            sangle = 90 - angle;

          /* conversion of "along the lane" position to 2D-Position
           *
           * TraCIPosition = struct{ double x, double y };
           */
	  std::cout << "Translating edgeID (" << edgeID << "), pos (" << pos << ") and laneID (" << laneID << ") into tdpos" << std::endl;
          TraCIAPI::TraCIPosition tdpos = traciclnt.simulation.convert2D(edgeID, pos, laneID);
	  std::cout << "tdpos is (" << tdpos.x << "," << tdpos.y << ")" << std::endl;
	  /* getting (x,y) of first track segment in sumo
	   *
	   * SUMO:     	SD2:
	   * --------  	+-------
	   * +
	   * --------	--------
	   */
	  TraCIAPI::TraCIPosition start = traciclnt.simulation.convert2D("0to1", 0, 0);
	  double width = traciclnt.lane.getWidth(std::get<0>(*it));
	  struct {
	    double x, y;
	  } newPos = {
	    (double)data[VEH_NAME]["x"] - ((double)data[VEH_NAME]["fsX"] - start.x),
	    (double)data[VEH_NAME]["y"] - ((double)data[VEH_NAME]["fsY"] - start.y) + width / 2
	  };

          /* move the vehicle to the equivalent position and set angle */
	  traciclnt.vehicle.moveToXY(VEH_NAME, edgeID, laneID, newPos.x, newPos.y, sangle, true);

          std::cout << "[SUMO] Moved vehicle " << VEH_NAME << " to (" << newPos.x << "," << newPos.y << ") with an angle of " << sangle << "°" << std::endl;

	  std::cout << "[SD2] (" << (double)data[VEH_NAME]["x"] << "," << (double)data[VEH_NAME]["y"] << "," << (double)data[VEH_NAME]["z"] << ")" << std::endl;
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
