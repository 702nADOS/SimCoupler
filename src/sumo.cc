#include <sumo.hh>

// traci
#include <utils/traci/TraCIAPI.h>
// protobuf
#include <track.pb.h>
//
#include <math.h>

#define SUMO_IP "localhost"
#define SUMO_PORT 2002

/* TraCIAPI client library: http://sumo.dlr.de/wiki/TraCI/C%2B%2BTraCIAPI */

SUMO::SUMO(protobuf::Track &track) {
  this->track = track;

  try {
    TraCIAPI::connect(SUMO_IP, SUMO_PORT);
  } catch (tcpip::SocketException &e) {
    // TODO logging
    std::cout << "TraCIAPI: " << e.what() << std::endl;
  }
};

SUMO::~SUMO() {
  try {
    TraCIAPI::close();
  } catch (tcpip::SocketException &e) {
    // TODO logging
    std::cout << "TraCIAPI: " << e.what() << std::endl;
  }
};

float SUMO::convertYawToAngle(float yaw) {
  float angle = yaw * 180.0 / M_PI;
  if (angle < 0) angle += 360.0;
  if (angle > 270)
  angle = 450 - angle;
  else
  angle = 90 - angle;
};
