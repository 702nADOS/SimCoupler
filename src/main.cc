#include <boost/asio.hpp>
#include <iostream>
#include <situation.pb.h>
#include <track.pb.h>
#include <utils/traci/TraCIAPI.h>
#include <math.h>

#define SUMO_PORT 2002
#define SUMO_IP "localhost"

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

/* 
 * SpeedDreams2 (SD2)
 *
 */
using namespace boost::asio;
using namespace std;

int main() {
  TraCIClient traciclnt;
  /* starting simulation */
  traciclnt.simulationStep(0);
  
  io_service io_service;
  ip::tcp::endpoint ep(ip::tcp::v4(), 9000);
  ip::tcp::acceptor ac(io_service, ep);

  ip::tcp::socket s(io_service);
  ac.accept(s);

  char data[2048];
  int bread = 0;
  bool once = false;
  while(true) {
    // situation
    uint32_t length;
    read(s, buffer(&length, 4));
    length = ntohl(length);
    bread = read(s, buffer(data, length));
    if (bread != length) cout << "Length mismatch! " << bread << " vs. " << length << endl;
    data[bread] = '\0';
    protobuf::Situation situation;
    situation.ParseFromArray(data, length);
    // track
    read(s, buffer(&length, 4));
    length = ntohl(length);
    bread = read(s, buffer(data, length));
    data[bread] = '\0';
    protobuf::Track track;
    track.ParseFromArray(data, length);

    // for(int i=0; i<track.segments(0).vertex_size(); i++) {
    //   cout << "x: " << track.segments(0).vertex(i).x()
    // 	   << "y: " << track.segments(0).vertex(i).y() << std::endl;
    // }

    cout << situation.vehicles_size() << ": ";
    for(int i=0; i<situation.vehicles_size(); i++) {
      if (i > 0)
    	cout << ", ";
      cout << situation.vehicles(i).name();
    }
    cout << endl;

    if (once == false) {
      traciclnt.vehicle.remove("veh0", 0);
      for(int i=0; i<situation.vehicles_size(); i++) {
	//cout << (string("veh") + std::to_string(i)).c_str() << endl;
	traciclnt.vehicle.add((string("veh") + std::to_string(i)).c_str(), "route0", "Car", "1");
      }
      traciclnt.simulationStep(0);
      once = true;
    }

    TraCIAPI::TraCIPosition start = traciclnt.simulation.convert2D("0to1", 0, 0);
    //cout << "TX: " << start.x << " TY: " << start.y << endl;
    //cout << std::abs(track.segments(0).vertex(0).y() - track.segments(0).vertex(1).y()) / 2 << endl;
    for(int i=0; i<situation.vehicles_size(); i++) {
      protobuf::Vehicle veh = situation.vehicles(i);

      struct {
	float x, y;
      } pos = {
	veh.mutable_position()->x() - (track.segments(0).vertex(0).x() - (float)start.x),
	veh.mutable_position()->y() - (track.segments(0).vertex(0).y() - (float)start.y) + std::abs(track.segments(0).vertex(0).y() - track.segments(0).vertex(1).y()) / 2
      };

      float angle = situation.vehicles(i).yaw() * 180.0 / M_PI;
      if (angle < 0) angle += 360.0;
      if (angle > 270)
	angle = 450 - angle;
      else
	angle = 90 - angle;
      try {
	traciclnt.vehicle.moveToXY((string("veh") + std::to_string(i)).c_str(), "", 0, pos.x, pos.y, angle, 2);
      } catch (tcpip::SocketException &e) {
	traciclnt.vehicle.add((string("veh") + std::to_string(i)).c_str(), "route0", "Car", "9000");
	traciclnt.vehicle.moveToXY((string("veh") + std::to_string(i)).c_str(), "", 0, pos.x, pos.y, angle, 2);
      }
    }
    traciclnt.simulationStep(0);
  }
}
