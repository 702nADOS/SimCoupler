#include <sd2.hh>
#include <sumo.hh>

#include <iostream>

#include <track.pb.h>

int main(int argc, char *argv[]) {
  SD2 sd2;
  protobuf::Track track = sd2.getTrack();
  SUMO sumo(track);

  protobuf::Situation situation;

  while(true) {
    sd2.simulationStep();
    situation = sd2.getCurrentSituation();

    for(int i=0; i<situation.vehicles_size(); i++) {
      std::cout << situation.vehicles(i).name() << std::endl;
    }
    // data acquisition
    // forward data
    // control for next simulation step
  }
}
