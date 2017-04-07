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

    sumo.simulationStep(situation);
  }
}
