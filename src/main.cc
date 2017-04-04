#include <sd2.hh>
#include <iostream>

#include <track.pb.h>

int main(int argc, char *argv[]) {
  SD2 sd2;
  protobuf::Track track = sd2.getTrack();
  std::cout << "We got a connection!" << std::endl;
  std::cout << track.segments(0).vertex(0).x() << std::endl;
  while(true) {
    // data acquisition
    // forward data
    // control for next simulation step
  }
}
