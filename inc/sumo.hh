#ifndef SUMO_HH
#define SUMO_HH

// XXX include order matters for protobuf and traci
// protobuf
#include <track.pb.h>
// traci
#include <utils/traci/TraCIAPI.h>

class SUMO : public TraCIAPI {
public:
  SUMO(protobuf::Track &track);
  ~SUMO();
  void simulationStep();

private:
  protobuf::Track track;
  float convertYawToAngle(float yaw);
};

#endif
