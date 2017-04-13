#ifndef SUMO_HH
#define SUMO_HH

// XXX include order matters for protobuf and traci
// protobuf
#include <track.pb.h>
#include <situation.pb.h>
// traci
#include <utils/traci/TraCIAPI.h>

class SUMO : public TraCIAPI {
public:
  SUMO(protobuf::Track &track);
  ~SUMO();
  void simulationStep(protobuf::Situation &situation);

private:
  float adjustX = 0, adjustY = 0;

  protobuf::Track track;
  float convertYawToAngle(float yaw);
  void moveVehicleToXY(std::string name, float x, float y, float angle);
};

#endif
