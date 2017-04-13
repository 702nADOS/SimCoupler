#ifndef VREP_HH
#define VREP_HH

class VREP {
public:
  VREP();
  ~VREP();
  void simulationStep();

private:
  int clientID = 0;
};

#endif
