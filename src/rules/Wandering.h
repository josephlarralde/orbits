#ifndef ORBITS_WANDERING_H
#define ORBITS_WANDERING_H

#include "../Orbits.h"

template <std::size_t dimension>
class Wandering : public Orbits<dimension>::Rule {


public:
  void setSize(std::size_t s) {
    //for (auto& p : particles) {}
  }

  setSpeed(float minSpeed, float maxSpeed, float frequency) {

  }

  setAngle(float minAngle, float maxAngle, float frequency) {
    
  }

  void process(Tree<dimension>* t) {

  }
};

#endif /* ORBITS_WANDERING_H */
