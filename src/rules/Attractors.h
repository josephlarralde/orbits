#ifndef ORBITS_ATTRACTORS_H
#define ORBITS_ATTRACTORS_H

#include "../Orbits.h"

// for now we will make simple : no tangent forces which would become
// hard to visualize if dimension > 3 (or not so much ? -> todo : check)
// also see : hairy ball theorem (sounds interesting)
// ideally we would have this : a vector computed from the normal to the
// sphere, but the hairy ball theorem says we can't have a uniform field, we
// will have some "spikes", even in dimension 3, so it might be the same
// in higher dimensions.

// More complex attractors will have to be defined for specific dimensions
// float nearForce[dimension];
// float farForce[dimension];

// for dimension 3 we can define the force to apply as a function of the
// spherical coordinates from the mass to the centre (azimuth/elevation/distance)
// returning a vector that should be added to the normal mass vector,
// this way we could easily "comb" vector hair along the surface


class Attractor {

};

template <std::size_t dimension>
class SimpleAttractor : public Attractor {
  std::vector<float> position;
  float radius;
  float nearForce;
  float farForce;

public:
  SimpleAttractor() :
    position(std::vector<float>(dimension, 0)),
    radius(1),
    nearForce(1),
    farForce(0) {}
};

template <std::size_t dimension>
class Attractors : public Orbits<dimension>::Rule {
  std::vector<SimpleAttractor<dimension>> attractors;

public:
  void process(Tree<dimension>* t) {
    for (auto& attractor : attractors)
    // t->getNeighborsAndDistances(attractor);
  }

  void addAttractor() {
    attractors.push_back()
  }
};

#endif /* ORBITS_ATTRACTORS_H */
