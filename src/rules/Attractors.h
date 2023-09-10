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

namespace Orbits {

// BASE ////////////////////////////////////////////////////////////////////////

template <std::size_t dimension>
class Attractor {
public:
  std::vector<float> position;

  Attractor() :
    position(std::vector<float>(dimension, 0)) {}
};

// SIMPLE //////////////////////////////////////////////////////////////////////

template <std::size_t dimension>
class SimpleAttractor : public Attractor<dimension> {
public:
  float radius;
  float radiusForce;
  float centerForce;
  float centerToRadiusCurveFactor; // exponent to apply to interpolation value

public:
  SimpleAttractor() : Attractor<dimension>(),
    radius(1),
    radiusForce(1),
    centerForce(0),
    centerToRadiusCurveFactor(1) {}
};

// MEMBRANE ////////////////////////////////////////////////////////////////////

template<std::size_t dimension>
class MembraneAttractor : public Attractor<dimension> {
public:
  mmm radius;
};

// SET OF ATTRACTORS ///////////////////////////////////////////////////////////

template <std::size_t dimension>
class Attractors : public Orbits<dimension>::Rule {
  std::vector<SimpleAttractor<dimension>> attractors;

public:
  // this should work (with a few adjustments)
  void process(Tree<dimension>* t) {
    std::vector<float> res(dimension, 0);

    for (auto& a : attractors) {
      auto neigh = t->getNeighborsAndDistances(a.position, a.radius);
      
      for (auto& [ mass, dist ] : neigh) { // massPtr, distance to mass
        auto& position = mass->getPosition();
        // if ratio is 0 we use nearForce, if it is 1 then farForce, otherwise
        // we interpolate
        float ratio = a.radius - dist;


        for (auto d = 0; d < dimension; ++d) {
          res[d] = (position[d] - a.position[d]) / dist;
        }

        for (auto d = 0; d < dimension; ++d) {
          // scale to attraction / repulsion range
        }        
        mass->applyForce(res);
      }
    }
  }

  void addAttractor() {
    attractors.push_back()
  }
};

}; // end of namespace orbits

#endif /* ORBITS_ATTRACTORS_H */
