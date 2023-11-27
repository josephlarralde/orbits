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
class Attractor : public Orbits<dimension>::Rule {
  std::vector<float> position;

public:
  Attractor() :
    Orbits<dimension>::Rule(),
    position(std::vector<float>(dimension, 0.f))
  {
    this->addParameter("position", [&](const std::vector<float>& v) {
      if (v.size() != dimension) return;
      position = v;
    });
  }

  virtual void setPosition(const std::vector<float>& p) { position = p; }
  virtual const std::vector<float>& getPosition() { return position; }
};

// SIMPLE //////////////////////////////////////////////////////////////////////

template <std::size_t dimension>
class SimpleAttractor : public Attractor<dimension> {
public:
  float radius;
  float radiusForce;
  float centerForce;
  float deltaForce;
  float centerToRadiusCurveFactor; // exponent to apply to interpolation value

public:
  SimpleAttractor() :
    Attractor<dimension>(),
    radius(1.f),
    radiusForce(1.f),
    centerForce(0.f),
    deltaForce(radiusForce - centerForce),
    centerToRadiusCurveFactor(1.f)
  {
    this->addParameter("radius", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      setRadius(v[0]);
    });

    this->addParameter("radiusForce", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      setRadiusForce(v[0]);
    });

    this->addParameter("centerForce", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      setCenterForce(v[0]);
    });

    this->addParameter("centerToRadiusCurveFactor", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      setCenterToRadiusCurveFactor(v[0]);
    });
  }

  void setRadius(float f) { radius = f; }

  void setRadiusForce(float f) {
    radiusForce = f;
    deltaForce = radiusForce - centerForce;
  }

  void setCenterForce(float f) {
    centerForce = f;
    deltaForce = radiusForce - centerForce;
  }

  void setCenterToRadiusCurveFactor(float f) {
    centerToRadiusCurveFactor = f;
  }

  void processParticles(Tree<dimension>* t,
                        std::vector<std::shared_ptr<Mass<dimension>>>& v,
                        float dt) override
  {
    auto neigh = t->getNeighborsAndDistances(this->getPosition(), radius);

    for (auto [ m, d ] : neigh) {
      float curve = powf(d / radius, centerToRadiusCurveFactor);
      m->applyForce(curve * deltaForce + centerForce);
    }
  }
};

// MEMBRANE ////////////////////////////////////////////////////////////////////

template<std::size_t dimension>
class MembraneAttractor : public Attractor<dimension> {
  mmm radii; // closest to centre, membrane radius, farthest from centre
  mmm forces;

public:
  MembraneAttractor() :
    Attractor<dimension>(),
    radii({0.f, 1.f, 2.f}),
    forces({0.f, 0.f, 0.f})
  {
    // todo
  }
};

// SET OF ATTRACTORS ///////////////////////////////////////////////////////////

/*
template <std::size_t dimension>
class Attractors : public Orbits<dimension>::Rule {
  std::vector<SimpleAttractor<dimension>> attractors;

public:
  // this should work (with a few adjustments)
  void processParticlesTree(Tree<dimension>* t) {
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

  void addAttractor(std::shared_ptr<SimpleAttractor<dimension>> a) {
    attractors.push_back(a);
  }
};
//*/

}; /* end namespace orbits */

#endif /* ORBITS_ATTRACTORS_H */
