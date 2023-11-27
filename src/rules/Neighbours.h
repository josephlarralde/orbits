#ifndef ORBITS_NEIGHBOURS_H
#define ORBITS_NEIGHBOURS_H

#include "../Orbits.h"

// this one could implement pmpd's iCircle<dimension>D behaviour,
// but it is more broad because this is where we find the closest neighbours
// and apply whatever force we want.
// maybe we could break this up into several rules, but then we would have to
// compute the closest neighbours multiple times

// NB : we could act on (and/)or react to neighbors, it is the same
// thing from a different perspective, so we decide to react to them

// UPDATE : after watching a very enlightening video explanation of the
// n nearest neighbours influence model (which seems more realistic than a
// radius-based model), we choose to implement this principle.
// We might use a strategy pattern here (the radius based approach could still
// make sense in certain situations, and there is also another "vision-cone"
// based approach which seems even more realistic but more computationally
// expensive ...)

namespace Orbits {

template <std::size_t dimension>
class Neighbours : public Orbits<dimension>::Rule
{
  std::size_t numberOfNearestNeighbors;
  float maximumRadius;

public:
  Neighbours() :
    numberOfNearestNeighbors(6),
    maximumRadius(1e+3f)
  {

  }

  void setNumberOfNearestNeighbors(std::size_t n) { numberOfNearestNeighbors = n; }

  void setMaximumRadius(float r) { maximumRadius = r; }

  void match(float amount, float r) { }

  void avoid(float amount, float r) { }

  void processParticles(Tree<dimension>* t,
                        std::vector<std::shared_ptr<Mass<dimension>>>& v,
                        float dt) override
  {
    // todo
  }
};

} /* end namespace Orbits */

#endif /* ORBITS_NEIGHBOURS_H */
