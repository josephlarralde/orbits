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

namespace Orbits {

template <std::size_t dimension>
class Neighbours : public Orbits<dimension>::Rule
{
  float radius;

public:
  void process(Tree<dimension>* t) override { }

  void setRadius(float r) { radius = r; }

  void match(float amount, float r) { }

  void avoid(float amount, float r) { }
};

} /* end namespace Orbits */

#endif /* ORBITS_NEIGHBOURS_H */
