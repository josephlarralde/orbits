#ifndef ORBITS_PHYSICS_H
#define ORBITS_PHYSICS_H

#include "../Orbits.h"

// NB : would attaching avery particle to the origin with a string work for
// inertia simulation ?
// wait ... maybe m, d and f mass parameters are already enough for this, but
// they are not a rule, they are basic physical parameters.
// but this one could make a strange attractor.

// we could also externalize Mass behaviour in a rule, and implement a Spring
// rule as well, and any other physical stuff (just as in pmpd)

namespace Orbits {

template <std::size_t dimension>
class Physics : public Orbits<dimension>::Rule
{
  void process(Tree<dimension>* t) { }
};

} /* end namespace Orbits */

#endif /* ORBITS_PHYSICS_H */