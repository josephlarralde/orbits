#include "../src/Orbits.h"
#include "../src/rules/Wandering.h"
#include "../src/rules/Neighbors.h"
#include "../src/rules/Attractors.h"

template <std::size_t dimension>
class Swarm {
  Orbits<dimension> orbits;

  Wandering<dimension> wander;
  Neighbors<dimension> neighbors;
  Attractors<dimension> attract;

public:
  Swarm() {
    // orbits = {};
    orbits.addRule(std::make_shared<Orbits<dimension>::Rule>(wander));
    orbits.addRule(std::make_shared<Orbits<dimension>::Rule>(neighbors));
    orbits.addRule(std::make_shared<Orbits<dimension>::Rule>(attract));
  }

  void setSize(std::size_t s) { orbits.setSize(s); }

  void update() {
    orbits.step();
  }
};