#ifndef ORBITS_WANDERING_H
#define ORBITS_WANDERING_H

#include "../Orbits.h"

// idea for a generic multidimensional algorithm :

// 1. make various oscillators control an increment value per dimension,
// then normalize the corresponding vector and apply a speed factor to it
// (also potentially driven by an oscillator).
// like this we have a continuously varying multidimensional vector, and we
// control the speed independently.

// 2. do the same but don't normalize the vector and don't use a global speed
// factor (and eventually clip the vector to a range)

// 3. the new wandering vector could also be computed from the current speed
// vector, we would then apply the increment values directly on it

// are there enough cases for a strategy pattern ? or a best case ?

template <std::size_t dimension>
class Wandering : public Orbits<dimension>::Rule {
  std::vector<float> frequencies;

public:
  void setSize(std::size_t s) {
    
  }

  void setSampleRate(std::uint16_t sr) {
    // rampleRate = sr;
  }

  // set each dimension's increment variation frequency
  void setFrequencies(std::vector<float>& f) {

  }

  // set each dimension's increment variation range
  void setRanges(std::vector<std::pair<float, float>>& r) {

  }

  void setGlobalSpeedOn(bool on) {
    // if set, normalize the wandering vector and appply the global speed factor
  }

  void setGlobalSpeedFrequency(float f) {

  }

  void setGlobalSpeedRange(std::pair<float, float>& r) {

  }

  // void setAngle(float minAngle, float maxAngle, float frequency) {
    
  // }

  void process(Tree<dimension>* t) {
    // for (auto& p : t->particles) {}
  }
};

#endif /* ORBITS_WANDERING_H */
