#ifndef Mass_h
#define Mass_h

#include <vector>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include "./Utilities.h"

#define MAX_FORCE 100       // min is 0
#define MAX_SPEED 340       // speed of sound :), min is also 0
#define MIN_MASS_VALUE 1e-3 // a gram
#define MAX_MASS_VALUE 1e3  // a ton

// base physical mass simulation
// use standard units :
// * force : in N (newtons)
// * mass : in kg (kilograms)
// * position : m (meters)
// * time : s (seconds)

namespace Orbits {

// was inside Mass declaration / definition
template <typename T> struct dimensionBounds {
  T min;
  T max;
};

struct mmm { // 1-dimensional min, mid and max
  float min;
  float mid;
  float max;
};

template <std::size_t dimension>
class Mass {
private:
  float mass; // mass
  float friction; // friction

  dimensionBounds<float> defaultDimensionBounds;
  std::vector<dimensionBounds<float>> bounds;
  std::vector<float> position;
  std::vector<float> force;
  std::vector<float> accel;

  std::vector<float> speed;           // a.k.a velocity
  std::vector<float> normalizedSpeed; // normalized speed vector
  float speedNorm;                    // length of speed vector


public:
  Mass() :
  mass(0.15f),
  friction(0.f),
  defaultDimensionBounds({ -2000, 2000 }),
  bounds(std::vector<dimensionBounds<float>>(dimension)),
  position(std::vector<float>(dimension, 0)),
  force(std::vector<float>(dimension, 0)),
  accel(std::vector<float>(dimension, 0)),
  speed(std::vector<float>(dimension, 0)),
  normalizedSpeed(std::vector<float>(dimension, 0)),
  speedNorm(0)
  {
    for (auto& dim : bounds) {
      dim.min = defaultDimensionBounds.min;
      dim.max = defaultDimensionBounds.max;
    }
  }
  
  ~Mass() {}
  
  void resetForce() {
    std::fill(force.begin(), force.end(), 0);
    std::fill(speed.begin(), speed.end(), 0);
  }

  // smells like old remains, should be removed
  std::size_t getDimension() const {
    return dimension;
  }

  // mass in kilograms
  float getMass() const { return mass; }

  void setMass(float m) {
    mass = std::clamp<float>(m, MIN_MASS_VALUE, MAX_MASS_VALUE);
  }
  
  // friction coefficient
  float getFriction() const { return friction; }

  void setFriction(float f) {
    friction = std::clamp<float>(f, 0, 1);
  }
  
  const std::vector<float>& getPosition() const { return position; }

  void setPosition(std::vector<float>& p) {
    if (!dimensionIsValid(p)) return;
    // do something about forces ? speed ? reset them or not ?
    // => depends on where this method will be called from
    position = p;
    // for (std::size_t i = 0; i < dimension; ++i) {
    //   position[i] = p[i];
    // }
  }

  const std::vector<float>& getSpeed() { return speed; }

  void setSpeed(std::vector<float>& s) {
    if (!dimensionIsValid(s)) return;
    speed = s;
    // for (std::size_t i = 0; i < dimension; ++i) {
    //   speed[i] = s[i];
    // }
  }

  float getSpeedNorm() { return speedNorm; }

  const std::vector<float>& getNormalizedSpeed() { return normalizedSpeed; }

  void applyForce(std::vector<float>& f) {
    if (!dimensionIsValid(f)) return; // throw exception ?

    for (std::size_t i = 0; i < dimension; ++i) force[i] += f[i];
  }

  /**
   * @param dt time since last call to update (<=> sampling period if constant across iterations)
   * @brief core of the Mass class, mostly inspired from www.red3d.com/cwr/steer/gdc99/
   */
  void update(float dt) {
    // clip force vector norm to MAX_FORCE
    const float forceNorm = computeVectorNorm(force);
    if (forceNorm > MAX_FORCE) scaleVector(force, MAX_FORCE / forceNorm);

    for (std::size_t i = 0; i < dimension; ++i) {
      // add simple (non physically realistic) friction coefficient
      force[i] *= -friction;
      // then compute acceleration, from Newton's second law : F=m.a <=> a=F/m
      accel[i] = force[i] / mass;
      // then, a = ds/dt <=> ds = a.dt <=> s = a.dt + prev_s
      speed[i] += accel[i] * dt;
    }

    // update speedNorm and clip speed vector
    speedNorm = computeVectorNorm(speed);
    if (speedNorm > MAX_SPEED) scaleVector(speed, MAX_SPEED / speedNorm);

    // update normalizedSpeed
    normalizedSpeed = speed;
    computeScaledVector(normalizedSpeed, 1 / speedNorm);

    // update position from new speed
    for (std::size_t i = 0; i < dimension; ++i) position[i] += speed[i];

    // reset forces for next iteration
    std::fill(force.begin(), force.end(), 0);
  }
  
private:
  bool dimensionIsValid(std::vector<float>& v) {
    return v.size() == dimension;
  }
};

}; // end of namespace Orbits

#endif /* Mass_h */
