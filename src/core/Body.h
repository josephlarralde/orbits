#ifndef Point_h
#define Point_h

#include <vector>
#include <cstdint>

template <std::size_t dimension>
class Point {
private:
  // float d; // global speed multiplier
  // float m; // mass
  // float f; // friction
  
  template <typename T> struct dimensionBounds {
    T min;
    T max;
  };
  
  dimensionBounds<float> defaultDimensionBounds;
  std::vector<dimensionBounds<float>> bounds;
  std::vector<float> position;
  std::vector<float> speed;
  std::vector<float> accel;
  std::vector<float> force;

public:
  Point(std::size_t dim = 2) :
    // d(0.95f),
    // m(0.15f),
    // f(0.f),
    defaultDimensionBounds({ -2000, 2000 }),
    bounds(std::vector<dimensionBounds<float>>(dimension)),
    position(std::vector<float>(dimension, 0)),
    speed(std::vector<float>(dimension, 0)),
    accel(std::vector<float>(dimension, 0)),
    force(std::vector<float>(dimension, 0))
  {
    for (auto& dim : bounds) {
      dim.min = defaultDimensionBounds.min;
      dim.max = defaultDimensionBounds.max;
    }
  }
  
  ~Point() {}
  
  void resetForce() {
    std::fill(force.begin(), force.end(), 0);
    std::fill(speed.begin(), speed.end(), 0);
  }
  
  std::size_t getDimension() const {
    return dimension;
  }
  
  // mass velocity damping :
  // float getD() const {  return d; }
  // void setD(float _d) { d = _d; }
  
  // float getM() const { return m; }
  // void setM(float _m) { m = _m; }
  
  // friction force
  // float getF() const { return f; }
  // void setF(float _f) { f = _f; }
  
  const std::vector<float>& getPosition() const {
    return position;
  }

  void setPosition(std::vector<float>& pos) {
    if (!dimensionIsValid(pos)) return; // throw exception ?
    
    for (std::uint8_t i = 0; i < dimension; ++i) {
      position[i] = pos[i];
      // do something about forces ?
    }
  }

  std::vector<float>& getSpeed() {
    return speed;
  }
  
  float getSpeedNorm() {
    float res = 0;
    for (auto& s : speed) {
      res += s * s;
    }
    return sqrt(res);
  }

  void applyForce(std::vector<float>& f) {
    if (!dimensionIsValid(f)) return; // throw exception ?

    for (std::size_t i = 0; i < dimension; ++i) {
      force[i] += f[i] * m;
    }
  }

  // void update() {
  //   for (std::size_t i = 0; i < dimension; ++i) {
  //     float s = std::min(130.f, std::max(-130.f, speed[i]));
  //     s *= d;
  //     s += force[i];
  //     s += -f * s;
  //     //s *= f;
  //     accel[i] = s - speed[i];
  //     speed[i] = s;
  //     //position[i] += speed[i];
  //     position[i] = clipPosition(position[i] + speed[i], i);
  //     force[i] = 0;
  //   }
  // }
  
  float clipPosition(float v, int d) {
    return std::min(bounds[d].max, std::max(bounds[d].min, v));
  }
    
private:
  bool dimensionIsValid(std::vector<float>& v) {
    return v.size() == dimension;
  }
};

#endif /* Point_h */
