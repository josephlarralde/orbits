#ifndef Spring_h
#define Spring_h

#include <cmath>
#include "Mass.h"

template <std::uint8_t dimension>
class Spring {
private:
  //std::uint8_t dimension;
  float k; // tension
  float d; // damping
  float minLength;
  float maxLength;
  
  typedef std::shared_ptr<Mass<dimension>> MassPtr;
  std::pair<MassPtr, MassPtr> masses;
  
  //std::pair<std::shared_ptr<Mass>, std::shared_ptr<Mass>> masses;
    
public:
  Spring(std::uint8_t dim = 2) :
  //dimension(dim),
  k(1.5f),
  d(0.f),
  minLength(0),
  maxLength(200),
  masses({ nullptr, nullptr }) {}
  
  ~Spring() {}
  
  void setMasses(
    //std::pair<std::shared_ptr<Mass<dimension>>,
    //          std::shared_ptr<Mass<dimension>>> m
    std::pair<MassPtr, MassPtr> m
  ) {
    /*
    // OBSOLETE DUE TO USAGE OF dimension TEMPLATE PARAMETER
    // MassPtr can only be nullptr or point to a Mass of dimension "dimension" :
      
    if ((m.first->getDimension() == dimension || m.first == nullptr) &&
        (m.second->getDimension() == dimension || m.second == nullptr)) {
        masses.first = m.first;
        masses.second = m.second;
    } else {
        // throw exception ?
        masses.first = nullptr;
        masses.second = nullptr;
    }
    //*/
    masses.first = m.first;
    masses.second = m.second;
  }
  
  float getK() { return k; }
  void setK(float _k) { k = _k; }

  // damping should be normalized by the critical damping value
  // critical damping formula : Cc = 2 * sqrt(k * m)
  // from where to apply this : to a spring ? to a mass ? makes more sense to a spring
  // because a mass doesn't have a "k" value
  float getD() { return d; }
  void setD(float _d) { d = _d; }
  
  void update() {
    if (!massesAreSet()) return; // throw exception ?
            
    applyForce(masses.first, masses.second);
    applyForce(masses.second, masses.first);
  }
    
private:
  bool massesAreSet() {
    return (masses.first != nullptr && masses.second != nullptr);
  }
  
  void applyForce(MassPtr massToApplyForceTo,
                  MassPtr otherMass) {
      
    float distance = 0;
    std::vector<float> distances(dimension, 0);

    for (std::uint8_t i = 0; i < dimension; ++i) {
      distances[i] =
        massToApplyForceTo->getPosition()[i] -
        otherMass->getPosition()[i];
      distance += (distances[i] * distances[i]);
    }
    
    distance = sqrt(distance);
    float ratio = fmin(1.f, fabs(distance) / maxLength);
    
    std::vector<float> force(dimension);

    for (std::uint8_t i = 0; i < dimension; ++i) {
      distance = distances[i] * ratio;
      force[i] = -k * distance;      
    }
    
    massToApplyForceTo->applyForce(force);
  }
};

#endif /* Spring_h */
