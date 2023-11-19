#ifndef ORBITS_WANDERING_H
#define ORBITS_WANDERING_H

#include "../Orbits.h"

// ideas for a generic multidimensional algorithm :

// 1. make various oscillators control an increment value per dimension,
// then normalize the corresponding vector and apply a speed factor to it
// (also driven by an oscillator).
// like this we have a continuously varying multidimensional vector, and we
// control the speed independently.

// 2. do the same but don't normalize the vector and don't use a global speed
// factor (and eventually clip the vector to a range)

// 3. the new wandering vector could also be computed from the current speed
// vector, we would then apply the increment values directly on it

// are there enough cases for a strategy pattern ? or a best case ?
// -> a strategy pattern seems pertinent here (use random, simple oscillators
//    or complex chains of oscillators as "brownian" movement generators),
//    although a template based solution might be just as fine.

// an intuitive description of the brownian movement is :
// "* between two shocks, the particle's trajectory is a straight line and moves
//    at constant speed
//  * the particle is accelerated (i.e. its speed changes) when it bounces on
//    another molecule"

// this could be achieved by sparsely impulsing random forces on the particles
// and let them drift until the next one, but would be very dependent on the
// friction and mass settings to achieve the desired behaviour ...

namespace Orbits
{

class DimensionWanderer {

};

template <std::size_t dimension>
class MassWanderer {

};

template <std::size_t dimension>
class Wandering : public Orbits<dimension>::Rule
{
  // BEGIN MASS WANDERER DEFINITION ////////////////////////////////////////////
  class MassWanderer {

    //--------------------------------------------------------------------------
    class DimensionWanderer {
      float increment;
      dimensionBounds<float> bounds;
      float width;
      float value;

    public:
      DimensionWanderer() :
      increment(0.f),
      bounds({ 0.f, 0.f }),
      width(0.f),
      value(0.f) {}

      void setIncrement(float inc) { increment = inc; }

      void setBounds(dimensionBounds<float> dimBounds) {
        bounds = dimBounds;
        width = bounds.max - bounds.min;
      }

      void step() {
        value += increment;
        if (value < 0.f) {
          value = fmod(1.f - value, 1.f);
        } else if (value > 1.f) {
          value = fmod(value, 1.f);
        }
      }

      float getValue() {
        // should use wave table for sin instead of math function !!!
        return (std::sin(M_2_PI) * 0.5f + 0.5f) * width + bounds.min;
      }
    };
    //--------------------------------------------------------------------------

    std::size_t samplerate;
    std::vector<DimensionWanderer> dimensionWanderers;

  public:
    MassWanderer() :
    samplerate(1.f),
    dimensionWanderers(dimension + 1) {}

    // void setSamplerate(std::size_t sr) { samplerate = sr; }

    void step() { for (auto& w : dimensionWanderers) w.step(); }

    void updateState(
      const std::vector<float>& freqs,
      const std::vector<float>& rfreqs,
      const std::vector<dimensionBounds<float>>& ranges,
      const std::vector<dimensionBounds<float>>& rranges
    ) {
      std::srand(std::time(0));
      for (auto i = 0; i < dimension + 1; ++i) {
        dimensionWanderers[i].setIncrement(
          (freqs[i] + rfreqs[i] * (std::rand() / RAND_MAX)) / samplerate
        );
        dimensionWanderers[i].setBounds({
          ranges[i].min + rranges[i].min * (std::rand() / RAND_MAX),
          ranges[i].max + rranges[i].max * (std::rand() / RAND_MAX)
        });
      }
    }

    std::vector<float> getForceToApply(float amount = 1.f) {
      std::vector<float> res(dimension);
      float norm = 0.f;
      for (auto i = 0; i < dimension; ++i) {
        res[i] = dimensionWanderers[i].getValue();
        norm += res[i] * res[i];
      }
      norm = std::sqrtf(norm);
      float speed = dimensionWanderers[dimension].getValue();
      for (auto i = 0; i < dimension; ++i) {
        res[i] = amount * speed * res[i] / norm;
      }
      return res;
    }
  };
  // END MASS WANDERER DEFINITION //////////////////////////////////////////////

  float samplerate;
  float amount;
  std::vector<float> frequencies;
  std::vector<float> randFrequencies;
  std::vector<dimensionBounds<float>> bounds;
  std::vector<dimensionBounds<float>> randBounds;

  std::vector<MassWanderer> massWanderers;

public:
  // TODO : take following parameters into account :
  //  frequencies, randFrequencies,
  //  valueRanges, randValueRanges
  //  and accept vectors of dimension "dimension + 1" (each dimension's value + speed vector norm)

  Wandering() : Orbits<dimension>::Rule(),
  samplerate(1.f),
  amount(1.f),
  frequencies(dimension + 1, 0),
  randFrequencies(dimension + 1, 0),
  bounds(dimension + 1, { 0, 0 }),
  randBounds(dimension + 1, { 0, 0 })
  {
    this->addParameter("samplerate", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      samplerate = std::max(v[0], 1e-3f);
      for (auto& w : massWanderers) {
        w.setSamplerate(samplerate);
      }
    });

    this->addParameter("amount", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      amount = std::clamp(v[0], 0.f, 1.f);
    });

    this->addParameter("frequencies", [&](const std::vector<float>& v) {
      if (v.size() != dimension + 1) return;
      frequencies = v;
    });

    this->addParameter("randFrequencies", [&](const std::vector<float>& v) {
      if (v.size() != dimension + 1) return;
      randFrequencies = v;
    });

    this->addParameter("bounds", [&](const std::vector<float>& v) {
      if (v.size() != 2 * (dimension + 1)) return;
      for (auto i = 0; i < v.size(); ++i) {
        bounds[i] = { v[2*i], v[2*i+1] };
      }
    });

    this->addParameter("randBounds", [&](const std::vector<float>& v) {
      if (v.size() != 2 * (dimension + 1)) return;
      for (auto i = 0; i < v.size(); ++i) {
        randBounds[i] = { v[2*i], v[2*i+1] };
      }
    });
  }

  virtual void setParameter(const char* paramName, const std::vector<float>& values) override {
    // Orbits<dimension>::Rule::setParameter(paramName, values);
    this->setParameter(paramName, values);
    updateWandererStates();
  }

  void setSize(std::size_t s) {
    massWanderers = std::vector<MassWanderer>(s);
    updateWandererStates();
  }

  virtual void processParticles(
    Tree<dimension>* t,
    std::vector<std::shared_ptr<Mass<dimension>>> v,
    float dt
  ) override {
    if (v.size() != massWanderers.size()) {
      setSize(v.size());
    }

    for (std::size_t i = 0; i < v.size(); ++i) {
      massWanderers[i].step();
      auto f = massWanderers[i].getForceToApply(amount);

      v[i]->applyForce(f);
    }
  }

private:
  void updateWandererStates() {
    for (auto i = 0; i < massWanderers.size(); ++i) {
      massWanderers[i].updateState(frequencies, randFrequencies, bounds, randBounds);
    }
  }
};

}; /* end namespace Orbits */

#endif /* ORBITS_WANDERING_H */
