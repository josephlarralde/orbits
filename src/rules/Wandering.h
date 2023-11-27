#ifndef ORBITS_WANDERING_H
#define ORBITS_WANDERING_H

#include <random>
#include "../Orbits.h"
#include "../core/Utilities.h"

////////////////////////////////////////////////////////////////////////////////

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
// OF COURSE, YES :/

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

////////////////////////////////////////////////////////////////////////////////

// Right now we just have an orientation vector (controlled by n - 1 varying
// angles) which we append to a scaled speed vector, as Craig Reynolds suggest
// in his description of the "wander" steering behaviour.

// SEE BELOW :

namespace Orbits
{

template <std::size_t dimension>
class Wandering : public Orbits<dimension>::Rule
{
  // MASS WANDERER STATE (TO BE SHARED GLOBALLY FROM WANDERING CLASS) //////////
  struct MassWandererState {
    float guideDistance;
    float guideRadius;
    float variationFrequency;
    float randomVariationFrequency;
    dimensionBounds<float> frequencyRange;
  };

  // MASS WANDERER DEFINITION //////////////////////////////////////////////////
  class MassWanderer {

    //-- BASE OSCILLATOR CLASS -------------------------------------------------
    class OscillatorBase {
    protected:
      float phase;
      //float period;
      float frequency;

    public:
      OscillatorBase() : phase(0.f), frequency(0.f) {}

      virtual void setFrequency(float f) { frequency = f; /* period = 1/f; */ }

      virtual void setPhase(float f) {
        phase = f;
        clipPhase();
      }

      virtual void step(float dt) {
        phase += (dt * frequency); // or dt / period
        clipPhase();
      }

    private:
      void clipPhase() {
        if (phase < 0.f) {
          phase = fmod(1.f - phase, 1.f);
        } else if (phase > 1.f) {
          phase = fmod(phase, 1.f);
        }
        // alternative :
        //if (phase < 0) {
        //  float absPhase = std::abs(phase);
        //  phase = 1 - (absPhase - std::floor(absPhase));
        //} else if (phase > 1) {
        //  phase -= std::floor(phase);
        //}
      }
    };
    //--------------------------------------------------------------------------

    //-- DIMENSION WANDERER ----------------------------------------------------
    class DimensionWanderer : public OscillatorBase {
      dimensionBounds<float> bounds;
      float width;

    public:
      DimensionWanderer() : OscillatorBase(),
      bounds({ 0.f, 0.f }),
      width(0.f) {}

      void setBounds(dimensionBounds<float> dimBounds) {
        bounds = dimBounds;
        width = bounds.max - bounds.min;
      }

      float getValue() {
        // should use wave table for sin instead of math function !!!
        return (std::sin(this->phase * M_2_PI) * 0.5f + 0.5f) * width + bounds.min;
      }
    };
    //--------------------------------------------------------------------------

    //-- ANGLE WANDERER --------------------------------------------------------
    class AngleWanderer : public OscillatorBase {
      float baseFrequency;
      float randomFrequency;
      DimensionWanderer frequencyRangeOscillator;

    public:
      AngleWanderer() : OscillatorBase(),
      baseFrequency(0.f),
      randomFrequency(0.f) {}

      void step(float dt) override {
        frequencyRangeOscillator.step(dt);
        OscillatorBase::setFrequency(frequencyRangeOscillator.getValue());
        OscillatorBase::step(dt);
      }

      // what about having a resetPhases(baseValue, randomizationFactor) method
      // on the Wandering class ?
      // this would generate random values between 0 and 0 to 2PI and call
      // resetPhase here with each of these values :
      void setPhase(float f) { this->phase = f; }

      void setVariationFrequency(float f) {
        this->baseFrequency = f;
        this->updateFrequency();
      }

      void setRandomVariationFrequency(float f) {
        this->randomFrequency = f;
        this->updateFrequency();
      }

      void setFrequencyRange(dimensionBounds<float> range) {
        frequencyRangeOscillator.setBounds(range);
      }

      float getValue() {
        return this->phase * M_2_PI;
      }

    private:
      void updateFrequency() {
        std::srand(std::time(nullptr));
        float randomValue = randomFrequency * std::rand() / RAND_MAX;
        this->frequency = baseFrequency + randomValue;
      }
    };
    //--------------------------------------------------------------------------

    float guideDistance;
    float guideRadius;
    std::vector<AngleWanderer> angleWanderers;
    std::vector<float> angles;

  public:
    explicit MassWanderer(MassWandererState s) :
    guideDistance(s.guideDistance),
    guideRadius(s.guideRadius),
    angleWanderers(dimension - 1),
    angles(dimension - 1, 0) {
      setVariationFrequency(s.variationFrequency);
      setRandomVariationFrequency(s.randomVariationFrequency);
      setFrequencyRange(s.frequencyRange);
    }

    void step(float dt) {
      for (std::size_t i = 0; i < dimension - 1; ++i) {
        angleWanderers[i].step(dt);
        angles[i] = angleWanderers[i].getValue();
      }
    }

    void setGuideDistance(float distance) { guideDistance = distance; }

    void setGuideRadius(float radius) { guideRadius = radius; }

    void setVariationFrequency(float f) {
      for (auto& w : angleWanderers) { w.setVariationFrequency(f); }
    }

    void setRandomVariationFrequency(float f) {
      for (auto& w : angleWanderers) { w.setRandomVariationFrequency(f); }
    }

    void setFrequencyRange(dimensionBounds<float> range) {
      for (auto& w : angleWanderers) { w.setFrequencyRange(range); }
    }

    // Here we actually just take the global deviation orientation and add it
    // to the direction to obtain the steering vector. But the angle doesn't
    // vary in the local coordinate system, to get this behaviour we should be
    // using a frenet system, which requires the use of the cross product.
    // ATM I have no idea how to expand frenet to n dimensions, but this post
    // might help :
    // https://math.stackexchange.com/questions/185991/is-the-vector-cross-product-only-defined-for-3d
    // Meanwhile, we should specialize this specific method for dimensions 2 and 3
    // to get a less erratic movement :
    std::vector<float> getForceToApply(std::shared_ptr<Mass<dimension>> m,
                                       float amount = 1.f) {
      const auto& direction = m->getNormalizedSpeed();
      scaleVector<float>(direction, guideDistance);

      auto deviation = sphericalToCartesian(angles);
      scaleVector<float>(deviation, guideRadius);

      std::vector<float> res = addVectors(direction, deviation);
      scaleVector<float>(res, amount);

      return res;
    }
  };
  // END MASS WANDERER DEFINITION //////////////////////////////////////////////

  float forceMultiplier;
  float rotationSpeedMultiplier;
  MassWandererState state;
  std::vector<MassWanderer> massWanderers;

public:
  Wandering() : Orbits<dimension>::Rule(),
  forceMultiplier(1.f),
  rotationSpeedMultiplier(1.f)
  {
    this->addParameter("forceMultiplier", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      setForceMultiplier(v[0]);
    });

    this->addParameter("rotationSpeedMultiplier", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      setRotationSpeedMultiplier(v[0]);
    });

    this->addParameter("guideDistance", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      setGuideDistance(v[0]);
    });

    this->addParameter("guideRadius", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      setGuideRadius(v[0]);
    });

    this->addParameter("variationFrequency", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      setVariationFrequency(v[0]);
    });

    this->addParameter("randomVariationFrequency", [&](const std::vector<float>& v) {
      if (v.empty()) return;
      setRandomVariationFrequency(v[0]);
    });

    this->addParameter("frequencyRange", [&](const std::vector<float>& v) {
      if (v.size() != 2) return;
      setFrequencyRange({v[0], v[1]});
    });
  }

  // various setters -----------------------------------------------------------

  void setForceMultiplier(float multiplier) {
    forceMultiplier = std::clamp(multiplier, 0.f, 1.f);
  }

  void setRotationSpeedMultiplier(float multiplier) {
    rotationSpeedMultiplier = std::clamp(multiplier, 0.f, 1.f);
  }

  void setGuideDistance(float distance) {
    state.guideDistance = distance;
    for (auto& w : massWanderers) { w.setGuideDistance(distance); }
  }

  void setGuideRadius(float radius) {
    state.guideRadius = radius;
    for (auto& w : massWanderers) { w.setGuideRadius(radius); }
  }

  void setVariationFrequency(float f) {
    state.variationFrequency = f;
    for (auto& w : massWanderers) { w.setVariationFrequency(f); }
  }

  void setRandomVariationFrequency(float f) {
    state.randomVariationFrequency = f;
    for (auto& w : massWanderers) { w.setRandomVariationFrequency(f); }
  }

  void setFrequencyRange(dimensionBounds<float>& r) {
    state.frequencyRange = r;
    for (auto& w : massWanderers) { w.setFrequencyRange(r); }
  }

  // kind of init function -----------------------------------------------------

  void setSize(std::size_t s) {
    massWanderers = std::vector<MassWanderer>(s, state);
  }

  // MAIN RULE CALLBACK ////////////////////////////////////////////////////////

  void processParticles(Tree<dimension>* t,
                        std::vector<std::shared_ptr<Mass<dimension>>>& v,
                        float dt) override
  {
    if (v.size() != massWanderers.size()) {
      setSize(v.size());
    }

    float scaledDt = dt * rotationSpeedMultiplier;

    for (std::size_t i = 0; i < v.size(); ++i) {
      massWanderers[i].step(scaledDt);
      auto f = massWanderers[i].getForceToApply(v[i], forceMultiplier);
      v[i]->applyForce(f);
    }
  }
};

}; /* end namespace Orbits */

#endif /* ORBITS_WANDERING_H */
