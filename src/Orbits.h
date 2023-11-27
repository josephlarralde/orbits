#ifndef ORBITS_ORBITS_H
#define ORBITS_ORBITS_H

#include <map>
#include <cmath>
#include <random>
#include <chrono>

#include "./core/Tree.h"

namespace Orbits {

template <std::size_t dimension>
class Orbits {
public:
  typedef Mass<dimension> Mass;
  typedef std::shared_ptr<Mass> MassPtr;

  // simple definition of Rule class ///////////////////////////////////////////

  class Rule {
  protected:
    std::map<const char*, std::function<void(const std::vector<float>&)>> setters;

  public:
    virtual void addParameter(const char* parameterName,
                              std::function<void(const std::vector<float>&)> setter) {
      setters[parameterName] = setter;
    }

    // virtual void addParameter(const char* parameterName,
    //                           std::function<void(float)> setter) {
    //   setters[parameterName] = [&](const std::vector<float>& v) {
    //     setter({ v });
    //   };
    // }

    virtual void setParameter(const char* parameterName, const std::vector<float>& values) {
      if (setters.find(parameterName) != setters.end()) {
        setters[parameterName](values);
      }
    }

    // clunky API until I figure out how to write a Mass iterator on the Tree to
    // get rid of the reference to particles here :
    virtual void processParticles(Tree<dimension>* t, std::vector<MassPtr>& v, float dt) = 0;
  };

private:
  std::chrono::time_point<std::chrono::steady_clock> lastTickDate;
  std::vector<MassPtr> particles; // the particles
  std::unique_ptr<Tree<dimension>> tree; // the tree to compute closest particles

  std::map<const char*, std::shared_ptr<Rule>> rulesById; // the navigation rules to apply
  std::vector<std::shared_ptr<Rule>> rules;

public:
  Orbits(std::size_t s) : tree(nullptr) {
    setSize(s);
  }

  void setSize(std::size_t s) {
    tree.reset(new Tree<dimension>());
    particles.resize(s);

    for (std::size_t i = 0; i < s; ++i) {
      particles[i] = std::make_shared<Mass>();
    }
  }

  std::size_t getSize() {
    return particles.size();
  }

  // void setBounds() {}
  // => bounds are deduced from all particles' positions, and clipping is
  // performed internally by Tree according to DEFAULT_MIN_DIMENSION_BOUND
  // and DEFAULT_MAX_DIMENSION_BOUND
  
  // set particles basic physical parameters :
  void setM(float m) { for (auto p : particles) p->setM(m); } // mass

  // damping doesn't make sense for a mass alone !!!
  // void setD(float d) { for (auto p : particles) p->setD(d); } // damping

  void setF(float f) { for (auto p : particles) p->setF(f); } // friction

  void addRule(const char* ruleName, std::shared_ptr<Rule> rule) {
    rulesById[ruleName] = rule;
    rules.push_back(rule);
  }

  void setParameter(const char* ruleName,
                    const char* parameterName,
                    const std::vector<float>& values) {
    if (rules.find(ruleName) != rules.end()) {
      rules[ruleName]->setParameter(parameterName, values);
    }
  }
  
  void step() {
    // deal with timing
    auto now = std::chrono::steady_clock::now();
    // if we wanted milliseconds units :
    // const std::chrono::duration<double, std::milli> dur = now - lastTickDate;
    // but we want the default, standard unit (seconds) :
    const std::chrono::duration<double> delta = now - lastTickDate;
    const float dt = delta.count();
    lastTickDate = now;

    // build tree
    tree.reset(new Tree<dimension>());
    tree->setup(particles);

    // apply rules
    for (std::size_t i = 0; i < rules.size(); ++i) {
      rules[i]->processParticles(tree.get(), particles, dt);
    }

    for (auto m : particles) m->update(dt);
  }

  std::vector<std::vector<float>>
  getPositions() {
    std::vector<std::vector<float>> res;
    for (auto& p : particles) {
      res.push_back(p.getPosition());
    }
    return res;
  }

  std::vector<std::pair<std::vector<float>, std::vector<float>>>
  getPositionsAndSpeeds() {
    std::vector<std::pair<std::vector<float>, std::vector<float>>> res;
    for (auto& p : particles) {
      res.push_back({ p.getPosition(), p.getSpeed() });
    }
    return res;
  }

private:
  void setRandomParticlePositions() {
    std::srand(std::time(nullptr));
    std::vector<float> position(3, 0.f);

    for (std::size_t i = 0; i < particles.size(); ++i) {
      particles[i] = std::make_shared<Mass>();
      for (std::size_t d = 0; d < dimension; ++d) {
        position[d] = static_cast<float>(std::rand()) / RAND_MAX;
      }
      particles[i]->setPosition(position);
    }
  }

  // todo implement this in an efficient way
  void setRandomParticleSpeeds() {

    // create a set of random points uniformly distributed on the
    // (dimension-1)-sphere (or n-ball).
    // see https://en.wikipedia.org/wiki/N-sphere#Uniformly_at_random_on_the_(n_%E2%88%92_1)-sphere
    // and https://en.wikipedia.org/wiki/N-sphere#Uniformly_at_random_within_the_n-ball
    // we use normal deviates computed from std::normal_distribution for each
    // dimension, then we normalized the obtain vectors, and voilà !
    // (actually we use a single normal deviate for all dimension, but it should
    // behave pretty much the same).

    std::vector<float> point(dimension);
    std::random_device seed_generator;
    std::default_random_engine generator(seed_generator);
    std::normal_distribution<float> distribution{0.5, 0.2};

    for (std::size_t i = 0; i < particles.size(); ++i) {
      for (std::size_t d = 0; d < dimension; ++d) {
        point[d] = distribution(generator);
      }
      normalizeVector(point);
      particles[i]->setSpeed(point);
    }
    std::vector<float> dist(particles.size());


  }
};

}; /* end namespace Orbits */

#endif /* ORBITS_ORBITS_H */