#ifndef ORBITS_ORBITS_H
#define ORBITS_ORBITS_H

#include <map>
#include <cmath>
#include <chrono>

#include "./core/Tree.h"

namespace Orbits {

template <std::size_t dimension>
class Orbits {
public:
  typedef Mass<dimension> Mass;
  typedef std::shared_ptr<Mass> MassPtr;

  // simple definition of Rule class
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
    virtual void processParticles(Tree<dimension>* t, std::vector<MassPtr>& v, float dt) {}
    // virtual void processParticlesTree(Tree<dimension>* t) {}
    // virtual void processParticlesVector(std::vector<MassPtr>& v) {}
  };

private:
  std::chrono::time_point<std::chrono::steady_clock> lastTickDate;
  std::vector<MassPtr> particles; // the particles
  std::unique_ptr<Tree<dimension>> tree; // the tree to compute closest particles
  std::map<const char*, std::shared_ptr<Rule>> rules; // the navigation rules to apply

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
    rules[ruleName] = rule;
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
    // todo : add optional dt argument to process / update methods
    for (auto& [ ruleName, rule ] : rules) {
      rule->processParticles(tree.get(), particles, dt);
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
};

}; /* end namespace Orbits */

#endif /* ORBITS_ORBITS_H */