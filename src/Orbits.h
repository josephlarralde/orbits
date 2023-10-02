#ifndef ORBITS_ORBITS_H
#define ORBITS_ORBITS_H

#include <map>
#include <cmath>

#include "./core/Tree.h"

namespace Orbits {

template <std::size_t dimension>
class Orbits {
public:
  // simple definition of rule class
  class Rule {
  public:
    virtual void setParameter(const char* parameterName, const std::vector<float>& values) {}
    virtual void process(Tree<dimension>* t) {}
  };

// private:
  typedef Mass<dimension> Mass;
  typedef std::shared_ptr<Mass> MassPtr;

private:
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
  void setM(float m) { for (auto p : tree.particles) p->setM(m); } // mass
  void setD(float m) { for (auto p : tree.particles) p->setD(m); } // damping
  void setF(float m) { for (auto p : tree.particles) p->setF(m); } // friction

  void addRule(const char* name, std::shared_ptr<Rule> rule) {
    rules[name] = rule;
  }

  void setParameter(const char* ruleName, const char* parameterName, const std::vector<float>& values) {
    // if (std::strcmp(parameterName, "stuff") == 0) {
    //   rules[ruleName]->setParameter(parameterValue);
    // }
    rules[ruleName]->setParameter(parameterName, values);
  }
  
  void step() {
    tree.reset(new Tree<dimension>());
    tree->setup(particles);

    for (auto& [ ruleName, rule ] : rules) {
      rule->process(tree.get());
    }
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

}; // end of namespace Orbits

#endif /* ORBITS_ORBITS_H */