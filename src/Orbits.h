#ifndef ORBITS_ORBITS_H
#define ORBITS_ORBITS_H

#include "core/Tree.h"

template <std::size_t dimension>
class Orbits {
  std::vector<Mass<dimension>> particles; // the particles
  std::unique_ptr<Tree<dimension>> tree; // the tree to compute closest particles
  std::vector<std::shared_ptr<Rule>> rules; // the navigation rules to apply

public:
  class Rule {
  public:
    virtual void process(Tree<dimension>* t) {}
  };

public:
  void setSize(std::size_t s) {
    for (std::size_t i = 0; i < s; ++i) {
      auto p = std::make_shared<Mass<dimension>>();
      particles.push_back(p);
    }
  }

  void setBounds() {}
  
  // set particles basic physical parameters :
  void setM(float m) { for (auto p : tree.particles) p->setM(m); } // mass
  void setD(float m) { for (auto p : tree.particles) p->setD(m); } // damping
  void setF(float m) { for (auto p : tree.particles) p->setF(m); } // friction

  void addRule(std::shared_ptr<Rule> rule) {
    rules.push_back(rule);
  }
  
  void step() {
    tree = std::unique_ptr<Tree<dimension>>(new Tree<dimension>());
    tree->setup(particles);

    for (auto& rule : rules) {
      rules.process(tree.get());
    }
  }

  std::vector<std::vector<float>>
  getPositions() {
    std::vector<std::vector<float>> res;
    for (auto& p : tree->particles) {
      res.push_back(p.getPosition());
    }
    return res;
  }

  std::vector<std::pair<std::vector<float>, std::vector<float>>>
  getPositionsAndSpeeds() {
    std::vector<std::pair<std::vector<float>, std::vector<float>>> res;
    for (auto& p : tree->particles) {
      res.push_back({ p.getPosition(), p.getSpeed() });
    }
    return res;
  }
};

#endif /* ORBITS_ORBITS_H */