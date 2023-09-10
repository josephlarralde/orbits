#ifndef ORBITS_ORBITS_H
#define ORBITS_ORBITS_H

#include <map>

// technique to allow definition of template class methods in cpp file
// and still be able to avoid "Symbol not found" errors
// the alternative is to drop the cpp file and put all the code in the header
#include "./core/Tree.cpp"

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
  //typedef std::shared_ptr<Tree<dimension>> TreePtr;
  
private:
  std::vector<MassPtr> particles; // the particles
  std::unique_ptr<Tree<dimension>> tree; // the tree to compute closest particles
  // Tree<dimension> tree;
  std::map<const char*, std::shared_ptr<Rule>> rules; // the navigation rules to apply

public:
  Orbits() {
    tree.reset(new Tree<dimension>());
  }

  void setSize(std::size_t s) {
    tree.reset(new Tree<dimension>());
    // particles.clear();
    particles.resize(s);
    for (std::size_t i = 0; i < s; ++i) {
      auto p = std::make_shared<Mass>();
      particles[i] = p;
    }
  }

  void setBounds() {}
  
  // set particles basic physical parameters :
  void setM(float m) { for (auto p : tree.particles) p->setM(m); } // mass
  void setD(float m) { for (auto p : tree.particles) p->setD(m); } // damping
  void setF(float m) { for (auto p : tree.particles) p->setF(m); } // friction

  void addRule(const char* name, std::shared_ptr<Rule> rule) {
    rules[name] = rule;
  }

  void setParameter(const char* ruleName, const char* parameterName, const std::vector<float>& values) {
    
    // if (std::strcmp(parameterName, "stuff") == 0) { rules[ruleName]->setParameter(parameterValue); }
    rules[ruleName]->setParameter(parameterName, values);
  }
  
  void step() {
    tree.reset(new Tree<dimension>());
    tree->setup(particles);

    for (auto& [ ruleName, rule ] : rules) {
      rule->process(tree.get());
      //rule->process(&tree);
    }
  }

  std::vector<std::vector<float>>
  getPositions() {
    std::vector<std::vector<float>> res;
    // for (auto& p : tree->particles) { // below should be equivalent
    for (auto& p : particles) {
      res.push_back(p.getPosition());
    }
    return res;
  }

  std::vector<std::pair<std::vector<float>, std::vector<float>>>
  getPositionsAndSpeeds() {
    std::vector<std::pair<std::vector<float>, std::vector<float>>> res;
    // for (auto& p : tree->particles) { // below should be equivalent
    for (auto& p : particles) {
      res.push_back({ p.getPosition(), p.getSpeed() });
    }
    return res;
  }
};

}; // end of namespace Orbits

#endif /* ORBITS_ORBITS_H */