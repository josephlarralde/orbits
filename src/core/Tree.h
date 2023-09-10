#ifndef Tree_h
#define Tree_h

#include "Mass.h"
#include <cmath>
#include <deque>

/*
 maxParticles describes how many particles are allowed
 in each leaf. a normal quadtree only has 1 particle per
 leaf. this can cause issues if two particles are very close
 together, as it causes a large number of subdivisions.
 in an evenly distributed system, if maxParticles has a high
 value, the tree will approximate a binning system.
 */
#define maxParticles 16

namespace Orbits {

#define DEFAULT_MIN_DIMENSION_BOUND -2000
#define DEFAULT_MAX_DIMENSION_BOUND 2000

template <std::size_t dimension>
class Tree {
  // struct mmm { // min, mid, max
  //   float min;
  //   float mid;
  //   float max;
  // };
  
  typedef Mass<dimension> Mass;
  typedef std::shared_ptr<Mass> MassPtr;
  typedef std::shared_ptr<Tree<dimension>> TreePtr;
  
  float clipToMinMaxDimensionBounds(float v);
    
public:
  bool hasChildren;
  int nParticles;

  // we use smart pointers instead of references to enable Body polymorphism :
  std::vector<MassPtr> particles;
  
  // min, mid, max for each dimension
  std::vector<mmm> bounds;

  // each bit if the subtree index represents a "side" of the dimension :
  // 0 and 1 mean below and above the dimension's bound.mid, respectively
  std::vector<TreePtr> subtrees;

  Tree();

  //----------------------------------------------------------------------------
  void initialize();

  //----------------------------------------------------------------------------
  void setBounds(const std::vector<std::pair<float,float>>& _bounds);
  void setMid();
  
  //----------------------------------------------------------------------------
  void add(MassPtr cur); // used by setup
  void setup(const std::vector<MassPtr>& all);
  
  //----------------------------------------------------------------------------
  std::vector<std::pair<MassPtr,float>>
  getNeighborsAndDistances(std::vector<float> targetPosition,
                           float radius);

  //----------------------------------------------------------------------------
  std::vector<MassPtr>
  getNeighbors(std::vector<float> targetPosition,
               float radius);

  //----------------------------------------------------------------------------
  void getIntersection(std::vector<MassPtr>& intersection,
                       const std::vector<float>& targetPosition,
                       float radius);
  
  //----------------------------------------------------------------------------
  // not working yet
  void addForce(std::vector<float> target, float radius, float scale);

  //----------------------------------------------------------------------------
  // void addForce(std::vector<float> targetPosition,
  //               float radius,
  //               float scale,
  //               ForceTypeE type);

  // void draw();
};

}; // end of namespace Orbits

#endif /* Tree_h */
