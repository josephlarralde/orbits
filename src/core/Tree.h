#ifndef Tree_h
#define Tree_h

#include "Mass.h"
#include <cmath>
#include <deque>
#include <bitset>

#define maxParticles 16

/*
 * Original comment from Kyle McDonald whose quad-tree
 * largely inspired this n-tree :
 * maxParticles describes how many particles are allowed
 * in each leaf. A normal quadtree only has 1 particle per
 * leaf. This can cause issues if two particles are very close
 * together, as it causes a large number of subdivisions.
 * in an evenly distributed system, if maxParticles has a high
 * value, the tree will approximate a binning system.
 */

/*
 * Improved tree building safety :
 * Experiments showed that the maximum safe number of
 * duplicate particle positions is maxParticles + 1, above this
 * value the tree from original code would grow indefinitely.
 * To prevent this, in the add() method, when a tree has no children
 * and reaches maxParticles particles, we check if all its particles
 * and the next particle we want to add have the same position.
 * If so, before dispatching the particles to the subtrees
 * we set the mid point to the common position, then on next
 * calls to add(), the duplicates will be randomly distributed
 * across subtrees thanks to a not-so-ugly hack (see beginning
 * of add method)
 */

namespace Orbits {

#define DEFAULT_MIN_DIMENSION_BOUND -2000
#define DEFAULT_MAX_DIMENSION_BOUND 2000

template <std::size_t dimension>
class Tree {
  typedef Mass<dimension> Mass;
  typedef std::shared_ptr<Mass> MassPtr;
  typedef std::shared_ptr<Tree<dimension>> TreePtr;

  float clipToMinMaxDimensionBounds(float v) {
    return std::max(
      static_cast<float>(DEFAULT_MIN_DIMENSION_BOUND),
      std::min(static_cast<float>(DEFAULT_MAX_DIMENSION_BOUND), v)
    );
  }

  static std::size_t addCount;
  std::size_t depth;

public:
  bool hasChildren;
  int nParticles;

  std::vector<MassPtr> particles;

  // min, mid, max for each dimension
  std::vector<mmm> bounds;

  // each bit if the subtree index represents a "side" of the dimension :
  // 0 and 1 mean below and above the dimension's bound.mid, respectively
  std::vector<TreePtr> subtrees;

  Tree(std::size_t d = 0) :
  depth(d),
  hasChildren(false),
  nParticles(0),
  particles(std::vector<MassPtr>(maxParticles, nullptr)),
  bounds(std::vector<mmm>(dimension, { 0.f, 0.f, 0.f })),
  subtrees(std::vector<TreePtr>(pow(2, dimension), nullptr))
  {
    // printf("new tree of depth %ld created\n", depth);
  }

  //----------------------------------------------------------------------------
  void setBounds(const std::vector<std::pair<float,float>>& _bounds) {
    if (_bounds.size() != bounds.size()) return; // throw exception ?

    for (std::size_t i = 0; i < _bounds.size(); ++i) {
      auto& dim = bounds[i];
      dim.min = clipToMinMaxDimensionBounds(_bounds[i].first);
      dim.max = clipToMinMaxDimensionBounds(_bounds[i].second);
    }
    setMid();
  }

  void setMid() {
    for (auto& dim : bounds) {
      dim.mid = (dim.min + dim.max) * 0.5f;
    }
  }

  void setMidFromPosition(const std::vector<float>& pos) {
    for (std::size_t d = 0; d < dimension; ++d) {
      auto& dim = bounds[d];
      dim.mid = pos[d];
    }
  }

  //----------------------------------------------------------------------------
  void add(MassPtr cur) { // used by setup
    if (hasChildren) {
      // we compute the subtree index value bit by bit (LSB to MSB),
      // using each dimension's position (below and above bounds[d].mid)
      std::bitset<sizeof(std::size_t) * CHAR_BIT> bitIndex{0};

      for (std::size_t d = 0; d < dimension; ++d) {
        // not-so-ugly "hack" to deal with too many duplicate positions
        // causing infinite recursive subtree creation
        // if allEqual was true and we set mid to the duplicate position
        // we will randomly distribute our duplicates to the subtrees
        // while maintaining coherency.
        // this should be rewritten with some kind of counter that evenly
        // distributes duplicates across the halves of each dimension
        if (cur->getPosition()[d] == bounds[d].mid) {
          bitIndex[d] = std::rand() > (RAND_MAX / 2);
        } else {
          bitIndex[d] = cur->getPosition()[d] > bounds[d].mid;
        }
      }

      subtrees[bitIndex.to_ulong()]->add(cur);
    } else {
      if (nParticles < maxParticles) {
        particles[nParticles].swap(cur);
        nParticles++;
      } else {
        bool tooManyDuplicates = true;
        auto zeroPosition = cur->getPosition();

        for (std::size_t i = 0; i < maxParticles; ++i) {
          // if (static_cast<std::vector<float>>(particles[i]->getPosition()) != zeroPosition) {
          if (particles[i]->getPosition() != zeroPosition) {
            tooManyDuplicates = false;
            break;
          }
        }

        if (tooManyDuplicates) { setMidFromPosition(zeroPosition); }

        for (std::size_t i = 0; i < subtrees.size(); ++i) {
          std::vector<std::pair<float,float>> subBounds(dimension);
          std::bitset<sizeof(std::size_t) * CHAR_BIT> bitIndex{
            static_cast<std::size_t>(i)
          };

          for (std::size_t d = 0; d < dimension; ++d) {
            std::pair<float,float> dimBounds{0,0};

            if (bitIndex[int(d)]) {
              dimBounds.first = bounds[d].mid;
              dimBounds.second = bounds[d].max;
            } else {
              dimBounds.first = bounds[d].min;
              dimBounds.second = bounds[d].mid;
            }

            subBounds[d] = dimBounds;
          }

          subtrees[i].reset(new Tree<dimension>(depth + 1));
          subtrees[i]->setBounds(subBounds);
        }

        hasChildren = true;

        for (int i = 0; i < nParticles; ++i) {
          add(particles[i]);
          particles[i] = nullptr;
        }

        nParticles = 0;
        add(cur);
      }
    }
  }

  void setup(const std::vector<MassPtr>& all) {
    int n = all.size();
    if (n == 0) return;

    std::vector<std::pair<float,float>> minMax(dimension, { 0.f, 0.f });

    for (auto d = 0; d < dimension; ++d) {
      minMax[d].first = all[0]->getPosition()[d];
      minMax[d].second = all[0]->getPosition()[d];

      for (auto i = 1; i < n; ++i) {
        auto& pos = all[i]->getPosition()[d];
        if (pos < minMax[d].first) {
          minMax[d].first = pos;
        }
        if (pos > minMax[d].second) {
          minMax[d].second = pos;
        }
      }
    }

    setBounds(minMax);

    // center and square boundaries
    float width, maxWidth = 0.f;

    for (auto& b : bounds) {
      maxWidth = std::max(b.max - b.min, maxWidth);
    }

    float halfSide = maxWidth * 0.5f;

    for (auto& b : bounds) {
      b.min = b.mid - halfSide;
      b.max = b.mid + halfSide;
    }

    for (auto particle : all) {
      add(particle);
    }
  }

  //----------------------------------------------------------------------------
  std::vector<std::pair<MassPtr,float>>
  getNeighborsAndDistances(std::vector<float> targetPosition,
                           float radius) {
    // delta, "squared dist" and "squared radius"
    float delta, sqdist, sqrad = radius * radius;
    std::vector<MassPtr> intersection;
    std::vector<std::pair<MassPtr,float>> neighbors;

    getIntersection(intersection, targetPosition, radius);

    for (auto cur : intersection) {
      sqdist = 0.f;

      for (auto d = 0; d < dimension; ++d) {
        delta = targetPosition[d] - cur->getPosition()[d];
        sqdist += delta * delta;
      }

      if (sqdist < sqrad) {
        neighbors.push_back({ cur, sqrt(sqdist) / radius });
      }
    }

    return neighbors;
  }

  //----------------------------------------------------------------------------
  std::vector<MassPtr>
  getNeighbors(std::vector<float> targetPosition,
               float radius) {
    // delta, "squared dist" and "squared radius"
    float delta, sqdist, sqradius = radius * radius;
    std::vector<MassPtr> intersection, neighbors;

    getIntersection(intersection, targetPosition, radius);

    for (auto cur : intersection) {
      sqdist = 0.f;

      for (auto d = 0; d < dimension; ++d) {
        delta = targetPosition[d] - cur->getPosition()[d];
        sqdist += delta * delta;
      }

      if (sqdist < sqradius) {
        neighbors.push_back(cur);
      }
    }

    return neighbors;
  }

  //----------------------------------------------------------------------------
  void getIntersection(std::vector<MassPtr>& intersection,
                       const std::vector<float>& targetPosition,
                       float radius) {
    if (nParticles > 0) {
      for (auto i = 0; i < nParticles; ++i) {
        auto p = particles[i];
        bool inside = true;

        for (auto d = 0; d < dimension; ++d) {
          if (targetPosition[d] < bounds[d].min - radius ||
             targetPosition[d] > bounds[d].max + radius) {
            inside = false;
            break;
          }
        }

        if (inside) {
          intersection.push_back(p);
        }
      }
    } else if (hasChildren) {
      for (auto subtree : subtrees) {
        subtree->getIntersection(
            intersection,
            targetPosition,
            radius
        );
      }
    }
  }

  //----------------------------------------------------------------------------

  /*
   * returns the n nearest neighbors with their respective distances sorted by
   * proximity, with the closest Mass optionally removed (e.g. if its position
   * is used as targetPosition)
   */

  std::vector<std::pair<MassPtr, float>>
  getNearestNeighbors(const std::vector<float> targetPosition,
                      std::size_t n,
                      bool removeFirstElement = true) {
    // radial dichotomic algorithm :
    // * we first initialize minRadius to 0 and maxRadius to the
    //   "hyper-diagonal" of (the longest possible distance in) the bounds
    // * we adjust min and max radius by a dichotomic process until one of the
    //   following conditions is met :
    //   * we find a radius where we have exactly n neighbors and proceed
    //     with it (by actually assigning it to maxRadius)
    //   * we exceed a max number of iterations and proceed with the
    //     current maxRadius, which should encompass a number of neighbors > n
    // * when we proceed, maxRadius represents a zone where we have n neighbors
    //   or more (but hopefully not too much), so we just sort the result of
    //   getNeighborsAndDistances() by distance, and only keep the n closest.
    //   the complexity of this algorithm can likely show great variations
    //   depending on the repartition of the masses ... if the repartition is
    //   "regular", the complexity shouldn't be too bad (to be tested, or find a
    //   better algorithm)

    float minRadius = 0.f;
    float maxRadius = 0.f; // max possible distance in bounds ("hyper-diagonal")
    float delta = 0.f;

    for (std::size_t d = 0; d < dimension; ++d) {
      delta = bounds[d].max - bounds[d].min;
      maxRadius += delta * delta;
    }
    maxRadius = std::sqrt(maxRadius);

    const std::size_t MAX_NNN_DICHOTOMIC_ITERATIONS = 5;

    bool gotExactMatch = false;
    std::vector<MassPtr> massesAtRadius;

    for (std::size_t i = 0; i < MAX_NNN_DICHOTOMIC_ITERATIONS; ++i) {
      float midRadius = (maxRadius + minRadius) * 0.5f;
      massesAtRadius = getNeighbors(targetPosition, midRadius);

      if (massesAtRadius.size() < n + 1) {
        minRadius = midRadius;
      } else if (massesAtRadius.size() > n + 1) {
        maxRadius = midRadius;
      } else { //  we've got an exact match
        maxRadius = midRadius;
        break;
      }
    }

    typedef std::pair<MassPtr, float> MassDist;
    std::vector<MassDist> res = getNeighborsAndDistances(targetPosition,
                                                         maxRadius);

    if (res.size() < n + 1) {
      // raise error ??? or simply return smaller vector ... ?
    }

    auto comp = [](MassDist a, MassDist b) -> bool {
      return a.second < b.second;
    };
    std::sort(res.begin(), res.end(), comp);

    if (removeFirstElement && res.size() > 0) {
      res.erase(res.begin());
    }

    if (res.size() > n) {
      res.resize(n);
    }

    return res;
  }

  //----------------------------------------------------------------------------
  // not working yet
  /*
  void addForce(std::vector<float> target, float radius, float scale) {
    std::deque<Tree::TreePtr> toProcess;
    toProcess.push_back(this);
    float length, effect, sqr = radius * radius;

    while (!toProcess.empty()) {
      Tree::TreePtr curTree = toProcess.front();
      toProcess.pop_front();
      bool inside = true;

      for (auto d = 0; d < dimension; ++d) {
        if (target[d] < curTree->bounds[d].min - radius ||
           target[d] > curTree->bounds[d].max + radius) {
          inside = false;
          break;
        }
      }

      if (inside && curTree->nParticles > 0) {
        for (auto p : curTree->particles) {
          for (auto d = 0; d < dimension; ++d) {
            float f = p->position[d] - target[d];
            // todo : continue ...
          }
        }
      }
    }
  }
  //*/

  //----------------------------------------------------------------------------
  // void addForce(std::vector<float> targetPosition,
  //               float radius,
  //               float scale,
  //               ForceTypeE type);

  // void draw();
};

template <std::size_t dimension>
std::size_t Tree<dimension>::addCount = 0;

}; // end of namespace Orbits

#endif /* Tree_h */
