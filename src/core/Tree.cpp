#include <bitset>
#include "Tree.h"

namespace Orbits {

template <std::size_t dimension>
float Tree<dimension>::clipToMinMaxDimensionBounds(float v)
{
  return std::max(
    static_cast<float>(DEFAULT_MIN_DIMENSION_BOUND),
    std::min(static_cast<float>(DEFAULT_MAX_DIMENSION_BOUND), v)
  );
}

template <std::size_t dimension>
Tree<dimension>::Tree()
{
  initialize();
}

template <std::size_t dimension>
void Tree<dimension>::initialize()
{
  hasChildren = false;
  nParticles = 0;
  particles = std::vector<MassPtr>(maxParticles, nullptr);
  bounds = std::vector<mmm>(dimension, { 0.f, 0.f, 0.f });
  subtrees = std::vector<TreePtr>(pow(2, dimension), nullptr);
}

template <std::size_t dimension>
void Tree<dimension>::setBounds(const std::vector<std::pair<float,float>>& _bounds)
{
  if (_bounds.size() != bounds.size()) return; // throw exception ?

  for (std::size_t i = 0; i < _bounds.size(); ++i) {
    auto& dim = bounds[i];
    dim.min = clipToMinMaxDimensionBounds(_bounds[i].first);
    dim.max = clipToMinMaxDimensionBounds(_bounds[i].second);
  }
  setMid();
}

template <std::size_t dimension>
void Tree<dimension>::setMid()
{
  for (auto& dim : bounds) {
    dim.mid = (dim.min + dim.max) * 0.5f;
  }
}

template <std::size_t dimension>
void Tree<dimension>::add(MassPtr cur)
{
  if (hasChildren) {
    // we compute the subtree index value bit by bit (LSB to MSB),
    // using each dimension's position (below and above bounds[d].mid)
    std::bitset<sizeof(std::size_t) * CHAR_BIT> bitIndex{0};

    for (std::size_t d = 0; d < dimension; ++d) {
      bitIndex[d] = cur->getPosition()[d] > bounds[d].mid;
    }

    subtrees[bitIndex.to_ulong()]->add(cur);
  } else {
    if (nParticles < maxParticles) {
      particles[nParticles].swap(cur);
      nParticles++;
    } else {
      for (auto i = 0; i < subtrees.size(); ++i) {
        std::vector<std::pair<float,float>> subBounds;
        std::bitset<sizeof(std::size_t) * CHAR_BIT> bitIndex{
          static_cast<std::size_t>(i)
        };

        for (auto d = 0; d < dimension; ++d) {
          std::pair<float,float> dimBounds;

          if (bitIndex[int(d)]) {
            dimBounds.first = bounds[d].mid;
            dimBounds.second = bounds[d].max;
          } else {
            dimBounds.first = bounds[d].min;
            dimBounds.second = bounds[d].mid;
          }

          subBounds.push_back(dimBounds);
        }

        subtrees[i].reset(new Tree<dimension>());
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

template <std::size_t dimension>
void Tree<dimension>::setup(const std::vector<MassPtr>& all) {
  int n = all.size();

  if (n > 0) {
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
}

template <std::size_t dimension>
std::vector<std::pair<typename Tree<dimension>::MassPtr,float>>
Tree<dimension>::getNeighborsAndDistances(std::vector<float> targetPosition,
                               float radius)
{
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

template <std::size_t dimension>
std::vector<typename Tree<dimension>::MassPtr>
Tree<dimension>::getNeighbors(std::vector<float> targetPosition,
                   float radius)
{
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

template <std::size_t dimension>
void
Tree<dimension>::getIntersection(std::vector<Tree::MassPtr>& intersection,
                      const std::vector<float>& targetPosition,
                      float radius)
{
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

template <std::size_t dimension>
void
Tree<dimension>::addForce(std::vector<float> target, float radius, float scale)
{
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

// void Tree::addForce(std::vector<float> targetPosition,
//                     float radius,
//                     float scale,
//                     ForceTypeE type)
// {
//   std::deque<Tree*> toProcess;
//   toProcess.push_back(this);
//   float xd, yd, length, effect;
//   float sqradius = radius * radius;

//   while(!toProcess.empty()) {
//     Tree& curTree = *(toProcess.front());
//     toProcess.pop_front();

//     if(targetX > curTree.minX - radius && targetX < curTree.maxX + radius &&
//        targetY > curTree.minY - radius && targetY < curTree.maxY + radius)
//     {
//       if(curTree.nParticles) {
//         for(int i = 0; i < curTree.nParticles; i++) {
//           Particle& curParticle = *(curTree.particles[i]);
//           xd = curParticle.x - targetX;
//           yd = curParticle.y - targetY;

//           if(xd != 0 && yd != 0) {
//             length = xd * xd + yd * yd;

//             if(length < sqradius) {
//               length = sqrtf(length);
//               xd /= length;
//               yd /= length;
//               effect = 1 - (length / radius);
//               effect *= scale;
//               curParticle.xf += effect * xd;
//               curParticle.yf += effect * yd;

//               if(type == SilhouetteForceE) {
//                 // set distance to silhouette here !!!!!!!!!
//                 curParticle.updateSilhouetteParameters(abs(effect));
//                 //cout << effect << endl;
//               }
//             }
//           }
//         }
//       } else if(curTree.hasChildren) {
//         toProcess.push_back(curTree.nw);
//         toProcess.push_back(curTree.ne);
//         toProcess.push_back(curTree.sw);
//         toProcess.push_back(curTree.se);
//       }
//     }
//   }
// }

// void draw() {
//     ofDrawRectangle(
//         bounds[0].min,
//         bounds[1].min,
//         bounds[0].max - bounds[0].min,
//         bounds[1].max - bounds[1].min
//     );
    
//     if(hasChildren) {
//         for (auto subtree : subtrees) {
//             subtree->draw();
//         }
//     }
// }

}; // end namespace Orbits
