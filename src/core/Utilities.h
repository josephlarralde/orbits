//
//  Utilities.h
//  ORBITS
//
//  Created by Joseph Larralde on 29/09/2022.
//

#ifndef Utilities_h
#define Utilities_h

// VECTOR OPERATIONS ///////////////////////////////////////////////////////////

template <typename T>
std::vector<T> addVectors(const std::vector<T>& v1, const std::vector<T>& v2) {
  std::size_t size = std::min(v1.size(), v2.size());
  std::vector<T> res(size, 0.f);
  for (std::size_t i = 0; i < size; ++i) {
    res[i] = v1[i] + v2[i];
  }
  return res;
}

template <typename T>
T computeVectorNorm(const std::vector<T>& v) {
  T res = 0;
  for (auto& e : v) res += e * e;
  return std::sqrt(res);
}

template <typename T>
void normalizeVector(std::vector<T>& v) {
  T normRatio = 1 / computeVectorNorm(v);
  for (auto& e : v) e *= normRatio;
}

template <typename T>
std::vector<T> computeNormalizedVector(const std::vector<T>& v) {
  std::vector<T> res = v;
  normalizeVector(res);
  return res;
}

template <typename T>
void scaleVector(std::vector<T>& v, T s) {
  for (auto& e : v) e *= s;
}

template <typename T>
std::vector<T> computeScaledVector(const std::vector<T>& v, T s) {
  std::vector<T> res = v;
  scaleVector(res, s);
  return res;
}

// N-SPHERE COORDINATES ////////////////////////////////////////////////////////

// see https://en.wikipedia.org/wiki/N-sphere#Spherical_coordinates

// accepts a vector containing [x1, x2, ..., xN]
// returns a vector containing [r, phi1, phi2, ..., phiN-1]
template <typename T>
std::vector<T> cartesianToSpherical(const std::vector<T>& coords) {
  T squareSum = 0;
  std::vector<T> res(coords.size(), 0);

  for (std::size_t i = coords.size() - 1; i > 0; --i) {
      squareSum += coords[i] * coords[i];
      res[i] = atan2<T>(sqrt<T>(squareSum), coords[i - 1]);
  }

  res[0] = sqrt<T>(squareSum + coords[0] * coords[0]);
  return res;
}

// accepts a vector of angles [phi1, phi2, ..., phiN-1] and an optional radius
// returns a vector containing [x1, x2, ..., xN]
template <typename T>
std::vector<T> sphericalToCartesian(const std::vector<T>& angles, T r = 1) {
  T sinProduct = 1;
  std::vector<T> res(angles.size() + 1, 0);

  for (std::size_t i = 0; i < angles.size(); ++i) {
    res[i] = r * sinProduct * cos<T>(angles[i]);
    sinProduct *= sin<T>(angles[i]);
  }

  res[angles.size()] = sinProduct;
  return res;
}

// SIMPLE RUNNING STATISTICS ///////////////////////////////////////////////////

/**
 * compute stats in one pass (running stats) :
 * taken from  https://www.johndcook.com/blog/standard_deviation/
 * see also https://www.johndcook.com/blog/skewness_kurtosis/ for a more
 * advanced method
 */

class RunningStats {
public:
    RunningStats() : m_n(0) {}

    void clear() {
        m_n = 0;
    }

    void push(double x) {
        m_n++;

        // See Knuth TAOCP vol 2, 3rd edition, page 232
        if (m_n == 1) {
            m_oldM = m_newM = x;
            m_oldS = 0.0;
        } else {
            m_newM = m_oldM + (x - m_oldM) / m_n;
            m_newS = m_oldS + (x - m_oldM) * (x - m_newM);

            // set up for next iteration
            m_oldM = m_newM;
            m_oldS = m_newS;
        }
    }

    int size() const {
        return m_n;
    }

    double getMean() const {
        return (m_n > 0) ? m_newM : 0.0;
    }

    double getVariance() const {
        return (m_n > 1) ? (m_newS / (m_n - 1)) : 0.0;
    }

    double getStdDev() const {
        return sqrt(getVariance());
    }

private:
    int m_n;
    double m_oldM, m_newM, m_oldS, m_newS;
};

// SIMPLE ONEPOLE FILTER ///////////////////////////////////////////////////////

class OnePole {
    float prevOutput;
    float alpha, oneMinusAlpha;

public:
    OnePole(float a = 0) : prevOutput(0) { setAlpha(a); }
    
    void setAlpha(float a) { alpha = a; oneMinusAlpha = 1.f - a; }
    void reset() { prevOutput = 0; }
    
    float process(float input) {
        prevOutput = alpha * input + oneMinusAlpha * prevOutput;
        return prevOutput;
    }
    
    float getValue() const { return prevOutput; }
};
#endif /* Utilities_h */
