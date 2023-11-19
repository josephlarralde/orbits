//
//  Utilities.h
//  Chordae
//
//  Created by Joseph Larralde on 29/09/2022.
//

#ifndef Utilities_h
#define Utilities_h

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
const std::vector<T> computeNormalizedVector(const std::vector<T>& v) {
  std::vector<T> res = v;
  normalizeVector(res);
  return res;
}

template <typename T>
void scaleVector(std::vector<T>& v, T s) {
  for (auto& e : v) e *= s;
}

template <typename T>
const std::vector<T> computeScaledVector(const std::vector<T>& v, T s) {
  std::vector<T> res = v;
  scaleVector(res, s);
  return res;
}

// simple running statistics ///////////////////////////////////////////////////

/**
 * compute stats in one pass (running stats) :
 * taken from  https://www.johndcook.com/blog/standard_deviation/
 * see also https://www.johndcook.com/blog/skewness_kurtosis/ for a more advanced method
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

// simple one pole filter //////////////////////////////////////////////////////

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
    
    float getValue() { return prevOutput; }
};
#endif /* Utilities_h */
