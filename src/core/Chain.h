#ifndef Chain_h
#define Chain_h

#include "Mass.h"
#include "Spring.h"
#include "Utilities.h"

template <std::uint8_t dimension>
class Chain {
public:
    struct Stats {
        float min = 0;
        float max = 0;
        float mean = 0;
        float stdDev = 0;
    };
    
    template <std::uint8_t d>
    struct StateT {
        Chain::Stats speed;
        std::vector<float> speedCentroid = std::vector<float>(d,0);
        std::vector<Chain::Stats> position = std::vector<Chain::Stats>(d);
    };
    typedef StateT<dimension> State;
    
private:
    typedef Mass<dimension> Mass;
    typedef Spring<dimension> Spring;
    
    typedef std::shared_ptr<Mass> MassPtr;
    typedef std::shared_ptr<Spring> SpringPtr;
    
    std::vector<MassPtr> masses;
    std::vector<SpringPtr> springs;
    
    RunningStats runningStats;    
    State state;

public:
    Chain(std::uint16_t nm = 3) {
        setNbOfMasses(nm);
    }

    ~Chain() {}
    
    std::uint16_t size() {
        return masses.size();
    }
    
    void setK(float _k) {
        for (auto s : springs) {
            s->setK(_k);
        }
    }
    
    void setF(float _f) {
        for (auto m : masses) {
            m->setF(_f);
        }
    }
    
    void setM(float _m) {
        for (auto m : masses) {
            m->setM(_m);
        }
    }
    
    void setD(float _d) {
        for (auto m : masses) {
            m->setD(_d);
        }
    }
    
    void resetForce() {
        for (auto m : masses) {
            m->resetForce();
        }
    }
    
    MassPtr getMass(std::uint16_t index) {
        if (index >= masses.size()) {
            index = 0;
        }
        
        return masses[index];
    }
    
    std::vector<MassPtr>& getMasses() {
        return masses;
    }
        
    const std::vector<float>& getMassPosition(std::uint16_t index) {
        if (index >= masses.size()) {
            index = 0;
        }
        
        return masses[index]->getPosition();
    }
    
    void setMassPosition(std::uint16_t index, std::vector<float> newPos) {
        if (index < masses.size()) {
            masses[index]->setPosition(newPos);
        }
    }
    
    /*
    void computeSpeedCentroid() {
        float maxSpeed = 0;
        float speedSum = 0;
        std::vector<float> dimSums(dimension + 1, 0);
        
        for (auto m : masses) {
            float speed = m->getSpeedNorm();
            maxSpeed = (speed > maxSpeed) ? speed : maxSpeed;
            speedSum += speed;
            for (std::uint8_t d = 0; d < dimension; ++d) {
                dimSums[d] += (speed * m->getPosition()[d]);
            }
        }
        
        for (auto& sum : dimSums) {
            sum /= speedSum;
        }
        
        dimSums[dimension] = speedSum / masses.size(); // maxSpeed;
        
        return dimSums;
    }
    //*/
    
    void computeState() {
        for (std::uint8_t d = 0; d < dimension + 1; ++d) {
            runningStats.clear();
            state.speedCentroid[d] = 0;
            auto& stats = (d < dimension) ? state.position[d] : state.speed;
            stats.min = std::numeric_limits<float>::max();
            stats.max = std::numeric_limits<float>::min();

            for (auto m : masses) {
                float val = (d < dimension)
                    ? m->getPosition()[d]
                    : m->getSpeedNorm();
                
                if (val < stats.min) stats.min = val;
                if (val > stats.max) stats.max = val;
                runningStats.push(val);
            }
            
            stats.mean = runningStats.getMean();
            stats.stdDev = runningStats.getStdDev();
        }
        
        float speedSum = 0;

        for (auto m : masses) {
            float speed = m->getSpeedNorm();
            speedSum += speed;
            for (std::uint8_t d = 0; d < dimension; ++d) {
                state.speedCentroid[d] += (speed * m->getPosition()[d]);
            }
        }
        
        for (auto& sum : state.speedCentroid) {
            sum /= speedSum;
        }
    }
    
    Chain::State& getState() {
        return state;
    }
    
    void setNbOfMasses(std::uint16_t nm) {
        masses = std::vector<MassPtr>(nm + 2, nullptr);
        springs = std::vector<SpringPtr>(nm + 1, nullptr);
        
        for (std::uint16_t i = 0; i < masses.size(); ++i) {
            masses[i] = std::make_shared<Mass>();
        }
        
        for (std::uint16_t i = 0; i < springs.size(); ++i) {
            springs[i] = std::make_shared<Spring>();
            springs[i]->setMasses({ masses[i], masses[i + 1] });
        }
    }
    
    std::pair<const std::vector<float>&, const std::vector<float>&> getEnds() {
        return {
            masses[0]->getPosition(),
            masses[masses.size() - 1]->getPosition()
        };
    }
    
    void setEnds(std::pair<std::vector<float>, std::vector<float>> pos)  {
        masses[0]->setPosition(pos.first);
        masses[masses.size() - 1]->setPosition(pos.second);
    }
    
    void update() {
        for (std::uint16_t i = 0; i < springs.size(); ++i) {
            springs[i]->update();
        }

        // masses[i]->update() has to be called last, after all forces
        // are applied (springs, gravity, collisions, force fields, etc ...)
        
        for (std::uint16_t i = 1; i < masses.size() - 1; ++i) {
            masses[i]->update();
        }
        
        computeState();
    }
};

#endif /* Chain_h */
