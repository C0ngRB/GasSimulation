#pragma once
#include <string>
#include <vector>

#include "Grid3D.h"
#include "ITerrain.h"
#include "SimulationConfig.h"

class ISimulationModel
{
public:
    virtual ~ISimulationModel() = default;

    virtual SimulationModelType modelType() const = 0;

    virtual bool initialize(
        const Grid3D& grid,
        const ITerrain* terrain,
        const SimulationConfig& config,
        std::string& err) = 0;

    virtual void reset() = 0;
    virtual double step() = 0;
    virtual double stableDt() const = 0;

    virtual double time() const = 0;
    virtual float maxC() const = 0;

    virtual const Grid3D& grid() const = 0;
    virtual const SimulationConfig& config() const = 0;

    virtual double currentWindSpeed() const = 0;
    virtual double currentWindDirDeg() const = 0;
    virtual double currentLeakRate() const = 0;

    virtual void extractSliceXY(int zIndex, std::vector<float>& outSlice, float& outMax) const = 0;
};
