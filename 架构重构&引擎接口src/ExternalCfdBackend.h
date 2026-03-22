#pragma once
#include <string>

#include "ISimulationModel.h"

class ExternalCfdBackend final : public ISimulationModel
{
public:
    ExternalCfdBackend() = default;
    ~ExternalCfdBackend() override = default;

    SimulationModelType modelType() const override
    {
        return SimulationModelType::CfdEulerian;
    }

    bool initialize(
        const Grid3D& grid,
        const ITerrain* terrain,
        const SimulationConfig& config,
        std::string& err) override
    {
        err = "ExternalCfdBackend: not implemented in this build.";
        return false;
    }

    void reset() override {}
    double step() override { return 0.0; }
    double stableDt() const override { return 1.0; }

    double time() const override { return 0.0; }
    float maxC() const override { return 0.0f; }

    const Grid3D& grid() const override { return grid_; }
    const SimulationConfig& config() const override { return config_; }

    double currentWindSpeed() const override { return 0.0; }
    double currentWindDirDeg() const override { return 0.0; }
    double currentLeakRate() const override { return 0.0; }

    void extractSliceXY(int zIndex, std::vector<float>& outSlice, float& outMax) const override
    {
        outSlice.clear();
        outMax = 0.0f;
    }

private:
    Grid3D grid_{};
    SimulationConfig config_{};
};
