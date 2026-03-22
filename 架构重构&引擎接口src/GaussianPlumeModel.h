#pragma once
#include "ISimulationModel.h"
#include "LeakSourceController.h"
#include "TerrainSampler.h"
#include "WindField.h"

class GaussianPlumeModel final : public ISimulationModel
{
public:
    GaussianPlumeModel() = default;

    SimulationModelType modelType() const override
    {
        return SimulationModelType::GaussianPlume;
    }

    bool initialize(
        const Grid3D& grid,
        const ITerrain* terrain,
        const SimulationConfig& config,
        std::string& err) override;

    void reset() override;
    double step() override;
    double stableDt() const override;

    double time() const override { return t_; }
    float maxC() const override { return maxC_; }

    const Grid3D& grid() const override { return grid_; }
    const SimulationConfig& config() const override { return config_; }

    double currentWindSpeed() const override { return windField_.representativeSpeed(); }
    double currentWindDirDeg() const override { return windField_.currentDirectionDeg(); }
    double currentLeakRate() const override { return sourceController_.currentLeakRate(); }

    void extractSliceXY(int zIndex, std::vector<float>& outSlice, float& outMax) const override;

private:
    double terrainTransmittance(double x, double y, double zWorld, double sigmaZ) const;
    double concentrationAt(double x, double y, double zWorld) const;

private:
    Grid3D grid_{};
    const ITerrain* terrainRaw_{nullptr};
    SimulationConfig config_{};

    double t_{0.0};
    mutable float maxC_{0.0f};

    TerrainSampler terrainSampler_{};
    TerrainAwareWindField windField_{};
    LeakSourceController sourceController_{};
};
