#pragma once
#include <vector>

#include "ISimulationModel.h"
#include "LeakSourceController.h"
#include "TerrainSampler.h"
#include "WindField.h"

class GaussianPuffModel final : public ISimulationModel
{
public:
    GaussianPuffModel() = default;

    SimulationModelType modelType() const override
    {
        return SimulationModelType::GaussianPuff;
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
    struct Puff
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double age = 0.0;
        double mass = 0.0;
        double sigma0 = 1.0;
    };

    double releaseInterval() const;
    void emitUntil(double tNow);
    void advectAll(double dt);

    double sigmaAt(const Puff& pf) const;
    double terrainTransmittance(const Puff& pf, double x, double y, double zWorld, double sigma) const;
    double contribution(const Puff& pf, double x, double y, double zWorld) const;

private:
    Grid3D grid_{};
    const ITerrain* terrainRaw_{nullptr};
    SimulationConfig config_{};

    double t_{0.0};
    double lastReleaseT_{0.0};
    mutable float maxC_{0.0f};

    TerrainSampler terrainSampler_{};
    TerrainAwareWindField windField_{};
    LeakSourceController sourceController_{};

    std::vector<Puff> puffs_;
    std::size_t maxPuffs_{4000};
};
