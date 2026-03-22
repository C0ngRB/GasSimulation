#pragma once
#include <algorithm>
#include <cmath>
#include <random>
#include <string>

#include "Grid3D.h"
#include "TerrainSampler.h"

struct WindSample
{
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
    double speed = 0.0;
    double dirDeg = 0.0;
};

class IWindField
{
public:
    virtual ~IWindField() = default;

    virtual bool initialize(const Grid3D& grid, const TerrainSampler* terrain, std::string& err) = 0;
    virtual void reset() = 0;
    virtual void advance(double dt) = 0;
    virtual WindSample sample(double x, double y, double z) const = 0;

    virtual double representativeSpeed() const = 0;
    virtual double currentDirectionDeg() const = 0;
    virtual double time() const = 0;
};

class TerrainAwareWindField final : public IWindField
{
public:
    struct NoiseParams
    {
        bool enabled = true;
        unsigned int seed = 42;
        double updateEvery_s = 1.0;
        double windSpeedSigma_mps = 0.30;
        double windDirSigma_deg = 7.0;
    };

    struct Params
    {
        double baseSpeed_mps = 2.0;
        double baseDir_deg = 0.0;
        double blendHeight_m = 6.0;
        double slopeSlipGain = 0.90;
        double ridgeSpeedupGain = 0.30;
        NoiseParams noise;
    };

    TerrainAwareWindField() = default;
    explicit TerrainAwareWindField(const Params& p) : params_(p) {}

    void setParams(const Params& p) { params_ = p; }
    const Params& params() const { return params_; }

    bool initialize(const Grid3D& grid, const TerrainSampler* terrain, std::string& err) override;
    void reset() override;
    void advance(double dt) override;
    WindSample sample(double x, double y, double z) const override;

    double representativeSpeed() const override;
    double currentDirectionDeg() const override { return effDir_deg_; }
    double time() const override { return t_; }

private:
    static double deg2rad(double d);
    static double rad2deg(double r);
    static double wrapAngleDeg(double a);

    void refreshNoise();
    void setDirectionFromDeg(double deg);

private:
    Grid3D grid_{};
    const TerrainSampler* terrain_{nullptr};
    Params params_{};

    double t_{0.0};
    double nextNoiseUpdateT_{0.0};

    double effSpeed_mps_{0.0};
    double effDir_deg_{0.0};
    double dirX_{1.0};
    double dirY_{0.0};

    std::mt19937 rng_{};
    std::normal_distribution<double> norm_{0.0, 1.0};
};
