#pragma once
#include <string>
#include <vector>
#include <random>

#include "Grid3D.h"
#include "ITerrain.h"

class GaussianPuffModel
{
public:
    struct Params
    {
        double windSpeed_mps = 2.0;
        double windDir_deg   = 0.0;

        double K_m2ps    = 2.0;
        double decay_1ps = 0.0;

        double srcX_m      = 0.0;
        double srcY_m      = 0.0;
        double srcZ_m      = 2.0;     // absolute world Z
        double srcRadius_m = 2.0;     // sigma0

        double leakRate_kgps = 1.0;

        double dt_s        = 0.05;
        double totalTime_s = 60.0;
        bool   autoClampDt = true;

        // If <=0, auto compute for continuity (derived from grid + wind).
        double releaseInterval_s = 0.0;

        // Same philosophy as plume: default no wind-dir noise; K already models mixing.
        struct Stochastic {
            bool enabled = false;
            unsigned int seed = 42;
            double updateEvery_s = 1.0;

            double windSpeedSigma_mps = 0.0;
            double windDirSigma_deg   = 0.0;
            double leakRelSigma       = 0.10;
        } noise;
    };

    bool initialize(const Grid3D& g, const ITerrain* terr, const Params& p, std::string& err);
    void reset();
    void step();

    double time() const { return t_; }
    const Grid3D& grid() const { return grid_; }
    Params params() const { return params_; }

    void extractSliceXY(int k, std::vector<float>& out, float& maxC) const;

private:
    struct Puff
    {
        double x=0, y=0, z=0;
        double t_emit=0;
        double mass=0;
        double sigma0=1.0;
    };

    static double deg2rad(double d);
    static double wrapAngleDeg(double a);

    void setEffectiveWind(double speed, double dirDeg);
    void refreshNoise();
    void updateNoiseIfNeeded();
    double stableDt() const;

    void buildTerrainCache();
    double Hbilinear(double x, double y) const;
    void gradH(double x, double y, double& dzdx, double& dzdy) const;
    double maxHAlong(double x0, double y0, double x1, double y1, int samples) const;

    bool isSolidAtZ(double x, double y, double z) const;
    void flowDirWithSlip(double x, double y, double zCenter, double& outDx, double& outDy) const;

    double releaseInterval() const;
    void emitUntil(double tNow);
    void advectAll(double dt);

    double sigmaAt(const Puff& pf, double age) const;

    double terrainTransmittance(double sx, double sy, double rx, double ry,
                                double zCenter, double sigmaZ) const;

    double contribution(const Puff& pf, double x, double y, double zWorld, double tNow) const;

private:
    Grid3D grid_;
    const ITerrain* terr_ = nullptr;
    Params params_;

    double t_ = 0.0;
    double lastReleaseT_ = 0.0;

    // effective
    double windSpeedEff_ = 0.0;
    double windDirEffDeg_ = 0.0;
    double u_ = 0.0;
    double v_ = 0.0;
    double leakRateEff_ = 0.0;

    // noise
    std::mt19937 rng_;
    std::normal_distribution<double> norm_{0.0, 1.0};
    double nextNoiseUpdateT_ = 0.0;

    // terrain cache
    std::vector<float> H_;

    // puffs
    std::vector<Puff> puffs_;
};
