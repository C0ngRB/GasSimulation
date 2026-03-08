#pragma once
#include <string>
#include <vector>
#include <random>

#include "Grid3D.h"
#include "ITerrain.h"

class GaussianPlumeModel
{
public:
    struct Params
    {
        double windSpeed_mps = 2.0;
        double windDir_deg   = 0.0;   // 0=+x, 90=+y

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

        // 物理上：K 已经代表随机湍混。在复杂地形下再对风向抖动会造成"分叉绕行"离谱。
        // 因此默认：只允许对 leakRate 做小扰动（可关），风速/风向扰动默认 0。
        struct Stochastic {
            bool enabled = false;
            unsigned int seed = 42;
            double updateEvery_s = 1.0;

            double windSpeedSigma_mps = 0.0; // 默认 0
            double windDirSigma_deg   = 0.0; // 默认 0
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
    // helpers
    static double deg2rad(double d);
    static double wrapAngleDeg(double a);

    void setEffectiveWind(double speed, double dirDeg);
    void refreshNoise();
    void updateNoiseIfNeeded();

    double stableDt() const;

    // terrain sampling
    void buildTerrainCache();
    double Hbilinear(double x, double y) const;
    void gradH(double x, double y, double& dzdx, double& dzdy) const;
    double maxHAlong(double x0, double y0, double x1, double y1, int samples) const;

    // obstacle/slip flow at z=srcZ
    bool isSolidAtZ(double x, double y, double z) const;
    void flowDirWithSlip(double x, double y, double zCenter, double& outDx, double& outDy) const;

    // centerline
    void rebuildCenterline();
    void ensureCenterline(double frontDist) const;
    bool projectToCenterline(double x, double y,
                             double& outS, double& outR,
                             double& outCx, double& outCy,
                             double& outUx, double& outUy) const;

    // transmittance (soft shadow)
    double terrainTransmittance(double cx, double cy, double rx, double ry,
                                double zCenter, double sigmaZ) const;

    // concentration
    double concentrationAt(double x, double y, double zWorld) const;

private:
    Grid3D grid_;
    const ITerrain* terr_ = nullptr;
    Params params_;

    double t_ = 0.0;

    // effective parameters
    double windSpeedEff_ = 0.0;
    double windDirEffDeg_ = 0.0;
    double u_ = 0.0;
    double v_ = 0.0;
    double leakRateEff_ = 0.0;

    // noise state
    std::mt19937 rng_;
    std::normal_distribution<double> norm_{0.0, 1.0};
    double nextNoiseUpdateT_ = 0.0;

    // terrain cache (Nx*Ny)
    std::vector<float> H_;

    // centerline polyline
    mutable std::vector<double> clx_, cly_, cls_;
};
