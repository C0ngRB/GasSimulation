#include "Simulator3D.h"

#include <algorithm>

bool Simulator3D::initialize(
    const Grid3D& grid,
    const ITerrain* terrain,
    const SimulationConfig& config,
    std::string& err)
{
    err.clear();
    grid_ = grid;
    terrainRaw_ = terrain;
    config_ = config;
    config_.model = SimulationModelType::CfdEulerian;
    t_ = 0.0;
    maxC_ = 0.0f;

    try {
        grid_.validate();
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }

    if (!terrainRaw_) {
        err = "Simulator3D: terrain is null.";
        return false;
    }
    if (!terrainRaw_->isValid()) {
        err = "Simulator3D: terrain invalid.";
        return false;
    }

    if (!terrainSampler_.initialize(grid_, terrainRaw_, err)) {
        return false;
    }

    const double groundAtSource = terrainSampler_.heightBilinear(config_.srcX_m, config_.srcY_m);
    if (config_.srcZ_m <= groundAtSource + 1.0e-6) {
        err = "Simulator3D: source Z is below or at terrain.";
        return false;
    }

    TerrainAwareWindField::Params windParams;
    windParams.baseSpeed_mps = config_.windSpeed_mps;
    windParams.baseDir_deg = config_.windDir_deg;
    windParams.noise.enabled = config_.noise.enabled;
    windParams.noise.seed = config_.noise.seed;
    windParams.noise.updateEvery_s = config_.noise.updateEvery_s;
    windParams.noise.windSpeedSigma_mps = config_.noise.windSpeedSigma_mps;
    windParams.noise.windDirSigma_deg = config_.noise.windDirSigma_deg;
    windField_.setParams(windParams);

    if (!windField_.initialize(grid_, &terrainSampler_, err)) {
        return false;
    }

    LeakSourceController::Params leakParams;
    leakParams.baseLeakRate = config_.leakRate_kgps;
    leakParams.noise.enabled = config_.noise.enabled;
    leakParams.noise.seed = config_.noise.seed + 7919U;
    leakParams.noise.updateEvery_s = config_.noise.updateEvery_s;
    leakParams.noise.leakRelSigma = config_.noise.leakRelSigma;
    sourceController_.setParams(leakParams);
    sourceController_.initialize();

    EulerianSolver3D::Params solverParams;
    solverParams.K_m2ps = config_.K_m2ps;
    solverParams.decay_1ps = config_.decay_1ps;
    solverParams.srcX_m = config_.srcX_m;
    solverParams.srcY_m = config_.srcY_m;
    solverParams.srcZ_m = config_.srcZ_m;
    solverParams.srcRadius_m = config_.srcRadius_m;

    if (!solver_.initialize(grid_, &terrainSampler_, solverParams, err)) {
        return false;
    }

    return true;
}

void Simulator3D::reset()
{
    t_ = 0.0;
    maxC_ = 0.0f;
    windField_.reset();
    sourceController_.reset();
    solver_.reset();
}

double Simulator3D::stableDt() const
{
    return solver_.stableDt(windField_.representativeSpeed());
}

double Simulator3D::step()
{
    if (t_ >= config_.totalTime_s - 1.0e-12) {
        return t_;
    }

    double dt = std::max(1.0e-9, config_.dt_s);
    if (config_.autoClampDt) {
        const double dtStable = stableDt();
        if (dtStable > 0.0) {
            dt = std::min(dt, dtStable);
        }
    }

    dt = std::min(dt, config_.totalTime_s - t_);
    dt = std::max(1.0e-9, dt);

    windField_.advance(dt);
    sourceController_.advance(dt);
    solver_.step(dt, windField_, sourceController_.currentLeakRate());

    t_ += dt;
    maxC_ = solver_.maxC();
    return t_;
}

void Simulator3D::extractSliceXY(int zIndex, std::vector<float>& outSlice, float& outMax) const
{
    solver_.extractSliceXY(zIndex, outSlice, outMax);
}
