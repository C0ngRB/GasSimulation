#include "GaussianPlumeModel.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.1415926535897932384626433832795;
}

bool GaussianPlumeModel::initialize(
    const Grid3D& grid,
    const ITerrain* terrain,
    const SimulationConfig& config,
    std::string& err)
{
    err.clear();
    grid_ = grid;
    terrainRaw_ = terrain;
    config_ = config;
    config_.model = SimulationModelType::GaussianPlume;
    t_ = 0.0;
    maxC_ = 0.0f;

    try {
        grid_.validate();
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }

    if (!terrainRaw_) {
        err = "GaussianPlumeModel: terrain is null.";
        return false;
    }
    if (!terrainRaw_->isValid()) {
        err = "GaussianPlumeModel: terrain invalid.";
        return false;
    }

    if (!terrainSampler_.initialize(grid_, terrainRaw_, err)) {
        return false;
    }

    const double groundAtSource = terrainSampler_.heightBilinear(config_.srcX_m, config_.srcY_m);
    if (config_.srcZ_m <= groundAtSource + 1.0e-6) {
        err = "GaussianPlumeModel: source Z is below or at terrain.";
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

    return true;
}

void GaussianPlumeModel::reset()
{
    t_ = 0.0;
    maxC_ = 0.0f;
    windField_.reset();
    sourceController_.reset();
}

double GaussianPlumeModel::stableDt() const
{
    const double U = std::max(0.25, windField_.representativeSpeed());
    const double h = std::min({grid_.dx, grid_.dy, grid_.dz});
    return 0.50 * h / U;
}

double GaussianPlumeModel::step()
{
    if (t_ >= config_.totalTime_s - 1.0e-12) {
        return t_;
    }

    double dt = std::max(1.0e-9, config_.dt_s);
    if (config_.autoClampDt) {
        dt = std::min(dt, stableDt());
    }
    dt = std::min(dt, config_.totalTime_s - t_);
    dt = std::max(1.0e-9, dt);

    windField_.advance(dt);
    sourceController_.advance(dt);
    t_ += dt;
    return t_;
}

double GaussianPlumeModel::terrainTransmittance(double x, double y, double zWorld, double sigmaZ) const
{
    const double hMax = terrainSampler_.maxHeightAlongRay(
        config_.srcX_m, config_.srcY_m, x, y, 24);

    const double zLine = 0.5 * (config_.srcZ_m + zWorld);
    const double clearance = zLine + sigmaZ - hMax;
    const double scale = std::max(0.5, 2.0 * sigmaZ);

    const double t = std::clamp(0.5 + 0.5 * clearance / scale, 0.05, 1.0);
    return t;
}

double GaussianPlumeModel::concentrationAt(double x, double y, double zWorld) const
{
    const WindSample ws = windField_.sample(config_.srcX_m, config_.srcY_m, config_.srcZ_m);
    const double U = std::max(0.25, ws.speed);
    if (U <= 0.0) {
        return 0.0;
    }

    const double dx = x - config_.srcX_m;
    const double dy = y - config_.srcY_m;

    const double dirX = ws.u / U;
    const double dirY = ws.v / U;

    const double s = dx * dirX + dy * dirY;
    if (s <= 0.0) {
        return 0.0;
    }

    const double cross = -dx * dirY + dy * dirX;
    const double travelTime = s / U;
    const double sigma0 = std::max(config_.srcRadius_m, 0.5 * std::min({grid_.dx, grid_.dy, grid_.dz}));

    const double sigmaY = std::sqrt(
        sigma0 * sigma0 +
        2.0 * std::max(0.0, config_.K_m2ps) * travelTime +
        std::pow(0.08 * s, 2.0));

    const double sigmaZ = std::sqrt(
        sigma0 * sigma0 +
        2.0 * std::max(0.0, config_.K_m2ps) * travelTime +
        std::pow(0.04 * s, 2.0));

    const double groundR = terrainSampler_.heightBilinear(x, y);
    const double sourceZ = std::max(config_.srcZ_m, terrainSampler_.heightBilinear(config_.srcX_m, config_.srcY_m) + 0.25 * grid_.dz);
    const double z = std::max(zWorld, groundR + 0.01);

    const double q = std::max(0.0, sourceController_.currentLeakRate());
    if (q <= 0.0) {
        return 0.0;
    }

    const double norm = q / (2.0 * kPi * U * sigmaY * sigmaZ);
    const double ey = std::exp(-0.5 * (cross * cross) / (sigmaY * sigmaY));

    const double ez1 = std::exp(-0.5 * std::pow((z - sourceZ) / sigmaZ, 2.0));
    const double ez2 = std::exp(-0.5 * std::pow((z - (2.0 * groundR - sourceZ)) / sigmaZ, 2.0));

    const double decay = std::exp(-std::max(0.0, config_.decay_1ps) * travelTime);
    const double startup = 1.0 - std::exp(-t_ / std::max(1.0, 0.1 * config_.totalTime_s));
    const double trans = terrainTransmittance(x, y, z, sigmaZ);

    double c = norm * ey * (ez1 + ez2) * decay * startup * trans;
    if (!std::isfinite(c) || c < 0.0) {
        c = 0.0;
    }
    return c;
}

void GaussianPlumeModel::extractSliceXY(int zIndex, std::vector<float>& outSlice, float& outMax) const
{
    zIndex = std::clamp(zIndex, 0, grid_.Nz - 1);
    outSlice.assign(static_cast<std::size_t>(grid_.Nx) * static_cast<std::size_t>(grid_.Ny), 0.0f);
    outMax = 0.0f;

    const double z = grid_.z(zIndex);
    for (int j = 0; j < grid_.Ny; ++j) {
        const double y = grid_.y(j);
        for (int i = 0; i < grid_.Nx; ++i) {
            const double x = grid_.x(i);
            const float c = static_cast<float>(concentrationAt(x, y, z));
            outSlice[static_cast<std::size_t>(i) + static_cast<std::size_t>(j) * static_cast<std::size_t>(grid_.Nx)] = c;
            outMax = std::max(outMax, c);
        }
    }

    maxC_ = outMax;
}
