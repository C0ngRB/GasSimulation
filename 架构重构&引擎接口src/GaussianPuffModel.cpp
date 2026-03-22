#include "GaussianPuffModel.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kSqrt2Pi3 = 15.749609945722419;
}

bool GaussianPuffModel::initialize(
    const Grid3D& grid,
    const ITerrain* terrain,
    const SimulationConfig& config,
    std::string& err)
{
    err.clear();
    grid_ = grid;
    terrainRaw_ = terrain;
    config_ = config;
    config_.model = SimulationModelType::GaussianPuff;
    t_ = 0.0;
    lastReleaseT_ = 0.0;
    maxC_ = 0.0f;
    puffs_.clear();

    try {
        grid_.validate();
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }

    if (!terrainRaw_) {
        err = "GaussianPuffModel: terrain is null.";
        return false;
    }
    if (!terrainRaw_->isValid()) {
        err = "GaussianPuffModel: terrain invalid.";
        return false;
    }

    if (!terrainSampler_.initialize(grid_, terrainRaw_, err)) {
        return false;
    }

    const double groundAtSource = terrainSampler_.heightBilinear(config_.srcX_m, config_.srcY_m);
    if (config_.srcZ_m <= groundAtSource + 1.0e-6) {
        err = "GaussianPuffModel: source Z is below or at terrain.";
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

void GaussianPuffModel::reset()
{
    t_ = 0.0;
    lastReleaseT_ = 0.0;
    maxC_ = 0.0f;
    puffs_.clear();
    windField_.reset();
    sourceController_.reset();
}

double GaussianPuffModel::stableDt() const
{
    const double U = std::max(0.25, windField_.representativeSpeed());
    const double h = std::min({grid_.dx, grid_.dy, grid_.dz});
    return 0.40 * h / U;
}

double GaussianPuffModel::releaseInterval() const
{
    if (config_.releaseInterval_s > 0.0) {
        return config_.releaseInterval_s;
    }

    const double U = std::max(0.50, windField_.representativeSpeed());
    const double h = std::min(grid_.dx, grid_.dy);
    return std::clamp(0.50 * h / U, 0.05, 0.75);
}

void GaussianPuffModel::emitUntil(double tNow)
{
    const double interval = releaseInterval();
    while (lastReleaseT_ + interval <= tNow + 1.0e-12) {
        lastReleaseT_ += interval;

        Puff pf;
        pf.x = config_.srcX_m;
        pf.y = config_.srcY_m;
        pf.z = std::max(
            config_.srcZ_m,
            terrainSampler_.heightBilinear(config_.srcX_m, config_.srcY_m) + 0.25 * grid_.dz);
        pf.age = 0.0;
        pf.mass = std::max(0.0, sourceController_.currentLeakRate()) * interval;
        pf.sigma0 = std::max(config_.srcRadius_m, 0.5 * std::min({grid_.dx, grid_.dy, grid_.dz}));

        if (pf.mass > 0.0) {
            puffs_.push_back(pf);
        }
    }

    if (puffs_.size() > maxPuffs_) {
        const std::size_t eraseCount = puffs_.size() - maxPuffs_;
        puffs_.erase(puffs_.begin(), puffs_.begin() + static_cast<std::ptrdiff_t>(eraseCount));
    }
}

void GaussianPuffModel::advectAll(double dt)
{
    const int subSteps = std::max(1, static_cast<int>(std::ceil(dt / 0.10)));
    const double h = dt / static_cast<double>(subSteps);

    for (Puff& pf : puffs_) {
        for (int s = 0; s < subSteps; ++s) {
            const WindSample ws = windField_.sample(pf.x, pf.y, pf.z);
            pf.x += ws.u * h;
            pf.y += ws.v * h;
            pf.z += ws.w * h;

            const double ground = terrainSampler_.heightBilinear(pf.x, pf.y);
            pf.z = std::max(pf.z, ground + 0.25 * grid_.dz);
            pf.age += h;

            if (config_.decay_1ps > 0.0) {
                pf.mass *= std::exp(-config_.decay_1ps * h);
            }
        }
    }

    const double marginX = 6.0 * (grid_.Nx * grid_.dx);
    const double marginY = 6.0 * (grid_.Ny * grid_.dy);

    puffs_.erase(
        std::remove_if(
            puffs_.begin(),
            puffs_.end(),
            [&](const Puff& pf) {
                if (pf.mass < 1.0e-12) return true;
                if (pf.age > 2.0 * config_.totalTime_s) return true;

                const bool outX = (pf.x < grid_.x0 - marginX) ||
                                  (pf.x > grid_.x0 + (grid_.Nx - 1) * grid_.dx + marginX);
                const bool outY = (pf.y < grid_.y0 - marginY) ||
                                  (pf.y > grid_.y0 + (grid_.Ny - 1) * grid_.dy + marginY);

                return outX || outY;
            }),
        puffs_.end());
}

double GaussianPuffModel::sigmaAt(const Puff& pf) const
{
    const double age = std::max(1.0e-6, pf.age);
    return std::sqrt(
        pf.sigma0 * pf.sigma0 +
        2.0 * std::max(0.0, config_.K_m2ps) * age +
        std::pow(0.03 * std::max(0.5, windField_.representativeSpeed()) * age, 2.0));
}

double GaussianPuffModel::terrainTransmittance(const Puff& pf, double x, double y, double zWorld, double sigma) const
{
    const double hMax = terrainSampler_.maxHeightAlongRay(pf.x, pf.y, x, y, 24);
    const double zLine = 0.5 * (pf.z + zWorld);
    const double clearance = zLine + sigma - hMax;
    const double scale = std::max(0.5, 2.0 * sigma);
    return std::clamp(0.5 + 0.5 * clearance / scale, 0.05, 1.0);
}

double GaussianPuffModel::contribution(const Puff& pf, double x, double y, double zWorld) const
{
    const double sigma = sigmaAt(pf);
    const double dz = zWorld - pf.z;
    if (std::abs(dz) > 4.0 * sigma) {
        return 0.0;
    }

    const double dx = x - pf.x;
    const double dy = y - pf.y;
    const double r2 = dx * dx + dy * dy + dz * dz;

    const double norm = pf.mass / (kSqrt2Pi3 * sigma * sigma * sigma);
    const double gauss = std::exp(-0.5 * r2 / (sigma * sigma));
    const double trans = terrainTransmittance(pf, x, y, zWorld, sigma);

    double c = norm * gauss * trans;
    if (!std::isfinite(c) || c < 0.0) {
        c = 0.0;
    }
    return c;
}

double GaussianPuffModel::step()
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

    const double tNext = t_ + dt;
    emitUntil(tNext);
    advectAll(dt);

    t_ = tNext;
    return t_;
}

void GaussianPuffModel::extractSliceXY(int zIndex, std::vector<float>& outSlice, float& outMax) const
{
    zIndex = std::clamp(zIndex, 0, grid_.Nz - 1);
    outSlice.assign(static_cast<std::size_t>(grid_.Nx) * static_cast<std::size_t>(grid_.Ny), 0.0f);
    outMax = 0.0f;

    const double zSlice = grid_.z(zIndex);

    for (const Puff& pf : puffs_) {
        const double sigma = sigmaAt(pf);
        if (std::abs(zSlice - pf.z) > 4.0 * sigma) {
            continue;
        }

        const double span = 4.0 * sigma;
        const int i0 = std::clamp(
            static_cast<int>(std::floor((pf.x - span - grid_.x0) / grid_.dx)),
            0, grid_.Nx - 1);
        const int i1 = std::clamp(
            static_cast<int>(std::ceil((pf.x + span - grid_.x0) / grid_.dx)),
            0, grid_.Nx - 1);
        const int j0 = std::clamp(
            static_cast<int>(std::floor((pf.y - span - grid_.y0) / grid_.dy)),
            0, grid_.Ny - 1);
        const int j1 = std::clamp(
            static_cast<int>(std::ceil((pf.y + span - grid_.y0) / grid_.dy)),
            0, grid_.Ny - 1);

        for (int j = j0; j <= j1; ++j) {
            const double y = grid_.y(j);
            for (int i = i0; i <= i1; ++i) {
                const double x = grid_.x(i);
                const double c = contribution(pf, x, y, zSlice);
                const std::size_t id = static_cast<std::size_t>(i) +
                                       static_cast<std::size_t>(j) * static_cast<std::size_t>(grid_.Nx);
                outSlice[id] += static_cast<float>(c);
            }
        }
    }

    for (float v : outSlice) {
        outMax = std::max(outMax, v);
    }
    maxC_ = outMax;
}
