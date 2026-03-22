#include "WindField.h"

double TerrainAwareWindField::deg2rad(double d)
{
    constexpr double kPi = 3.1415926535897932384626433832795;
    return d * kPi / 180.0;
}

double TerrainAwareWindField::rad2deg(double r)
{
    constexpr double kPi = 3.1415926535897932384626433832795;
    return r * 180.0 / kPi;
}

double TerrainAwareWindField::wrapAngleDeg(double a)
{
    while (a >= 180.0) a -= 360.0;
    while (a < -180.0) a += 360.0;
    return a;
}

bool TerrainAwareWindField::initialize(const Grid3D& grid, const TerrainSampler* terrain, std::string& err)
{
    err.clear();
    grid_ = grid;
    terrain_ = terrain;

    if (!terrain_) {
        err = "TerrainAwareWindField: terrain sampler is null.";
        return false;
    }

    t_ = 0.0;
    rng_ = std::mt19937(params_.noise.seed);
    nextNoiseUpdateT_ = 0.0;
    refreshNoise();
    return true;
}

void TerrainAwareWindField::reset()
{
    t_ = 0.0;
    rng_ = std::mt19937(params_.noise.seed);
    nextNoiseUpdateT_ = 0.0;
    refreshNoise();
}

void TerrainAwareWindField::advance(double dt)
{
    t_ += std::max(0.0, dt);
    if (!params_.noise.enabled) {
        return;
    }

    if (t_ + 1.0e-12 >= nextNoiseUpdateT_) {
        refreshNoise();
    }
}

void TerrainAwareWindField::setDirectionFromDeg(double deg)
{
    effDir_deg_ = wrapAngleDeg(deg);
    const double th = deg2rad(effDir_deg_);
    dirX_ = std::cos(th);
    dirY_ = std::sin(th);
}

void TerrainAwareWindField::refreshNoise()
{
    double speed = params_.baseSpeed_mps;
    double dir = params_.baseDir_deg;

    if (params_.noise.enabled) {
        speed += norm_(rng_) * params_.noise.windSpeedSigma_mps;
        dir += norm_(rng_) * params_.noise.windDirSigma_deg;
    }

    effSpeed_mps_ = std::max(0.0, speed);
    setDirectionFromDeg(dir);
    nextNoiseUpdateT_ = t_ + std::max(1.0e-6, params_.noise.updateEvery_s);
}

double TerrainAwareWindField::representativeSpeed() const
{
    return effSpeed_mps_ * std::clamp(1.0 + params_.ridgeSpeedupGain, 1.0, 2.0);
}

WindSample TerrainAwareWindField::sample(double x, double y, double z) const
{
    WindSample out;
    if (!terrain_) {
        return out;
    }

    const double ground = terrain_->heightBilinear(x, y);
    const double zAgl = std::max(0.0, z - ground);

    const double blendH = std::max(0.25, params_.blendHeight_m);
    const double verticalFactor = std::clamp(
        0.25 + 0.75 * (1.0 - std::exp(-zAgl / blendH)),
        0.25,
        1.0);

    double fx = dirX_;
    double fy = dirY_;

    double dzdx = 0.0;
    double dzdy = 0.0;
    terrain_->gradient(x, y, dzdx, dzdy);

    const double slopeMag = std::hypot(dzdx, dzdy);
    if (slopeMag > 1.0e-12) {
        const double nx = dzdx / slopeMag;
        const double ny = dzdy / slopeMag;

        const double intoSlope = std::max(0.0, fx * nx + fy * ny);
        fx -= params_.slopeSlipGain * intoSlope * nx;
        fy -= params_.slopeSlipGain * intoSlope * ny;

        double fn = std::hypot(fx, fy);
        if (fn < 1.0e-12) {
            double tx = -ny;
            double ty = nx;
            if (tx * dirX_ + ty * dirY_ < 0.0) {
                tx = -tx;
                ty = -ty;
            }
            fx = tx;
            fy = ty;
            fn = std::hypot(fx, fy);
        }
        fx /= fn;
        fy /= fn;
    }

    const double terrainSpeedup = std::clamp(
        1.0 + params_.ridgeSpeedupGain * std::min(1.0, slopeMag),
        0.5,
        2.0);

    const double speed = effSpeed_mps_ * verticalFactor * terrainSpeedup;
    out.u = speed * fx;
    out.v = speed * fy;
    out.w = 0.0;
    out.speed = std::hypot(out.u, out.v);
    out.dirDeg = wrapAngleDeg(rad2deg(std::atan2(out.v, out.u)));
    return out;
}
