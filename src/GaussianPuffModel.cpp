#include "GaussianPuffModel.h"

#include <algorithm>
#include <cmath>
#include <limits>

static constexpr double kPi = 3.1415926535897932384626433832795;

double GaussianPuffModel::deg2rad(double d) { return d * kPi / 180.0; }

double GaussianPuffModel::wrapAngleDeg(double a)
{
    while (a >= 180.0) a -= 360.0;
    while (a <  -180.0) a += 360.0;
    return a;
}

void GaussianPuffModel::setEffectiveWind(double speed, double dirDeg)
{
    windSpeedEff_ = std::max(0.0, speed);
    windDirEffDeg_ = wrapAngleDeg(dirDeg);
    const double th = deg2rad(windDirEffDeg_);
    u_ = windSpeedEff_ * std::cos(th);
    v_ = windSpeedEff_ * std::sin(th);
}

void GaussianPuffModel::refreshNoise()
{
    double ws = params_.windSpeed_mps;
    double wd = params_.windDir_deg;
    double lr = params_.leakRate_kgps;

    if (params_.noise.enabled)
    {
        const double z1 = norm_(rng_);
        const double z2 = norm_(rng_);
        const double z3 = norm_(rng_);

        ws = params_.windSpeed_mps + z1 * params_.noise.windSpeedSigma_mps;
        if (ws < 0.0) ws = 0.0;

        wd = wrapAngleDeg(params_.windDir_deg + z2 * params_.noise.windDirSigma_deg);

        const double rel = 1.0 + z3 * params_.noise.leakRelSigma;
        lr = params_.leakRate_kgps * std::max(0.0, rel);
    }

    leakRateEff_ = std::max(0.0, lr);
    setEffectiveWind(ws, wd);

    nextNoiseUpdateT_ = t_ + std::max(1e-6, params_.noise.updateEvery_s);
}

void GaussianPuffModel::updateNoiseIfNeeded()
{
    if (!params_.noise.enabled) return;
    if (t_ + 1e-12 >= nextNoiseUpdateT_) refreshNoise();
}

double GaussianPuffModel::stableDt() const
{
    const double eps = 1e-12;
    const double cfl = 0.4;
    const double diff = 0.2;

    double dt_adv = 1e9;
    if (std::abs(u_) > eps) dt_adv = std::min(dt_adv, cfl * grid_.dx / std::abs(u_));
    if (std::abs(v_) > eps) dt_adv = std::min(dt_adv, cfl * grid_.dy / std::abs(v_));

    double dt_diff = 1e9;
    if (params_.K_m2ps > eps)
    {
        const double h2 = std::min(grid_.dx*grid_.dx, grid_.dy*grid_.dy);
        dt_diff = diff * h2 / (4.0 * params_.K_m2ps);
    }

    return std::min(dt_adv, dt_diff);
}

// -------- terrain cache --------

void GaussianPuffModel::buildTerrainCache()
{
    H_.assign((size_t)grid_.Nx * grid_.Ny, 0.0f);
    if (!terr_) return;
    for (int j = 0; j < grid_.Ny; ++j)
    {
        const double y = grid_.y(j);
        for (int i = 0; i < grid_.Nx; ++i)
        {
            const double x = grid_.x(i);
            H_[(size_t)i + (size_t)j * grid_.Nx] = terr_->height(x, y);
        }
    }
}

double GaussianPuffModel::Hbilinear(double x, double y) const
{
    if (!terr_) return 0.0;
    if (grid_.Nx <= 1 || grid_.Ny <= 1) return terr_->height(x, y);

    const double fx = (x - grid_.x0) / grid_.dx;
    const double fy = (y - grid_.y0) / grid_.dy;

    int i0 = (int)std::floor(fx);
    int j0 = (int)std::floor(fy);
    double tx = fx - i0;
    double ty = fy - j0;

    i0 = std::clamp(i0, 0, grid_.Nx - 2);
    j0 = std::clamp(j0, 0, grid_.Ny - 2);
    tx = std::clamp(tx, 0.0, 1.0);
    ty = std::clamp(ty, 0.0, 1.0);

    const int i1 = i0 + 1;
    const int j1 = j0 + 1;

    const float h00 = H_[(size_t)i0 + (size_t)j0 * grid_.Nx];
    const float h10 = H_[(size_t)i1 + (size_t)j0 * grid_.Nx];
    const float h01 = H_[(size_t)i0 + (size_t)j1 * grid_.Nx];
    const float h11 = H_[(size_t)i1 + (size_t)j1 * grid_.Nx];

    const double hx0 = h00*(1.0-tx) + h10*tx;
    const double hx1 = h01*(1.0-tx) + h11*tx;
    return hx0*(1.0-ty) + hx1*ty;
}

void GaussianPuffModel::gradH(double x, double y, double& dzdx, double& dzdy) const
{
    const double h = std::max(grid_.dx, grid_.dy);
    const double hx1 = Hbilinear(x + h, y);
    const double hx0 = Hbilinear(x - h, y);
    const double hy1 = Hbilinear(x, y + h);
    const double hy0 = Hbilinear(x, y - h);
    dzdx = (hx1 - hx0) / (2.0*h);
    dzdy = (hy1 - hy0) / (2.0*h);
}

double GaussianPuffModel::maxHAlong(double x0, double y0, double x1, double y1, int samples) const
{
    samples = std::max(2, samples);
    double hMax = -1e30;
    for (int s = 0; s < samples; ++s)
    {
        const double t = (double)s / (double)(samples - 1);
        const double x = x0*(1-t) + x1*t;
        const double y = y0*(1-t) + y1*t;
        hMax = std::max(hMax, Hbilinear(x, y));
    }
    return hMax;
}

// -------- obstacle/slip flow --------

bool GaussianPuffModel::isSolidAtZ(double x, double y, double z) const
{
    const double eps = 0.5 * grid_.dz;
    return Hbilinear(x, y) >= (z - eps);
}

void GaussianPuffModel::flowDirWithSlip(double x, double y, double zCenter, double& outDx, double& outDy) const
{
    const double U = std::hypot(u_, v_);
    if (U < 1e-12) { outDx = 0.0; outDy = 0.0; return; }

    double dx = u_ / U;
    double dy = v_ / U;

    const double step = 0.5 * std::min(grid_.dx, grid_.dy);
    const double x1 = x + step * dx;
    const double y1 = y + step * dy;

    if (!isSolidAtZ(x1, y1, zCenter))
    {
        outDx = dx; outDy = dy; return;
    }

    double dzdx = 0.0, dzdy = 0.0;
    gradH(x, y, dzdx, dzdy);
    const double gn = std::hypot(dzdx, dzdy);
    if (gn < 1e-12)
    {
        outDx = -dy; outDy = dx; return;
    }
    const double nx = dzdx / gn;
    const double ny = dzdy / gn;

    const double t1x = -ny, t1y = nx;
    const double t2x =  ny, t2y = -nx;

    const double dot1 = t1x*dx + t1y*dy;
    const double dot2 = t2x*dx + t2y*dy;
    double tx = (dot1 >= dot2) ? t1x : t2x;
    double ty = (dot1 >= dot2) ? t1y : t2y;

    if (tx*dx + ty*dy < 0.0) { tx = -tx; ty = -ty; }
    outDx = tx; outDy = ty;
}

// -------- puff release / advection --------

double GaussianPuffModel::releaseInterval() const
{
    if (params_.releaseInterval_s > 0.0)
        return std::min(params_.releaseInterval_s, std::max(1e-6, params_.dt_s));

    const double U = std::max(1e-9, windSpeedEff_);
    // 连续性：相邻 puff 的平流间距 ≤ 0.25*网格
    const double ds = 0.25 * std::min(grid_.dx, grid_.dy);
    const double dt = ds / U;
    // 防止过小导致 puff 数量爆炸：下限 0.01s，上限不超过 dt
    return std::min(std::max(0.01, dt), std::max(1e-6, params_.dt_s));
}

void GaussianPuffModel::emitUntil(double tNow)
{
    const double dtRel = releaseInterval();
    while (tNow - lastReleaseT_ >= dtRel - 1e-12)
    {
        lastReleaseT_ += dtRel;
        Puff pf;
        pf.x = params_.srcX_m;
        pf.y = params_.srcY_m;
        pf.z = params_.srcZ_m;
        pf.t_emit = lastReleaseT_;
        pf.sigma0 = std::max(params_.srcRadius_m, 0.5 * std::min(grid_.dx, grid_.dy));
        pf.mass = std::max(0.0, leakRateEff_) * dtRel;
        puffs_.push_back(pf);
    }
}

void GaussianPuffModel::advectAll(double dt)
{
    const double U = std::max(0.0, windSpeedEff_);
    if (U < 1e-9) return;

    // substep for obstacle + CFL
    const double dtSt = stableDt();
    const double h = (dtSt > 0.0) ? std::min(dt, dtSt) : dt;
    const int n = std::max(1, (int)std::ceil(dt / std::max(1e-9, h)));
    const double sub = dt / (double)n;

    for (int s = 0; s < n; ++s)
    {
        for (auto& pf : puffs_)
        {
            double dx=0.0, dy=0.0;
            flowDirWithSlip(pf.x, pf.y, pf.z, dx, dy);

            double x1 = pf.x + U * dx * sub;
            double y1 = pf.y + U * dy * sub;

            if (isSolidAtZ(x1, y1, pf.z))
            {
                // try shrink step a few times
                double sub2 = sub;
                bool ok = false;
                for (int k = 0; k < 6; ++k)
                {
                    sub2 *= 0.5;
                    x1 = pf.x + U * dx * sub2;
                    y1 = pf.y + U * dy * sub2;
                    if (!isSolidAtZ(x1, y1, pf.z)) { ok = true; break; }
                }
                if (!ok)
                {
                    // stuck: no move this substep
                    x1 = pf.x; y1 = pf.y;
                }
            }

            pf.x = x1;
            pf.y = y1;
            pf.z = params_.srcZ_m; // keep fixed absolute height for this simplified CALPUFF-like mode
        }
    }
}

double GaussianPuffModel::sigmaAt(const Puff& pf, double age) const
{
    const double K = std::max(1e-9, params_.K_m2ps);
    const double s2 = pf.sigma0*pf.sigma0 + 2.0*K*std::max(0.0, age);
    return std::sqrt(std::max(1e-12, s2));
}

double GaussianPuffModel::terrainTransmittance(double sx, double sy, double rx, double ry,
                                               double zCenter, double sigmaZ) const
{
    const double hMax = maxHAlong(sx, sy, rx, ry, 16);
    if (hMax <= zCenter) return 1.0;
    const double sig = std::max(1e-6, sigmaZ);
    const double arg = (hMax - zCenter) / (std::sqrt(2.0) * sig);
    return 0.5 * std::erfc(arg);
}

double GaussianPuffModel::contribution(const Puff& pf, double x, double y, double zWorld, double tNow) const
{
    const double age = tNow - pf.t_emit;
    if (age <= 0.0) return 0.0;

    const double hRec = Hbilinear(x, y);
    if (zWorld < hRec) return 0.0;

    const double sig = sigmaAt(pf, age);
    const double sig2 = sig*sig;

    // soft shadow from terrain between puff center and receptor
    const double T = terrainTransmittance(pf.x, pf.y, x, y, pf.z, sig);
    if (T <= 1e-6) return 0.0;

    const double lam = std::max(0.0, params_.decay_1ps);
    const double massEff = (lam > 0.0) ? (pf.mass * std::exp(-lam * age)) : pf.mass;
    if (massEff <= 0.0) return 0.0;

    const double dx = x - pf.x;
    const double dy = y - pf.y;
    const double dz = zWorld - pf.z;

    const double inv2 = 1.0 / (2.0 * sig2);
    const double norm = massEff / (std::pow(2.0 * kPi, 1.5) * sig*sig*sig);

    auto core = [&](double dzr)->double {
        const double r2 = dx*dx + dy*dy + dzr*dzr;
        return norm * std::exp(-r2 * inv2);
    };

    // mirror at local ground
    const double zMirror = 2.0*hRec - pf.z;
    const double dzm = zWorld - zMirror;

    return std::max(0.0, (core(dz) + core(dzm)) * T);
}

// -------- public API --------

bool GaussianPuffModel::initialize(const Grid3D& g, const ITerrain* terr, const Params& p, std::string& err)
{
    err.clear();
    grid_ = g;
    terr_ = terr;
    params_ = p;

    if (!terr_) { err = "Terrain is null"; return false; }
    try { grid_.validate(); }
    catch (const std::exception& e) { err = e.what(); return false; }

    buildTerrainCache();

    const double hSrc = Hbilinear(params_.srcX_m, params_.srcY_m);
    if (params_.srcZ_m <= hSrc + 1e-6)
    {
        err = "GaussianPuff: srcZ is below/at ground.";
        return false;
    }

    t_ = 0.0;
    lastReleaseT_ = 0.0;
    puffs_.clear();

    rng_ = std::mt19937(params_.noise.seed);
    refreshNoise();

    // seed one puff at t=0 for visibility
    Puff pf;
    pf.x=params_.srcX_m; pf.y=params_.srcY_m; pf.z=params_.srcZ_m;
    pf.t_emit=0.0;
    pf.sigma0=std::max(params_.srcRadius_m, 0.5 * std::min(grid_.dx, grid_.dy));
    pf.mass=std::max(0.0, leakRateEff_) * releaseInterval();
    puffs_.push_back(pf);

    return true;
}

void GaussianPuffModel::reset()
{
    t_ = 0.0;
    lastReleaseT_ = 0.0;
    puffs_.clear();

    rng_ = std::mt19937(params_.noise.seed);
    refreshNoise();

    Puff pf;
    pf.x=params_.srcX_m; pf.y=params_.srcY_m; pf.z=params_.srcZ_m;
    pf.t_emit=0.0;
    pf.sigma0=std::max(params_.srcRadius_m, 0.5 * std::min(grid_.dx, grid_.dy));
    pf.mass=std::max(0.0, leakRateEff_) * releaseInterval();
    puffs_.push_back(pf);
}

void GaussianPuffModel::step()
{
    updateNoiseIfNeeded();

    double dt = std::max(1e-6, params_.dt_s);
    if (params_.autoClampDt)
    {
        const double dtSt = stableDt();
        if (dtSt > 0.0 && dt > dtSt) dt = dtSt;
    }

    t_ = std::min(params_.totalTime_s, t_ + dt);

    emitUntil(t_);
    advectAll(dt);

    // prune old
    const double maxAge = std::max(params_.totalTime_s, 60.0);
    puffs_.erase(std::remove_if(puffs_.begin(), puffs_.end(),
                                [&](const Puff& p){ return (t_ - p.t_emit) > maxAge || p.mass <= 0.0; }),
                 puffs_.end());
}

void GaussianPuffModel::extractSliceXY(int k, std::vector<float>& out, float& maxC) const
{
    k = std::clamp(k, 0, grid_.Nz - 1);
    const double zWorld = grid_.z(k);

    const int Nx = grid_.Nx;
    const int Ny = grid_.Ny;
    out.assign((size_t)Nx * Ny, 0.0f);
    maxC = 0.0f;

    // Efficient splatting: update only within 3σ box per puff
    for (const auto& pf : puffs_)
    {
        const double age = t_ - pf.t_emit;
        if (age <= 0.0) continue;
        const double sig = sigmaAt(pf, age);
        const double rMax = 3.0 * sig;

        const int i0 = std::clamp((int)std::floor((pf.x - rMax - grid_.x0) / grid_.dx), 0, Nx-1);
        const int i1 = std::clamp((int)std::ceil ((pf.x + rMax - grid_.x0) / grid_.dx), 0, Nx-1);
        const int j0 = std::clamp((int)std::floor((pf.y - rMax - grid_.y0) / grid_.dy), 0, Ny-1);
        const int j1 = std::clamp((int)std::ceil ((pf.y + rMax - grid_.y0) / grid_.dy), 0, Ny-1);

        for (int j = j0; j <= j1; ++j)
        {
            const double y = grid_.y(j);
            for (int i = i0; i <= i1; ++i)
            {
                const double x = grid_.x(i);
                const double c = contribution(pf, x, y, zWorld, t_);
                if (c <= 0.0) continue;
                float& cell = out[(size_t)i + (size_t)j * Nx];
                cell += (float)c;
                maxC = std::max(maxC, cell);
            }
        }
    }
}
