#include "GaussianPlumeModel.h"

#include <algorithm>
#include <cmath>
#include <limits>

static constexpr double kPi = 3.1415926535897932384626433832795;

double GaussianPlumeModel::deg2rad(double d) { return d * kPi / 180.0; }

double GaussianPlumeModel::wrapAngleDeg(double a)
{
    while (a >= 180.0) a -= 360.0;
    while (a <  -180.0) a += 360.0;
    return a;
}

void GaussianPlumeModel::setEffectiveWind(double speed, double dirDeg)
{
    windSpeedEff_ = std::max(0.0, speed);
    windDirEffDeg_ = wrapAngleDeg(dirDeg);
    const double th = deg2rad(windDirEffDeg_);
    u_ = windSpeedEff_ * std::cos(th);
    v_ = windSpeedEff_ * std::sin(th);
}

void GaussianPlumeModel::refreshNoise()
{
    // base
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

void GaussianPlumeModel::updateNoiseIfNeeded()
{
    if (!params_.noise.enabled) return;
    if (t_ + 1e-12 >= nextNoiseUpdateT_) refreshNoise();
}

double GaussianPlumeModel::stableDt() const
{
    // Keep consistency with your CFD autoClamp idea (CFL + diffusion).
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

void GaussianPlumeModel::buildTerrainCache()
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

double GaussianPlumeModel::Hbilinear(double x, double y) const
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

void GaussianPlumeModel::gradH(double x, double y, double& dzdx, double& dzdy) const
{
    // finite difference on terrain height
    const double h = std::max(grid_.dx, grid_.dy);
    const double hx1 = Hbilinear(x + h, y);
    const double hx0 = Hbilinear(x - h, y);
    const double hy1 = Hbilinear(x, y + h);
    const double hy0 = Hbilinear(x, y - h);
    dzdx = (hx1 - hx0) / (2.0*h);
    dzdy = (hy1 - hy0) / (2.0*h);
}

double GaussianPlumeModel::maxHAlong(double x0, double y0, double x1, double y1, int samples) const
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

bool GaussianPlumeModel::isSolidAtZ(double x, double y, double z) const
{
    // 关键修正：障碍不再用 srcRadius 扩大，只用 dz/2 的几何容差
    const double eps = 0.5 * grid_.dz;
    return Hbilinear(x, y) >= (z - eps);
}

void GaussianPlumeModel::flowDirWithSlip(double x, double y, double zCenter, double& outDx, double& outDy) const
{
    // base wind unit
    const double U = std::hypot(u_, v_);
    if (U < 1e-12) { outDx = 0.0; outDy = 0.0; return; }
    double dx = u_ / U;
    double dy = v_ / U;

    // if next step would penetrate solid, slide along contour tangent
    const double step = 0.5 * std::min(grid_.dx, grid_.dy);
    const double x1 = x + step * dx;
    const double y1 = y + step * dy;

    if (!isSolidAtZ(x1, y1, zCenter))
    {
        outDx = dx; outDy = dy; return;
    }

    // compute terrain gradient at current point (normal to contour)
    double dzdx = 0.0, dzdy = 0.0;
    gradH(x, y, dzdx, dzdy);
    const double gn = std::hypot(dzdx, dzdy);
    if (gn < 1e-12)
    {
        // fallback: rotate 90 deg
        outDx = -dy; outDy = dx; return;
    }
    const double nx = dzdx / gn;
    const double ny = dzdy / gn;

    // tangents
    const double t1x = -ny, t1y = nx;
    const double t2x =  ny, t2y = -nx;

    // choose tangent with larger downwind projection
    const double dot1 = t1x*dx + t1y*dy;
    const double dot2 = t2x*dx + t2y*dy;
    double tx = (dot1 >= dot2) ? t1x : t2x;
    double ty = (dot1 >= dot2) ? t1y : t2y;

    // ensure still makes progress downwind (avoid loops)
    const double prog = tx*dx + ty*dy;
    if (prog < 0.0)
    {
        tx = -tx; ty = -ty;
    }

    outDx = tx; outDy = ty;
}

// -------- centerline --------

void GaussianPlumeModel::rebuildCenterline()
{
    clx_.clear(); cly_.clear(); cls_.clear();
    clx_.push_back(params_.srcX_m);
    cly_.push_back(params_.srcY_m);
    cls_.push_back(0.0);
}

void GaussianPlumeModel::ensureCenterline(double frontDist) const
{
    const double U = std::max(0.0, windSpeedEff_);
    if (U < 1e-9) return;

    const double step = 0.5 * std::min(grid_.dx, grid_.dy);
    const int need = (int)std::ceil(frontDist / std::max(1e-9, step)) + 3;

    while ((int)clx_.size() < need)
    {
        const double x0 = clx_.back();
        const double y0 = cly_.back();

        double dx = 0.0, dy = 0.0;
        flowDirWithSlip(x0, y0, params_.srcZ_m, dx, dy);

        double x1 = x0 + step * dx;
        double y1 = y0 + step * dy;

        // if still solid, try reducing step
        if (isSolidAtZ(x1, y1, params_.srcZ_m))
        {
            double step2 = step;
            bool ok = false;
            for (int k = 0; k < 6; ++k)
            {
                step2 *= 0.5;
                x1 = x0 + step2 * dx;
                y1 = y0 + step2 * dy;
                if (!isSolidAtZ(x1, y1, params_.srcZ_m)) { ok = true; break; }
            }
            if (!ok)
            {
                // give up extending further
                break;
            }
        }

        clx_.push_back(x1);
        cly_.push_back(y1);
        cls_.push_back(cls_.back() + step);

        if ((int)clx_.size() > 20000) break;
    }
}

bool GaussianPlumeModel::projectToCenterline(double x, double y,
                                            double& outS, double& outR,
                                            double& outCx, double& outCy,
                                            double& outUx, double& outUy) const
{
    if (clx_.size() < 2) return false;

    double bestD2 = std::numeric_limits<double>::infinity();
    double bestS = 0.0, bestR = 0.0;
    double bestCx = clx_[0], bestCy = cly_[0];
    double bestUx = 1.0, bestUy = 0.0;

    for (size_t i = 0; i + 1 < clx_.size(); ++i)
    {
        const double ax = clx_[i], ay = cly_[i];
        const double bx = clx_[i+1], by = cly_[i+1];
        const double abx = bx-ax, aby = by-ay;
        const double ab2 = abx*abx + aby*aby;
        if (ab2 < 1e-12) continue;

        const double apx = x-ax, apy = y-ay;
        const double t = std::clamp((apx*abx + apy*aby)/ab2, 0.0, 1.0);

        const double cx = ax + t*abx;
        const double cy = ay + t*aby;

        const double dx = x-cx, dy = y-cy;
        const double d2 = dx*dx + dy*dy;
        if (d2 < bestD2)
        {
            bestD2 = d2;
            bestCx = cx; bestCy = cy;

            const double segLen = std::sqrt(ab2);
            bestS = cls_[i] + t * segLen;

            // crosswind sign
            const double ux = abx/segLen, uy = aby/segLen;
            const double cross = ux*(y-cy) - uy*(x-cx);
            bestR = (cross >= 0.0 ? 1.0 : -1.0) * std::sqrt(std::max(0.0, d2));

            bestUx = ux; bestUy = uy;
        }
    }

    outS = bestS;
    outR = bestR;
    outCx = bestCx;
    outCy = bestCy;
    outUx = bestUx;
    outUy = bestUy;
    return true;
}

double GaussianPlumeModel::terrainTransmittance(double cx, double cy, double rx, double ry,
                                                double zCenter, double sigmaZ) const
{
    // soft shadow: fraction of vertical Gaussian above the maximum ridge height along the ray
    const double hMax = maxHAlong(cx, cy, rx, ry, 16);
    if (hMax <= zCenter) return 1.0;

    const double sig = std::max(1e-6, sigmaZ);
    const double arg = (hMax - zCenter) / (std::sqrt(2.0) * sig);
    // 0.5 * erfc(arg) in [0,1]
    return 0.5 * std::erfc(arg);
}

// -------- concentration --------

double GaussianPlumeModel::concentrationAt(double x, double y, double zWorld) const
{
    if (!terr_) return 0.0;
    const double hRec = Hbilinear(x, y);
    if (zWorld < hRec) return 0.0;

    const double U = std::max(0.0, windSpeedEff_);
    if (U < 1e-9) return 0.0;

    const double front = U * t_;
    if (front <= 0.0) return 0.0;

    ensureCenterline(front);

    double s=0.0, r=0.0, cx=0.0, cy=0.0, ux=1.0, uy=0.0;
    if (!projectToCenterline(x, y, s, r, cx, cy, ux, uy)) return 0.0;
    if (s <= 1e-9 || s > front) return 0.0;

    // dispersion
    const double K = std::max(1e-9, params_.K_m2ps);
    const double age = s / U;

    const double sigma0 = std::max(params_.srcRadius_m, 0.5 * std::min(grid_.dx, grid_.dy));
    const double sig2 = sigma0*sigma0 + 2.0*K*age;
    const double sig  = std::sqrt(std::max(1e-12, sig2));

    // soft terrain shadow across the ridge between centerline point and receptor
    const double T = terrainTransmittance(cx, cy, x, y, params_.srcZ_m, sig);
    if (T <= 1e-6) return 0.0;

    // ALOHA-like constant-K Gaussian plume (sigma_y=sigma_z=sig)
    const double Q = std::max(0.0, leakRateEff_);
    if (Q <= 0.0) return 0.0;

    const double lam = std::max(0.0, params_.decay_1ps);
    const double decay = (lam > 0.0) ? std::exp(-lam * age) : 1.0;

    // Vertical reflection in local AGL coordinates
    const double hSrc = Hbilinear(params_.srcX_m, params_.srcY_m);
    const double H = params_.srcZ_m - hSrc;   // release AGL at source
    const double zAGL = zWorld - hRec;

    const double denom = 2.0 * kPi * U * sig * sig;
    if (denom < 1e-30) return 0.0;

    const double yTerm = std::exp(-(r*r) / (2.0*sig2));
    const double z1 = zAGL - H;
    const double z2 = zAGL + H;
    const double zTerm = std::exp(-(z1*z1)/(2.0*sig2)) + std::exp(-(z2*z2)/(2.0*sig2));

    const double c = (Q/denom) * yTerm * zTerm * decay * T;
    return std::max(0.0, c);
}

// -------- public API --------

bool GaussianPlumeModel::initialize(const Grid3D& g, const ITerrain* terr, const Params& p, std::string& err)
{
    err.clear();
    grid_ = g;
    terr_ = terr;
    params_ = p;
    t_ = 0.0;

    if (!terr_) { err = "Terrain is null"; return false; }
    try { grid_.validate(); }
    catch (const std::exception& e) { err = e.what(); return false; }

    buildTerrainCache();

    // source must be above ground
    const double hSrc = Hbilinear(params_.srcX_m, params_.srcY_m);
    if (params_.srcZ_m <= hSrc + 1e-6)
    {
        err = "GaussianPlume: srcZ is below/at ground.";
        return false;
    }

    rng_ = std::mt19937(params_.noise.seed);
    refreshNoise();

    rebuildCenterline();
    return true;
}

void GaussianPlumeModel::reset()
{
    t_ = 0.0;
    rng_ = std::mt19937(params_.noise.seed);
    refreshNoise();
    rebuildCenterline();
}

void GaussianPlumeModel::step()
{
    updateNoiseIfNeeded();

    double dt = std::max(1e-6, params_.dt_s);
    if (params_.autoClampDt)
    {
        const double dtSt = stableDt();
        if (dtSt > 0.0 && dt > dtSt) dt = dtSt;
    }
    t_ = std::min(params_.totalTime_s, t_ + dt);

    // centerline is param-dependent only on wind; if you enable wind noise, it will naturally curve over time.
    // With default (no wind noise), it stays stable.
}

void GaussianPlumeModel::extractSliceXY(int k, std::vector<float>& out, float& maxC) const
{
    k = std::clamp(k, 0, grid_.Nz - 1);
    const double z = grid_.z(k);

    const int Nx = grid_.Nx;
    const int Ny = grid_.Ny;
    out.assign((size_t)Nx * Ny, 0.0f);
    maxC = 0.0f;

    for (int j = 0; j < Ny; ++j)
    {
        const double y = grid_.y(j);
        for (int i = 0; i < Nx; ++i)
        {
            const double x = grid_.x(i);
            const float c = (float)concentrationAt(x, y, z);
            out[(size_t)i + (size_t)j * Nx] = c;
            maxC = std::max(maxC, c);
        }
    }
}
