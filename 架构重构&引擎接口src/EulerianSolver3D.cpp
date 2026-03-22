#include "EulerianSolver3D.h"

#include <cmath>
#include <limits>

bool EulerianSolver3D::initialize(const Grid3D& grid, const TerrainSampler* terrain, const Params& p, std::string& err)
{
    err.clear();
    grid_ = grid;
    terrain_ = terrain;
    params_ = p;

    if (!terrain_) {
        err = "EulerianSolver3D: terrain sampler is null.";
        return false;
    }

    try {
        grid_.validate();
    } catch (const std::exception& e) {
        err = e.what();
        return false;
    }

    const std::size_t n = static_cast<std::size_t>(grid_.Nx)
                        * static_cast<std::size_t>(grid_.Ny)
                        * static_cast<std::size_t>(grid_.Nz);

    C_.assign(n, 0.0f);
    Cnew_.assign(n, 0.0f);
    solid_.assign(n, 0);
    sourceMask_.assign(n, 0);

    buildSolidMask();
    buildSourceMask();
    maxC_ = 0.0f;
    return true;
}

void EulerianSolver3D::reset()
{
    std::fill(C_.begin(), C_.end(), 0.0f);
    std::fill(Cnew_.begin(), Cnew_.end(), 0.0f);
    maxC_ = 0.0f;
}

void EulerianSolver3D::buildSolidMask()
{
    for (int j = 0; j < grid_.Ny; ++j) {
        const double y = grid_.y(j);
        for (int i = 0; i < grid_.Nx; ++i) {
            const double x = grid_.x(i);
            const double h = terrain_->heightBilinear(x, y);
            for (int k = 0; k < grid_.Nz; ++k) {
                const double z = grid_.z(k);
                solid_[grid_.idx(i, j, k)] = (z <= h) ? 1 : 0;
            }
        }
    }
}

void EulerianSolver3D::buildSourceMask()
{
    sourceCellCount_ = 0;
    const double r = std::max(0.0, params_.srcRadius_m);
    const double r2 = r * r;

    for (int k = 0; k < grid_.Nz; ++k) {
        const double z = grid_.z(k);
        for (int j = 0; j < grid_.Ny; ++j) {
            const double y = grid_.y(j);
            for (int i = 0; i < grid_.Nx; ++i) {
                const double x = grid_.x(i);
                const double d2 =
                    (x - params_.srcX_m) * (x - params_.srcX_m) +
                    (y - params_.srcY_m) * (y - params_.srcY_m) +
                    (z - params_.srcZ_m) * (z - params_.srcZ_m);

                const bool inside = (d2 <= r2) && !isSolid(i, j, k);
                sourceMask_[grid_.idx(i, j, k)] = inside ? 1 : 0;
                if (inside) {
                    ++sourceCellCount_;
                }
            }
        }
    }

    if (sourceCellCount_ == 0) {
        double bestD2 = std::numeric_limits<double>::infinity();
        std::size_t bestId = 0;
        bool found = false;

        for (int k = 0; k < grid_.Nz; ++k) {
            const double z = grid_.z(k);
            for (int j = 0; j < grid_.Ny; ++j) {
                const double y = grid_.y(j);
                for (int i = 0; i < grid_.Nx; ++i) {
                    if (isSolid(i, j, k)) continue;
                    const double x = grid_.x(i);
                    const double d2 =
                        (x - params_.srcX_m) * (x - params_.srcX_m) +
                        (y - params_.srcY_m) * (y - params_.srcY_m) +
                        (z - params_.srcZ_m) * (z - params_.srcZ_m);
                    if (d2 < bestD2) {
                        bestD2 = d2;
                        bestId = grid_.idx(i, j, k);
                        found = true;
                    }
                }
            }
        }

        if (found) {
            sourceMask_[bestId] = 1;
            sourceCellCount_ = 1;
        }
    }
}

float EulerianSolver3D::fluidNeighborOrSelf(int ni, int nj, int nk, float self) const
{
    if (ni < 0 || ni >= grid_.Nx || nj < 0 || nj >= grid_.Ny || nk < 0 || nk >= grid_.Nz) {
        return self;
    }
    if (isSolid(ni, nj, nk)) {
        return self;
    }
    return cell(ni, nj, nk);
}

double EulerianSolver3D::stableDt(double representativeSpeed) const
{
    const double eps = 1.0e-12;
    const double cfl = 0.35;
    const double diff = 0.15;

    double dtAdv = 1.0e9;
    if (representativeSpeed > eps) {
        const double h = std::min({grid_.dx, grid_.dy, grid_.dz});
        dtAdv = cfl * h / representativeSpeed;
    }

    double dtDiff = 1.0e9;
    if (params_.K_m2ps > eps) {
        const double h2 = std::min({
            grid_.dx * grid_.dx,
            grid_.dy * grid_.dy,
            grid_.dz * grid_.dz
        });
        dtDiff = diff * h2 / (6.0 * params_.K_m2ps);
    }

    return std::min(dtAdv, dtDiff);
}

void EulerianSolver3D::step(double dt, const IWindField& windField, double leakRate_kgps)
{
    dt = std::max(1.0e-9, dt);

    const double K = std::max(0.0, params_.K_m2ps);
    const double decay = std::max(0.0, params_.decay_1ps);

    const float sourceAdd = (sourceCellCount_ > 0)
        ? static_cast<float>((std::max(0.0, leakRate_kgps) * dt) / (grid_.cellVolume() * static_cast<double>(sourceCellCount_)))
        : 0.0f;

    maxC_ = 0.0f;

    for (int k = 0; k < grid_.Nz; ++k) {
        const double z = grid_.z(k);
        for (int j = 0; j < grid_.Ny; ++j) {
            const double y = grid_.y(j);
            for (int i = 0; i < grid_.Nx; ++i) {
                const std::size_t id = grid_.idx(i, j, k);
                if (solid_[id]) {
                    Cnew_[id] = 0.0f;
                    continue;
                }

                const float Cc = C_[id];
                const WindSample ws = windField.sample(grid_.x(i), y, z);

                const float Cxm = fluidNeighborOrSelf(i - 1, j, k, Cc);
                const float Cxp = fluidNeighborOrSelf(i + 1, j, k, Cc);
                const float Cym = fluidNeighborOrSelf(i, j - 1, k, Cc);
                const float Cyp = fluidNeighborOrSelf(i, j + 1, k, Cc);
                const float Czm = fluidNeighborOrSelf(i, j, k - 1, Cc);
                const float Czp = fluidNeighborOrSelf(i, j, k + 1, Cc);

                float dCdx = 0.0f;
                if (ws.u >= 0.0) dCdx = (Cc - Cxm) / static_cast<float>(grid_.dx);
                else             dCdx = (Cxp - Cc) / static_cast<float>(grid_.dx);

                float dCdy = 0.0f;
                if (ws.v >= 0.0) dCdy = (Cc - Cym) / static_cast<float>(grid_.dy);
                else             dCdy = (Cyp - Cc) / static_cast<float>(grid_.dy);

                float dCdz = 0.0f;
                if (ws.w >= 0.0) dCdz = (Cc - Czm) / static_cast<float>(grid_.dz);
                else             dCdz = (Czp - Cc) / static_cast<float>(grid_.dz);

                const float d2Cdx2 = (Cxp - 2.0f * Cc + Cxm) / static_cast<float>(grid_.dx * grid_.dx);
                const float d2Cdy2 = (Cyp - 2.0f * Cc + Cym) / static_cast<float>(grid_.dy * grid_.dy);
                const float d2Cdz2 = (Czp - 2.0f * Cc + Czm) / static_cast<float>(grid_.dz * grid_.dz);

                const float adv = static_cast<float>(-(ws.u * dCdx + ws.v * dCdy + ws.w * dCdz));
                const float dif = static_cast<float>(K * (d2Cdx2 + d2Cdy2 + d2Cdz2));
                const float dec = static_cast<float>(-decay * Cc);

                float Cn = Cc + static_cast<float>(dt) * (adv + dif + dec);
                if (isSourceCell(i, j, k)) {
                    Cn += sourceAdd;
                }
                if (Cn < 0.0f) {
                    Cn = 0.0f;
                }

                Cnew_[id] = Cn;
                maxC_ = std::max(maxC_, Cn);
            }
        }
    }

    C_.swap(Cnew_);
}

void EulerianSolver3D::extractSliceXY(int zIndex, std::vector<float>& outSlice, float& outMax) const
{
    zIndex = std::clamp(zIndex, 0, grid_.Nz - 1);
    outSlice.assign(static_cast<std::size_t>(grid_.Nx) * static_cast<std::size_t>(grid_.Ny), 0.0f);
    outMax = 0.0f;

    for (int j = 0; j < grid_.Ny; ++j) {
        for (int i = 0; i < grid_.Nx; ++i) {
            const float v = C_[grid_.idx(i, j, zIndex)];
            outSlice[static_cast<std::size_t>(i) + static_cast<std::size_t>(j) * static_cast<std::size_t>(grid_.Nx)] = v;
            outMax = std::max(outMax, v);
        }
    }
}
