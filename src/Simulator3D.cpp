#include "Simulator3D.h"
#include <cmath>
#include <algorithm>

/**
 * @file Simulator3D.cpp
 * @brief 三维气体扩散模拟器实现（含默认随机扰动）
 */

static inline double deg2rad(double deg) {
    constexpr double kPi = 3.1415926535897932384626433832795;
    return deg * kPi / 180.0;
}

double Simulator3D::wrapAngleDeg(double a) {
    // 归一到 [-180, 180)
    while (a >= 180.0) a -= 360.0;
    while (a <  -180.0) a += 360.0;
    return a;
}

void Simulator3D::setEffectiveWind(double speed, double dirDeg) {
    windSpeedEff_ = std::max(0.0, speed);
    windDirEffDeg_ = wrapAngleDeg(dirDeg);

    const double rad = deg2rad(windDirEffDeg_);
    u_ = windSpeedEff_ * std::cos(rad);
    v_ = windSpeedEff_ * std::sin(rad);
    w_ = 0.0;
}

void Simulator3D::refreshStochasticNow() {
    // 基准值
    double ws = p_.windSpeed_mps;
    double wd = p_.windDir_deg;
    double lr = p_.leakRate;

    if (p_.noise.enabled) {
        const double z1 = norm_(rng_);
        const double z2 = norm_(rng_);
        const double z3 = norm_(rng_);

        // 风速：绝对扰动（m/s）
        ws = p_.windSpeed_mps + z1 * p_.noise.windSpeedSigma_mps;
        if (ws < 0.0) ws = 0.0;

        // 风向：角度扰动（deg）
        wd = wrapAngleDeg(p_.windDir_deg + z2 * p_.noise.windDirSigma_deg);

        // 源强：相对扰动（乘法），例如 10% -> 1 + N(0,0.1)
        const double rel = 1.0 + z3 * p_.noise.leakRelSigma;
        lr = p_.leakRate * std::max(0.0, rel);
    }

    leakRateEff_ = std::max(0.0, lr);
    setEffectiveWind(ws, wd);

    // 下一次更新时间
    const double dtUpd = std::max(1e-6, p_.noise.updateEvery_s);
    nextNoiseUpdateT_ = t_ + dtUpd;
}

void Simulator3D::updateStochasticIfNeeded() {
    if (!p_.noise.enabled) return;
    if (t_ + 1e-12 >= nextNoiseUpdateT_) {
        refreshStochasticNow();
    }
}

bool Simulator3D::initialize(const Grid3D& grid, const ITerrain* terrain, const Params& p, QString& errOut) {
    errOut.clear();

    // Step 1: 检查网格有效性
    grid_ = grid;
    try { grid_.validate(); }
    catch (const std::exception& e) { errOut = e.what(); return false; }

    // Step 2: 保存地形指针和参数副本
    terrain_ = terrain;
    if (!terrain_) { errOut = "Terrain is null"; return false; }
    if (!terrain_->isValid()) { errOut = "Terrain invalid"; return false; }

    p_ = p;

    // Step 3: 初始化随机数（默认启用扰动）
    rng_.seed(p_.noise.seed);
    t_ = 0.0;
    nextNoiseUpdateT_ = 0.0;
    refreshStochasticNow(); // 这里会设置 u_, v_, leakRateEff_

    // Step 4: 预分配浓度数组
    const std::size_t n = static_cast<std::size_t>(grid_.Nx) * grid_.Ny * grid_.Nz;
    C_.assign(n, 0.0f);
    Cnew_ = C_;

    // Step 5: 构建固体掩码（地形以下为固体）
    solid_.assign(n, 0);
    buildSolidMask();

    maxC_ = 0.0f;
    return true;
}

void Simulator3D::reset() {
    std::fill(C_.begin(), C_.end(), 0.0f);
    std::fill(Cnew_.begin(), Cnew_.end(), 0.0f);
    maxC_ = 0.0f;

    t_ = 0.0;
    rng_.seed(p_.noise.seed);
    nextNoiseUpdateT_ = 0.0;
    refreshStochasticNow();
}

void Simulator3D::buildSolidMask() {
    // 遍历所有 (i,j,k) 网格点：
    // 1) 查询地形高度 h = terrain(x(i), y(j))
    // 2) 若 z(k) <= h，则标记为固体
    for (int j = 0; j < grid_.Ny; ++j) {
        const double y = grid_.y(j);
        for (int i = 0; i < grid_.Nx; ++i) {
            const double x = grid_.x(i);
            const float h = terrain_->height(x, y);
            for (int k = 0; k < grid_.Nz; ++k) {
                const double z = grid_.z(k);
                solid_[grid_.idx(i,j,k)] = (z <= h) ? 1 : 0;
            }
        }
    }
}

float Simulator3D::sampleC(int i, int j, int k) const {
    // 边界外返回0（开放边界粗略近似）
    if (i < 0 || i >= grid_.Nx || j < 0 || j >= grid_.Ny || k < 0 || k >= grid_.Nz) return 0.0f;
    return C_[grid_.idx(i,j,k)];
}

double Simulator3D::stableDt() const {
    const double eps = 1e-12;
    const double cfl = 0.4;   // 对流CFL数（小于1保证稳定）
    const double diff = 0.2;  // 扩散CFL数（小于0.25保证稳定）

    // Step 1: 对流稳定时间步长：dt <= CFL * dx / |u|
    double dt_adv = 1e9;
    if (std::abs(u_) > eps) dt_adv = std::min(dt_adv, cfl * grid_.dx / std::abs(u_));
    if (std::abs(v_) > eps) dt_adv = std::min(dt_adv, cfl * grid_.dy / std::abs(v_));
    if (std::abs(w_) > eps) dt_adv = std::min(dt_adv, cfl * grid_.dz / std::abs(w_));

    // Step 2: 扩散稳定时间步长：dt <= diff * h^2 / (6K)
    double dt_diff = 1e9;
    if (p_.K_m2ps > eps) {
        const double h2 = std::min({grid_.dx*grid_.dx, grid_.dy*grid_.dy, grid_.dz*grid_.dz});
        dt_diff = diff * h2 / (6.0 * p_.K_m2ps);
    }

    return std::min(dt_adv, dt_diff);
}

double Simulator3D::step() {
    // Step 0: 到达更新时间则更新随机扰动（影响 u_, v_, leakRateEff_）
    updateStochasticIfNeeded();

    // Step 1: 确定时间步长
    double dt = p_.dt_s;
    const double dtStable = stableDt();
    if (p_.autoClampDt && dtStable > 0 && dt > dtStable) dt = dtStable;

    // Step 2: 获取物理参数
    const double K = p_.K_m2ps;
    const double kdecay = p_.decay_1ps;

    // Step 3: 计算源区域内的流体网格数
    const double r = std::max(0.0, p_.srcRadius_m);
    const double r2 = r * r;

    int nSrcCells = 0;
    for (int k = 0; k < grid_.Nz; ++k) {
        const double z = grid_.z(k);
        for (int j = 0; j < grid_.Ny; ++j) {
            const double y = grid_.y(j);
            for (int i = 0; i < grid_.Nx; ++i) {
                const double x = grid_.x(i);
                const double d2 = (x - p_.srcX_m)*(x - p_.srcX_m)
                                + (y - p_.srcY_m)*(y - p_.srcY_m)
                                + (z - p_.srcZ_m)*(z - p_.srcZ_m);
                if (d2 <= r2 && !isSolid(i,j,k)) ++nSrcCells;
            }
        }
    }

    // Step 4: 计算每个源网格单元的浓度增量（用扰动后的 leakRateEff_）
    const float sourceAdd = (nSrcCells > 0)
        ? static_cast<float>((leakRateEff_ * dt) / (grid_.cellVolume() * nSrcCells))
        : 0.0f;

    // Step 5: 遍历更新
    maxC_ = 0.0f;

    for (int k = 0; k < grid_.Nz; ++k) {
        for (int j = 0; j < grid_.Ny; ++j) {
            for (int i = 0; i < grid_.Nx; ++i) {
                const std::size_t id = grid_.idx(i,j,k);

                if (solid_[id]) { Cnew_[id] = 0.0f; continue; }

                const float Cc = C_[id];

                // 对流：上风差分
                float dCdx = 0.0f;
                if (u_ >= 0) dCdx = (Cc - sampleC(i-1,j,k)) / static_cast<float>(grid_.dx);
                else         dCdx = (sampleC(i+1,j,k) - Cc) / static_cast<float>(grid_.dx);

                float dCdy = 0.0f;
                if (v_ >= 0) dCdy = (Cc - sampleC(i,j-1,k)) / static_cast<float>(grid_.dy);
                else         dCdy = (sampleC(i,j+1,k) - Cc) / static_cast<float>(grid_.dy);

                float dCdz = 0.0f;
                if (w_ >= 0) dCdz = (Cc - sampleC(i,j,k-1)) / static_cast<float>(grid_.dz);
                else         dCdz = (sampleC(i,j,k+1) - Cc) / static_cast<float>(grid_.dz);

                // 扩散：中心差分
                const float d2Cdx2 = (sampleC(i+1,j,k) - 2.0f*Cc + sampleC(i-1,j,k)) / static_cast<float>(grid_.dx*grid_.dx);
                const float d2Cdy2 = (sampleC(i,j+1,k) - 2.0f*Cc + sampleC(i,j-1,k)) / static_cast<float>(grid_.dy*grid_.dy);
                const float d2Cdz2 = (sampleC(i,j,k+1) - 2.0f*Cc + sampleC(i,j,k-1)) / static_cast<float>(grid_.dz*grid_.dz);

                const float adv = static_cast<float>(-u_*dCdx - v_*dCdy - w_*dCdz);
                const float dif = static_cast<float>( K * (d2Cdx2 + d2Cdy2 + d2Cdz2) );
                const float dec = static_cast<float>(-kdecay * Cc);

                float Cn = Cc + static_cast<float>(dt) * (adv + dif + dec);

                // 源项注入（在源半径内）
                if (sourceAdd > 0.0f) {
                    const double x = grid_.x(i);
                    const double y = grid_.y(j);
                    const double z = grid_.z(k);
                    const double d2 = (x - p_.srcX_m)*(x - p_.srcX_m)
                                    + (y - p_.srcY_m)*(y - p_.srcY_m)
                                    + (z - p_.srcZ_m)*(z - p_.srcZ_m);
                    if (d2 <= r2) Cn += sourceAdd;
                }

                if (Cn < 0.0f) Cn = 0.0f;
                Cnew_[id] = Cn;
                if (Cn > maxC_) maxC_ = Cn;
            }
        }
    }

    C_.swap(Cnew_);
    t_ += dt;
    return t_;
}

void Simulator3D::extractSliceXY(int zIndex, std::vector<float>& outSlice, float& outMax) const {
    zIndex = std::clamp(zIndex, 0, grid_.Nz - 1);
    outSlice.assign(static_cast<std::size_t>(grid_.Nx) * grid_.Ny, 0.0f);
    outMax = 0.0f;

    for (int j = 0; j < grid_.Ny; ++j) {
        for (int i = 0; i < grid_.Nx; ++i) {
            const float v = C_[grid_.idx(i,j,zIndex)];
            outSlice[static_cast<std::size_t>(i) + static_cast<std::size_t>(j) * grid_.Nx] = v;
            outMax = std::max(outMax, v);
        }
    }
}
