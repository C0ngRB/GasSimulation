#include "Simulator.h"

Simulator::Simulator(const SimParams &p)
{
  reset(p);
}

void Simulator::reset(const SimParams &p)
{
  p_ = p;
  Nx_ = std::max(10, p_.Nx);
  Ny_ = std::max(10, p_.Ny);

  dx_ = p_.Lx_m / (Nx_ - 1);
  dy_ = p_.Ly_m / (Ny_ - 1);

  // wind components
  constexpr double kPi = 3.1415926535897932384626433832795;
  const double rad = p_.windDir_deg * kPi / 180.0;

  u_ = p_.windSpeed_mps * std::cos(rad);
  v_ = p_.windSpeed_mps * std::sin(rad);

  C_.assign((size_t)Nx_ * (size_t)Ny_, 0.0f);
  Cnew_ = C_;
  t_ = 0.0;
  maxC_ = 0.0f;

  computeSourceIndex();
}

void Simulator::computeSourceIndex()
{
  src_i_ = (int)std::round(p_.srcX_m / dx_);
  src_j_ = (int)std::round(p_.srcY_m / dy_);
  src_i_ = std::clamp(src_i_, 0, Nx_ - 1);
  src_j_ = std::clamp(src_j_, 0, Ny_ - 1);
}

float Simulator::sample(int i, int j) const
{
  if (i < 0 || i >= Nx_ || j < 0 || j >= Ny_)
    return 0.0f; // outside treated as 0
  return C_[idx(i, j)];
}

double Simulator::stableDt() const
{
  // 对流CFL：dt <= cfl * min(dx/|u|, dy/|v|)
  // 扩散稳定：dt <= diff * min(dx^2, dy^2) / (4D)
  const double eps = 1e-12;
  const double cfl = 0.5;
  const double diff = 0.25;

  double dt_adv = 1e9;
  if (std::abs(u_) > eps)
    dt_adv = std::min(dt_adv, cfl * dx_ / std::abs(u_));
  if (std::abs(v_) > eps)
    dt_adv = std::min(dt_adv, cfl * dy_ / std::abs(v_));

  double dt_diff = 1e9;
  if (p_.D_m2ps > eps)
  {
    const double h2 = std::min(dx_ * dx_, dy_ * dy_);
    dt_diff = diff * h2 / p_.D_m2ps;
  }

  return std::min(dt_adv, dt_diff);
}

double Simulator::step()
{
  // 自动钳制 dt
  double dt = p_.dt_s;
  const double dtStable = stableDt();
  if (p_.autoClampDt && dtStable > 0 && dt > dtStable)
  {
    dt = dtStable;
  }

  // 预计算源项注入：把 leakRate（kg/s）近似为 2D 单元浓度增量：
  // dC += (Q * dt) / (dx*dy*H)
  // 若你不关心量纲，可把 leakRate 看作“相对注入强度”
  const double cellVol = dx_ * dy_ * std::max(1e-6, p_.effectiveHeight_m);
  const float sourceAdd = (float)((p_.leakRate * dt) / cellVol);

  const double D = p_.D_m2ps;
  const double k = p_.decay_1ps;

  maxC_ = 0.0f;

  for (int j = 0; j < Ny_; ++j)
  {
    for (int i = 0; i < Nx_; ++i)
    {
      const float Cc = sample(i, j);

      // --- Upwind gradients ---
      float dCdx = 0.0f;
      if (u_ >= 0)
        dCdx = (Cc - sample(i - 1, j)) / (float)dx_;
      else
        dCdx = (sample(i + 1, j) - Cc) / (float)dx_;

      float dCdy = 0.0f;
      if (v_ >= 0)
        dCdy = (Cc - sample(i, j - 1)) / (float)dy_;
      else
        dCdy = (sample(i, j + 1) - Cc) / (float)dy_;

      // --- Laplacian (central) ---
      float d2Cdx2 = (sample(i + 1, j) - 2.0f * Cc + sample(i - 1, j)) / (float)(dx_ * dx_);
      float d2Cdy2 = (sample(i, j + 1) - 2.0f * Cc + sample(i, j - 1)) / (float)(dy_ * dy_);

      const float adv = (float)(-u_ * dCdx - v_ * dCdy);
      const float dif = (float)(D * (d2Cdx2 + d2Cdy2));
      const float dec = (float)(-k * Cc);

      float Cn = Cc + (float)dt * (adv + dif + dec);

      // 源项注入（体源/点近似）
      if (i == src_i_ && j == src_j_)
      {
        Cn += sourceAdd;
      }

      // 物理下限：浓度不为负
      if (Cn < 0.0f)
        Cn = 0.0f;

      Cnew_[idx(i, j)] = Cn;
      if (Cn > maxC_)
        maxC_ = Cn;
    }
  }

  C_.swap(Cnew_);
  t_ += dt;
  return t_;
}
