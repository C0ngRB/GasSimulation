#pragma once
#include <vector>
#include <cstddef>
#include <algorithm>
#include <cmath>
#include <string>

/**
 * @brief 2D 对流-扩散仿真参数
 *
 * 物理模型（被动标量）：
 *   dC/dt + u dC/dx + v dC/dy = D ∇²C - kC + S
 *
 * 说明：
 * - u,v: 由风速与风向确定（恒定均匀）
 * - D: 有效扩散系数（可视作分子扩散 + 湍扩散的合并）
 * - k: 一阶衰减（可用于沉降/反应的粗略近似）
 * - S: 源项注入（在源点所在网格单元内注入）
 */
struct SimParams
{
  // Domain
  double Lx_m = 200.0;
  double Ly_m = 100.0;
  int Nx = 300;
  int Ny = 150;

  // Time
  double totalTime_s = 60.0;
  double dt_s = 0.05;
  bool autoClampDt = true;

  // Wind
  double windSpeed_mps = 2.0;
  double windDir_deg = 0.0; // 0=向东(+x), 90=向北(+y)

  // Dispersion
  double D_m2ps = 1.0;    // 有效扩散系数
  double decay_1ps = 0.0; // 衰减

  // Source
  double srcX_m = 20.0;
  double srcY_m = 50.0;
  double leakRate = 1.0;          // 注入强度（相对单位或 kg/s 量纲由你自己解释）
  double effectiveHeight_m = 1.0; // 2D->3D等效厚度（用于把 kg/s 转成 2D体积浓度增量）
};

/**
 * @brief 数值仿真核心（显式迎风 + 扩散中心差分）
 */
class Simulator
{
public:
  explicit Simulator(const SimParams &p);

  void reset(const SimParams &p);

  // 单步推进；返回当前时间（推进后）
  double step();

  // 读取浓度场（行主序：i + j*Nx）
  const std::vector<float> &field() const { return C_; }

  int Nx() const { return Nx_; }
  int Ny() const { return Ny_; }
  double time() const { return t_; }

  // 统计量
  float maxC() const { return maxC_; }

  // 稳定步长上限（建议 dt <= dtStable）
  double stableDt() const;

  // 获取当前风速分量
  double u() const { return u_; }
  double v() const { return v_; }

private:
  SimParams p_;
  int Nx_{}, Ny_{};
  double dx_{}, dy_{};
  double t_{0.0};
  double u_{0.0}, v_{0.0}; // wind components

  std::vector<float> C_;
  std::vector<float> Cnew_;

  int src_i_{0}, src_j_{0};
  float maxC_{0.0f};

  inline int idx(int i, int j) const { return i + j * Nx_; }

  float sample(int i, int j) const;
  void computeSourceIndex();
};
