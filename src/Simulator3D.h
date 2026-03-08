#pragma once
#include <vector>
#include <cstdint>
#include <QString>
#include <random>

#include "Grid3D.h"
#include "ITerrain.h"

/**
 * @file Simulator3D.h
 * @brief 三维气体扩散模拟器
 *
 * 【设计目的】
 * 在不依赖外部CFD引擎的前提下，实现可复现、可扩展的三维对流-扩散模拟，
 * 支持平地/虚拟构造地形/DEM地形（通过 ITerrain 接口统一抽象）。
 *
 * 【物理模型】
 * 控制方程 - 三维瞬态对流-扩散方程：
 *   ∂C/∂t + u·∇C = K·∇²C - k·C + S
 *
 * - C: 气体浓度（可视为无量纲或等效浓度单位）
 * - u: 风速矢量 (m/s)
 * - K: 有效扩散系数 (m²/s)
 * - k: 一阶衰减系数 (1/s)
 * - S: 源项（在有限体积源区内均匀注入）
 *
 * 【随机扰动（默认启用，无UI调参）】
 * 为了生成更鲁棒的数据以拟合元胞自动机（CA）参数，本模拟器默认对：
 * - 风速 windSpeed
 * - 风向 windDir
 * - 源强 leakRate
 * 施加分段常值的随机扰动（每 updateEvery_s 秒重采样一次，Gaussian）。
 *
 * 默认推荐值（可复现，seed=42）：
 * - updateEvery_s = 1.0 s
 * - σ(windSpeed) = 0.30 m/s
 * - σ(windDir)   = 7.0 deg
 * - σ(leakRate)  = 10%（相对扰动）
 */
class Simulator3D {
public:
    struct Params {
        // 时间控制
        double totalTime_s = 60.0;    ///< 模拟总时长（秒）
        double dt_s = 0.05;           ///< 时间步长（秒）
        bool autoClampDt = true;      ///< 自动调整dt保持稳定

        // 基准风场
        double windSpeed_mps = 2.0;   ///< 风速大小（m/s）
        double windDir_deg = 0.0;     ///< 风向角（deg）。当前实现按向量方向分解为(u,v)。

        // 扩散/衰减
        double K_m2ps = 1.0;          ///< 有效扩散系数（m²/s）
        double decay_1ps = 0.0;       ///< 一阶衰减系数（1/s）

        // 源项（3D）
        double srcX_m = 0.0;          ///< 源点X坐标（m）
        double srcY_m = 0.0;          ///< 源点Y坐标（m）
        double srcZ_m = 0.0;          ///< 源点Z坐标（m）
        double srcRadius_m = 2.0;     ///< 源作用半径（m）
        double leakRate = 1.0;        ///< 基准源强（单位/秒）

        // 随机扰动（默认启用，无UI调参）
        struct Stochastic {
            bool enabled = true;            ///< 默认启用扰动
            unsigned int seed = 42;         ///< 固定随机种子，保证可复现
            double updateEvery_s = 1.0;     ///< 每隔多少秒更新一次扰动（分段常值）

            double windSpeedSigma_mps = 0.30; ///< 风速高斯扰动标准差（m/s）
            double windDirSigma_deg = 7.0;    ///< 风向高斯扰动标准差（deg）
            double leakRelSigma = 0.10;       ///< 源强相对扰动标准差（0.10=10%）
        } noise;
    };

    Simulator3D() = default;

    /**
     * @brief 初始化模拟器
     * @param grid 计算网格定义
     * @param terrain 地形数据接口（用于构建固体掩码）
     * @param p 模拟参数（含随机扰动默认值）
     * @param errOut 错误信息输出
     * @return true成功，false失败
     */
    bool initialize(const Grid3D& grid, const ITerrain* terrain, const Params& p, QString& errOut);

    /**
     * @brief 重置模拟器状态（t=0，浓度清零）
     *
     * 注意：重置会用相同 seed 重新开始随机序列，保证同参数下可复现实验。
     */
    void reset();

    /**
     * @brief 执行一个时间步推进
     * @return 当前时间（秒）
     */
    double step();

    /**
     * @brief 计算数值稳定的最大时间步长
     * @return 稳定dt上限（秒）
     */
    double stableDt() const;

    const Grid3D& grid() const { return grid_; }
    const Params& params() const { return p_; }
    double time() const { return t_; }
    float maxC() const { return maxC_; }

    // 当前生效（扰动后）的驱动参数（用于日志/状态显示/数据复现）
    double currentWindSpeed() const { return windSpeedEff_; }
    double currentWindDirDeg() const { return windDirEffDeg_; }
    double currentLeakRate() const { return leakRateEff_; }

    /**
     * @brief 提取指定Z层的XY切片（用于可视化/CSV导出）
     * @param zIndex Z索引 k
     * @param outSlice 输出数组（Nx*Ny）
     * @param outMax 输出切片最大值
     */
    void extractSliceXY(int zIndex, std::vector<float>& outSlice, float& outMax) const;

private:
    Grid3D grid_;                 ///< 计算网格副本
    Params p_;                    ///< 模拟参数副本
    const ITerrain* terrain_{nullptr};  ///< 地形数据接口指针

    double t_{0.0};              ///< 当前模拟时间（秒）

    // 风速分量（扰动后）
    double u_{0.0}, v_{0.0}, w_{0.0};

    // 扰动后当前生效参数
    double windSpeedEff_{0.0};
    double windDirEffDeg_{0.0};
    double leakRateEff_{0.0};

    // 随机数（Gaussian）
    std::mt19937 rng_;
    std::normal_distribution<double> norm_{0.0, 1.0};
    double nextNoiseUpdateT_{0.0};

    std::vector<float> C_;       ///< 当前时刻浓度场
    std::vector<float> Cnew_;    ///< 下一时刻浓度场
    std::vector<std::uint8_t> solid_;  ///< 固体掩码（1=固体，0=流体）

    float maxC_{0.0f};           ///< 当前浓度场最大值

    inline bool isSolid(int i, int j, int k) const {
        return solid_[grid_.idx(i,j,k)] != 0;
    }

    float sampleC(int i, int j, int k) const;

    void buildSolidMask();

    // ---- stochastic helpers ----
    static double wrapAngleDeg(double a);
    void setEffectiveWind(double speed, double dirDeg);
    void refreshStochasticNow();      ///< 立即重采样一次扰动并更新(u,v,leakRateEff)
    void updateStochasticIfNeeded();  ///< 到达 nextNoiseUpdateT_ 时更新
};
