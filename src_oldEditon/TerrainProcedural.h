#pragma once
#include "ITerrain.h"
#include <vector>
#include <cmath>

/**
 * @file TerrainProcedural.h
 * @brief 程序化生成地形
 *
 * 【设计目的】
 * 通过数学函数实时生成各类地形，用于：
 * - 无需外部数据即可测试模拟器
 * - 生成特定形状的地形（山脊、多个山峰等）
 * - 参数化研究地形对气体扩散的影响
 *
 * 【支持的模式】
 * 1. Flat（平地）：z = baseZ，基准模式
 * 2. GaussianHill（高斯山丘）：单峰高斯函数
 * 3. Ridge（山脊）：沿X方向延伸的线性山脊
 * 4. MultiGaussian（多峰）：多个高斯山峰叠加
 *
 * 【数学模型】
 *
 * 高斯山丘（GaussianHill）：
 *   z(x,y) = baseZ + A * exp(-((x-xc)² + (y-yc)²) / (2σ²))
 *
 *   参数说明：
 *   - A: 峰值高度（米），山丘最高点
 *   - σ: 标准差（米），控制山丘的水平范围
 *   - (xc, yc): 峰顶中心坐标
 *
 * 山脊（Ridge）：
 *   z(x,y) = baseZ + A * exp(-(x-xr)² / (2σx²))
 *
 *   与高斯山丘的区别：山脊在Y方向无限延伸
 *
 * 多峰叠加（MultiGaussian）：
 *   z(x,y) = baseZ + Σ Ai * exp(-ri² / (2σi²))
 *
 *   用于模拟复杂山脉或多个独立地形特征
 *
 * 【物理意义】
 * - 地形起伏影响气体扩散路径
 * - 山丘可能阻挡或绕流气体
 * - 高海拔地区风速通常更大（地形抬升效应）
 *
 * 【使用场景】
 * - MainWindow选择"Procedural"模式时使用
 * - 用户通过sbProcBaseZ/PeakA/Sigma控件调节参数
 * - 默认生成以域中心为峰顶的高斯山丘
 *
 * 【性能特点】
 * - 实时计算，无需预生成或存储
 * - 计算复杂度 O(1)（单峰）或 O(n)（多峰，n=峰数量）
 */
class TerrainProcedural final : public ITerrain {
public:
    /**
     * @brief 程序化地形模式枚举
     */
    enum class Mode {
        Flat,          ///< 平地（基准高度）
        GaussianHill,   ///< 高斯山丘（单峰）
        Ridge,         ///< 山脊（沿X方向延伸）
        MultiGaussian  ///< 多峰叠加
    };

    /**
     * @brief 高斯山峰参数结构体
     *
     * 用于描述单个高斯山峰的几何特征。
     * 可用于GaussianHill模式（单峰）或MultiGaussian模式（多峰叠加）。
     */
    struct Gaussian {
        double x0 = 0.0;      ///< 峰顶X坐标（米）
        double y0 = 0.0;      ///< 峰顶Y坐标（米）
        double A = 50.0;      ///< 峰值高度（米，相对于基准面）
        double sigma = 200.0; ///< 标准差（米，控制水平范围）
    };

    /**
     * @brief 设置地形模式
     * @param m 模式类型
     */
    void setMode(Mode m) { mode_ = m; }

    /**
     * @brief 设置基准高度
     * @param z0 地形基准面高度（米，相对于海平面）
     */
    void setBaseZ(float z0) { baseZ_ = z0; }

    /**
     * @brief 设置高斯山峰参数（用于GaussianHill或MultiGaussian模式）
     * @param g 高斯山峰参数
     *
     * 【使用说明】
     * - GaussianHill模式：使用g1_作为主山峰
     * - MultiGaussian模式：gs_中所有山峰叠加，g1_可能被忽略或作为参考
     */
    void setGaussian(const Gaussian& g) { g1_ = g; }

    /**
     * @brief 设置山脊参数
     * @param x0 山脊轴线X坐标（米）
     * @param A 山脊最大高度（米）
     * @param sigmaX 山脊在X方向的标准差（米）
     *
     * 【地形特征】
     * 山脊沿Y方向无限延伸，在X方向呈高斯分布。
     * 适合模拟山脉、断层等线性地形特征。
     */
    void setRidge(double x0, double A, double sigmaX) {
        ridgeX0_ = x0;
        ridgeA_ = A;
        ridgeSigmaX_ = sigmaX;
    }

    /**
     * @brief 设置多个山峰（用于MultiGaussian模式）
     * @param gs 高斯山峰参数列表
     *
     * 【叠加规则】
     * 最终地形高度 = baseZ_ + Σ height(gi)
     * 多个山峰的高度直接相加，可能形成复杂地形。
     */
    void setMultiGaussians(const std::vector<Gaussian>& gs) { gs_ = gs; }

    /**
     * @brief 计算指定位置的地形高度
     * @param x X坐标（米）
     * @param y Y坐标（米）
     * @return 地形高度（米，相对于海平面）
     *
     * 【实现逻辑】
     * 根据当前模式选择对应的计算公式：
     * - Flat: 返回 baseZ_
     * - GaussianHill: 计算单峰高斯函数
     * - Ridge: 计算沿Y方向延伸的山脊
     * - MultiGaussian: 叠加所有山峰
     */
    float height(double x, double y) const override {
        const double zBase = baseZ_;
        switch (mode_) {
        case Mode::Flat:
            return static_cast<float>(zBase);

        case Mode::GaussianHill: {
            // 单峰高斯函数
            const double dx = x - g1_.x0;
            const double dy = y - g1_.y0;
            const double r2 = dx*dx + dy*dy;
            const double s2 = g1_.sigma * g1_.sigma;
            const double h = g1_.A * std::exp(-r2 / s2);
            return static_cast<float>(zBase + h);
        }

        case Mode::Ridge: {
            // 沿Y方向无限延伸的山脊
            const double dx = x - ridgeX0_;
            const double s2 = ridgeSigmaX_ * ridgeSigmaX_;
            const double h = ridgeA_ * std::exp(-(dx*dx) / s2);
            return static_cast<float>(zBase + h);
        }

        case Mode::MultiGaussian: {
            // 多峰叠加
            double hsum = 0.0;
            for (const auto& g : gs_) {
                const double dx = x - g.x0;
                const double dy = y - g.y0;
                const double r2 = dx*dx + dy*dy;
                const double s2 = g.sigma * g.sigma;
                hsum += g.A * std::exp(-r2 / s2);
            }
            return static_cast<float>(zBase + hsum);
        }
        }
        return static_cast<float>(zBase);
    }

private:
    Mode mode_{Mode::Flat};      ///< 当前地形模式
    float baseZ_{0.0f};          ///< 基准高度（米）

    Gaussian g1_{};              ///< 单峰参数（GaussianHill模式）

    double ridgeX0_{0.0};       ///< 山脊轴线X坐标（米）
    double ridgeA_{50.0};        ///< 山脊最大高度（米）
    double ridgeSigmaX_{200.0};  ///< 山脊X方向标准差（米）

    std::vector<Gaussian> gs_;  ///< 多峰参数列表（MultiGaussian模式）
};
