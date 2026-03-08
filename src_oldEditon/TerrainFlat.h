#pragma once
#include "ITerrain.h"

/**
 * @file TerrainFlat.h
 * @brief 平地地形实现
 *
 * 【设计目的】
 * 提供最简单的地形实现——恒定高度平地。
 * 适用于基线模拟场景（无地形影响）或教学演示。
 *
 * 【物理意义】
 * - 地形高度 z(x,y) = z0（常数）
 * - 底部边界为平面，气体在水平面上扩散
 * - 无地形遮挡效应，风场不受地形影响
 *
 * 【使用场景】
 * - 快速验证模拟器功能（排除地形干扰）
 * - 开阔平原或水域上的扩散模拟
 * - 作为其他地形类型的基准对比
 *
 * 【与模拟器的交互】
 * - Simulator3D::buildSolidMask() 查询地形高度
 * - 所有网格点的地形高度相同
 * - 固体掩码：z < z0 的点标记为固体
 *
 * 【性能特点】
 * - 零计算开销：height()直接返回常数
 * - 内存占用：仅存储一个浮点数
 */
class TerrainFlat final : public ITerrain {
public:
    /**
     * @brief 构造函数
     * @param z0 恒定高度（米，相对于海平面）
     */
    explicit TerrainFlat(float z0 = 0.0f) : z0_(z0) {}

    /**
     * @brief 设置地形高度
     * @param z0 恒定高度（米，相对于海平面）
     */
    void setZ0(float z0) { z0_ = z0; }

    /**
     * @brief 查询地形高度
     * @param x X坐标（米，被忽略）
     * @param y Y坐标（米，被忽略）
     * @return 恒定地形高度 z0_（米，相对于海平面）
     *
     * 【实现说明】
     * 直接返回成员变量z0_，无任何计算。
     */
    float height(double, double) const override { return z0_; }

private:
    float z0_{0.0f};  ///< 恒定地形高度（米，相对于海平面）
};
