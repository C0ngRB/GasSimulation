#pragma once
#include <algorithm>
#include <cstddef>
#include <stdexcept>

/**
 * @file Grid3D.h
 * @brief 三维笛卡尔均匀网格定义
 *
 * 【设计目的】
 * 定义气体扩散模拟所需的三维计算网格，采用结构化（struct）存储最小必要信息。
 * 网格采用均匀间距，适合有限差分方法求解对流-扩散方程。
 *
 * 【坐标系约定】
 * - 原点 (x0, y0, z0) 为网格角点，向正方向延伸
 * - X轴：东西向，右东左西
 * - Y轴：南北向，前北后南（惯例）
 * - Z轴：垂直向上，重力反方向
 *
 * 【单位】
 * - 水平距离：米（m）
 * - 垂直距离：米（m）
 * - 时间步长：由 Simulator3D 控制
 *
 * 【网格拓扑】
 * 网格点总数 = Nx × Ny × Nz
 * - Nx: X方向网格点数
 * - Ny: Y方向网格点数
 * - Nz: Z方向网格点数
 *
 * 【索引映射】
 * 三维索引 (i,j,k) 映射到一维存储索引：
 *   idx = i + Nx * (j + Ny * k)
 * 其中 0 ≤ i < Nx, 0 ≤ j < Ny, 0 ≤ k < Nz
 *
 * 【边界条件类型】
 * - 水平边界（X/Y）：开放边界，气体可自由流出
 * - 底部边界（Z=z0）：地形表面，固体边界（气体不可穿透）
 * - 顶部边界（Z=zTop）：开放边界，模拟域上盖
 *
 * 【与地形的交互】
 * - 网格的 z0 通常设置为略低于地形最低点
 * - 地形高度通过 ITerrain 接口查询，用于标记固体单元格
 */
struct Grid3D {
    int Nx = 0;              ///< X方向网格点数（必须 > 1）
    int Ny = 0;              ///< Y方向网格点数（必须 > 1）
    int Nz = 0;              ///< Z方向网格点数（必须 > 1）

    double x0 = 0.0;         ///< 网格原点在X方向的坐标（米，投影坐标系）
    double y0 = 0.0;         ///< 网格原点在Y方向的坐标（米，投影坐标系）
    double z0 = 0.0;         ///< 网格底面Z坐标（米，通常略低于地形最低点）

    double dx = 1.0;         ///< X方向网格间距（米，必须 > 0）
    double dy = 1.0;         ///< Y方向网格间距（米，必须 > 0）
    double dz = 1.0;         ///< Z方向网格间距（米，必须 > 0）

    /**
     * @brief 计算单个网格单元的体积
     * @return 体积（立方米）
     *
     * 【物理意义】
     * 用于质量守恒计算：源强(kg/s) / 单元体积(m³) = 源浓度增量(kg/m³/s)
     */
    double cellVolume() const { return dx * dy * dz; }

    /**
     * @brief 将三维网格索引转换为一维存储索引
     * @param i X方向索引 [0, Nx-1]
     * @param j Y方向索引 [0, Ny-1]
     * @param k Z方向索引 [0, Nz-1]
     * @return 一维数组中的位置索引
     *
     * 【使用场景】
     * Simulator3D 中使用一维向量存储三维浓度场 C(i,j,k)，
     * 调用此函数进行索引转换以访问特定网格点的数据。
     */
    inline std::size_t idx(int i, int j, int k) const {
        return static_cast<std::size_t>(i)
             + static_cast<std::size_t>(Nx) * (static_cast<std::size_t>(j)
             + static_cast<std::size_t>(Ny) * static_cast<std::size_t>(k));
    }

    /**
     * @brief 验证网格参数的有效性
     * @throw std::runtime_error 当参数无效时抛出异常
     *
     * 【检查项目】
     * - Nx, Ny, Nz 必须大于1（保证网格至少2×2×2）
     * - dx, dy, dz 必须大于0（保证网格间距有效）
     *
     * 【调用时机】
     * 通常在 Simulator3D::initialize() 中调用，确保模拟开始前网格有效。
     */
    void validate() const {
        if (Nx <= 1 || Ny <= 1 || Nz <= 1)
            throw std::runtime_error("Grid3D: Nx/Ny/Nz must be > 1");
        if (dx <= 0 || dy <= 0 || dz <= 0)
            throw std::runtime_error("Grid3D: dx/dy/dz must be > 0");
    }

    /**
     * @brief 获取指定X方向索引对应的实际坐标
     * @param i X方向索引 [0, Nx-1]
     * @return X坐标（米）
     */
    double x(int i) const { return x0 + dx * i; }

    /**
     * @brief 获取指定Y方向索引对应的实际坐标
     * @param j Y方向索引 [0, Ny-1]
     * @return Y坐标（米）
     */
    double y(int j) const { return y0 + dy * j; }

    /**
     * @brief 获取指定Z方向索引对应的实际坐标
     * @param k Z方向索引 [0, Nz-1]
     * @return Z坐标（米）
     */
    double z(int k) const { return z0 + dz * k; }
};
