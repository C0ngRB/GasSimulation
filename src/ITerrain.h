#pragma once

/**
 * @file ITerrain.h
 * @brief 地形高度查询接口
 *
 * 【设计目的】
 * 提供统一的地形高度查询抽象层，使气体扩散模拟器能够与任意地形数据源
 * （解析GeoTIFF、程序生成、平地等）进行解耦交互。
 *
 * 【坐标系约定】
 * - 采用右手笛卡尔坐标系
 * - X轴向右为正，Y轴向前为正（地图视图的传统朝向）
 * - Z轴向上为正，代表地形高度（海平面以上）
 *
 * 【单位】
 * - 水平距离：米（m）
 * - 高度：米（m），相对于海平面的绝对高度
 *
 * 【数据流】
 * MainWindow → Simulator3D → ITerrain（查询地形约束）
 * 模拟器在构建网格时查询每个节点的地面高度，用于标记固体单元格
 */
class ITerrain {
public:
    virtual ~ITerrain() = default;

    /**
     * @brief 查询指定坐标点的地形高度
     * @param x X坐标（米，相对于地形原点）
     * @param y Y坐标（米，相对于地形原点）
     * @return 地形高度（米，相对于海平面）
     *
     * 【物理意义】
     * 返回地面以上的高度值，用于判断模拟网格点是否位于地形表面以下。
     * 位于地形以下的点被标记为固体边界，气体无法穿透。
     *
     * 【边界条件】
     * - 输入坐标超出地形范围：返回地形边缘外推值或NaN，由具体实现决定
     * - 地形无效时：isValid()返回false，模拟器会跳过该地形
     */
    virtual float height(double x, double y) const = 0;

    /**
     * @brief 检查地形数据是否有效
     * @return true=有效，false=无效（如数据未加载、格式错误等）
     *
     * 【用途】
     * 在模拟开始前进行有效性检查，防止因地形数据问题导致模拟崩溃。
     */
    virtual bool isValid() const { return true; }
};
