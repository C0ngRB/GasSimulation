#pragma once
#include <QString>
#include <vector>
#include "ITerrain.h"

/**
 * @file TerrainDem.h
 * @brief 基于GeoTIFF数字高程模型的地形实现
 *
 * 【设计目的】
 * 从GeoTIFF格式的数字高程模型（DEM）文件加载真实地形数据。
 * GeoTIFF是GIS领域标准的高程数据格式，支持投影坐标系和地理参考信息。
 *
 * 【数据来源】
 * DEM数据通常来自：
 * - SRTM（Shuttle Radar Topography Mission）：全球30m/90m分辨率
 * - ASTER GDEM：全球30m分辨率
 * - LiDAR点云生成的DEM：高精度局部区域
 * - 国家测绘部门提供的官方DEM数据
 *
 * 【支持的GeoTIFF特性】
 * - 投影坐标系（如UTM）
 * - 像素存储方式（PixelIsArea 或 PixelIsPoint）
 * - NoData值处理
 * - 多波段数据的单波段提取
 *
 * 【坐标系】
 * - 加载后转换为本地网格坐标系：(x0, y0) 为左下角
 * - X方向向右增加，Y方向向前增加
 * - 原始投影信息保存在epsg字段中
 *
 * 【文件格式】
 * 为减少内存占用和加快加载速度，采用分离存储：
 * - dem_meta.json：元数据（尺寸、分辨率、范围、EPSG代码）
 * - dem_data.bin：纯二进制高程数据（float32格式）
 *
 * 【使用流程】
 * 1. Python脚本 tools/dem_convert.py 预处理GeoTIFF：
 *    - 读取GeoTIFF和投影信息
 *    - 重采样（可选，stride参数控制下采样倍数）
 *    - 输出JSON元数据和二进制高程数据
 * 2. TerrainDem::load() 加载预处理后的数据
 * 3. height() 使用双线性插值查询任意点高程
 *
 * 【与模拟器的交互】
 * - Simulator3D::buildSolidMask() 查询每个网格点的地形高度
 * - 地形高度决定固体/流体边界面
 *
 * 【性能考虑】
 * - 预处理后的二进制格式加载速度远快于直接解析GeoTIFF
 * - 双线性插值提供平滑的地形表示
 * - stride参数可减少数据量（适合大范围低分辨率模拟）
 */
class TerrainDem : public ITerrain {
public:
    /**
     * @brief DEM元数据结构体
     *
     * 存储从GeoTIFF提取的地形元信息，
     * 用于坐标转换和网格构建。
     */
    struct Meta {
        int width = 0;          ///< DEM宽度（像素数）
        int height = 0;         ///< DEM高度（像素数）
        double origin_x = 0.0;  ///< 左上角X坐标（投影坐标系，单位：米）
        double origin_y = 0.0;  ///< 左上角Y坐标（投影坐标系，单位：米）
        double dx = 1.0;        ///< X方向分辨率（米/像素，通常为正）
        double dy = -1.0;       ///< Y方向分辨率（米/像素，通常为负，影像坐标系）
        int epsg = 0;           ///< EPSG投影代码（如32650=UTM 50N）
        float nodata = -3.4e38f; ///< NoData标记值（无效高程）
        double z_min = 0.0;      ///< 高程最小值（米）
        double z_max = 0.0;      ///< 高程最大值（米）
    };

    /**
     * @brief 加载预处理后的DEM数据
     * @param metaPath JSON元数据文件路径
     * @param binPath 二进制高程数据文件路径
     * @param errOut 错误信息输出
     * @return true=成功，false=失败
     *
     * 【加载流程】
     * 1. 解析JSON元数据文件
     * 2. 读取二进制float32高程数组
     * 3. 验证数据完整性
     *
     * 【数据验证】
     * - 检查数据数组大小是否与meta匹配
     * - 统计有效高程范围
     */
    bool load(const QString& metaPath, const QString& binPath, QString& errOut);

    /// @brief 获取元数据（只读）
    const Meta& meta() const { return meta_; }

    /// @brief 获取高程数据（只读）
    const std::vector<float>& data() const { return data_; }

    /**
     * @brief 获取X方向最小坐标
     * @return X坐标最小值（米）
     */
    double minX() const { return meta_.origin_x; }

    /**
     * @brief 获取X方向最大坐标
     * @return X坐标最大值（米）
     *
     * 【计算公式】
     * origin_x + (width-1) * dx
     */
    double maxX() const { return meta_.origin_x + (meta_.width - 1) * meta_.dx; }

    /**
     * @brief 获取Y方向最大坐标
     * @return Y坐标最大值（米）
     *
     * 【注意】
     * GeoTIFF通常使用影像坐标系，Y向下增加。
     * 此处dy通常为负值，所以maxY实际上是较小的数值。
     */
    double maxY() const { return meta_.origin_y; }

    /**
     * @brief 获取Y方向最小坐标
     * @return Y坐标最小值（米）
     *
     * 【计算公式】
     * origin_y + (height-1) * dy
     */
    double minY() const { return meta_.origin_y + (meta_.height - 1) * meta_.dy; }

    /**
     * @brief 使用双线性插值查询任意点高程
     * @param x X坐标（米，投影坐标系）
     * @param y Y坐标（米，投影坐标系）
     * @return 高程值（米，相对于海平面）
     *
     * 【插值算法】
     * 1. 将(x,y)从世界坐标转换为像素坐标
     * 2. 确定四个相邻像素：(i,j), (i+1,j), (i,j+1), (i+1,j+1)
     * 3. 在四个像素值之间进行双线性插值
     *
     * 【边界处理】
     * - 坐标超出DEM范围：返回边缘像素值（外推）
     * - NoData值：尝试使用相邻有效值替代
     */
    float sampleBilinear(double x, double y) const;

    /**
     * @brief 查询地形高度（接口实现）
     * @param x X坐标（米）
     * @param y Y坐标（米）
     * @return 高程值（米）
     *
     * 【封装说明】
     * 直接调用sampleBilinear()，提供ITerrain接口的统一访问方式。
     */
    float height(double x, double y) const override { return sampleBilinear(x, y); }

    /**
     * @brief 检查DEM数据是否有效
     * @return true=已加载且有效，false=未加载或无效
     *
     * 【有效性条件】
     * - 高程数据数组非空
     * - EPSG代码非零（表示已正确解析投影信息）
     */
    bool isValid() const override { return !data_.empty() && meta_.epsg != 0; }

private:
    Meta meta_;                     ///< DEM元数据
    std::vector<float> data_;      ///< 高程数据数组（行优先存储）

    /**
     * @brief 安全访问像素值
     * @param i 像素X索引
     * @param j 像素Y索引
     * @return 高程值，索引无效返回NaN
     *
     * 【边界检查】
     * 索引超出范围时返回NaN，由调用者处理边界情况。
     */
    float at(int i, int j) const;
};
