#pragma once
#include <QString>
#include <vector>

/**
 * @file ExporterCsv.h
 * @brief CSV格式数据导出器
 *
 * 【设计目的】
 * 将模拟结果导出为CSV格式，便于第三方软件（如Excel、MATLAB、Python）
 * 进行后处理分析。
 *
 * 【导出格式】
 * CSV文件包含：
 * 1. 头部注释行（以#开头）：包含坐标系、网格参数、时间信息
 * 2. 数据行：Nx列 × Ny行的浓度值矩阵
 *
 * 【文件结构示例】
 * @code
 * # crs=EPSG:32650
 * # origin_x=0 origin_y=0 dx=2 dy=2 z=2
 * # Nx=100 Ny=50 t=10.5
 * 0.001,0.002,0.003,...
 * 0.001,0.002,0.004,...
 * ...
 * @endcode
 *
 * 【坐标系说明】
 * - EPSG代码：标识投影坐标系（如32650=UTM Zone 50N）
 * - origin_x, origin_y：网格左下角坐标（米）
 * - dx, dy：网格间距（米）
 * - z：切片高度（米，相对于海平面）
 *
 * 【数据组织】
 * - 行索引 j：对应Y方向，从南向北递增
 * - 列索引 i：对应X方向，从西向东递增
 * - 值：对应位置的气体浓度（kg/m³）
 *
 * 【使用场景】
 * - MainWindow::onTick() 导出仿真帧
 * - 支持双切片导出（zslice和agl两种高度）
 * - 可在外部进行时序分析或可视化
 *
 * 【与其他系统的集成】
 * - Python: pandas.read_csv() 直接读取
 * - MATLAB: readmatrix() 或 csvread()
 * - Excel: 直接打开或导入
 */
class ExporterCsv {
public:
    /**
     * @brief 帧元数据结构体
     *
     * 描述单个时间切片的数据信息。
     * 这些元数据以注释行形式写入CSV文件头部。
     */
    struct FrameMeta {
        int epsg = 0;          ///< EPSG投影代码（如32650=UTM Zone 50N）
        double origin_x = 0.0; ///< 网格原点X坐标（米）
        double origin_y = 0.0; ///< 网格原点Y坐标（米）
        double dx = 1.0;      ///< X方向网格间距（米）
        double dy = 1.0;      ///< Y方向网格间距（米）
        double z  = 0.0;      ///< 切片高度（米，相对于海平面）
        int Nx = 0;           ///< X方向网格点数
        int Ny = 0;           ///< Y方向网格点数
        double t = 0.0;       ///< 模拟时间（秒）
    };

    /**
     * @brief 导出二维网格切片到CSV文件
     * @param path 输出文件路径
     * @param meta 帧元数据
     * @param gridValues 二维浓度数组（大小 Nx × Ny）
     * @param errOut 错误信息输出
     * @return true=成功，false=失败
     *
     * 【导出流程】
     * 1. 验证数据大小是否与Nx×Ny匹配
     * 2. 创建/覆盖CSV文件
     * 3. 写入头部注释行（坐标系、网格、时间信息）
     * 4. 按行写入浓度数据（科学计数法）
     *
     * 【数据精度】
     * - 使用科学计数法表示浓度值
     * - 精度：6位有效数字
     *
     * 【错误处理】
     * - 数据大小不匹配：返回false并设置错误信息
     * - 文件无法创建：返回false并设置错误信息
     */
    static bool writeGridFrame(const QString& path,
                               const FrameMeta& meta,
                               const std::vector<float>& gridValues,
                               QString& errOut);
};
