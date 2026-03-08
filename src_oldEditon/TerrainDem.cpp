#include "TerrainDem.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QtGlobal>
#include <algorithm>
#include <cmath>
#include <cstring>

/**
 * @file TerrainDem.cpp
 * @brief TerrainDem 实现文件
 *
 * 【实现说明】
 * 实现从预处理格式加载DEM数据的逻辑。
 * 采用JSON元数据 + 二进制高程数据的分离存储格式，
 * 以提高加载速度和减少内存占用。
 *
 * 【文件格式】
 * - dem_meta.json: 使用Qt JSON库解析，包含所有元信息
 * - dem_data.bin: 原始float32二进制数据，直接内存映射读取
 *
 * 【数据验证】
 * - EPSG代码检查：确保DEM使用投影坐标系（单位：米）
 * - 数据大小验证：确保二进制文件大小与声明匹配
 *
 * 【双线性插值算法】
 * 用于在离散DEM栅格上获取任意连续位置的高程值：
 * 1. 世界坐标 → 像素坐标转换
 * 2. 确定四个相邻像素网格
 * 3. 两次线性插值（先X方向，再Y方向）
 *
 * 【坐标系转换】
 * - 世界坐标 (x,y) 单位：米（投影坐标系）
 * - 像素坐标 (i,j) 单位：像素索引
 * - 转换公式：i = (x - origin_x) / dx
 */
namespace {

/**
 * @brief 读取文件全部内容
 * @param path 文件路径
 * @param out 输出字节数组
 * @param err 错误信息
 * @return true=成功，false=失败
 */
static bool readAllBytes(const QString& path, QByteArray& out, QString& err) {
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        err = "Cannot open: " + path;
        return false;
    }
    out = f.readAll();
    return true;
}

} // anonymous namespace

bool TerrainDem::load(const QString& metaPath, const QString& binPath, QString& errOut) {
    errOut.clear();

    // Step 1: 读取并解析JSON元数据
    QByteArray metaBytes;
    if (!readAllBytes(metaPath, metaBytes, errOut)) return false;

    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(metaBytes, &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
        errOut = "Invalid meta json: " + pe.errorString();
        return false;
    }
    const QJsonObject o = doc.object();

    // Step 2: 从JSON提取元数据字段
    auto getInt = [&](const char* k, int& v)->bool{
        if (!o.contains(k) || !o.value(k).isDouble()) return false;
        v = o.value(k).toInt();
        return true;
    };
    auto getDouble = [&](const char* k, double& v)->bool{
        if (!o.contains(k) || !o.value(k).isDouble()) return false;
        v = o.value(k).toDouble();
        return true;
    };

    Meta m;
    if (!getInt("width", m.width) || !getInt("height", m.height)) {
        errOut = "meta missing width/height";
        return false;
    }
    if (!getDouble("origin_x", m.origin_x) || !getDouble("origin_y", m.origin_y)) {
        errOut = "meta missing origin";
        return false;
    }
    if (!getDouble("dx", m.dx) || !getDouble("dy", m.dy)) {
        errOut = "meta missing dx/dy";
        return false;
    }
    getInt("epsg", m.epsg);
    if (o.contains("nodata") && o.value("nodata").isDouble())
        m.nodata = static_cast<float>(o.value("nodata").toDouble());
    getDouble("z_min", m.z_min);
    getDouble("z_max", m.z_max);

    // Step 3: 验证必需的投影信息
    if (m.epsg == 0) {
        errOut = "EPSG is missing (epsg=0). DEM must be projected (meters).";
        return false;
    }

    // Step 4: 读取二进制高程数据
    QByteArray binBytes;
    if (!readAllBytes(binPath, binBytes, errOut)) return false;

    // Step 5: 验证数据大小
    const std::size_t n = static_cast<std::size_t>(m.width) * static_cast<std::size_t>(m.height);
    const std::size_t expectedBytes = n * sizeof(float);
    if (static_cast<std::size_t>(binBytes.size()) != expectedBytes) {
        errOut = "dem_data.bin size mismatch";
        return false;
    }

    // Step 6: 复制数据到成员变量
    data_.resize(n);
    std::memcpy(data_.data(), binBytes.constData(), expectedBytes);
    meta_ = m;

    return true;
}

float TerrainDem::at(int i, int j) const {
    // 安全边界检查：超出范围时返回边界值
    i = std::clamp(i, 0, meta_.width - 1);
    j = std::clamp(j, 0, meta_.height - 1);
    return data_[static_cast<std::size_t>(i) + static_cast<std::size_t>(j) * static_cast<std::size_t>(meta_.width)];
}

float TerrainDem::sampleBilinear(double x, double y) const {
    // Step 1: 世界坐标 → 像素坐标
    // 注意：dy通常为负值（GeoTIFF惯例），所以fy方向相反
    const double fx = (x - meta_.origin_x) / meta_.dx;
    const double fy = (y - meta_.origin_y) / meta_.dy;

    // Step 2: 钳制到有效范围（边界外推）
    const double fxC = std::clamp(fx, 0.0, static_cast<double>(meta_.width - 1));
    const double fyC = std::clamp(fy, 0.0, static_cast<double>(meta_.height - 1));

    // Step 3: 确定四个相邻像素索引
    const int x0 = static_cast<int>(std::floor(fxC));
    const int y0 = static_cast<int>(std::floor(fyC));
    const int x1 = std::min(x0 + 1, meta_.width - 1);
    const int y1 = std::min(y0 + 1, meta_.height - 1);

    // Step 4: 计算插值权重
    const double tx = fxC - x0;  // X方向权重 [0,1]
    const double ty = fyC - y0;  // Y方向权重 [0,1]

    // Step 5: 获取四个角点的高程值
    const double v00 = at(x0, y0);  // 左下角
    const double v10 = at(x1, y0); // 右下角
    const double v01 = at(x0, y1); // 左上角
    const double v11 = at(x1, y1); // 右上角

    // Step 6: 双线性插值
    // 先在X方向插值得到两行结果
    const double v0 = v00 * (1.0 - tx) + v10 * tx;
    const double v1 = v01 * (1.0 - tx) + v11 * tx;
    // 再在Y方向插值得到最终结果
    return static_cast<float>(v0 * (1.0 - ty) + v1 * ty);
}
