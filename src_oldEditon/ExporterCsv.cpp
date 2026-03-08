#include "ExporterCsv.h"
#include <QFile>
#include <QTextStream>

/**
 * @file ExporterCsv.cpp
 * @brief CSV格式数据导出实现
 *
 * 【实现说明】
 * 实现CSV文件的写入功能。
 * 使用Qt的QTextStream进行文本输出，保证跨平台兼容性。
 *
 * 【数据格式】
 * - 使用逗号分隔符（CSV标准）
 * - 科学计数法表示浓度值（适合大范围浓度值）
 * - 每行以换行符结束
 *
 * 【头部注释】
 * 以#开头的行作为注释，包含：
 * - crs: EPSG投影代码
 * - origin/dx/dy/z: 网格信息
 * - Nx/Ny/t: 切片信息
 */
bool ExporterCsv::writeGridFrame(const QString& path,
                                 const FrameMeta& meta,
                                 const std::vector<float>& gridValues,
                                 QString& errOut)
{
    errOut.clear();

    // Step 1: 验证数据大小
    const std::size_t expected = static_cast<std::size_t>(meta.Nx) * meta.Ny;
    if (gridValues.size() != expected) {
        errOut = "Grid values size mismatch";
        return false;
    }

    // Step 2: 打开文件
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Text)) {
        errOut = "Cannot write: " + path;
        return false;
    }

    // Step 3: 配置输出流
    QTextStream out(&f);
    out.setRealNumberNotation(QTextStream::ScientificNotation);
    out.setRealNumberPrecision(6);

    // Step 4: 写入头部注释
    out << "# crs=EPSG:" << meta.epsg << "\n";
    out << "# origin_x=" << meta.origin_x << " origin_y=" << meta.origin_y
        << " dx=" << meta.dx << " dy=" << meta.dy << " z=" << meta.z << "\n";
    out << "# Nx=" << meta.Nx << " Ny=" << meta.Ny << " t=" << meta.t << "\n";

    // Step 5: 写入浓度数据
    for (int j = 0; j < meta.Ny; ++j) {
        for (int i = 0; i < meta.Nx; ++i) {
            const float v = gridValues[static_cast<std::size_t>(i) + static_cast<std::size_t>(j) * meta.Nx];
            out << v;
            if (i + 1 < meta.Nx) out << ",";
        }
        out << "\n";
    }

    return true;
}
