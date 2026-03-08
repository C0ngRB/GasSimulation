#pragma once

#include <QString>
#include <vector>

/**
 * @brief 读取外部 CFD 输出的 CSV 帧。
 *
 * CSV 约定：
 * - 可选头行：# Nx=300 Ny=150 t=0.50
 * - 后续 Ny 行，每行 Nx 个 float，用逗号分隔（行=Y，列=X）
 */
class CsvFrameReader {
public:
    struct Frame {
        int Nx = 0;
        int Ny = 0;
        double t = 0.0;
        std::vector<float> data; // size = Nx*Ny
    };

    /**
     * @brief 读取一个 CSV 文件。
     * @param path CSV 文件路径
     * @param out 输出帧
     * @param err 错误信息（失败时）
     * @return true 成功；false 失败
     */
    static bool read(const QString& path, Frame& out, QString& err);
};
