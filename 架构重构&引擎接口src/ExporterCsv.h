#pragma once
#include <string>
#include <vector>

class ExporterCsv
{
public:
    struct Meta
    {
        int epsg = 0;
        double origin_x = 0;
        double origin_y = 0;
        double dx = 1;
        double dy = 1;
        double z = 0;
        double t = 0;
    };

    static bool writeGridFrame(const std::string &path,
                               const Meta &meta,
                               int Nx, int Ny,
                               const std::vector<float> &data,
                               std::string &err);
};
