#include "ExporterCsv.h"

#include <fstream>
#include <iomanip>

bool ExporterCsv::writeGridFrame(const std::string &path,
                                 const Meta &meta,
                                 int Nx, int Ny,
                                 const std::vector<float> &data,
                                 std::string &err)
{
    if ((int)data.size() != Nx * Ny)
    {
        err = "data size mismatch";
        return false;
    }

    std::ofstream f(path);
    if (!f)
    {
        err = "open failed";
        return false;
    }

    f << "# epsg: " << meta.epsg << "\n";
    f << "# origin_x: " << meta.origin_x << "\n";
    f << "# origin_y: " << meta.origin_y << "\n";
    f << "# dx: " << meta.dx << "\n";
    f << "# dy: " << meta.dy << "\n";
    f << "# z: " << meta.z << "\n";
    f << "# Nx: " << Nx << "\n";
    f << "# Ny: " << Ny << "\n";
    f << "# t: " << meta.t << "\n";

    f << std::setprecision(6) << std::scientific;
    for (int j = 0; j < Ny; ++j)
    {
        for (int i = 0; i < Nx; ++i)
        {
            const float v = data[j * Nx + i];
            f << v;
            if (i + 1 < Nx)
                f << ",";
        }
        f << "\n";
    }
    return true;
}
