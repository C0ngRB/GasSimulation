#pragma once
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "Grid3D.h"
#include "ITerrain.h"

class TerrainSampler final
{
public:
    TerrainSampler() = default;

    bool initialize(const Grid3D& grid, const ITerrain* terrain, std::string& err)
    {
        err.clear();
        grid_ = grid;
        terrain_ = terrain;

        if (!terrain_) {
            err = "TerrainSampler: terrain is null.";
            return false;
        }

        try {
            grid_.validate();
        } catch (const std::exception& e) {
            err = e.what();
            return false;
        }

        H_.assign(static_cast<std::size_t>(grid_.Nx) * static_cast<std::size_t>(grid_.Ny), 0.0f);
        for (int j = 0; j < grid_.Ny; ++j) {
            const double y = grid_.y(j);
            for (int i = 0; i < grid_.Nx; ++i) {
                const double x = grid_.x(i);
                H_[gridIndex(i, j)] = terrain_->height(x, y);
            }
        }
        return true;
    }

    void clear()
    {
        terrain_ = nullptr;
        H_.clear();
        grid_ = Grid3D{};
    }

    bool isValid() const
    {
        return terrain_ != nullptr && !H_.empty();
    }

    const Grid3D& grid() const
    {
        return grid_;
    }

    double heightAtNode(int i, int j) const
    {
        if (!isValid()) return 0.0;
        i = std::clamp(i, 0, grid_.Nx - 1);
        j = std::clamp(j, 0, grid_.Ny - 1);
        return H_[gridIndex(i, j)];
    }

    double heightBilinear(double x, double y) const
    {
        if (!isValid()) return 0.0;
        if (grid_.Nx <= 1 || grid_.Ny <= 1) {
            return terrain_->height(x, y);
        }

        const double fx = (x - grid_.x0) / grid_.dx;
        const double fy = (y - grid_.y0) / grid_.dy;

        int i0 = static_cast<int>(std::floor(fx));
        int j0 = static_cast<int>(std::floor(fy));
        double tx = fx - static_cast<double>(i0);
        double ty = fy - static_cast<double>(j0);

        i0 = std::clamp(i0, 0, grid_.Nx - 2);
        j0 = std::clamp(j0, 0, grid_.Ny - 2);
        tx = std::clamp(tx, 0.0, 1.0);
        ty = std::clamp(ty, 0.0, 1.0);

        const int i1 = i0 + 1;
        const int j1 = j0 + 1;

        const double h00 = H_[gridIndex(i0, j0)];
        const double h10 = H_[gridIndex(i1, j0)];
        const double h01 = H_[gridIndex(i0, j1)];
        const double h11 = H_[gridIndex(i1, j1)];

        const double hx0 = h00 * (1.0 - tx) + h10 * tx;
        const double hx1 = h01 * (1.0 - tx) + h11 * tx;
        return hx0 * (1.0 - ty) + hx1 * ty;
    }

    void gradient(double x, double y, double& dzdx, double& dzdy) const
    {
        if (!isValid()) {
            dzdx = 0.0;
            dzdy = 0.0;
            return;
        }

        const double h = std::max(grid_.dx, grid_.dy);
        const double hx1 = heightBilinear(x + h, y);
        const double hx0 = heightBilinear(x - h, y);
        const double hy1 = heightBilinear(x, y + h);
        const double hy0 = heightBilinear(x, y - h);

        dzdx = (hx1 - hx0) / (2.0 * h);
        dzdy = (hy1 - hy0) / (2.0 * h);
    }

    double maxHeightAlongRay(double x0, double y0, double x1, double y1, int samples = 16) const
    {
        if (!isValid()) return 0.0;

        samples = std::max(2, samples);
        double hMax = -1.0e30;
        for (int s = 0; s < samples; ++s) {
            const double t = static_cast<double>(s) / static_cast<double>(samples - 1);
            const double x = x0 * (1.0 - t) + x1 * t;
            const double y = y0 * (1.0 - t) + y1 * t;
            hMax = std::max(hMax, heightBilinear(x, y));
        }
        return hMax;
    }

private:
    std::size_t gridIndex(int i, int j) const
    {
        return static_cast<std::size_t>(i)
             + static_cast<std::size_t>(grid_.Nx) * static_cast<std::size_t>(j);
    }

private:
    Grid3D grid_{};
    const ITerrain* terrain_{nullptr};
    std::vector<float> H_;
};
