#pragma once
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

#include "Grid3D.h"
#include "TerrainSampler.h"
#include "WindField.h"

class EulerianSolver3D
{
public:
    struct Params
    {
        double K_m2ps = 1.0;
        double decay_1ps = 0.0;
        double srcX_m = 0.0;
        double srcY_m = 0.0;
        double srcZ_m = 0.0;
        double srcRadius_m = 2.0;
    };

    bool initialize(const Grid3D& grid, const TerrainSampler* terrain, const Params& p, std::string& err);
    void reset();
    double stableDt(double representativeSpeed) const;
    void step(double dt, const IWindField& windField, double leakRate_kgps);

    void extractSliceXY(int zIndex, std::vector<float>& outSlice, float& outMax) const;
    void extractColumnIntegralXY(std::vector<float>& outGrid, float& outMax) const;

    const Grid3D& grid() const { return grid_; }
    float maxC() const { return maxC_; }

private:
    bool isSolid(int i, int j, int k) const
    {
        return solid_[grid_.idx(i, j, k)] != 0;
    }

    bool isSourceCell(int i, int j, int k) const
    {
        return sourceMask_[grid_.idx(i, j, k)] != 0;
    }

    float cell(int i, int j, int k) const
    {
        return C_[grid_.idx(i, j, k)];
    }

    float fluidNeighborOrSelf(int ni, int nj, int nk, float self) const;
    void buildSolidMask();
    void buildSourceMask();

private:
    Grid3D grid_{};
    const TerrainSampler* terrain_{nullptr};
    Params params_{};

    std::vector<float> C_;
    std::vector<float> Cnew_;
    std::vector<std::uint8_t> solid_;
    std::vector<std::uint8_t> sourceMask_;

    int sourceCellCount_{0};
    float maxC_{0.0f};
};
