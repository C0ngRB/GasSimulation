#pragma once
#include <algorithm>
#include <string>
#include <vector>

#include "ITerrain.h"

class TerrainDem : public ITerrain
{
public:
    bool load(const std::string& metaPath, const std::string& binPath);

    float height(double x, double y) const override;
    bool isValid() const override { return valid_; }

    int width() const { return w_; }
    int height() const { return h_; }

    double xMin() const { return xMin_; }
    double yMin() const { return yMin_; }
    double xMax() const { return xMax_; }
    double yMax() const { return yMax_; }

    double dx() const { return dx_; }
    double dy() const { return dy_; }
    double cellSize() const { return std::max(std::abs(dx_), std::abs(dy_)); }

    double centerX() const { return 0.5 * (xMin_ + xMax_); }
    double centerY() const { return 0.5 * (yMin_ + yMax_); }

    double minX() const { return xMin_; }
    double minY() const { return yMin_; }

    double minElevation() const;

    int nx() const { return w_; }
    int ny() const { return h_; }

private:
    float at(int i, int j) const;
    float sampleBilinear(double x, double y) const;

private:
    bool valid_ = false;
    int w_ = 0;
    int h_ = 0;

    double xMin_ = 0.0;
    double yMin_ = 0.0;
    double xMax_ = 0.0;
    double yMax_ = 0.0;
    double dx_ = 1.0;
    double dy_ = 1.0;

    std::vector<float> data_;
};
