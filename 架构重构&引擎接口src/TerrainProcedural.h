#pragma once
#include "ITerrain.h"

#include <cmath>

class TerrainProcedural : public ITerrain
{
public:
    enum class Mode
    {
        Flat,
        GaussianHill,
        Sombrero
    };

    struct Gaussian
    {
        double xc = 0.0;
        double yc = 0.0;
        double A = 80.0;
        double sigma = 50.0;
    };

    void setMode(Mode m) { mode_ = m; }
    void setBaseZ(float z) { baseZ_ = z; }
    void setGaussian(const Gaussian &g) { gaussian_ = g; }

    void setGaussianHill(double baseZ, double peakA, double sigma)
    {
        mode_ = Mode::GaussianHill;
        baseZ_ = baseZ;
        gaussian_.A = peakA;
        gaussian_.sigma = sigma;
    }

    void setSombrero(double baseZ, double amplitude, double radius)
    {
        mode_ = Mode::Sombrero;
        baseZ_ = baseZ;
        sombreroAmp_ = amplitude;
        sombreroR_ = radius;
    }

    float height(double x, double y) const override
    {
        if (mode_ == Mode::Flat)
        {
            return baseZ_;
        }

        if (mode_ == Mode::GaussianHill)
        {
            const double dx = x - gaussian_.xc;
            const double dy = y - gaussian_.yc;
            const double r2 = dx * dx + dy * dy;
            const double s2 = gaussian_.sigma * gaussian_.sigma;
            const double h = baseZ_ + gaussian_.A * std::exp(-0.5 * r2 / std::max(1e-12, s2));
            return (float)h;
        }

        if (mode_ == Mode::Sombrero)
        {
            const double dx = x - gaussian_.xc;
            const double dy = y - gaussian_.yc;
            const double r = std::sqrt(dx * dx + dy * dy);
            const double r0 = sombreroR_;
            const double xsi = r / r0;
            const double h = baseZ_ + sombreroAmp_ * ((1.0 - xsi) * (1.0 + xsi) * (1.0 - xsi) * (1.0 + xsi));
            return (float)h;
        }

        return baseZ_;
    }

private:
    Mode mode_ = Mode::Flat;
    float baseZ_ = 0.0f;
    Gaussian gaussian_{0.0, 0.0, 80.0, 50.0};
    double sombreroAmp_ = 80.0;
    double sombreroR_ = 50.0;
};
