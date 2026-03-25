#pragma once
#include "ITerrain.h"

#include <algorithm>
#include <cmath>

class TerrainProcedural : public ITerrain
{
public:
    enum class Mode
    {
        Flat,
        GaussianHill,
        Sombrero,
        SphereCap
    };

    struct Gaussian
    {
        double xc = 0.0;
        double yc = 0.0;
        double A = 80.0;
        double sigma = 50.0;
    };

    struct SphereCap
    {
        double xc = 0.0;
        double yc = 0.0;
        double capHeight = 60.0;
        double capRadius = 80.0;
    };

    void setMode(Mode m) { mode_ = m; }
    void setBaseZ(float z) { baseZ_ = z; }
    void setGaussian(const Gaussian &g) { gaussian_ = g; }

    void setGaussianHill(double baseZ, double peakA, double sigma)
    {
        mode_ = Mode::GaussianHill;
        baseZ_ = static_cast<float>(baseZ);
        gaussian_.A = peakA;
        gaussian_.sigma = sigma;
    }

    void setSombrero(double baseZ, double amplitude, double radius)
    {
        mode_ = Mode::Sombrero;
        baseZ_ = static_cast<float>(baseZ);
        sombreroAmp_ = amplitude;
        sombreroR_ = radius;
    }

    void setSombrero(double baseZ, double amplitude, double radius, double xc, double yc)
    {
        mode_ = Mode::Sombrero;
        baseZ_ = static_cast<float>(baseZ);
        gaussian_.xc = xc;
        gaussian_.yc = yc;
        sombreroAmp_ = amplitude;
        sombreroR_ = radius;
    }

    void setSphereCap(const SphereCap& sc)
    {
        mode_ = Mode::SphereCap;
        sphereCap_ = sc;
    }

    void setSphereCap(double baseZ, double xc, double yc, double capHeight, double capRadius)
    {
        mode_ = Mode::SphereCap;
        baseZ_ = static_cast<float>(baseZ);
        sphereCap_.xc = xc;
        sphereCap_.yc = yc;
        sphereCap_.capHeight = capHeight;
        sphereCap_.capRadius = capRadius;
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
            return static_cast<float>(h);
        }

        if (mode_ == Mode::Sombrero)
        {
            const double dx = x - gaussian_.xc;
            const double dy = y - gaussian_.yc;
            const double r = std::sqrt(dx * dx + dy * dy);
            const double r0 = std::max(1e-12, sombreroR_);
            const double xsi = r / r0;
            const double ring = (1.0 - xsi) * (1.0 + xsi) * (1.0 - xsi) * (1.0 + xsi);
            const double h = baseZ_ + sombreroAmp_ * ring;
            return static_cast<float>(h);
        }

        if (mode_ == Mode::SphereCap)
        {
            const double hCap = std::max(1e-12, sphereCap_.capHeight);
            const double a = std::max(1e-12, sphereCap_.capRadius);

            const double dx = x - sphereCap_.xc;
            const double dy = y - sphereCap_.yc;
            const double r2 = dx * dx + dy * dy;

            if (r2 > a * a)
            {
                return baseZ_;
            }

            const double R = (a * a + hCap * hCap) / (2.0 * hCap);
            const double zc = static_cast<double>(baseZ_) - (R - hCap);
            const double under = std::max(0.0, R * R - r2);
            const double z = zc + std::sqrt(under);
            return static_cast<float>(std::max(z, static_cast<double>(baseZ_)));
        }

        return baseZ_;
    }

private:
    Mode mode_ = Mode::Flat;
    float baseZ_ = 0.0f;
    Gaussian gaussian_{0.0, 0.0, 80.0, 50.0};
    double sombreroAmp_ = 80.0;
    double sombreroR_ = 50.0;
    SphereCap sphereCap_{0.0, 0.0, 60.0, 80.0};
};
