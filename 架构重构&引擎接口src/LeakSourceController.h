#pragma once
#include <algorithm>
#include <random>

class LeakSourceController
{
public:
    struct NoiseParams
    {
        bool enabled = true;
        unsigned int seed = 12143;
        double updateEvery_s = 1.0;
        double leakRelSigma = 0.10;
    };

    struct Params
    {
        double baseLeakRate = 1.0;
        NoiseParams noise;
    };

    LeakSourceController() = default;
    explicit LeakSourceController(const Params& p) : params_(p) {}

    void setParams(const Params& p) { params_ = p; }
    const Params& params() const { return params_; }

    void initialize()
    {
        reset();
    }

    void reset()
    {
        t_ = 0.0;
        rng_ = std::mt19937(params_.noise.seed);
        nextNoiseUpdateT_ = 0.0;
        refreshNoise();
    }

    void advance(double dt)
    {
        t_ += std::max(0.0, dt);
        if (!params_.noise.enabled) return;
        if (t_ + 1.0e-12 >= nextNoiseUpdateT_) {
            refreshNoise();
        }
    }

    double currentLeakRate() const { return effLeakRate_; }
    double time() const { return t_; }

private:
    void refreshNoise()
    {
        double leak = params_.baseLeakRate;
        if (params_.noise.enabled) {
            const double rel = 1.0 + norm_(rng_) * params_.noise.leakRelSigma;
            leak *= std::max(0.0, rel);
        }
        effLeakRate_ = std::max(0.0, leak);
        nextNoiseUpdateT_ = t_ + std::max(1.0e-6, params_.noise.updateEvery_s);
    }

private:
    Params params_{};
    double t_{0.0};
    double nextNoiseUpdateT_{0.0};
    double effLeakRate_{0.0};
    std::mt19937 rng_{};
    std::normal_distribution<double> norm_{0.0, 1.0};
};
