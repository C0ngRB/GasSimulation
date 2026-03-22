#pragma once

enum class SimulationModelType
{
    CfdEulerian = 0,
    GaussianPlume = 1,
    GaussianPuff = 2
};

inline const char* simulationModelTypeName(SimulationModelType t)
{
    switch (t) {
    case SimulationModelType::CfdEulerian:
        return "Terrain-aware Eulerian";
    case SimulationModelType::GaussianPlume:
        return "Terrain-aware Gaussian plume";
    case SimulationModelType::GaussianPuff:
        return "Terrain-aware Gaussian puff";
    default:
        return "Unknown";
    }
}

struct SimulationStochasticConfig
{
    bool enabled = true;
    unsigned int seed = 42;
    double updateEvery_s = 1.0;
    double windSpeedSigma_mps = 0.30;
    double windDirSigma_deg = 7.0;
    double leakRelSigma = 0.10;
};

struct SimulationConfig
{
    SimulationModelType model = SimulationModelType::CfdEulerian;

    double totalTime_s = 60.0;
    double dt_s = 0.05;
    bool autoClampDt = true;

    double windSpeed_mps = 2.0;
    double windDir_deg = 0.0;

    double K_m2ps = 1.0;
    double decay_1ps = 0.0;

    double srcX_m = 0.0;
    double srcY_m = 0.0;
    double srcZ_m = 2.0;
    double srcRadius_m = 2.0;
    double leakRate_kgps = 1.0;

    double releaseInterval_s = 0.0;

    SimulationStochasticConfig noise;
};
