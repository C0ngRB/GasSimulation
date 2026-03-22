#pragma once
#include <memory>
#include <string>

#include "Grid3D.h"
#include "ISimulationModel.h"
#include "ITerrain.h"
#include "SimulationConfig.h"

class PlumeEngine
{
public:
    PlumeEngine();
    ~PlumeEngine();

    void setModel(SimulationModelType t);
    SimulationModelType model() const;

    bool initialize(const Grid3D& g, const ITerrain* terr, const SimulationConfig& p, std::string& err);
    void reset();
    double step();

    double time() const;
    const Grid3D& grid() const;
    SimulationConfig config() const;

    double currentWindSpeed() const;
    double currentWindDirDeg() const;
    double currentLeakRate() const;

    void extractSliceXY(int k, std::vector<float>& out, float& maxC) const;

private:
    Grid3D grid_;
    const ITerrain* terr_ = nullptr;
    SimulationConfig config_;

    std::unique_ptr<ISimulationModel> model_;
    SimulationModelType modelType_ = SimulationModelType::CfdEulerian;
};
