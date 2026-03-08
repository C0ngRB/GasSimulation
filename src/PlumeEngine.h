#pragma once
#include <string>
#include <vector>

#include "Grid3D.h"
#include "ITerrain.h"
#include "Simulator3D.h"
#include "GaussianPlumeModel.h"
#include "GaussianPuffModel.h"

class PlumeEngine
{
public:
    enum class ModelType
    {
        CfdEulerian = 0,
        GaussianPlume = 1,
        LagrangianPuff = 2,
    };

    struct Params
    {
        double windSpeed_mps = 2.0;
        double windDir_deg = 0.0;

        double K_m2ps = 2.0;
        double decay_1ps = 0.0;

        double srcX_m = 0.0;
        double srcY_m = 0.0;
        double srcZ_m = 2.0;
        double srcRadius_m = 2.0;

        double leakRate_kgps = 1.0;

        double dt_s = 0.05;
        double totalTime_s = 60.0;
        bool autoClampDt = true;
    };

    PlumeEngine();

    void setModel(ModelType t);
    ModelType model() const;

    bool initialize(const Grid3D &g, const ITerrain *terr, const Params &p, std::string &err);
    void reset();
    void step();

    double time() const;
    const Grid3D &grid() const;
    Params params() const;

    void extractSliceXY(int k, std::vector<float> &out, float &maxC) const;

private:
    ModelType modelType_ = ModelType::CfdEulerian;

    Grid3D grid_;
    const ITerrain *terr_ = nullptr;
    Params params_;
    double t_ = 0.0;

    Simulator3D cfd_;
    GaussianPlumeModel gauss_;
    GaussianPuffModel puff_;
};
