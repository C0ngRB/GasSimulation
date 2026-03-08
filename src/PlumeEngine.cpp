#include "PlumeEngine.h"

#include <algorithm>

PlumeEngine::PlumeEngine()
{
}

void PlumeEngine::setModel(ModelType t)
{
    modelType_ = t;
}

PlumeEngine::ModelType PlumeEngine::model() const
{
    return modelType_;
}

bool PlumeEngine::initialize(const Grid3D &g, const ITerrain *terr, const Params &p, std::string &err)
{
    grid_ = g;
    terr_ = terr;
    params_ = p;
    t_ = 0.0;

    if (modelType_ == ModelType::CfdEulerian)
    {
        Simulator3D::Params pp;
        pp.windSpeed_mps = p.windSpeed_mps;
        pp.windDir_deg = p.windDir_deg;
        pp.K_m2ps = p.K_m2ps;
        pp.decay_1ps = p.decay_1ps;
        pp.srcX_m = p.srcX_m;
        pp.srcY_m = p.srcY_m;
        pp.srcZ_m = p.srcZ_m;
        pp.srcRadius_m = p.srcRadius_m;
        pp.leakRate = p.leakRate_kgps;
        pp.dt_s = p.dt_s;
        pp.totalTime_s = p.totalTime_s;
        pp.autoClampDt = p.autoClampDt;
        QString qerr;
        bool ok = cfd_.initialize(g, terr, pp, qerr);
        if (!ok)
        {
            err = qerr.toStdString();
            return false;
        }
        return true;
    }
    else if (modelType_ == ModelType::GaussianPlume)
    {
        GaussianPlumeModel::Params pp;
        pp.windSpeed_mps = p.windSpeed_mps;
        pp.windDir_deg = p.windDir_deg;
        pp.K_m2ps = p.K_m2ps;
        pp.decay_1ps = p.decay_1ps;
        pp.srcX_m = p.srcX_m;
        pp.srcY_m = p.srcY_m;
        pp.srcZ_m = p.srcZ_m;
        pp.srcRadius_m = p.srcRadius_m;
        pp.leakRate_kgps = p.leakRate_kgps;
        pp.dt_s = p.dt_s;
        pp.totalTime_s = p.totalTime_s;
        pp.autoClampDt = p.autoClampDt;
        return gauss_.initialize(g, terr, pp, err);
    }
    else
    {
        GaussianPuffModel::Params pp;
        pp.windSpeed_mps = p.windSpeed_mps;
        pp.windDir_deg = p.windDir_deg;
        pp.K_m2ps = p.K_m2ps;
        pp.decay_1ps = p.decay_1ps;
        pp.srcX_m = p.srcX_m;
        pp.srcY_m = p.srcY_m;
        pp.srcZ_m = p.srcZ_m;
        pp.srcRadius_m = p.srcRadius_m;
        pp.leakRate_kgps = p.leakRate_kgps;
        pp.dt_s = p.dt_s;
        pp.totalTime_s = p.totalTime_s;
        pp.autoClampDt = p.autoClampDt;
        return puff_.initialize(g, terr, pp, err);
    }
}

void PlumeEngine::reset()
{
    t_ = 0.0;
    if (modelType_ == ModelType::CfdEulerian)
        cfd_.reset();
    else if (modelType_ == ModelType::GaussianPlume)
        gauss_.reset();
    else
        puff_.reset();
}

void PlumeEngine::step()
{
    if (modelType_ == ModelType::CfdEulerian)
    {
        cfd_.step();
        t_ = cfd_.time();
    }
    else if (modelType_ == ModelType::GaussianPlume)
    {
        gauss_.step();
        t_ = gauss_.time();
    }
    else
    {
        puff_.step();
        t_ = puff_.time();
    }
}

double PlumeEngine::time() const
{
    if (modelType_ == ModelType::CfdEulerian)
        return cfd_.time();
    if (modelType_ == ModelType::GaussianPlume)
        return gauss_.time();
    return puff_.time();
}

const Grid3D &PlumeEngine::grid() const
{
    if (modelType_ == ModelType::CfdEulerian)
        return cfd_.grid();
    if (modelType_ == ModelType::GaussianPlume)
        return gauss_.grid();
    return puff_.grid();
}

PlumeEngine::Params PlumeEngine::params() const
{
    return params_;
}

void PlumeEngine::extractSliceXY(int k, std::vector<float> &out, float &maxC) const
{
    if (modelType_ == ModelType::CfdEulerian)
        cfd_.extractSliceXY(k, out, maxC);
    else if (modelType_ == ModelType::GaussianPlume)
        gauss_.extractSliceXY(k, out, maxC);
    else
        puff_.extractSliceXY(k, out, maxC);
}
