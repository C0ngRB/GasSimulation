#include "PlumeEngine.h"

#include "GaussianPlumeModel.h"
#include "GaussianPuffModel.h"
#include "Simulator3D.h"

PlumeEngine::PlumeEngine() = default;
PlumeEngine::~PlumeEngine() = default;

void PlumeEngine::setModel(SimulationModelType t)
{
    modelType_ = t;
}

SimulationModelType PlumeEngine::model() const
{
    return modelType_;
}

bool PlumeEngine::initialize(const Grid3D& g, const ITerrain* terr, const SimulationConfig& p, std::string& err)
{
    grid_ = g;
    terr_ = terr;
    config_ = p;

    switch (modelType_) {
    case SimulationModelType::CfdEulerian:
        model_ = std::make_unique<Simulator3D>();
        break;
    case SimulationModelType::GaussianPlume:
        model_ = std::make_unique<GaussianPlumeModel>();
        break;
    case SimulationModelType::GaussianPuff:
        model_ = std::make_unique<GaussianPuffModel>();
        break;
    default:
        err = "PlumeEngine: unknown model type.";
        return false;
    }

    return model_->initialize(grid_, terr_, config_, err);
}

void PlumeEngine::reset()
{
    if (model_) model_->reset();
}

double PlumeEngine::step()
{
    if (!model_) return 0.0;
    return model_->step();
}

double PlumeEngine::time() const
{
    return model_ ? model_->time() : 0.0;
}

const Grid3D& PlumeEngine::grid() const
{
    return model_ ? model_->grid() : grid_;
}

SimulationConfig PlumeEngine::config() const
{
    return model_ ? model_->config() : config_;
}

double PlumeEngine::currentWindSpeed() const
{
    return model_ ? model_->currentWindSpeed() : 0.0;
}

double PlumeEngine::currentWindDirDeg() const
{
    return model_ ? model_->currentWindDirDeg() : 0.0;
}

double PlumeEngine::currentLeakRate() const
{
    return model_ ? model_->currentLeakRate() : 0.0;
}

void PlumeEngine::extractSliceXY(int k, std::vector<float>& out, float& maxC) const
{
    if (model_) {
        model_->extractSliceXY(k, out, maxC);
    } else {
        out.clear();
        maxC = 0.0f;
    }
}
