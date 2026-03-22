#pragma once
#include "ITerrain.h"

struct TerrainFlat : public ITerrain
{
    float z = 0.0f;
    float height(double, double) const override { return z; }
};
