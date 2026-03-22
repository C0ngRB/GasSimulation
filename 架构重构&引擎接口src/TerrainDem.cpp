#include "TerrainDem.h"

#include <QByteArray>
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

namespace {

static bool readAllBytes(const QString& path, QByteArray& out, QString& err)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        err = "Cannot open: " + path;
        return false;
    }
    out = f.readAll();
    return true;
}

} // namespace

bool TerrainDem::load(const std::string& metaPath, const std::string& binPath)
{
    valid_ = false;
    w_ = 0;
    h_ = 0;
    xMin_ = 0.0;
    yMin_ = 0.0;
    xMax_ = 0.0;
    yMax_ = 0.0;
    dx_ = 1.0;
    dy_ = 1.0;
    data_.clear();

    QString errOut;
    const QString metaPathQ = QString::fromStdString(metaPath);
    const QString binPathQ = QString::fromStdString(binPath);

    QByteArray metaBytes;
    if (!readAllBytes(metaPathQ, metaBytes, errOut)) {
        return false;
    }

    QJsonParseError pe;
    const QJsonDocument doc = QJsonDocument::fromJson(metaBytes, &pe);
    if (pe.error != QJsonParseError::NoError || !doc.isObject()) {
        return false;
    }

    const QJsonObject o = doc.object();

    auto getInt = [&](const char* k, int& v) -> bool {
        if (!o.contains(k) || !o.value(k).isDouble()) return false;
        v = o.value(k).toInt();
        return true;
    };

    auto getDouble = [&](const char* k, double& v) -> bool {
        if (!o.contains(k) || !o.value(k).isDouble()) return false;
        v = o.value(k).toDouble();
        return true;
    };

    int width = 0;
    int height = 0;
    double originX = 0.0;
    double originY = 0.0;
    double ddx = 0.0;
    double ddy = 0.0;
    int epsg = 0;

    if (!getInt("width", width) || !getInt("height", height)) {
        return false;
    }
    if (!getDouble("origin_x", originX) || !getDouble("origin_y", originY)) {
        return false;
    }
    if (!getDouble("dx", ddx) || !getDouble("dy", ddy)) {
        return false;
    }
    if (o.contains("epsg") && o.value("epsg").isDouble()) {
        epsg = o.value("epsg").toInt();
    }

    if (width <= 0 || height <= 0) {
        return false;
    }
    if (std::abs(ddx) < 1e-12 || std::abs(ddy) < 1e-12) {
        return false;
    }
    if (epsg == 0) {
        return false;
    }

    QByteArray binBytes;
    if (!readAllBytes(binPathQ, binBytes, errOut)) {
        return false;
    }

    const std::size_t n =
        static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    const std::size_t expectedBytes = n * sizeof(float);

    if (static_cast<std::size_t>(binBytes.size()) != expectedBytes) {
        return false;
    }

    data_.resize(n);
    std::memcpy(data_.data(), binBytes.constData(), expectedBytes);

    w_ = width;
    h_ = height;
    dx_ = ddx;
    dy_ = ddy;
    xMin_ = originX;
    yMin_ = originY;
    xMax_ = originX + (static_cast<double>(w_ - 1) * dx_);
    yMax_ = originY + (static_cast<double>(h_ - 1) * dy_);

    valid_ = true;
    return true;
}

float TerrainDem::at(int i, int j) const
{
    if (w_ <= 0 || h_ <= 0 || data_.empty()) {
        return 0.0f;
    }

    i = std::clamp(i, 0, w_ - 1);
    j = std::clamp(j, 0, h_ - 1);

    return data_[static_cast<std::size_t>(i) +
                 static_cast<std::size_t>(j) * static_cast<std::size_t>(w_)];
}

float TerrainDem::sampleBilinear(double x, double y) const
{
    if (!valid_ || data_.empty() || w_ <= 0 || h_ <= 0) {
        return 0.0f;
    }

    if (w_ == 1 && h_ == 1) {
        return data_[0];
    }

    const double fx = (x - xMin_) / dx_;
    const double fy = (y - yMin_) / dy_;

    const double fxC = std::clamp(fx, 0.0, static_cast<double>(w_ - 1));
    const double fyC = std::clamp(fy, 0.0, static_cast<double>(h_ - 1));

    const int x0 = static_cast<int>(std::floor(fxC));
    const int y0 = static_cast<int>(std::floor(fyC));
    const int x1 = std::min(x0 + 1, w_ - 1);
    const int y1 = std::min(y0 + 1, h_ - 1);

    const double tx = fxC - static_cast<double>(x0);
    const double ty = fyC - static_cast<double>(y0);

    const float q00 = at(x0, y0);
    const float q10 = at(x1, y0);
    const float q01 = at(x0, y1);
    const float q11 = at(x1, y1);

    const double v0 = static_cast<double>(q00) * (1.0 - tx) + static_cast<double>(q10) * tx;
    const double v1 = static_cast<double>(q01) * (1.0 - tx) + static_cast<double>(q11) * tx;

    return static_cast<float>(v0 * (1.0 - ty) + v1 * ty);
}

double TerrainDem::minElevation() const
{
    if (data_.empty()) return 0.0;

    float minVal = data_[0];
    for (float v : data_) {
        if (v < minVal) {
            minVal = v;
        }
    }
    return static_cast<double>(minVal);
}

float TerrainDem::height(double x, double y) const
{
    if (!valid_) return 0.0f;
    return sampleBilinear(x, y);
}
