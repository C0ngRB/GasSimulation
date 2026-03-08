/**
 * @file MainWindow.cpp
 * @brief GasDispersionDesktop 主窗口实现
 *
 * 【实现说明】
 * 实现MainWindow类的所有功能，包括：
 * - UI构建和布局管理
 * - 参数验证和地形预览
 * - 仿真生命周期管理
 * - 可视化渲染（地形+浓度切片）
 * - 数据导出
 *
 * 【UI组件组织】
 * 使用Qt Designer风格的代码布局：
 * 1. 构造函数：创建所有UI组件并建立信号槽连接
 * 2. 地形相关槽函数：处理DEM加载、预览等
 * 3. 仿真控制槽函数：Run/Pause/Reset
 * 4. 渲染函数：buildTerrainPreview, renderTerrainOnly, renderTerrainAndSlice
 * 5. 辅助函数：坐标系转换、单位转换等
 *
 * 【渲染流程】
 * 1. buildTerrainPreview():
 *    - 根据地形模式选择数据源
 *    - 构建二维高程数组
 *    - 计算高程范围（minZ, maxZ）
 *
 * 2. renderTerrainOnly():
 *    - 对每个像素计算对应的网格位置
 *    - 使用hillshade算法渲染地形
 *    - 添加固定padding(0.35)拉远视野
 *
 * 3. renderTerrainAndSlice():
 *    - 调用Simulator3D::extractSliceXY()获取浓度切片
 *    - 根据背景模式选择底色（地形灰度/纯蓝）
 *    - 使用colorMap()将浓度映射为颜色
 *    - 半透明叠加显示
 *
 * 【坐标系转换】
 * 网格坐标 (i,j) → 世界坐标 (x,y):
 *   x = x0 + dx * i
 *   y = y0 + dy * j
 *
 * 像素坐标 (px,py) → 网格坐标 (i,j):
 *   i = round(px * scale)
 *   j = round(py * scale)
 *
 * 【与Simulator3D的交互】
 * - buildSimulation(): 创建Grid3D，调用Simulator3D::initialize()
 * - onTick(): 调用Simulator3D::step()推进仿真
 * - renderTerrainAndSlice(): 调用Simulator3D::extractSliceXY()获取数据
 */

#include "MainWindow.h"
#include "ColorMap.h"
#include "ExporterCsv.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QFileDialog>

#include <QDir>
#include <QPainter>

#include <algorithm>
#include <cmath>
#include <limits>

static QDoubleSpinBox* makeDsb(double minV, double maxV, double step, double val, int decimals=3) {
    auto* sb = new QDoubleSpinBox();
    sb->setRange(minV, maxV);
    sb->setDecimals(decimals);
    sb->setSingleStep(step);
    sb->setValue(val);
    return sb;
}
static QSpinBox* makeSsb(int minV, int maxV, int step, int val) {
    auto* sb = new QSpinBox();
    sb->setRange(minV, maxV);
    sb->setSingleStep(step);
    sb->setValue(val);
    return sb;
}

static QImage applyFixedPadding(const QImage& img, double padFrac, const QColor& fill) {
    const int W = img.width();
    const int H = img.height();
    const int padX = (int)std::round(W * padFrac);
    const int padY = (int)std::round(H * padFrac);
    const int CW = W + 2 * padX;
    const int CH = H + 2 * padY;

    QImage canvas(CW, CH, QImage::Format_RGB32);
    canvas.fill(fill);

    QPainter p(&canvas);
    p.drawImage(padX, padY, img);
    return canvas;
}

MainWindow::TerrainMode MainWindow::terrainMode() const {
    return static_cast<TerrainMode>(cbTerrainMode_->currentIndex());
}

const ITerrain* MainWindow::currentTerrain() const {
    const auto mode = terrainMode();
    if (mode == TerrainMode::Dem) {
        if (hasDem_ && dem_.isValid()) return &dem_;
        return nullptr;
    }
    if (mode == TerrainMode::Procedural) return &proc_;
    return &flat_;
}

QString MainWindow::framesDir() const {
    return QDir::current().filePath("outputs/frames");
}

void MainWindow::appendLog(const QString& s) {
    log_->appendPlainText(s);
}

void MainWindow::updateDomainInfo() {
    const double Lx = sbLx_->value();
    const double Ly = sbLy_->value();
    const double dx = std::max(1e-9, sbDx_->value());
    const double dy = std::max(1e-9, sbDy_->value());
    const int Nx = std::max(2, (int)std::floor(Lx/dx) + 1);
    const int Ny = std::max(2, (int)std::floor(Ly/dy) + 1);
    domainInfo_->setText(QString("Nx=%1 Ny=%2 (non-DEM)").arg(Nx).arg(Ny));
}

void MainWindow::enforceSourceCenterIfNeeded() {
    if (!cbAutoCenterSrc_ || !cbAutoCenterSrc_->isChecked()) return;
    if (terrainMode() == TerrainMode::Dem) return;
    if (!terrainPreviewReady_) return;

    const double xMin = tx0_;
    const double yMin = ty0_;
    const double xMax = tx0_ + (tNx_ - 1) * tdx_;
    const double yMax = ty0_ + (tNy_ - 1) * tdy_;
    const double cx = 0.5 * (xMin + xMax);
    const double cy = 0.5 * (yMin + yMax);

    const double x = sbSrcX_->value();
    const double y = sbSrcY_->value();

    const double eps = 1e-9;

    const bool out =
        (x < xMin - eps) || (x > xMax + eps) ||
        (y < yMin - eps) || (y > yMax + eps);

    const bool atCorner =
        (std::abs(x - xMin) < 1e-6) && (std::abs(y - yMin) < 1e-6);

    if (out || atCorner) {
        sbSrcX_->setValue(cx);
        sbSrcY_->setValue(cy);
        appendLog(QString("[SRC] auto-centered to domain center: (%1, %2)")
                  .arg(cx, 0, 'f', 2).arg(cy, 0, 'f', 2));
        syncSliceWithSourceIfNeeded();
    }
}

double MainWindow::clampSliceToFluid(double z) const {
    if (!terrainPreviewReady_) return z;
    const double zMin = (double)tMin_;
    const double zTop = (double)tMax_ + sbZTopMargin_->value();
    return std::clamp(z, zMin, zTop);
}

double MainWindow::effectiveZSlice() const {
    double z = sbZSlice_->value();
    return clampSliceToFluid(z);
}

double MainWindow::aglSliceZ() const {
    const double agl = sbAgl_->value();
    return clampSliceToFluid((double)groundAtSource() + agl);
}

int MainWindow::zToK(double z) const {
    const auto& g = sim_.grid();
    if (g.Nz <= 1) return 0;
    const double z0 = g.z0;
    const double dz = g.dz;
    const int k = (int)std::round((z - z0) / dz);
    return std::clamp(k, 0, g.Nz - 1);
}

void MainWindow::onFollowSliceToggled(bool on) {
    if (on) syncSliceWithSourceIfNeeded();
}

void MainWindow::syncSliceWithSourceIfNeeded() {
    if (!cbFollowSlice_->isChecked()) return;
    sbZSlice_->blockSignals(true);
    sbZSlice_->setValue(sbSrcZ_->value());
    sbZSlice_->blockSignals(false);
}

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    setWindowTitle("GasDispersionDesktop - Scrollable UI + Terrain Preview + Overlay");
    resize(1400, 800);

    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* root = new QVBoxLayout(central);

    auto* splitter = new QSplitter(Qt::Horizontal, central);
    root->addWidget(splitter, 1);

    leftScroll_ = new QScrollArea(splitter);
    leftScroll_->setWidgetResizable(true);

    auto* leftInner = new QWidget();
    leftScroll_->setWidget(leftInner);

    auto* leftLayout = new QVBoxLayout(leftInner);
    leftLayout->setSpacing(8);

    auto* right = new QWidget(splitter);
    auto* rightLayout = new QVBoxLayout(right);
    rightLayout->setContentsMargins(6, 6, 6, 6);

    view_ = new QLabel();
    view_->setMinimumSize(640, 520);
    view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    view_->setAlignment(Qt::AlignCenter);
    view_->setText("No terrain yet. Click Preview.");

    status_ = new QLabel();
    status_->setText("Ready.");

    rightLayout->addWidget(view_, 1);
    rightLayout->addWidget(status_, 0);

    splitter->setStretchFactor(0, 0);
    splitter->setStretchFactor(1, 1);

    leftScroll_->setMinimumWidth(360);
    leftInner->setMinimumWidth(340);
    splitter->setCollapsible(0, false);
    splitter->setSizes(QList<int>{400, 1000});
    splitter->setHandleWidth(8);

    auto* gbTerrain = new QGroupBox("Terrain", leftInner);
    auto* terrainForm = new QFormLayout(gbTerrain);

    cbTerrainMode_ = new QComboBox(gbTerrain);
    cbTerrainMode_->addItem("Flat");
    cbTerrainMode_->addItem("DEM (GeoTIFF)");
    cbTerrainMode_->addItem("Procedural");
    terrainForm->addRow("Mode", cbTerrainMode_);

    leDemTif_ = new QLineEdit(gbTerrain);
    btnPickDem_ = new QPushButton("Pick DEM .tif", gbTerrain);
    btnConvertLoadDem_ = new QPushButton("Convert+Load DEM", gbTerrain);
    sbDemStride_ = makeSsb(1, 50, 1, 2);
    demInfo_ = new QLabel("DEM: (none)", gbTerrain);

    terrainForm->addRow("DEM path", leDemTif_);
    terrainForm->addRow(btnPickDem_);
    terrainForm->addRow("Stride (downsample)", sbDemStride_);
    terrainForm->addRow(btnConvertLoadDem_);
    terrainForm->addRow(demInfo_);

    sbFlatZ_ = makeDsb(-1000, 10000, 0.5, 0.0, 2);
    terrainForm->addRow("Flat z", sbFlatZ_);

    sbProcBaseZ_ = makeDsb(-1000, 10000, 0.5, 0.0, 2);
    sbProcPeakA_ = makeDsb(0, 1000, 1.0, 100.0, 2);
    sbProcSigma_ = makeDsb(1, 100000, 1.0, 200.0, 2);
    terrainForm->addRow("Proc base z", sbProcBaseZ_);
    terrainForm->addRow("Proc peak A", sbProcPeakA_);
    terrainForm->addRow("Proc sigma", sbProcSigma_);

    btnPreviewTerrain_ = new QPushButton("Preview Terrain", gbTerrain);
    terrainForm->addRow(btnPreviewTerrain_);

    leftLayout->addWidget(gbTerrain);

    auto* gbDomain = new QGroupBox("Domain (Cartesian)", leftInner);
    auto* domForm = new QFormLayout(gbDomain);

    sbX0_ = makeDsb(-1e6, 1e6, 1.0, 0.0, 2);
    sbY0_ = makeDsb(-1e6, 1e6, 1.0, 0.0, 2);
    sbLx_ = makeDsb(1.0, 1e6, 1.0, 200.0, 2);
    sbLy_ = makeDsb(1.0, 1e6, 1.0, 200.0, 2);
    sbDx_ = makeDsb(0.1, 1000.0, 0.1, 1.0, 2);
    sbDy_ = makeDsb(0.1, 1000.0, 0.1, 1.0, 2);
    domainInfo_ = new QLabel("Nx=? Ny=?", gbDomain);

    domForm->addRow("x0", sbX0_);
    domForm->addRow("y0", sbY0_);
    domForm->addRow("Lx", sbLx_);
    domForm->addRow("Ly", sbLy_);
    domForm->addRow("dx", sbDx_);
    domForm->addRow("dy", sbDy_);
    domForm->addRow(domainInfo_);

    leftLayout->addWidget(gbDomain);

    auto* gbZ = new QGroupBox("Vertical", leftInner);
    auto* zForm = new QFormLayout(gbZ);

    sbDz_ = makeDsb(0.1, 1000.0, 0.1, 1.0, 2);
    sbZTopMargin_ = makeDsb(0.0, 20000.0, 1.0, 50.0, 2);
    sbNzMax_ = makeSsb(2, 2000, 1, 120);

    sbZSlice_ = makeDsb(-1000.0, 20000.0, 0.5, 2.0, 2);
    cbFollowSlice_ = new QCheckBox("Follow slice Z = srcZ", gbZ);
    cbFollowSlice_->setChecked(true);

    zForm->addRow("dz", sbDz_);
    zForm->addRow("Z top margin", sbZTopMargin_);
    zForm->addRow("Nz max", sbNzMax_);
    zForm->addRow("Z slice", sbZSlice_);
    zForm->addRow(cbFollowSlice_);

    leftLayout->addWidget(gbZ);

    auto* gbPhys = new QGroupBox("Physics", leftInner);
    auto* physForm = new QFormLayout(gbPhys);

    sbWindSpeed_ = makeDsb(0.0, 100.0, 0.1, 2.0, 2);
    sbWindDir_   = makeDsb(0.0, 360.0, 1.0, 0.0, 1);
    sbK_         = makeDsb(0.0, 1000.0, 0.01, 2.0, 4);
    sbDecay_     = makeDsb(0.0, 10.0, 0.001, 0.0, 6);

    physForm->addRow("Wind speed (m/s)", sbWindSpeed_);
    physForm->addRow("Wind dir (deg)", sbWindDir_);
    physForm->addRow("K (m^2/s)", sbK_);
    physForm->addRow("Decay (1/s)", sbDecay_);

    leftLayout->addWidget(gbPhys);

    auto* gbTime = new QGroupBox("Time & Output", leftInner);
    auto* timeForm = new QFormLayout(gbTime);

    sbTotalTime_ = makeDsb(1.0, 36000.0, 10.0, 60.0, 1);
    sbDt_        = makeDsb(1e-4, 10.0, 0.01, 0.05, 4);
    cbAutoClampDt_ = new QCheckBox("Auto clamp dt (stable)", gbTime);
    cbAutoClampDt_->setChecked(true);
    cbExportCsv_ = new QCheckBox("Export CSV frames", gbTime);
    cbExportCsv_->setChecked(true);
    sbExportInterval_ = makeDsb(0.01, 10.0, 0.05, 0.20, 2);

    cbExportTwoSlices_ = new QCheckBox("Export TWO slices per frame (zSlice + AGL)", gbTime);
    cbExportTwoSlices_->setChecked(true);

    timeForm->addRow("Total (s)", sbTotalTime_);
    timeForm->addRow("dt (s)", sbDt_);
    timeForm->addRow(cbAutoClampDt_);
    timeForm->addRow(cbExportCsv_);
    timeForm->addRow("Export interval (s)", sbExportInterval_);
    timeForm->addRow(cbExportTwoSlices_);

    leftLayout->addWidget(gbTime);

    auto* gbViz = new QGroupBox("Visualization", leftInner);
    auto* vizForm = new QFormLayout(gbViz);

    cbBackgroundMode_ = new QComboBox(gbViz);
    cbBackgroundMode_->addItem("Terrain grayscale + concentration overlay");
    cbBackgroundMode_->addItem("Blue background (concentration only)");
    cbBackgroundMode_->setCurrentIndex(1);
    vizForm->addRow("Background", cbBackgroundMode_);

    sbDisplayCutoffRel_ = makeDsb(0.0, 0.5, 1e-3, 1e-3, 6);
    vizForm->addRow("Cutoff (relative to sliceMax)", sbDisplayCutoffRel_);

    connect(cbBackgroundMode_, &QComboBox::currentIndexChanged, this, [this](int) {
        if (simReady_) renderTerrainAndSlice();
        else renderTerrainOnly();
    });
    connect(sbDisplayCutoffRel_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
        if (simReady_) renderTerrainAndSlice();
    });

    leftLayout->addWidget(gbViz);

    auto* gbSrc = new QGroupBox("3D Source", leftInner);
    auto* srcForm = new QFormLayout(gbSrc);

    sbSrcX_ = makeDsb(-1e12, 1e12, 1.0, 0.0, 2);
    sbSrcY_ = makeDsb(-1e12, 1e12, 1.0, 0.0, 2);
    sbSrcZ_ = makeDsb(-1e12, 1e12, 0.5, 2.0, 2);
    sbSrcRadius_ = makeDsb(0.01, 1000.0, 0.05, 1.0, 2);
    sbLeak_ = makeDsb(0.0, 1e9, 1.0, 100.0, 2);
    sbAgl_  = makeDsb(0.0, 20000.0, 0.5, 2.0, 2);

    btnSetSrcZFromGround_ = new QPushButton("Set srcZ = ground + AGL", gbSrc);

    cbAutoCenterSrc_ = new QCheckBox("Auto center source in non-DEM (recommended)", gbSrc);
    cbAutoCenterSrc_->setChecked(true);

    srcForm->addRow("srcX", sbSrcX_);
    srcForm->addRow("srcY", sbSrcY_);
    srcForm->addRow("srcZ", sbSrcZ_);
    srcForm->addRow("srcRadius", sbSrcRadius_);
    srcForm->addRow("Leak strength", sbLeak_);
    srcForm->addRow("AGL slice (m)", sbAgl_);
    srcForm->addRow(btnSetSrcZFromGround_);
    srcForm->addRow(cbAutoCenterSrc_);

    leftLayout->addWidget(gbSrc);

    auto* gbCtrl = new QGroupBox("Control", leftInner);
    auto* ctrlLayout = new QVBoxLayout(gbCtrl);

    btnRun_ = new QPushButton("Run", gbCtrl);
    btnPause_ = new QPushButton("Pause", gbCtrl);
    btnReset_ = new QPushButton("Reset", gbCtrl);

    ctrlLayout->addWidget(btnRun_);
    ctrlLayout->addWidget(btnPause_);
    ctrlLayout->addWidget(btnReset_);

    log_ = new QPlainTextEdit(gbCtrl);
    log_->setReadOnly(true);
    log_->setMaximumBlockCount(2000);
    ctrlLayout->addWidget(log_, 1);

    leftLayout->addWidget(gbCtrl);
    leftLayout->addStretch(1);

    connect(btnPickDem_, &QPushButton::clicked, this, &MainWindow::onPickDemClicked);
    connect(btnConvertLoadDem_, &QPushButton::clicked, this, &MainWindow::onConvertLoadDemClicked);
    connect(btnPreviewTerrain_, &QPushButton::clicked, this, &MainWindow::onPreviewTerrainClicked);

    connect(cbTerrainMode_, &QComboBox::currentIndexChanged, this, &MainWindow::onTerrainParamsChanged);

    connect(sbFlatZ_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbProcBaseZ_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbProcPeakA_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbProcSigma_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);

    connect(sbX0_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbY0_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbLx_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbLy_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbDx_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbDy_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);

    connect(btnSetSrcZFromGround_, &QPushButton::clicked, this, &MainWindow::onSetSrcZFromGroundClicked);

    connect(btnRun_, &QPushButton::clicked, this, &MainWindow::onRunClicked);
    connect(btnPause_, &QPushButton::clicked, this, &MainWindow::onPauseClicked);
    connect(btnReset_, &QPushButton::clicked, this, &MainWindow::onResetClicked);

    connect(cbFollowSlice_, &QCheckBox::toggled, this, &MainWindow::onFollowSliceToggled);

    timer_ = new QTimer(this);
    timer_->setInterval(15);
    connect(timer_, &QTimer::timeout, this, &MainWindow::onTick);

    updateDomainInfo();
    appendLog("[Init] ready. Click Preview Terrain first.");
}

void MainWindow::onPickDemClicked() {
    const QString f = QFileDialog::getOpenFileName(this, "Pick DEM GeoTIFF", QDir::currentPath(), "GeoTIFF (*.tif *.tiff)");
    if (!f.isEmpty()) leDemTif_->setText(f);
}

void MainWindow::onConvertLoadDemClicked() {
    const QString tifPath = leDemTif_->text().trimmed();
    if (tifPath.isEmpty()) { appendLog("[DEM] path empty."); return; }

    const QString outDir = QDir::current().filePath("outputs/dem_cache");
    QDir().mkpath(outDir);

    const QString cmd = QString("python tools/dem_convert.py --in \"%1\" --out_dir \"%2\"").arg(tifPath).arg(outDir);
    appendLog("[DEM] convert: " + cmd);

    const int code = std::system(cmd.toLocal8Bit().constData());
    if (code != 0) {
        appendLog("[DEM] convert failed. Ensure rasterio or gdal is installed (conda-forge).");
        return;
    }

    QString err;
    const QString metaPath = QDir(outDir).filePath("dem_meta.json");
    const QString binPath  = QDir(outDir).filePath("dem_data.bin");

    if (!dem_.load(metaPath, binPath, err)) {
        appendLog("[DEM] load failed: " + err);
        demInfo_->setText("DEM: load failed");
        hasDem_ = false;
        return;
    }

    hasDem_ = true;
    const auto& m = dem_.meta();
    demInfo_->setText(QString("DEM: ok | z=[%1,%2]")
                      .arg(m.z_min, 0, 'f', 2)
                      .arg(m.z_max, 0, 'f', 2));
    appendLog("[DEM] loaded ok.");
}

void MainWindow::onTerrainParamsChanged() {
    flat_.setZ0((float)sbFlatZ_->value());

    const double x0 = sbX0_->value();
    const double y0 = sbY0_->value();
    const double Lx = sbLx_->value();
    const double Ly = sbLy_->value();
    const double xc = x0 + 0.5 * Lx;
    const double yc = y0 + 0.5 * Ly;

    proc_.setMode(TerrainProcedural::Mode::GaussianHill);
    proc_.setBaseZ((float)sbProcBaseZ_->value());
    proc_.setGaussian(TerrainProcedural::Gaussian{(double)xc, (double)yc, sbProcPeakA_->value(), sbProcSigma_->value()});

    updateDomainInfo();
}

void MainWindow::onPreviewTerrainClicked() {
    QString err;
    if (!buildTerrainPreview(err)) {
        appendLog("[Terrain] preview failed: " + err);
        return;
    }
    enforceSourceCenterIfNeeded();
    renderTerrainOnly();
    appendLog("[Terrain] preview ok.");
}

bool MainWindow::buildTerrainPreview(QString& errOut) {
    errOut.clear();
    const ITerrain* terr = currentTerrain();
    if (!terr) { errOut = "Terrain is null (DEM not loaded?)"; return false; }
    if (!terr->isValid()) { errOut = "Terrain invalid"; return false; }

    if (terrainMode() == TerrainMode::Dem) {
        if (!hasDem_) { errOut = "DEM mode selected but DEM not loaded"; return false; }

        const auto& m = dem_.meta();
        const int stride = std::max(1, sbDemStride_->value());

        tNx_ = std::max(2, m.width / stride);
        tNy_ = std::max(2, m.height / stride);

        tdx_ = std::abs(m.dx) * stride;
        tdy_ = std::abs(m.dy) * stride;

        tx0_ = std::min(dem_.minX(), dem_.maxX());
        ty0_ = std::min(dem_.minY(), dem_.maxY());
    } else {
        const double Lx = sbLx_->value();
        const double Ly = sbLy_->value();
        tdx_ = std::max(1e-9, sbDx_->value());
        tdy_ = std::max(1e-9, sbDy_->value());
        tNx_ = std::max(2, (int)std::floor(Lx/tdx_) + 1);
        tNy_ = std::max(2, (int)std::floor(Ly/tdy_) + 1);
        tx0_ = sbX0_->value();
        ty0_ = sbY0_->value();
    }

    terrainXY_.assign((std::size_t)tNx_ * tNy_, 0.0f);
    tMin_ = std::numeric_limits<float>::infinity();
    tMax_ = -std::numeric_limits<float>::infinity();

    for (int j = 0; j < tNy_; ++j) {
        const double y = ty0_ + tdy_ * j;
        for (int i = 0; i < tNx_; ++i) {
            const double x = tx0_ + tdx_ * i;
            const float h = terr->height(x, y);
            terrainXY_[(std::size_t)i + (std::size_t)j * tNx_] = h;
            tMin_ = std::min(tMin_, h);
            tMax_ = std::max(tMax_, h);
        }
    }

    terrainPreviewReady_ = true;
    return true;
}

void MainWindow::renderTerrainOnly() {
    if (!terrainPreviewReady_) return;

    const int Nx = tNx_;
    const int Ny = tNy_;

    const int maxW = 950, maxH = 650;
    const double sx = (double)Nx / maxW;
    const double sy = (double)Ny / maxH;
    const double s = std::max(1.0, std::max(sx, sy));
    const int W = std::max(1, (int)std::round(Nx / s));
    const int H = std::max(1, (int)std::round(Ny / s));

    QImage img(W, H, QImage::Format_RGB32);

    const float denom = std::max(1e-6f, tMax_ - tMin_);

    const double lx = -1.0, ly = -1.0, lz = 1.0;
    const double ln = std::sqrt(lx*lx + ly*ly + lz*lz);
    const double Lx = lx/ln, Ly = ly/ln, Lz = lz/ln;

    auto Hxy = [&](int i, int j)->float {
        i = std::clamp(i, 0, Nx-1);
        j = std::clamp(j, 0, Ny-1);
        return terrainXY_[(std::size_t)i + (std::size_t)j * Nx];
    };

    for (int y = 0; y < H; ++y) {
        int j = std::clamp((int)std::round(y * s), 0, Ny - 1);
        for (int x = 0; x < W; ++x) {
            int i = std::clamp((int)std::round(x * s), 0, Nx - 1);

            const float h = Hxy(i,j);
            const float n = (h - tMin_) / denom;

            const float dzdx = (Hxy(i+1,j) - Hxy(i-1,j)) / (float)(2.0 * tdx_);
            const float dzdy = (Hxy(i,j+1) - Hxy(i,j-1)) / (float)(2.0 * tdy_);

            double nx = -dzdx, ny = -dzdy, nz = 1.0;
            const double nn = std::sqrt(nx*nx + ny*ny + nz*nz);
            nx /= nn; ny /= nn; nz /= nn;

            const double intensity = std::max(0.0, nx*Lx + ny*Ly + nz*Lz);
            const double shade = 0.45 + 0.55 * intensity;

            int g = (int)std::round((40.0 + 180.0 * n) * shade);
            g = std::clamp(g, 0, 255);

            img.setPixelColor(x, H - 1 - y, QColor(g, g, g));
        }
    }

    static constexpr double kPad = 0.35;
    const QImage canvas = applyFixedPadding(img, kPad, QColor(0,0,0));
    view_->setPixmap(QPixmap::fromImage(canvas).scaled(view_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    status_->setText(QString("TERRAIN | mode=%1 | z=[%2,%3] | Nx=%4 Ny=%5")
                     .arg(cbTerrainMode_->currentText())
                     .arg(tMin_, 0, 'f', 2)
                     .arg(tMax_, 0, 'f', 2)
                     .arg(Nx).arg(Ny));
}

Simulator3D::Params MainWindow::readSimParams() const {
    Simulator3D::Params p;
    p.totalTime_s = sbTotalTime_->value();
    p.dt_s = sbDt_->value();
    p.autoClampDt = cbAutoClampDt_->isChecked();

    p.windSpeed_mps = sbWindSpeed_->value();
    p.windDir_deg = sbWindDir_->value();

    p.K_m2ps = sbK_->value();
    p.decay_1ps = sbDecay_->value();

    p.srcX_m = sbSrcX_->value();
    p.srcY_m = sbSrcY_->value();
    p.srcZ_m = sbSrcZ_->value();
    p.srcRadius_m = sbSrcRadius_->value();
    p.leakRate = sbLeak_->value();

    return p;
}

bool MainWindow::buildSimulation(QString& errOut) {
    errOut.clear();
    const ITerrain* terr = currentTerrain();
    if (!terr) { errOut = "Terrain null (DEM not loaded?)"; return false; }
    if (!terr->isValid()) { errOut = "Terrain invalid"; return false; }

    if (!terrainPreviewReady_) {
        if (!buildTerrainPreview(errOut)) return false;
    }

    Grid3D g;
    g.Nx = tNx_;
    g.Ny = tNy_;
    g.x0 = tx0_;
    g.y0 = ty0_;
    g.dx = tdx_;
    g.dy = tdy_;

    const double dz = sbDz_->value();
    const double z0 = tMin_;
    const double zTop = (double)tMax_ + sbZTopMargin_->value();

    g.z0 = z0;
    g.dz = dz;

    int Nz = (int)std::floor((zTop - g.z0) / g.dz) + 1;
    Nz = std::clamp(Nz, 2, sbNzMax_->value());
    g.Nz = Nz;

    auto p = readSimParams();
    const double zMin = clampSliceToFluid(p.srcZ_m);
    if (p.srcZ_m < zMin) {
        appendLog(QString("[S1] srcZ clamped from %1 to %2 (avoid solid layer)")
                  .arg(p.srcZ_m, 0, 'f', 2).arg(zMin, 0, 'f', 2));
        p.srcZ_m = zMin;
        sbSrcZ_->setValue(zMin);
        syncSliceWithSourceIfNeeded();
    }

    if (!sim_.initialize(g, terr, p, errOut)) return false;

    simReady_ = true;
    running_ = false;
    timer_->stop();
    nextExportT_ = 0.0;
    frameId_ = 0;

    appendLog(QString("[SIM] grid Nx=%1 Ny=%2 Nz=%3 | dx=%4 dy=%5 dz=%6")
              .arg(g.Nx).arg(g.Ny).arg(g.Nz).arg(g.dx).arg(g.dy).arg(g.dz));

    renderTerrainAndSlice();
    return true;
}

float MainWindow::groundAtSource() const {
    const ITerrain* terr = currentTerrain();
    if (!terr || !terr->isValid()) return 0.0f;
    const double x = sbSrcX_->value();
    const double y = sbSrcY_->value();
    return terr->height(x, y);
}

void MainWindow::renderTerrainAndSlice() {
    if (!simReady_ || !terrainPreviewReady_) { renderTerrainOnly(); return; }

    const auto& g = sim_.grid();

    const double zEff = effectiveZSlice();
    const int k = zToK(zEff);

    sim_.extractSliceXY(k, slice_, sliceMax_);
    const int Nx = g.Nx;
    const int Ny = g.Ny;

    const int maxW = 950, maxH = 650;
    const double sx = (double)Nx / maxW;
    const double sy = (double)Ny / maxH;
    const double s = std::max(1.0, std::max(sx, sy));
    const int W = std::max(1, (int)std::round(Nx / s));
    const int H = std::max(1, (int)std::round(Ny / s));

    QImage img(W, H, QImage::Format_RGB32);

    const float denomH = std::max(1e-6f, tMax_ - tMin_);
    const float maxC = std::max(1e-12f, sliceMax_);
    const float relCut = sbDisplayCutoffRel_ ? (float)sbDisplayCutoffRel_->value() : 0.0f;
    const float cCut = relCut * maxC;

    const double lx = -1.0, ly = -1.0, lz = 1.0;
    const double ln = std::sqrt(lx*lx + ly*ly + lz*lz);
    const double Lx = lx/ln, Ly = ly/ln, Lz = lz/ln;

    auto Hxy = [&](int i, int j)->float {
        i = std::clamp(i, 0, Nx-1);
        j = std::clamp(j, 0, Ny-1);
        return terrainXY_[(std::size_t)i + (std::size_t)j * Nx];
    };

    const int bgMode = cbBackgroundMode_ ? cbBackgroundMode_->currentIndex() : 0;
    const QColor kBlueBg(20, 35, 80);

    for (int y = 0; y < H; ++y) {
        int j = std::clamp((int)std::round(y * s), 0, Ny - 1);
        for (int x = 0; x < W; ++x) {
            int i = std::clamp((int)std::round(x * s), 0, Nx - 1);

            double r = 0.0, gch = 0.0, b = 0.0;

            if (bgMode == 0) {
                const float h = Hxy(i,j);
                const float hn = (h - tMin_) / denomH;

                const float dzdx = (Hxy(i+1,j) - Hxy(i-1,j)) / (float)(2.0 * g.dx);
                const float dzdy = (Hxy(i,j+1) - Hxy(i,j-1)) / (float)(2.0 * g.dy);

                double nx = -dzdx, ny = -dzdy, nz = 1.0;
                const double nn = std::sqrt(nx*nx + ny*ny + nz*nz);
                nx/=nn; ny/=nn; nz/=nn;
                const double intensity = std::max(0.0, nx*Lx + ny*Ly + nz*Lz);
                const double shade = 0.45 + 0.55 * intensity;

                int bg = (int)std::round((40.0 + 180.0 * hn) * shade);
                bg = std::clamp(bg, 0, 255);

                r = bg; gch = bg; b = bg;
            } else {
                r = kBlueBg.red();
                gch = kBlueBg.green();
                b = kBlueBg.blue();
            }

            const float c = slice_[(std::size_t)i + (std::size_t)j * Nx];
            if (c > cCut) {
                const float cn = std::max(0.0f, std::min(1.0f, c / maxC));
                QColor cc = colorMap(cn);
                const double a = 0.15 + 0.75 * std::sqrt(cn);
                r = (1.0 - a) * r + a * cc.red();
                gch = (1.0 - a) * gch + a * cc.green();
                b = (1.0 - a) * b + a * cc.blue();
            }

            img.setPixelColor(x, H - 1 - y, QColor((int)r, (int)gch, (int)b));
        }
    }

    static constexpr double kPad = 0.35;
    const QColor padFill = (bgMode == 0) ? QColor(0,0,0) : kBlueBg;
    const QImage canvas = applyFixedPadding(img, kPad, padFill);
    view_->setPixmap(QPixmap::fromImage(canvas).scaled(view_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    const double gsrc = (double)groundAtSource();
    const double zAgl = aglSliceZ();

    status_->setText(QString("SIM | t=%1 s | zEff=%2 (k=%3/%4) | sliceMax=%5 | srcZ=%6 | ground=%7 | zAGL=%8")
                     .arg(sim_.time(), 0, 'f', 3)
                     .arg(zEff, 0, 'f', 2)
                     .arg(k).arg(g.Nz - 1)
                     .arg(sliceMax_, 0, 'g', 6)
                     .arg(sbSrcZ_->value(), 0, 'f', 2)
                     .arg(gsrc, 0, 'f', 2)
                     .arg(zAgl, 0, 'f', 2));
}

void MainWindow::onSetSrcZFromGroundClicked() {
    const ITerrain* terr = currentTerrain();
    if (!terr || !terr->isValid()) { appendLog("[SRC] terrain invalid."); return; }
    const double x = sbSrcX_->value();
    const double y = sbSrcY_->value();
    const float gz = terr->height(x, y);
    const double agl = sbAgl_->value();
    const double newZ = clampSliceToFluid(gz + agl);
    sbSrcZ_->setValue(newZ);
    syncSliceWithSourceIfNeeded();
    appendLog(QString("[SRC] srcZ = clamp(ground(%.2f)+AGL(%.2f)) => %.2f")
              .arg(gz,0,'f',2).arg(agl,0,'f',2).arg(newZ,0,'f',2));
}

void MainWindow::onRunClicked() {
    syncSliceWithSourceIfNeeded();

    if (!simReady_) {
        QString err;
        if (!buildSimulation(err)) {
            appendLog("[Run] buildSimulation failed: " + err);
            return;
        }
    }
    running_ = true;
    timer_->start();
    appendLog("[Run] start.");
}

void MainWindow::onPauseClicked() {
    running_ = false;
    timer_->stop();
    appendLog("[Pause] stopped.");
}

void MainWindow::onResetClicked() {
    running_ = false;
    timer_->stop();
    if (simReady_) sim_.reset();
    simReady_ = false;
    appendLog("[Reset] done.");
    renderTerrainOnly();
}

void MainWindow::onTick() {
    if (!running_ || !simReady_) return;

    sim_.step();

    const auto p = sim_.params();
    if (sim_.time() >= p.totalTime_s - 1e-12) {
        running_ = false;
        timer_->stop();
        appendLog("[Run] finished.");
        return;
    }

    if (cbExportCsv_->isChecked()) {
        const double t = sim_.time();
        if (t >= nextExportT_ - 1e-12) {
            QDir().mkpath(framesDir());

            const auto& g = sim_.grid();

            auto writeSlice = [&](const QString& suffix, double zWorld) {
                int k = zToK(zWorld);
                std::vector<float> grid2d;
                float mx = 0.0f;
                sim_.extractSliceXY(k, grid2d, mx);

                ExporterCsv::FrameMeta meta;
                meta.epsg = (terrainMode() == TerrainMode::Dem && hasDem_) ? dem_.meta().epsg : 0;
                meta.origin_x = g.x0;
                meta.origin_y = g.y0;
                meta.dx = g.dx;
                meta.dy = g.dy;
                meta.z  = g.z(k);
                meta.Nx = g.Nx;
                meta.Ny = g.Ny;
                meta.t  = t;

                const QString f = QDir(framesDir()).filePath(
                    QString("frame_%1_%2.csv").arg(frameId_, 4, 10, QLatin1Char('0')).arg(suffix)
                );
                QString err;
                if (!ExporterCsv::writeGridFrame(f, meta, grid2d, err)) {
                    appendLog("[CSV] write failed: " + err);
                } else {
                    appendLog("[CSV] wrote: " + f);
                }
            };

            const double zEff = effectiveZSlice();
            writeSlice("zslice", zEff);

            if (cbExportTwoSlices_->isChecked()) {
                const double zAgl = aglSliceZ();
                writeSlice("agl", zAgl);
            }

            frameId_++;
            nextExportT_ += sbExportInterval_->value();
        }
    }

    renderTerrainAndSlice();
}
