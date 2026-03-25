#include "MainWindow.h"
#include "ColorMap.h"
#include "ExporterCsv.h"

#include <QtWidgets/QWidget>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QSizePolicy>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QPainter>
#include <QTextStream>
#include <QUuid>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

static QDoubleSpinBox* makeDsb(double minV, double maxV, double step, double val, int decimals = 3)
{
    auto* sb = new QDoubleSpinBox();
    sb->setRange(minV, maxV);
    sb->setDecimals(decimals);
    sb->setSingleStep(step);
    sb->setValue(val);
    return sb;
}

static QSpinBox* makeSsb(int minV, int maxV, int step, int val)
{
    auto* sb = new QSpinBox();
    sb->setRange(minV, maxV);
    sb->setSingleStep(step);
    sb->setValue(val);
    return sb;
}

static QImage applyFixedPadding(const QImage& img, double padFrac, const QColor& fill)
{
    const int W = img.width();
    const int H = img.height();
    const int padX = static_cast<int>(std::round(W * padFrac));
    const int padY = static_cast<int>(std::round(H * padFrac));
    const int CW = W + 2 * padX;
    const int CH = H + 2 * padY;

    QImage canvas(CW, CH, QImage::Format_RGB32);
    canvas.fill(fill);

    QPainter p(&canvas);
    p.drawImage(padX, padY, img);
    return canvas;
}

MainWindow::TerrainMode MainWindow::terrainMode() const
{
    return static_cast<TerrainMode>(cbTerrainMode_->currentIndex());
}

const ITerrain* MainWindow::currentTerrain() const
{
    const auto mode = terrainMode();
    if (mode == TerrainMode::Dem) {
        if (hasDem_ && dem_.isValid()) return &dem_;
        return nullptr;
    }
    if (mode == TerrainMode::Procedural) return &proc_;
    return &flat_;
}

QString MainWindow::framesDir() const
{
    return QDir::current().filePath("outputs/frames");
}

QString MainWindow::logsDir() const
{
    return QDir::current().filePath("outputs/log");
}

QString MainWindow::createExperimentId() const
{
    const QString timePart = QDateTime::currentDateTime().toString("yyyyMMdd-HHmmsszzz");
    const QString uuidPart = QUuid::createUuid().toString(QUuid::WithoutBraces).left(8);
    return QString("EXP-%1-%2").arg(timePart, uuidPart);
}

void MainWindow::appendExperimentLogLine(const QString& s)
{
    if (!currentExperimentActive_ || currentExperimentLogPath_.isEmpty()) {
        return;
    }

    QFile f(currentExperimentLogPath_);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        return;
    }

    QTextStream ts(&f);
    const QString t = QDateTime::currentDateTime().toString("HH:mm:ss");
    ts << "- [" << t << "] " << s << "\n";
}

void MainWindow::startExperimentLogSession()
{
    QDir().mkpath(logsDir());

    const QDateTime now = QDateTime::currentDateTime();
    currentExperimentId_ = createExperimentId();
    currentExperimentStartMinute_ = now.toString("yyyy-MM-dd HH:mm");
    currentExperimentLogPath_ = QDir(logsDir()).filePath(currentExperimentId_ + ".md");
    currentExperimentActive_ = true;

    QFile f(currentExperimentLogPath_);
    if (f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        QTextStream ts(&f);
        ts << "# GasDispersionDesktop 实验日志\n\n";
        ts << "- 实验编号：`" << currentExperimentId_ << "`\n";
        ts << "- 开始时间：`" << currentExperimentStartMinute_ << "`\n";
        ts << "- 计算模型：`" << cbModel_->currentText() << "`\n";
        ts << "- 地形模式：`" << cbTerrainMode_->currentText() << "`\n";
        if (terrainMode() == TerrainMode::Procedural && cbProcShape_) {
            ts << "- 程序地形形状：`" << cbProcShape_->currentText() << "`\n";
        }
        ts << "- 日志文件：`" << currentExperimentLogPath_ << "`\n\n";
        ts << "## 事件记录\n\n";
    }

    appendExperimentLogLine("实验日志会话已创建。");
}

void MainWindow::closeExperimentLogSession()
{
    currentExperimentActive_ = false;
    currentExperimentId_.clear();
    currentExperimentStartMinute_.clear();
    currentExperimentLogPath_.clear();
}

void MainWindow::appendLog(const QString& s)
{
    log_->appendPlainText(s);
    appendExperimentLogLine(s);
}

void MainWindow::updateDomainInfo()
{
    const double Lx = sbLx_->value();
    const double Ly = sbLy_->value();
    const double dx = std::max(1e-9, sbDx_->value());
    const double dy = std::max(1e-9, sbDy_->value());
    const int Nx = std::max(2, static_cast<int>(std::floor(Lx / dx)) + 1);
    const int Ny = std::max(2, static_cast<int>(std::floor(Ly / dy)) + 1);
    domainInfo_->setText(QString("Nx=%1 Ny=%2 (non-DEM)").arg(Nx).arg(Ny));
}

void MainWindow::updateProcShapeUi()
{
    const bool procSelected = (terrainMode() == TerrainMode::Procedural);
    if (cbProcShape_) {
        cbProcShape_->setEnabled(procSelected);
    }

    const int shape = cbProcShape_ ? cbProcShape_->currentIndex() : 0;
    if (gbProcGaussian_) {
        gbProcGaussian_->setVisible(procSelected && shape == 0);
    }
    if (gbProcSombrero_) {
        gbProcSombrero_->setVisible(procSelected && shape == 1);
    }
    if (gbProcSphereCap_) {
        gbProcSphereCap_->setVisible(procSelected && shape == 2);
    }
}

void MainWindow::enforceSourceCenterIfNeeded()
{
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

double MainWindow::clampSliceToFluid(double z) const
{
    if (!terrainPreviewReady_) return z;
    const double zMin = static_cast<double>(tMin_);
    const double zTop = static_cast<double>(tMax_) + sbZTopMargin_->value();
    return std::clamp(z, zMin, zTop);
}

double MainWindow::effectiveZSlice() const
{
    return clampSliceToFluid(sbZSlice_->value());
}

double MainWindow::aglSliceZ() const
{
    return clampSliceToFluid(static_cast<double>(groundAtSource()) + sbAgl_->value());
}

int MainWindow::zToK(double z) const
{
    const auto& g = engine_.grid();
    if (g.Nz <= 1) return 0;
    const int k = static_cast<int>(std::round((z - g.z0) / g.dz));
    return std::clamp(k, 0, g.Nz - 1);
}

void MainWindow::onFollowSliceToggled(bool on)
{
    if (on) syncSliceWithSourceIfNeeded();
}

void MainWindow::syncSliceWithSourceIfNeeded()
{
    if (!cbFollowSlice_->isChecked()) return;
    sbZSlice_->blockSignals(true);
    sbZSlice_->setValue(sbSrcZ_->value());
    sbZSlice_->blockSignals(false);
}

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
{
    setWindowTitle("GasDispersionDesktop");
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

    cbProcShape_ = new QComboBox(gbTerrain);
    cbProcShape_->addItem("Gaussian hill");
    cbProcShape_->addItem("Sombrero");
    cbProcShape_->addItem("Sphere cap");
    terrainForm->addRow("Procedural shape", cbProcShape_);

    gbProcGaussian_ = new QGroupBox("Gaussian hill parameters", gbTerrain);
    auto* procGaussianForm = new QFormLayout(gbProcGaussian_);
    sbProcBaseZ_ = makeDsb(-1000, 10000, 0.5, 0.0, 2);
    sbProcPeakA_ = makeDsb(0, 1000, 1.0, 100.0, 2);
    sbProcSigma_ = makeDsb(1, 100000, 1.0, 200.0, 2);
    procGaussianForm->addRow("Base z", sbProcBaseZ_);
    procGaussianForm->addRow("Peak A", sbProcPeakA_);
    procGaussianForm->addRow("Sigma", sbProcSigma_);
    terrainForm->addRow(gbProcGaussian_);

    gbProcSombrero_ = new QGroupBox("Sombrero parameters", gbTerrain);
    auto* procSombreroForm = new QFormLayout(gbProcSombrero_);
    sbProcSombreroBaseZ_ = makeDsb(-1000, 10000, 0.5, 0.0, 2);
    sbProcSombreroAmp_ = makeDsb(0, 1000, 1.0, 100.0, 2);
    sbProcSombreroRadius_ = makeDsb(1, 100000, 1.0, 200.0, 2);
    procSombreroForm->addRow("Base z", sbProcSombreroBaseZ_);
    procSombreroForm->addRow("Amplitude", sbProcSombreroAmp_);
    procSombreroForm->addRow("Radius", sbProcSombreroRadius_);
    terrainForm->addRow(gbProcSombrero_);

    gbProcSphereCap_ = new QGroupBox("Sphere cap parameters", gbTerrain);
    auto* procSphereCapForm = new QFormLayout(gbProcSphereCap_);
    sbCapBaseZ_ = makeDsb(-1000, 10000, 0.5, 0.0, 2);
    sbCapCenterX_ = makeDsb(-1e6, 1e6, 1.0, 0.0, 2);
    sbCapCenterY_ = makeDsb(-1e6, 1e6, 1.0, 0.0, 2);
    sbCapHeight_ = makeDsb(0.1, 100000, 1.0, 60.0, 2);
    sbCapRadius_ = makeDsb(0.1, 100000, 1.0, 80.0, 2);
    procSphereCapForm->addRow("Base z", sbCapBaseZ_);
    procSphereCapForm->addRow("Center X", sbCapCenterX_);
    procSphereCapForm->addRow("Center Y", sbCapCenterY_);
    procSphereCapForm->addRow("Cap height", sbCapHeight_);
    procSphereCapForm->addRow("Cap radius", sbCapRadius_);
    terrainForm->addRow(gbProcSphereCap_);

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

    sbCapCenterX_->setValue(sbX0_->value() + 0.5 * sbLx_->value());
    sbCapCenterY_->setValue(sbY0_->value() + 0.5 * sbLy_->value());

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

    cbModel_ = new QComboBox(gbPhys);
    cbModel_->addItem("Terrain-aware Eulerian (CFD)");
    cbModel_->addItem("Terrain-aware Gaussian plume");
    cbModel_->addItem("Terrain-aware Gaussian puff");

    sbWindSpeed_ = makeDsb(0.0, 100.0, 0.1, 2.0, 2);
    sbWindDir_   = makeDsb(0.0, 360.0, 1.0, 0.0, 1);
    sbK_         = makeDsb(0.0, 1000.0, 0.01, 2.0, 4);
    sbDecay_     = makeDsb(0.0, 10.0, 0.001, 0.0, 6);

    physForm->addRow("Model", cbModel_);
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

    cbExportColumn_ = new QCheckBox("Export column integral (vertical integral)", gbTime);
    cbExportColumn_->setChecked(true);

    timeForm->addRow("Total (s)", sbTotalTime_);
    timeForm->addRow("dt (s)", sbDt_);
    timeForm->addRow(cbAutoClampDt_);
    timeForm->addRow(cbExportCsv_);
    timeForm->addRow("Export interval (s)", sbExportInterval_);
    timeForm->addRow(cbExportTwoSlices_);
    timeForm->addRow(cbExportColumn_);

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

    connect(cbTerrainMode_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
        updateProcShapeUi();
        onTerrainParamsChanged();
    });

    connect(cbProcShape_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
        updateProcShapeUi();
        onTerrainParamsChanged();
    });

    connect(sbFlatZ_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbProcBaseZ_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbProcPeakA_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbProcSigma_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);

    connect(sbProcSombreroBaseZ_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbProcSombreroAmp_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbProcSombreroRadius_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);

    connect(sbCapBaseZ_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbCapCenterX_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbCapCenterY_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbCapHeight_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbCapRadius_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);

    connect(sbX0_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbY0_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbLx_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbLy_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbDx_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);
    connect(sbDy_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, &MainWindow::onTerrainParamsChanged);

    connect(cbModel_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
        running_ = false;
        timer_->stop();
        simReady_ = false;
        closeExperimentLogSession();
        if (terrainPreviewReady_) renderTerrainOnly();
        appendLog("[Model] changed. Rebuild on next Run.");
    });

    connect(cbBackgroundMode_, qOverload<int>(&QComboBox::currentIndexChanged), this, [this](int) {
        if (simReady_) renderTerrainAndSlice();
        else if (terrainPreviewReady_) renderTerrainOnly();
    });

    connect(sbDisplayCutoffRel_, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double) {
        if (simReady_) renderTerrainAndSlice();
    });

    connect(btnSetSrcZFromGround_, &QPushButton::clicked, this, &MainWindow::onSetSrcZFromGroundClicked);

    connect(btnRun_, &QPushButton::clicked, this, &MainWindow::onRunClicked);
    connect(btnPause_, &QPushButton::clicked, this, &MainWindow::onPauseClicked);
    connect(btnReset_, &QPushButton::clicked, this, &MainWindow::onResetClicked);

    connect(cbFollowSlice_, &QCheckBox::toggled, this, &MainWindow::onFollowSliceToggled);

    timer_ = new QTimer(this);
    timer_->setInterval(15);
    connect(timer_, &QTimer::timeout, this, &MainWindow::onTick);

    updateProcShapeUi();
    updateDomainInfo();
    appendLog("[Init] ready. Click Preview Terrain first.");
}

void MainWindow::onPickDemClicked()
{
    const QString f = QFileDialog::getOpenFileName(
        this,
        "Pick DEM GeoTIFF",
        QDir::currentPath(),
        "GeoTIFF (*.tif *.tiff)");
    if (!f.isEmpty()) {
        leDemTif_->setText(f);
    }
}

void MainWindow::onConvertLoadDemClicked()
{
    const QString tifPath = leDemTif_->text().trimmed();
    if (tifPath.isEmpty()) {
        appendLog("[DEM] path empty.");
        return;
    }

    const QString outDir = QDir::current().filePath("outputs/dem_cache");
    QDir().mkpath(outDir);

    const QString cmd = QString("python tools/dem_convert.py --in \"%1\" --out_dir \"%2\"")
                            .arg(tifPath)
                            .arg(outDir);
    appendLog("[DEM] convert: " + cmd);

    const int code = std::system(cmd.toLocal8Bit().constData());
    if (code != 0) {
        appendLog("[DEM] convert failed. Ensure rasterio or gdal is installed (conda-forge).");
        return;
    }

    const QString metaPath = QDir(outDir).filePath("dem_meta.json");
    const QString binPath  = QDir(outDir).filePath("dem_data.bin");

    const bool ok = dem_.load(metaPath.toStdString(), binPath.toStdString());
    if (!ok) {
        appendLog("[DEM] load failed.");
        demInfo_->setText("DEM: load failed");
        hasDem_ = false;
        return;
    }

    hasDem_ = true;
    demInfo_->setText(
        QString("DEM: ok | Nx=%1 Ny=%2 | cell=%3 | zmin=%4")
            .arg(dem_.width())
            .arg(dem_.height())
            .arg(dem_.cellSize(), 0, 'f', 3)
            .arg(dem_.minElevation(), 0, 'f', 2));
    appendLog("[DEM] loaded ok.");

    onTerrainParamsChanged();
}

void MainWindow::onTerrainParamsChanged()
{
    flat_.z = static_cast<float>(sbFlatZ_->value());
    updateProcShapeUi();

    const double x0 = sbX0_->value();
    const double y0 = sbY0_->value();
    const double Lx = sbLx_->value();
    const double Ly = sbLy_->value();
    const double xc = x0 + 0.5 * Lx;
    const double yc = y0 + 0.5 * Ly;

    const int procShape = cbProcShape_ ? cbProcShape_->currentIndex() : 0;
    if (procShape == 0) {
        proc_.setMode(TerrainProcedural::Mode::GaussianHill);
        proc_.setBaseZ(static_cast<float>(sbProcBaseZ_->value()));
        proc_.setGaussian(TerrainProcedural::Gaussian{xc, yc, sbProcPeakA_->value(), sbProcSigma_->value()});
    } else if (procShape == 1) {
        proc_.setSombrero(
            sbProcSombreroBaseZ_->value(),
            sbProcSombreroAmp_->value(),
            sbProcSombreroRadius_->value(),
            xc,
            yc);
    } else {
        proc_.setSphereCap(
            sbCapBaseZ_->value(),
            sbCapCenterX_->value(),
            sbCapCenterY_->value(),
            sbCapHeight_->value(),
            sbCapRadius_->value());
    }

    running_ = false;
    timer_->stop();
    simReady_ = false;
    terrainPreviewReady_ = false;
    closeExperimentLogSession();

    updateDomainInfo();
}

void MainWindow::onPreviewTerrainClicked()
{
    QString err;
    if (!buildTerrainPreview(err)) {
        appendLog("[Terrain] preview failed: " + err);
        return;
    }

    enforceSourceCenterIfNeeded();
    renderTerrainOnly();
    appendLog("[Terrain] preview ok.");
}

bool MainWindow::buildTerrainPreview(QString& errOut)
{
    errOut.clear();
    const ITerrain* terr = currentTerrain();
    if (!terr) {
        errOut = "Terrain is null (DEM not loaded?)";
        return false;
    }
    if (!terr->isValid()) {
        errOut = "Terrain invalid";
        return false;
    }

    if (terrainMode() == TerrainMode::Dem) {
        if (!hasDem_) {
            errOut = "DEM mode selected but DEM not loaded";
            return false;
        }

        const int stride = std::max(1, sbDemStride_->value());

        tNx_ = std::max(2, (dem_.width() - 1) / stride + 1);
        tNy_ = std::max(2, (dem_.height() - 1) / stride + 1);

        tdx_ = std::abs(dem_.dx()) * stride;
        tdy_ = std::abs(dem_.dy()) * stride;

        tx0_ = std::min(dem_.xMin(), dem_.xMax());
        ty0_ = std::min(dem_.yMin(), dem_.yMax());
    } else {
        const double Lx = sbLx_->value();
        const double Ly = sbLy_->value();
        tdx_ = std::max(1e-9, sbDx_->value());
        tdy_ = std::max(1e-9, sbDy_->value());
        tNx_ = std::max(2, static_cast<int>(std::floor(Lx / tdx_)) + 1);
        tNy_ = std::max(2, static_cast<int>(std::floor(Ly / tdy_)) + 1);
        tx0_ = sbX0_->value();
        ty0_ = sbY0_->value();
    }

    terrainXY_.assign(static_cast<std::size_t>(tNx_) * static_cast<std::size_t>(tNy_), 0.0f);
    tMin_ = std::numeric_limits<float>::infinity();
    tMax_ = -std::numeric_limits<float>::infinity();

    for (int j = 0; j < tNy_; ++j) {
        const double y = ty0_ + tdy_ * j;
        for (int i = 0; i < tNx_; ++i) {
            const double x = tx0_ + tdx_ * i;
            const float h = terr->height(x, y);
            terrainXY_[static_cast<std::size_t>(i) + static_cast<std::size_t>(j) * static_cast<std::size_t>(tNx_)] = h;
            tMin_ = std::min(tMin_, h);
            tMax_ = std::max(tMax_, h);
        }
    }

    terrainPreviewReady_ = true;
    return true;
}

void MainWindow::renderTerrainOnly()
{
    if (!terrainPreviewReady_) return;

    const int Nx = tNx_;
    const int Ny = tNy_;

    const int maxW = 950;
    const int maxH = 650;
    const double sx = static_cast<double>(Nx) / maxW;
    const double sy = static_cast<double>(Ny) / maxH;
    const double s = std::max(1.0, std::max(sx, sy));
    const int W = std::max(1, static_cast<int>(std::round(Nx / s)));
    const int H = std::max(1, static_cast<int>(std::round(Ny / s)));

    QImage img(W, H, QImage::Format_RGB32);

    const float denom = std::max(1e-6f, tMax_ - tMin_);

    const double lx = -1.0, ly = -1.0, lz = 1.0;
    const double ln = std::sqrt(lx * lx + ly * ly + lz * lz);
    const double Lx = lx / ln, Ly = ly / ln, Lz = lz / ln;

    auto Hxy = [&](int i, int j) -> float {
        i = std::clamp(i, 0, Nx - 1);
        j = std::clamp(j, 0, Ny - 1);
        return terrainXY_[static_cast<std::size_t>(i) + static_cast<std::size_t>(j) * static_cast<std::size_t>(Nx)];
    };

    for (int y = 0; y < H; ++y) {
        const int j = std::clamp(static_cast<int>(std::round(y * s)), 0, Ny - 1);
        for (int x = 0; x < W; ++x) {
            const int i = std::clamp(static_cast<int>(std::round(x * s)), 0, Nx - 1);

            const float h = Hxy(i, j);
            const float n = (h - tMin_) / denom;

            const float dzdx = (Hxy(i + 1, j) - Hxy(i - 1, j)) / static_cast<float>(2.0 * tdx_);
            const float dzdy = (Hxy(i, j + 1) - Hxy(i, j - 1)) / static_cast<float>(2.0 * tdy_);

            double nx = -dzdx, ny = -dzdy, nz = 1.0;
            const double nn = std::sqrt(nx * nx + ny * ny + nz * nz);
            nx /= nn;
            ny /= nn;
            nz /= nn;

            const double intensity = std::max(0.0, nx * Lx + ny * Ly + nz * Lz);
            const double shade = 0.45 + 0.55 * intensity;

            int g = static_cast<int>(std::round((40.0 + 180.0 * n) * shade));
            g = std::clamp(g, 0, 255);

            img.setPixelColor(x, H - 1 - y, QColor(g, g, g));
        }
    }

    static constexpr double kPad = 0.35;
    const QImage canvas = applyFixedPadding(img, kPad, QColor(0, 0, 0));
    view_->setPixmap(QPixmap::fromImage(canvas).scaled(view_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    status_->setText(
        QString("TERRAIN | mode=%1 | z=[%2,%3] | Nx=%4 Ny=%5")
            .arg(cbTerrainMode_->currentText())
            .arg(tMin_, 0, 'f', 2)
            .arg(tMax_, 0, 'f', 2)
            .arg(Nx)
            .arg(Ny));
}

SimulationConfig MainWindow::readSimConfig() const
{
    SimulationConfig p;
    p.model = static_cast<SimulationModelType>(cbModel_->currentIndex());

    p.totalTime_s = sbTotalTime_->value();
    p.dt_s = sbDt_->value();
    p.autoClampDt = cbAutoClampDt_->isChecked();

    p.windSpeed_mps = sbWindSpeed_->value();
    p.windDir_deg   = sbWindDir_->value();
    p.K_m2ps        = sbK_->value();
    p.decay_1ps     = sbDecay_->value();

    p.srcX_m        = sbSrcX_->value();
    p.srcY_m        = sbSrcY_->value();
    p.srcZ_m        = sbSrcZ_->value();
    p.srcRadius_m   = sbSrcRadius_->value();
    p.leakRate_kgps = sbLeak_->value();

    p.releaseInterval_s = 0.0;

    p.noise.enabled = true;
    p.noise.seed = 42;
    p.noise.updateEvery_s = 1.0;
    p.noise.windSpeedSigma_mps = 0.30;
    p.noise.windDirSigma_deg = 7.0;
    p.noise.leakRelSigma = 0.10;

    return p;
}

bool MainWindow::buildSimulation(QString& errOut)
{
    errOut.clear();
    const ITerrain* terr = currentTerrain();
    if (!terr) {
        errOut = "Terrain null (DEM not loaded?)";
        return false;
    }
    if (!terr->isValid()) {
        errOut = "Terrain invalid";
        return false;
    }

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

    g.z0 = static_cast<double>(tMin_);
    g.dz = sbDz_->value();

    const double zTop = static_cast<double>(tMax_) + sbZTopMargin_->value();
    int Nz = static_cast<int>(std::floor((zTop - g.z0) / g.dz)) + 1;
    Nz = std::clamp(Nz, 2, sbNzMax_->value());
    g.Nz = Nz;

    auto cfg = readSimConfig();

    const double ground = terr->height(cfg.srcX_m, cfg.srcY_m);
    const double minSrcZ = std::max(g.z0, ground + 0.5 * g.dz);
    const double maxSrcZ = g.z0 + (g.Nz - 1) * g.dz;

    const double srcZOld = cfg.srcZ_m;
    cfg.srcZ_m = std::clamp(cfg.srcZ_m, minSrcZ, maxSrcZ);
    if (std::abs(cfg.srcZ_m - srcZOld) > 1e-9) {
        appendLog(QString("[SRC] srcZ clamped from %1 to %2")
                  .arg(srcZOld, 0, 'f', 2)
                  .arg(cfg.srcZ_m, 0, 'f', 2));
        sbSrcZ_->setValue(cfg.srcZ_m);
        syncSliceWithSourceIfNeeded();
    }

    engine_.setModel(cfg.model);

    std::string err;
    if (!engine_.initialize(g, terr, cfg, err)) {
        errOut = QString::fromStdString(err);
        return false;
    }

    simReady_ = true;
    running_ = false;
    timer_->stop();
    nextExportT_ = 0.0;
    frameId_ = 0;

    appendLog(QString("[SIM] model=%1 | grid Nx=%2 Ny=%3 Nz=%4 | dx=%5 dy=%6 dz=%7")
              .arg(QString::fromLatin1(simulationModelTypeName(cfg.model)))
              .arg(g.Nx).arg(g.Ny).arg(g.Nz)
              .arg(g.dx).arg(g.dy).arg(g.dz));

    renderTerrainAndSlice();
    return true;
}

float MainWindow::groundAtSource() const
{
    const ITerrain* terr = currentTerrain();
    if (!terr || !terr->isValid()) return 0.0f;
    return terr->height(sbSrcX_->value(), sbSrcY_->value());
}

void MainWindow::renderTerrainAndSlice()
{
    if (!simReady_ || !terrainPreviewReady_) {
        renderTerrainOnly();
        return;
    }

    const auto& g = engine_.grid();
    const double zEff = effectiveZSlice();
    const int k = zToK(zEff);

    engine_.extractSliceXY(k, slice_, sliceMax_);

    const int Nx = g.Nx;
    const int Ny = g.Ny;

    const int maxW = 950;
    const int maxH = 650;
    const double sx = static_cast<double>(Nx) / maxW;
    const double sy = static_cast<double>(Ny) / maxH;
    const double s = std::max(1.0, std::max(sx, sy));
    const int W = std::max(1, static_cast<int>(std::round(Nx / s)));
    const int H = std::max(1, static_cast<int>(std::round(Ny / s)));

    QImage img(W, H, QImage::Format_RGB32);

    const float denomH = std::max(1e-6f, tMax_ - tMin_);
    const float maxC = std::max(1e-12f, sliceMax_);
    const float relCut = sbDisplayCutoffRel_ ? static_cast<float>(sbDisplayCutoffRel_->value()) : 0.0f;
    const float cCut = relCut * maxC;

    const double lx = -1.0, ly = -1.0, lz = 1.0;
    const double ln = std::sqrt(lx * lx + ly * ly + lz * lz);
    const double Lx = lx / ln, Ly = ly / ln, Lz = lz / ln;

    auto Hxy = [&](int i, int j) -> float {
        i = std::clamp(i, 0, Nx - 1);
        j = std::clamp(j, 0, Ny - 1);
        return terrainXY_[static_cast<std::size_t>(i) + static_cast<std::size_t>(j) * static_cast<std::size_t>(Nx)];
    };

    const int bgMode = cbBackgroundMode_ ? cbBackgroundMode_->currentIndex() : 0;
    const QColor kBlueBg(20, 35, 80);

    for (int y = 0; y < H; ++y) {
        const int j = std::clamp(static_cast<int>(std::round(y * s)), 0, Ny - 1);
        for (int x = 0; x < W; ++x) {
            const int i = std::clamp(static_cast<int>(std::round(x * s)), 0, Nx - 1);

            double r = 0.0;
            double gch = 0.0;
            double b = 0.0;

            if (bgMode == 0) {
                const float h = Hxy(i, j);
                const float hn = (h - tMin_) / denomH;

                const float dzdx = (Hxy(i + 1, j) - Hxy(i - 1, j)) / static_cast<float>(2.0 * g.dx);
                const float dzdy = (Hxy(i, j + 1) - Hxy(i, j - 1)) / static_cast<float>(2.0 * g.dy);

                double nx = -dzdx;
                double ny = -dzdy;
                double nz = 1.0;
                const double nn = std::sqrt(nx * nx + ny * ny + nz * nz);
                nx /= nn;
                ny /= nn;
                nz /= nn;

                const double intensity = std::max(0.0, nx * Lx + ny * Ly + nz * Lz);
                const double shade = 0.45 + 0.55 * intensity;

                int bg = static_cast<int>(std::round((40.0 + 180.0 * hn) * shade));
                bg = std::clamp(bg, 0, 255);

                r = bg;
                gch = bg;
                b = bg;
            } else {
                r = kBlueBg.red();
                gch = kBlueBg.green();
                b = kBlueBg.blue();
            }

            const float c = slice_[static_cast<std::size_t>(i) + static_cast<std::size_t>(j) * static_cast<std::size_t>(Nx)];
            if (c > cCut) {
                const float cn = std::clamp(c / maxC, 0.0f, 1.0f);
                const QColor cc = colorMap(cn);
                const double a = 0.15 + 0.75 * std::sqrt(cn);
                r = (1.0 - a) * r + a * cc.red();
                gch = (1.0 - a) * gch + a * cc.green();
                b = (1.0 - a) * b + a * cc.blue();
            }

            img.setPixelColor(x, H - 1 - y, QColor(static_cast<int>(r), static_cast<int>(gch), static_cast<int>(b)));
        }
    }

    static constexpr double kPad = 0.35;
    const QColor padFill = (bgMode == 0) ? QColor(0, 0, 0) : kBlueBg;
    const QImage canvas = applyFixedPadding(img, kPad, padFill);
    view_->setPixmap(QPixmap::fromImage(canvas).scaled(view_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));

    const auto cfg = engine_.config();
    const QString modelName = QString::fromLatin1(simulationModelTypeName(cfg.model));

    const double gsrc = static_cast<double>(groundAtSource());
    const double zAgl = aglSliceZ();

    status_->setText(
        QString("SIM | model=%1 | t=%2 s | wind=%3 m/s @ %4 deg | leak=%5 | zEff=%6 (k=%7/%8) | sliceMax=%9 | srcZ=%10 | ground=%11 | zAGL=%12 | solve(last/avg)= %13 / %14 ms")
            .arg(modelName)
            .arg(engine_.time(), 0, 'f', 3)
            .arg(engine_.currentWindSpeed(), 0, 'f', 3)
            .arg(engine_.currentWindDirDeg(), 0, 'f', 2)
            .arg(engine_.currentLeakRate(), 0, 'g', 6)
            .arg(zEff, 0, 'f', 2)
            .arg(k).arg(g.Nz - 1)
            .arg(sliceMax_, 0, 'g', 6)
            .arg(sbSrcZ_->value(), 0, 'f', 2)
            .arg(gsrc, 0, 'f', 2)
            .arg(zAgl, 0, 'f', 2)
            .arg(engine_.lastSolveMs(), 0, 'f', 3)
            .arg(engine_.averageSolveMs(), 0, 'f', 3));
}

void MainWindow::onSetSrcZFromGroundClicked()
{
    const ITerrain* terr = currentTerrain();
    if (!terr || !terr->isValid()) {
        appendLog("[SRC] terrain invalid.");
        return;
    }

    const float gz = terr->height(sbSrcX_->value(), sbSrcY_->value());
    const double agl = sbAgl_->value();
    const double newZ = clampSliceToFluid(gz + agl);
    sbSrcZ_->setValue(newZ);
    syncSliceWithSourceIfNeeded();

    appendLog(QString("[SRC] srcZ = clamp(ground(%1)+AGL(%2)) => %3")
              .arg(gz, 0, 'f', 2)
              .arg(agl, 0, 'f', 2)
              .arg(newZ, 0, 'f', 2));
}

void MainWindow::onRunClicked()
{
    syncSliceWithSourceIfNeeded();

    if (!simReady_) {
        QString err;
        if (!buildSimulation(err)) {
            appendLog("[Run] buildSimulation failed: " + err);
            return;
        }
    }

    if (!currentExperimentActive_) {
        startExperimentLogSession();
    }

    running_ = true;
    timer_->start();
    appendLog("[Run] start.");
}

void MainWindow::onPauseClicked()
{
    running_ = false;
    timer_->stop();
    appendLog("[Pause] stopped.");
}

void MainWindow::onResetClicked()
{
    running_ = false;
    timer_->stop();
    if (simReady_) {
        engine_.reset();
    }
    simReady_ = false;
    appendLog("[Reset] done.");
    closeExperimentLogSession();
    renderTerrainOnly();
}

void MainWindow::onTick()
{
    if (!running_ || !simReady_) return;

    engine_.step();

    const auto p = engine_.config();
    if (engine_.time() >= p.totalTime_s - 1e-12) {
        running_ = false;
        timer_->stop();
        appendLog(QString("[Run] finished. solve steps=%1 | total solve=%2 ms | avg solve=%3 ms")
                  .arg(static_cast<qulonglong>(engine_.solveStepCount()))
                  .arg(engine_.totalSolveMs(), 0, 'f', 3)
                  .arg(engine_.averageSolveMs(), 0, 'f', 3));
        closeExperimentLogSession();
        renderTerrainAndSlice();
        return;
    }

    if (cbExportCsv_->isChecked()) {
        const double t = engine_.time();
        if (t >= nextExportT_ - 1e-12) {
            QDir().mkpath(framesDir());

            const auto& g = engine_.grid();

            auto writeSlice = [&](const QString& suffix, double zWorld) {
                const int k = zToK(zWorld);
                std::vector<float> grid2d;
                float mx = 0.0f;
                engine_.extractSliceXY(k, grid2d, mx);

                ExporterCsv::Meta meta;
                meta.epsg = 0;
                meta.origin_x = g.x0;
                meta.origin_y = g.y0;
                meta.dx = g.dx;
                meta.dy = g.dy;
                meta.z = g.z(k);
                meta.t = t;
                meta.product = "slice_xy";
                meta.quantity = "mass_concentration";

                const QString f = QDir(framesDir()).filePath(
                    QString("frame_%1_%2.csv").arg(frameId_, 4, 10, QLatin1Char('0')).arg(suffix));

                std::string err;
                if (!ExporterCsv::writeGridFrame(
                        f.toStdString(),
                        meta,
                        g.Nx,
                        g.Ny,
                        grid2d,
                        err)) {
                    appendLog("[CSV] write failed: " + QString::fromStdString(err));
                } else {
                    appendLog("[CSV] wrote: " + f);
                }
            };

            auto writeColumn = [&]() {
                std::vector<float> grid2d;
                float mx = 0.0f;
                engine_.extractColumnIntegralXY(grid2d, mx);

                ExporterCsv::Meta meta;
                meta.epsg = 0;
                meta.origin_x = g.x0;
                meta.origin_y = g.y0;
                meta.dx = g.dx;
                meta.dy = g.dy;
                meta.z = -1.0;
                meta.t = t;
                meta.product = "column_integral";
                meta.quantity = "mass_per_area";

                const QString f = QDir(framesDir()).filePath(
                    QString("frame_%1_column.csv").arg(frameId_, 4, 10, QLatin1Char('0')));

                std::string err;
                if (!ExporterCsv::writeGridFrame(
                        f.toStdString(),
                        meta,
                        g.Nx,
                        g.Ny,
                        grid2d,
                        err)) {
                    appendLog("[CSV] column write failed: " + QString::fromStdString(err));
                } else {
                    appendLog("[CSV] wrote: " + f);
                }
            };

            writeSlice("zslice", effectiveZSlice());
            if (cbExportTwoSlices_->isChecked()) {
                writeSlice("agl", aglSliceZ());
            }
            if (cbExportColumn_->isChecked()) {
                writeColumn();
            }

            frameId_++;
            nextExportT_ += sbExportInterval_->value();
        }
    }

    renderTerrainAndSlice();
}
