#pragma once
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QLabel>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QPlainTextEdit>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QScrollArea>
#include <QTimer>

#include <vector>

#include "TerrainDem.h"
#include "TerrainFlat.h"
#include "TerrainProcedural.h"
#include "PlumeEngine.h"

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);

private slots:
    void onPickDemClicked();
    void onConvertLoadDemClicked();
    void onTerrainParamsChanged();
    void onPreviewTerrainClicked();

    void onSetSrcZFromGroundClicked();
    void onRunClicked();
    void onPauseClicked();
    void onResetClicked();
    void onTick();

    void onFollowSliceToggled(bool on);

private:
    enum class TerrainMode { Flat = 0, Dem = 1, Procedural = 2 };

    QLabel* view_{nullptr};
    QLabel* status_{nullptr};

    QScrollArea* leftScroll_{nullptr};

    QComboBox* cbTerrainMode_{nullptr};

    QLineEdit* leDemTif_{nullptr};
    QPushButton* btnPickDem_{nullptr};
    QPushButton* btnConvertLoadDem_{nullptr};
    QSpinBox* sbDemStride_{nullptr};
    QLabel* demInfo_{nullptr};

    QDoubleSpinBox* sbFlatZ_{nullptr};

    QDoubleSpinBox* sbProcBaseZ_{nullptr};
    QDoubleSpinBox* sbProcPeakA_{nullptr};
    QDoubleSpinBox* sbProcSigma_{nullptr};

    QPushButton* btnPreviewTerrain_{nullptr};

    QDoubleSpinBox* sbX0_{nullptr};
    QDoubleSpinBox* sbY0_{nullptr};
    QDoubleSpinBox* sbLx_{nullptr};
    QDoubleSpinBox* sbLy_{nullptr};
    QDoubleSpinBox* sbDx_{nullptr};
    QDoubleSpinBox* sbDy_{nullptr};
    QLabel* domainInfo_{nullptr};

    QDoubleSpinBox* sbDz_{nullptr};
    QDoubleSpinBox* sbZTopMargin_{nullptr};
    QSpinBox* sbNzMax_{nullptr};
    QDoubleSpinBox* sbZSlice_{nullptr};
    QCheckBox* cbFollowSlice_{nullptr};

    QComboBox* cbModel_{nullptr};            // 新增：计算模型选择控件
    QDoubleSpinBox* sbWindSpeed_{nullptr};
    QDoubleSpinBox* sbWindDir_{nullptr};
    QDoubleSpinBox* sbK_{nullptr};
    QDoubleSpinBox* sbDecay_{nullptr};

    QDoubleSpinBox* sbTotalTime_{nullptr};
    QDoubleSpinBox* sbDt_{nullptr};
    QCheckBox* cbAutoClampDt_{nullptr};
    QCheckBox* cbExportCsv_{nullptr};
    QDoubleSpinBox* sbExportInterval_{nullptr};
    QCheckBox* cbExportTwoSlices_{nullptr};

    QDoubleSpinBox* sbSrcX_{nullptr};
    QDoubleSpinBox* sbSrcY_{nullptr};
    QDoubleSpinBox* sbSrcZ_{nullptr};
    QDoubleSpinBox* sbSrcRadius_{nullptr};
    QDoubleSpinBox* sbLeak_{nullptr};
    QDoubleSpinBox* sbAgl_{nullptr};
    QPushButton* btnSetSrcZFromGround_{nullptr};
    QCheckBox* cbAutoCenterSrc_{nullptr};

    QComboBox* cbBackgroundMode_{nullptr};
    QDoubleSpinBox* sbDisplayCutoffRel_{nullptr};

    QPushButton* btnRun_{nullptr};
    QPushButton* btnPause_{nullptr};
    QPushButton* btnReset_{nullptr};
    QPlainTextEdit* log_{nullptr};

    TerrainDem dem_;
    bool hasDem_{false};

    TerrainFlat flat_{};
    TerrainProcedural proc_{};

    bool terrainPreviewReady_{false};
    int tNx_{0}, tNy_{0};
    double tx0_{0.0}, ty0_{0.0}, tdx_{1.0}, tdy_{1.0};
    float tMin_{0.0f}, tMax_{0.0f};
    std::vector<float> terrainXY_;

    PlumeEngine engine_;                     // 改为统一引擎
    bool simReady_{false};
    QTimer* timer_{nullptr};
    bool running_{false};

    double nextExportT_{0.0};
    int frameId_{0};

    std::vector<float> slice_;
    float sliceMax_{0.0f};

    void appendLog(const QString& s);
    QString framesDir() const;

    TerrainMode terrainMode() const;
    const ITerrain* currentTerrain() const;

    void updateDomainInfo();
    bool buildTerrainPreview(QString& errOut);
    void renderTerrainOnly();
    void renderTerrainAndSlice();

    bool buildSimulation(QString& errOut);
    SimulationConfig readSimConfig() const;

    float groundAtSource() const;
    double clampSliceToFluid(double z) const;
    double effectiveZSlice() const;
    double aglSliceZ() const;
    int zToK(double z) const;
    void syncSliceWithSourceIfNeeded();
    void enforceSourceCenterIfNeeded();
};
