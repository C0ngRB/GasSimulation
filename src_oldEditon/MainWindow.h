#pragma once
/**
 * @file MainWindow.h
 * @brief GasDispersionDesktop 主窗口
 *
 * 【设计目的】
 * 提供Qt GUI界面，让用户能够：
 * 1. 选择和配置地形（Flat/DEM/Procedural）
 * 2. 设置模拟参数（风场、扩散、源项、时间）
 * 3. 预览地形和气体扩散结果
 * 4. 导出仿真数据（CSV格式）
 *
 * 【UI布局】
 * 采用水平分割布局（QSplitter）：
 * - 左侧：可滚动的参数配置面板（QScrollArea）
 *   · Terrain分组：地形模式选择和参数
 *   · Domain分组：笛卡尔网格参数
 *   · Vertical分组：垂直网格和切片设置
 *   · Physics分组：风场和扩散参数
 *   · Time & Output分组：时间步长和导出设置
 *   · Visualization分组：可视化模式选择
 *   · 3D Source分组：泄漏源位置和强度
 *   · Control分组：运行/暂停/重置按钮和日志
 *
 * - 右侧：可视化显示区域
 *   · 上部：地形/浓度切片渲染视图（QLabel）
 *   · 底部：状态栏，显示当前模拟状态
 *
 * 【坐标系】
 * 与Simulator3D一致：
 * - X: 右东
 * - Y: 向前北（地图惯例）
 * - Z: 向上
 * - 单位：米
 *
 * 【数据流】
 * 用户操作 → UI控件 → MainWindow成员变量
 *                                    ↓
 *                         buildSimulation()
 *                                    ↓
 *                         Simulator3D::initialize()
 *                                    ↓
 *                         onTick() [定时器回调]
 *                                    ↓
 *                         Simulator3D::step()
 *                                    ↓
 *                         Simulator3D::extractSliceXY()
 *                                    ↓
 *                         renderTerrainAndSlice()
 *                                    ↓
 *                         QLabel显示
 *
 * 【关键功能】
 * - 地形预览：点击Preview Terrain后显示地形渲染
 * - 仿真控制：Run/Pause/Reset按钮
 * - 自动居中：非DEM模式下自动将源点移到域中心
 * - 双切片导出：同时导出zslice和agl两种高度的浓度
 * - 背景模式：可选地形灰度背景或纯蓝背景
 */

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
#include "Simulator3D.h"

class MainWindow : public QMainWindow {
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

    TerrainFlat flat_{0.0f};
    TerrainProcedural proc_{};

    bool terrainPreviewReady_{false};
    int tNx_{0}, tNy_{0};
    double tx0_{0}, ty0_{0}, tdx_{1}, tdy_{1};
    float tMin_{0.0f}, tMax_{0.0f};
    std::vector<float> terrainXY_;

    Simulator3D sim_;
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
    Simulator3D::Params readSimParams() const;

    float groundAtSource() const;
    double clampSliceToFluid(double z) const;
    double effectiveZSlice() const;
    double aglSliceZ() const;
    int zToK(double z) const;
    void syncSliceWithSourceIfNeeded();
    void enforceSourceCenterIfNeeded();
};
