#pragma once

#include <QObject>
#include <QProcess>
#include <QDateTime>
#include <QStringList>
#include <QDir>
#include <QTimer>

#include "Simulator.h"
#include "CsvFrameReader.h"

/**
 * @brief 外部 CFD 引擎后端：写 params.json -> 运行 runner -> 轮询 frames 输出并逐帧可视化
 *
 * runner 约定：
 *   runner.bat --params <path/to/params.json> --out <path/to/frames>
 */
class ExternalCfdBackend : public QObject {
    Q_OBJECT
public:
    struct Config {
        QString runnerPath;   // e.g. tools/run_mock_cfd.bat
        QString workRootDir;  // e.g. work
        int pollIntervalMs = 100; // 轮询 frames 目录间隔
    };

    explicit ExternalCfdBackend(QObject* parent = nullptr);
    void start(const SimParams& p, const Config& cfg);
    void stop();
    bool isRunning() const;
signals:
    void logLine(const QString& s);
    void newFrameAvailable(const CsvFrameReader::Frame& f);
    void finished(bool ok, const QString& message);
private slots:
    void onProcStdout();
    void onProcStderr();
    void onProcFinished(int exitCode, QProcess::ExitStatus status);
    void onPoll();
private:
    bool writeParamsJson(const SimParams& p, const QString& path, QString& err);
    QString runDir_;
    QString framesDir_;
    QString paramsPath_;
    QStringList knownFrames_;
    QProcess* proc_{nullptr};
    QTimer* pollTimer_{nullptr};
};
