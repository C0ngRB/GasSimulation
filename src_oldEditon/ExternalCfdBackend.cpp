#include "ExternalCfdBackend.h"

#include <QFile>
#include <QJsonObject>
#include <QJsonDocument>
#include <QTimer>

ExternalCfdBackend::ExternalCfdBackend(QObject* parent) : QObject(parent) {
    proc_ = new QProcess(this);
    pollTimer_ = new QTimer(this);
    connect(pollTimer_, &QTimer::timeout, this, &ExternalCfdBackend::onPoll);

    connect(proc_, &QProcess::readyReadStandardOutput, this, &ExternalCfdBackend::onProcStdout);
    connect(proc_, &QProcess::readyReadStandardError, this, &ExternalCfdBackend::onProcStderr);
    connect(proc_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &ExternalCfdBackend::onProcFinished);
}

bool ExternalCfdBackend::isRunning() const {
    return proc_->state() != QProcess::NotRunning;
}

bool ExternalCfdBackend::writeParamsJson(const SimParams& p, const QString& path, QString& err) {
    QJsonObject o;
    o["Lx_m"] = p.Lx_m; o["Ly_m"] = p.Ly_m; o["Nx"] = p.Nx; o["Ny"] = p.Ny;
    o["totalTime_s"] = p.totalTime_s; o["dt_s"] = p.dt_s;
    o["windSpeed_mps"] = p.windSpeed_mps; o["windDir_deg"] = p.windDir_deg;
    o["D_m2ps"] = p.D_m2ps; o["decay_1ps"] = p.decay_1ps;
    o["srcX_m"] = p.srcX_m; o["srcY_m"] = p.srcY_m;
    o["leakRate"] = p.leakRate; o["effectiveHeight_m"] = p.effectiveHeight_m;

    QJsonDocument doc(o);
    QFile f(path);
    if (!f.open(QIODevice::WriteOnly)) {
        err = "Cannot write params.json: " + path;
        return false;
    }
    f.write(doc.toJson(QJsonDocument::Indented));
    return true;
}

void ExternalCfdBackend::start(const SimParams& p, const Config& cfg) {
    if (isRunning()) stop();

    const QString ts = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    runDir_ = QDir(cfg.workRootDir).filePath("run_" + ts);
    framesDir_ = QDir(runDir_).filePath("frames");
    paramsPath_ = QDir(runDir_).filePath("params.json");

    QDir().mkpath(framesDir_);

    QString err;
    if (!writeParamsJson(p, paramsPath_, err)) {
        emit finished(false, err);
        return;
    }

    emit logLine("[External] runDir=" + runDir_);
    emit logLine("[External] params=" + paramsPath_);
    emit logLine("[External] frames=" + framesDir_);
    emit logLine("[External] runner=" + cfg.runnerPath);

    // 运行 bat/cmd 最稳：cmd.exe /c <runner> --params ... --out ...
    QString program = "cmd.exe";
    QStringList args;
    args << "/c" << QDir::toNativeSeparators(cfg.runnerPath)
         << "--params" << QDir::toNativeSeparators(paramsPath_)
         << "--out"    << QDir::toNativeSeparators(framesDir_);

    proc_->setWorkingDirectory(runDir_);
    proc_->start(program, args);

    pollTimer_->start(cfg.pollIntervalMs);
}

void ExternalCfdBackend::stop() {
    pollTimer_->stop();
    if (isRunning()) {
        proc_->kill();
        proc_->waitForFinished(2000);
    }
}

void ExternalCfdBackend::onProcStdout() {
    const QByteArray b = proc_->readAllStandardOutput();
    if (!b.isEmpty()) emit logLine(QString::fromLocal8Bit(b).trimmed());
}

void ExternalCfdBackend::onProcStderr() {
    const QByteArray b = proc_->readAllStandardError();
    if (!b.isEmpty()) emit logLine(QString::fromLocal8Bit(b).trimmed());
}

void ExternalCfdBackend::onProcFinished(int exitCode, QProcess::ExitStatus status) {
    pollTimer_->stop();
    const bool ok = (status == QProcess::NormalExit && exitCode == 0);
    emit finished(ok, ok ? "External CFD finished OK" : QString("External CFD failed, exitCode=%1").arg(exitCode));
}

void ExternalCfdBackend::onPoll() {
    QDir d(framesDir_);
    if (!d.exists()) return;

    // frame_0000.csv, frame_0001.csv...
    QStringList files = d.entryList(QStringList() << "frame_*.csv", QDir::Files, QDir::Name);
    if (files.isEmpty()) return;

    // 找出新帧
    for (const auto& f : files) {
        if (!knownFrames_.contains(f)) {
            knownFrames_ << f;

            CsvFrameReader::Frame frame;
            QString err;
            const QString full = d.filePath(f);
            if (CsvFrameReader::read(full, frame, err)) {
                emit newFrameAvailable(frame);
            } else {
                emit logLine("[External] read frame failed: " + err);
            }
        }
    }
}
