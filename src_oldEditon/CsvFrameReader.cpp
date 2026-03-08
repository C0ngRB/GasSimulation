#include "CsvFrameReader.h"
#include <QFile>
#include <QRegularExpression>
#include <QStringList>

static bool parseHeader(const QString& line, int& Nx, int& Ny, double& t)
{
    // line: "# Nx=300 Ny=150 t=0.50"
    const QString s = line.trimmed();
    if (!s.startsWith("#")) return false;

    const QString body = s.mid(1).trimmed();
    const QStringList parts = body.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);

    for (const QString& p : parts) {
        if (p.startsWith("Nx=")) {
            bool ok = false;
            const int v = p.mid(3).toInt(&ok);
            if (ok) Nx = v;
        }
        else if (p.startsWith("Ny=")) {
            bool ok = false;
            const int v = p.mid(3).toInt(&ok);
            if (ok) Ny = v;
        }
        else if (p.startsWith("t=")) {
            bool ok = false;
            const double v = p.mid(2).toDouble(&ok);
            if (ok) t = v;
        }
    }
    return true;
}

bool CsvFrameReader::read(const QString& path, Frame& out, QString& err)
{
    QFile f(path);
    if (!f.open(QIODevice::ReadOnly)) {
        err = "Cannot open file: " + path;
        return false;
    }

    // Qt6 里 QTextStream::setCodec 已不推荐/可能不可用；直接按 UTF-8 读取更稳。
    const QByteArray bytes = f.readAll();
    const QString text = QString::fromUtf8(bytes);

    // 兼容 Windows \r\n：用正则按换行切
    QStringList lines = text.split(QRegularExpression("[\r\n]+"), Qt::SkipEmptyParts);
    if (lines.isEmpty()) {
        err = "Empty CSV data: " + path;
        return false;
    }

    int Nx = 0, Ny = 0;
    double t = 0.0;

    int startLine = 0;
    if (lines[0].trimmed().startsWith("#")) {
        parseHeader(lines[0], Nx, Ny, t);
        startLine = 1;
        if (lines.size() <= 1) {
            err = "CSV has header but no data: " + path;
            return false;
        }
    }

    // 收集数据行
    QStringList dataLines;
    for (int k = startLine; k < lines.size(); ++k) {
        const QString line = lines[k].trimmed();
        if (!line.isEmpty()) dataLines << line;
    }

    if (dataLines.isEmpty()) {
        err = "No data rows in CSV: " + path;
        return false;
    }

    // Deduce Nx if not provided
    if (Nx <= 0) {
        const QStringList cols = dataLines[0].split(",", Qt::KeepEmptyParts);
        Nx = cols.size();
    }
    // Deduce Ny if not provided
    if (Ny <= 0) {
        Ny = static_cast<int>(dataLines.size());
    }
    else {
        // 修复 std::min(int, qsizetype) 的类型冲突
        Ny = std::min(Ny, static_cast<int>(dataLines.size()));
    }

    if (Nx <= 0 || Ny <= 0) {
        err = "Invalid Nx/Ny after parsing: " + path;
        return false;
    }

    out.Nx = Nx;
    out.Ny = Ny;
    out.t = t;
    out.data.assign(static_cast<size_t>(Nx) * static_cast<size_t>(Ny), 0.0f);

    for (int j = 0; j < Ny; ++j) {
        const QStringList cols = dataLines[j].split(",", Qt::KeepEmptyParts);
        if (cols.size() < Nx) {
            err = "Row has fewer columns than Nx in: " + path;
            return false;
        }
        for (int i = 0; i < Nx; ++i) {
            bool ok = false;
            float v = cols[i].toFloat(&ok);
            if (!ok) v = 0.0f;
            out.data[static_cast<size_t>(i) + static_cast<size_t>(j) * static_cast<size_t>(Nx)] = v;
        }
    }

    return true;
}
