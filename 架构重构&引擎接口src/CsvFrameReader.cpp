#include "CsvFrameReader.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string trim(const std::string& s)
{
    std::size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) {
        ++b;
    }

    std::size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) {
        --e;
    }

    return s.substr(b, e - b);
}

bool parseHeaderLine(const std::string& line, std::string& key, std::string& value)
{
    if (line.empty() || line[0] != '#') {
        return false;
    }

    const std::string body = trim(line.substr(1));

    std::size_t pos = body.find(':');
    if (pos == std::string::npos) {
        pos = body.find('=');
    }
    if (pos == std::string::npos) {
        return false;
    }

    key = trim(body.substr(0, pos));
    value = trim(body.substr(pos + 1));
    return !key.empty();
}

bool parseIntMeta(const std::map<std::string, std::string>& meta, const std::string& key, int& out)
{
    auto it = meta.find(key);
    if (it == meta.end()) return false;

    try {
        out = std::stoi(it->second);
        return true;
    } catch (...) {
        return false;
    }
}

std::vector<float> parseCsvRow(const std::string& line, bool& ok)
{
    ok = true;
    std::vector<float> row;

    std::stringstream ss(line);
    std::string cell;

    while (std::getline(ss, cell, ',')) {
        const std::string v = trim(cell);
        try {
            row.push_back(v.empty() ? 0.0f : std::stof(v));
        } catch (...) {
            ok = false;
            return {};
        }
    }

    if (line.find(',') != std::string::npos && !line.empty() && line.back() == ',') {
        row.push_back(0.0f);
    }

    return row;
}

} // namespace

bool CsvFrameReader::read(const std::string& path, Frame& out, std::string& err)
{
    err.clear();
    out = Frame{};

    std::ifstream fin(path);
    if (!fin) {
        err = "Cannot open file: " + path;
        return false;
    }

    std::string line;
    std::vector<std::vector<float>> rows;

    while (std::getline(fin, line)) {
        const std::string s = trim(line);
        if (s.empty()) {
            continue;
        }

        std::string key;
        std::string value;
        if (parseHeaderLine(s, key, value)) {
            out.meta[key] = value;
            continue;
        }

        bool ok = false;
        std::vector<float> row = parseCsvRow(s, ok);
        if (!ok) {
            err = "Failed to parse numeric row in: " + path;
            return false;
        }
        if (row.empty()) {
            continue;
        }
        rows.push_back(std::move(row));
    }

    if (rows.empty()) {
        err = "No grid rows in CSV: " + path;
        return false;
    }

    int nxHeader = 0;
    int nyHeader = 0;
    parseIntMeta(out.meta, "Nx", nxHeader);
    parseIntMeta(out.meta, "Ny", nyHeader);

    const int nxData = static_cast<int>(rows.front().size());
    const int nyData = static_cast<int>(rows.size());

    for (const auto& row : rows) {
        if (static_cast<int>(row.size()) != nxData) {
            err = "Inconsistent row width in CSV: " + path;
            return false;
        }
    }

    if (nxHeader > 0 && nxHeader != nxData) {
        err = "Header Nx does not match data width in: " + path;
        return false;
    }
    if (nyHeader > 0 && nyHeader != nyData) {
        err = "Header Ny does not match data height in: " + path;
        return false;
    }

    out.Nx = (nxHeader > 0) ? nxHeader : nxData;
    out.Ny = (nyHeader > 0) ? nyHeader : nyData;
    out.grid = std::move(rows);

    return true;
}
