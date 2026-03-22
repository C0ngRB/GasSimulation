#pragma once
#include <map>
#include <string>
#include <vector>

#include <QMetaType>

class CsvFrameReader
{
public:
    struct Frame
    {
        std::map<std::string, std::string> meta;
        int Nx = 0;
        int Ny = 0;
        std::vector<std::vector<float>> grid;
    };

    static bool read(const std::string& path, Frame& out, std::string& err);
};

Q_DECLARE_METATYPE(CsvFrameReader::Frame)
