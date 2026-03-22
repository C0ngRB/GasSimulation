cmake_minimum_required(VERSION 3.20)
project(GasDispersionDesktop VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets)

set(SOURCES
    src/main.cpp
    src/MainWindow.cpp
    src/Simulator3D.cpp
    src/TerrainDem.cpp
    src/ExporterCsv.cpp
    src/CsvFrameReader.cpp
    src/WindField.cpp
    src/EulerianSolver3D.cpp
    src/GaussianPlumeModel.cpp
    src/GaussianPuffModel.cpp
    src/PlumeEngine.cpp
)

set(HEADERS
    src/MainWindow.h
    src/Simulator3D.h
    src/Grid3D.h
    src/ITerrain.h
    src/TerrainDem.h
    src/TerrainFlat.h
    src/TerrainProcedural.h
    src/ColorMap.h
    src/ExporterCsv.h
    src/CsvFrameReader.h
    src/ExternalCfdBackend.h
    src/SimulationConfig.h
    src/ISimulationModel.h
    src/TerrainSampler.h
    src/WindField.h
    src/LeakSourceController.h
    src/EulerianSolver3D.h
    src/GaussianPlumeModel.h
    src/GaussianPuffModel.h
    src/PlumeEngine.h
)

add_executable(GasDispersionDesktop ${SOURCES} ${HEADERS})

target_link_libraries(GasDispersionDesktop Qt6::Core Qt6::Widgets)

target_include_directories(GasDispersionDesktop PRIVATE src)

if(MSVC)
    target_compile_options(GasDispersionDesktop PRIVATE /W4)
else()
    target_compile_options(GasDispersionDesktop PRIVATE -Wall -Wextra -Wpedantic)
endif()
if(WIN32)
    target_compile_definitions(GasDispersionDesktop PRIVATE _USE_MATH_DEFINES)
endif()

add_custom_target(run COMMAND GasDispersionDesktop)
