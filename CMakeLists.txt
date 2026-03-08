# =============================================================================
# @file CMakeLists.txt
# @brief GasDispersionDesktop 构建配置
#
# 【项目概述】
# GasDispersionDesktop - 基于Qt6的三维气体扩散仿真可视化桌面应用
#
# 【依赖项】
# - CMake: 3.20+
# - C++编译器: 支持C++20的编译器（MSVC, GCC, Clang）
# - Qt6: Core, Widgets 模块
#
# 【构建特性】
# - CMAKE_CXX_STANDARD 20: 使用C++20标准
# - CMAKE_AUTOMOC/AUTORCC/AUTOUIC: Qt元对象编译器自动处理
#
# 【源文件组织】
# SOURCES: 实现文件（.cpp）
#   - main.cpp: 程序入口
#   - MainWindow.cpp: GUI主窗口
#   - Simulator3D.cpp: 三维扩散模拟核心
#   - TerrainDem.cpp: DEM地形加载
#   - ExporterCsv.cpp: CSV数据导出
#
# HEADERS: 头文件（.h/.hpp）
#   - MainWindow.h: 主窗口声明
#   - Simulator3D.h: 模拟器声明
#   - Grid3D.h: 三维网格定义
#   - ITerrain.h: 地形抽象接口
#   - TerrainDem.h/TerrainFlat.h/TerrainProcedural.h: 地形实现
#   - ColorMap.h: 颜色映射（可视化）
#   - ExporterCsv.h: CSV导出声明
#
# 【编译选项】
# - MSVC: /W4 (高警告级别)
# - GCC/Clang: -Wall -Wextra -Wpedantic (严格警告)
#
# 【输出配置】
# - 可执行文件: GasDispersionDesktop.exe
# - 安装目录: bin/
#
# 【构建命令示例】
#   mkdir build && cd build
#   cmake .. -G "Ninja" -DCMAKE_BUILD_TYPE=Release
#   cmake --build .
#
# =============================================================================

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
)

add_executable(GasDispersionDesktop ${SOURCES} ${HEADERS})

target_link_libraries(GasDispersionDesktop Qt6::Core Qt6::Widgets)

target_include_directories(GasDispersionDesktop PRIVATE src)

if(MSVC)
    target_compile_options(GasDispersionDesktop PRIVATE /W4)
else()
    target_compile_options(GasDispersionDesktop PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS GasDispersionDesktop
    RUNTIME DESTINATION bin
)
