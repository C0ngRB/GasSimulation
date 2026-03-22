/**
 * @file main.cpp
 * @brief GasDispersionDesktop 程序入口
 *
 * 【程序说明】
 * GasDispersionDesktop - 基于Qt6的三维气体扩散仿真可视化桌面应用
 *
 * 【功能概述】
 * 本程序实现了一个交互式的三维气体扩散仿真环境，用户可以：
 * - 配置地形（平地/DEM/程序化生成）
 * - 设置风场、扩散系数、源项参数
 * - 实时预览地形和浓度分布
 * - 导出仿真数据用于后处理分析
 *
 * 【程序流程】
 * 1. 创建QApplication对象（管理Qt应用程序状态）
 * 2. 创建MainWindow主窗口对象
 * 3. 显示主窗口
 * 4. 进入Qt事件循环（等待用户交互）
 *
 * 【依赖项】
 * - Qt6 Core: Qt核心功能
 * - Qt6 Widgets: GUI组件库
 * - MainWindow: 主窗口类（包含所有UI和仿真逻辑）
 *
 * 【编译要求】
 * - C++20编译器
 * - Qt6库（Core + Widgets模块）
 *
 * 【运行说明】
 * 直接运行生成的GasDispersionDesktop可执行文件即可启动程序。
 */

#include <QtWidgets/QApplication>
#include "MainWindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    return app.exec();
}
