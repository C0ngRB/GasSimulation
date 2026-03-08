# GasDispersionDesktop (Qt6)

一个桌面端交互平台：设置风向/风速/点源/泄漏强度/扩散参数，点击运行后动态可视化气体扩散态势。
数值内核为 2D 对流-扩散方程（被动标量）显式有限差分：迎风对流 + 中心差分扩散。

> **重要定位**：当前内置求解器解的是 *被动标量的平流-扩散-衰减方程*，不是全 Navier‑Stokes。它适合快速原型、可视化与参数敏感性分析；需要"真实 CFD"时再替换求解器即可。

## 依赖

- Qt 6.x (Widgets)
- CMake >= 3.20
- C++17 编译器（MSVC/Clang/GCC）

---

## 坐标系与单位约定（必须看）

- **坐标系**：**右手直角坐标系**（Cartesian）
  - **x / y**：水平面（单位 m）
  - **z**：竖直向上（单位 m）
- **风向定义**（`Wind dir (deg)`）：
  - `0°` 指向 **+x** 方向
  - `90°` 指向 **+y** 方向
  - （即角度从 +x 轴逆时针旋转）
- 地形高度 `terrain.height(x,y)` 与源位置 `srcX/srcY/srcZ` 均使用同一套（x,y,z）单位与参考系。

---

## DEM地形导入指南

> **重要操作顺序**：
> 1. 点击 `Pick...` 选择DEM文件
> 2. 点击 `Convert+Load` 转换并加载
> 3. 点击 `Preview Terrain` 预览地形
> 4. 设置Domain和Vertical参数

### 演示数据参数（src/tif/紫金山_演示数据.tif）

| 参数 | 值 | 说明 |
|------|-----|------|
| dx | 12.5 | 网格X方向间距 (m) |
| dy | 12.5 | 网格Y方向间距 (m) |
| Lx | 3475.0 | 域X方向长度 = 278 × 12.5 (m) |
| Ly | 2387.5 | 域Y方向长度 = 191 × 12.5 (m) |
| Z top margin | 200 | 顶部余量 (m) |
| Nz max | 120 | 最大Z层数（建议≥120） |
| dz | ≥4 | Z方向间距（建议≥4，内存友好） |

### 坐标系说明

- 演示数据使用 **UTM坐标系**
- DEM左下角坐标：X₀=669886.156, Y₀=3547787.750
- **srcX/srcY必须使用绝对坐标**（非相对坐标）

### 源点坐标示例

| 参数 | 绝对坐标 | 计算方式 |
|------|----------|----------|
| srcX | ≈ 671086.156 | X₀ + 1200 |
| srcY | ≈ 3548587.750 | Y₀ + 800 |
| srcZ | > 地形高度 | 地面高度 + AGL |

> **注意**：srcZ 必须高于地表，否则气体可能被地形遮挡。如果看不见气体，可以增加 srcZ 试试，但不要超过 tmax + ZtopMargin。

---

## 地形输入规范

### DEM地形（不依赖GDAL的本地格式）

程序内部 DEM 格式：

1. `dem_meta.json`（元数据）
2. `dem_data.bin`（float32 高程数组，little-endian，row-major）

**dem_meta.json 字段：**

| 字段 | 类型 | 含义 |
|---|---:|---|
| width / height | int | 栅格宽高（像素数） |
| origin_x / origin_y | double | **左上角像素中心**坐标（m） |
| dx / dy | double | 像素分辨率（m）；常见 north-up 数据 `dy` 为负 |
| epsg | int | 投影坐标 EPSG（必须非 0） |
| nodata | float | 无效值（可选） |
| z_min / z_max | double | 最小/最大高程（可选，显示用） |

**dem_data.bin 约定：**
- `float32`，小端序
- 行优先（row-major），第 `j` 行第 `i` 列索引为 `i + j*width`

> GUI 的 "Convert/Load" 按钮会调用 `tools/dem_convert.py` 将 GeoTIFF 转成上述格式并加载。

### 虚拟构造地形（Procedural，不依赖 DEM）

在 **Terrain → Mode = Procedural** 时使用：
- `Proc base z`：地形基准高程
- `Proc peak A`：峰值高度（例如高斯丘的振幅）
- `Proc sigma`：尺度（控制坡度与起伏范围）

> 目前实现的生成模式在 `TerrainProcedural.h` 中（Flat / GaussianHill / Ridge / MultiGaussian）。
> 后续要加复杂地形（分形噪声、阶梯、障碍物等），也建议只在这一层扩展。

### 默认平地

当 **不导入 DEM** 且 **不使用 Procedural** 时：
- 使用 `Flat z` 作为常数高程地面（默认 0）。

---

## 模拟区域（Domain）与网格（Vertical）

### Domain（XY水平域）

| 控件 | 含义 |
|---|---|
| x0 / y0 | 计算域左上角（或参考原点）坐标（m） |
| Lx / Ly | 计算域在 x/y 方向长度（m） |
| dx / dy | 网格步长（m） |

派生关系（程序内部会算）：
- `Nx ≈ floor(Lx/dx)+1`
- `Ny ≈ floor(Ly/dy)+1`

> dx/dy 越小：更细致，但更慢；并且显式时间步长稳定性会更严格。

### Vertical（Z竖向网格 + 切片）

| 控件 | 含义 |
|---|---|
| dz | 竖向网格步长（m） |
| Z top margin | 在最大地形上方额外加的高度余量（m），防止顶盖太低 |
| Nz max | 最大竖向层数上限（防止过大内存） |
| Z slice | 可视化/导出的切片高度（m） |
| Follow slice | 勾选后切片高度会跟随源高度（常用于观察羽流"主层"） |

---

## 物理参数（Physics）

| 控件 | 单位 | 含义 |
|---|---|---|
| Wind speed | m/s | 常量风速大小 |
| Wind dir | deg | 常量风向（0°=+x，90°=+y） |
| K | m²/s | 有效扩散系数（可理解为湍扩散/数值扩散的合并参数） |
| Decay | 1/s | 一阶衰减系数（模拟化学反应/沉降等） |

---

## 时间与输出（Time / Output）

| 控件 | 含义 |
|---|---|
| Total (s) | 总模拟时长 |
| dt (s) | 时间步长（显式 Euler） |
| Auto clamp dt (stable) | 自动将 dt 限制在稳定范围内（建议开启） |
| Export CSV frames | 导出 CSV 帧到 `outputs/frames/` |
| Export interval (s) | 导出间隔（例如 0.2s） |
| Export two slices | 同时导出两张切片：`Z slice` 与 `AGL slice` |

---

## 三维泄漏源（3D Source）

| 控件 | 单位 | 含义 |
|---|---|---|
| srcX / srcY / srcZ | m | 泄漏源中心坐标 |
| srcRadius | m | 源球半径（注入体积） |
| Leak strength | (相对量)/s | 注入强度（质量/秒的意义；可按相对大小比较） |
| AGL slice (m) | m | "距地高度"切片：z = terrain(x_src,y_src) + AGL |
| Set srcZ = ground + AGL | — | 一键把 srcZ 设置为地面高度 + AGL |
| Auto center src | — | 强制 srcX/srcY 自动居中（避免源跑到角落） |

---

## 可视化（Visualization）

| 控件 | 含义 |
|---|---|
| Background | 背景模式：地形灰度+气体叠加 / 纯蓝底浓度图 |
| Cutoff (relative to sliceMax) | 相对阈值：低于 `cutoff * maxC` 的像素透明（抑制"虚胖"雾化） |

### 显示范围（固定 padding）

为了让羽流看起来不"胖"，渲染时会在图像四周加固定比例 padding。
该比例写死在 `MainWindow.cpp` 的 `kPad = 0.35`（**不是控件**）。
如需更大范围，直接调大该常量。

---

## 常见修改入口（给维护者）

- **UI 布局 / 控件增加**：`MainWindow.cpp` 构造函数（创建 GroupBox 与 FormLayout 的地方）
- **切片渲染 / 叠加**：`MainWindow::renderTerrainAndSlice()`
- **求解器与数值格式**：`Simulator3D.*`
- **地形接口与实现**：`ITerrain.h`, `TerrainDem.*`, `TerrainProcedural.h`, `TerrainFlat.h`
- **CSV 导出/读取**：`ExporterCsv.*`, `CsvFrameReader.*`

---

## 构建说明（强烈建议统一工具链）

你必须保证 **编译器** 与 **Qt 套件**匹配，否则会出现典型错误：
> "Qt requires a C++17 compiler … On MSVC, you must pass /Zc:__cplusplus …"
（根因是：用 MSVC 生成器却指向了 Qt MinGW 头文件/库）

### 使用 Qt MinGW 套件（推荐你当前环境）

```powershell
# 1) 配置（生成 build-mingw 构建树）
cmake -S . -B build-mingw -G "MinGW Makefiles" `
  -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_PREFIX_PATH="C:\Qt\6.9.3\mingw_64"

# 2) 构建（生成 exe）
cmake --build build-mingw -- -j 8
```

### 使用 Qt MSVC 套件

需要安装对应的 Qt MSVC 版本（不是 mingw_64）。
```powershell
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_PREFIX_PATH=<Qt MSVC path>
cmake --build build --config Release
```

---

## 输出文件（CSV）

默认导出目录：
- `outputs/frames/frame_XXXX.csv`

每个 CSV：
- 以 `#` 开头的三行元数据（CRS、origin、分辨率、t 等）
- 后续 Ny 行、每行 Nx 个数值（逗号分隔）
