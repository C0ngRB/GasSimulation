#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dem_convert.py

将"规范 GeoTIFF DEM（投影坐标、单位米）"转换为 GasDispersionDesktop 可稳定读取的内部格式：
- dem_meta.json
- dem_data.bin  (float32, row-major, top->bottom)

依赖（建议二选一）：
- rasterio（推荐）：conda install -c conda-forge rasterio
- 或 osgeo.gdal：conda install -c conda-forge gdal

用法：
  python tools/dem_convert.py --in dem.tif --out_dir work/run_xxx/dem

注意：
- 如果检测到 nodata，脚本会直接报错（要求你先做 DEM 填洞/裁剪）。这是为了保证"输入规范可复现"。
"""
import argparse
import json
import os
import sys

def try_rasterio(path):
    import rasterio
    with rasterio.open(path) as ds:
        if ds.count != 1:
            raise RuntimeError("DEM must be single-band")
        crs = ds.crs
        if crs is None:
            raise RuntimeError("DEM has no CRS")
        epsg = crs.to_epsg()
        if epsg is None:
            raise RuntimeError("Cannot determine EPSG (must be projected meters)")

        nodata = ds.nodata
        if nodata is not None:
            raise RuntimeError(f"DEM has nodata={nodata}. Please fill/crop nodata before converting.")

        data = ds.read(1).astype("float32")
        transform = ds.transform
        origin_x = transform.c + transform.a * 0.5
        origin_y = transform.f + transform.e * 0.5
        dx = float(transform.a)
        dy = float(transform.e)

        meta = dict(
            width=int(ds.width),
            height=int(ds.height),
            origin_x=float(origin_x),
            origin_y=float(origin_y),
            dx=float(dx),
            dy=float(dy),
            epsg=int(epsg),
            nodata=-3.4e38,
            z_min=float(data.min()),
            z_max=float(data.max()),
        )
        return meta, data

def try_gdal(path):
    from osgeo import gdal, osr
    ds = gdal.Open(path, gdal.GA_ReadOnly)
    if ds is None:
        raise RuntimeError("Cannot open DEM with GDAL")
    if ds.RasterCount != 1:
        raise RuntimeError("DEM must be single-band")
    band = ds.GetRasterBand(1)
    nodata = band.GetNoDataValue()
    if nodata is not None:
        raise RuntimeError(f"DEM has nodata={nodata}. Please fill/crop nodata before converting.")

    gt = ds.GetGeoTransform()
    if abs(gt[2]) > 1e-12 or abs(gt[4]) > 1e-12:
        raise RuntimeError("DEM must be north-up (no rotation). Please warp to north-up.")

    dx = float(gt[1])
    dy = float(gt[5])
    origin_x = float(gt[0] + dx * 0.5)
    origin_y = float(gt[3] + dy * 0.5)

    w = ds.RasterXSize
    h = ds.RasterYSize

    wkt = ds.GetProjection()
    if not wkt:
        raise RuntimeError("DEM has no CRS WKT")
    srs = osr.SpatialReference()
    srs.ImportFromWkt(wkt)
    epsg = srs.GetAttrValue("AUTHORITY", 1)
    if epsg is None:
        raise RuntimeError("Cannot determine EPSG (must be projected meters)")
    epsg = int(epsg)

    data = band.ReadAsArray().astype("float32")
    meta = dict(
        width=int(w),
        height=int(h),
        origin_x=float(origin_x),
        origin_y=float(origin_y),
        dx=float(dx),
        dy=float(dy),
        epsg=int(epsg),
        nodata=-3.4e38,
        z_min=float(data.min()),
        z_max=float(data.max()),
    )
    return meta, data

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="inp", required=True, help="Input GeoTIFF DEM (projected meters)")
    ap.add_argument("--out_dir", required=True, help="Output directory")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    meta_path = os.path.join(args.out_dir, "dem_meta.json")
    bin_path = os.path.join(args.out_dir, "dem_data.bin")

    meta = None
    data = None
    last_err = None

    try:
        meta, data = try_rasterio(args.inp)
    except Exception as e:
        last_err = e
        try:
            meta, data = try_gdal(args.inp)
        except Exception as e2:
            raise RuntimeError(f"Rasterio failed: {last_err}\nGDAL failed: {e2}")

    with open(meta_path, "w", encoding="utf-8") as f:
        json.dump(meta, f, ensure_ascii=False, indent=2)

    data.tofile(bin_path)

    print(f"[OK] meta: {meta_path}")
    print(f"[OK] bin : {bin_path}")
    return 0

if __name__ == "__main__":
    sys.exit(main())
