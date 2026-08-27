#!/usr/bin/env python3
"""从 LIO-SAM 的 GlobalMap.pcd 生成同源导航地图包。

地图包包含三件套：
- GlobalMap.pcd：NDT/GICP 定位地图；
- map.yaml / map.pgm：Nav2 全局代价地图（由上游
  lidar_localization_ros2 的 generate_occupancy_map_from_pcd.py 生成）；
- map_bundle.yaml：记录 frame_id、各文件 SHA-256、生成时间与建图来源，
  供 validate_map_bundle 在启动前校验。

用法：
    ros2 run go2_navigation build_map_bundle --map-dir $GO2_PROJECT_ROOT/go2_maps/latest
"""

import argparse
import datetime
import hashlib
import os
import subprocess
import sys
import shutil
import tempfile
import yaml

from ament_index_python.packages import get_package_prefix

from go2_navigation.map_utils import MAP_BUNDLE_SCHEMA_VERSION, MAP_FILES, default_map_dir


def sha256(path: str) -> str:
    """计算文件 SHA-256，用于完整性校验。"""
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def find_upstream_map_script() -> str:
    """定位上游 PCD 转二维地图脚本的绝对路径。"""
    prefix = get_package_prefix("lidar_localization_ros2")
    candidates = [
        os.path.join(prefix, "lib", "lidar_localization_ros2",
                     "generate_occupancy_map_from_pcd.py"),
        os.path.join(prefix, "share", "lidar_localization_ros2", "scripts",
                     "generate_occupancy_map_from_pcd.py"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(
        "未找到上游 generate_occupancy_map_from_pcd.py，请先构建 "
        "lidar_localization_ros2。"
    )


def run_upstream_map_generation(
    pcd_path: str,
    output_dir: str,
    map_name: str,
    resolution: float,
    obstacle_height_m: float,
    inflate_radius_m: float,
    min_points_per_cell: int = 1,
    bounds: dict | None = None,
) -> None:
    """调用上游脚本生成 Nav2 可用的 pgm + yaml 地图。

    bounds 提供 {x_min,x_max,y_min,y_max} 时按该范围裁剪，避免远处点把
    地图放大到稀疏区域、导致大部分单元被标为 unknown。
    """
    script = find_upstream_map_script()
    cmd = [
        sys.executable, script,
        "--pcd", pcd_path,
        "--output-dir", output_dir,
        "--map-name", map_name,
        "--resolution", str(resolution),
        "--obstacle-height-m", str(obstacle_height_m),
        "--inflate-radius-m", str(inflate_radius_m),
        "--min-points-per-cell", str(min_points_per_cell),
    ]
    for key in ("x_min", "x_max", "y_min", "y_max"):
        if bounds and bounds.get(key) is not None:
            cmd += ["--%s" % key.replace("_", "-"), str(bounds[key])]
    print("运行上游地图生成脚本：\n  " + " ".join(cmd))
    result = subprocess.run(cmd, text=True, capture_output=True)
    if result.returncode != 0:
        sys.stderr.write(result.stdout)
        sys.stderr.write(result.stderr)
        raise RuntimeError("上游地图生成失败（退出码 %d）。" % result.returncode)


def write_bundle(
    map_dir: str,
    map_name: str,
    source: str,
    generated_files: dict,
) -> None:
    """把地图包清单写入 map_bundle.yaml。"""
    files = {}
    for rel_name, role in MAP_FILES.items():
        path = os.path.join(map_dir, rel_name)
        files[rel_name] = {
            "path": rel_name,
            "sha256": sha256(path),
            "role": role,
        }

    bundle = {
        "schema_version": MAP_BUNDLE_SCHEMA_VERSION,
        "frame_id": "map",
        "created_at": datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "source": source,
        "map_name": map_name,
        "files": files,
        "generation": generated_files,
    }
    bundle_path = os.path.join(map_dir, "map_bundle.yaml")
    with open(bundle_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(bundle, f, allow_unicode=True, sort_keys=False)
    print("已生成地图包清单：%s" % bundle_path)
    print("生成文件：%s" % ", ".join(MAP_FILES))


def backup_existing_map_files(map_dir: str) -> str | None:
    """把将被替换的地图派生产物复制到可恢复的时间戳目录。"""
    existing = [name for name in ("map.yaml", "map.pgm", "map_bundle.yaml",
                                  "map_stats.json")
                if os.path.isfile(os.path.join(map_dir, name))]
    if not existing:
        return None
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    backup_dir = os.path.join(map_dir, "map_backup_" + stamp)
    os.makedirs(backup_dir, exist_ok=False)
    for name in existing:
        shutil.copy2(os.path.join(map_dir, name), os.path.join(backup_dir, name))
    return backup_dir


def enrich_map_yaml(path: str, source: str) -> None:
    """补齐由上游脚本生成的 Nav2 元数据，保持 map 坐标系可追溯。"""
    with open(path, "r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream)
    if not isinstance(metadata, dict):
        raise RuntimeError("上游生成的 map.yaml 不是对象")
    metadata.update({
        "frame_id": "map",
        "created_at": datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S"),
        "source": source,
    })
    with open(path, "w", encoding="utf-8") as stream:
        yaml.safe_dump(metadata, stream, allow_unicode=True, sort_keys=False)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map-dir",
        default=default_map_dir(),
        help="存放 GlobalMap.pcd 并输出地图包的目录",
    )
    parser.add_argument(
        "--map-name", default="map", help="输出 2D 地图基础名（不含扩展名）"
    )
    parser.add_argument(
        "--resolution", type=float, default=0.10,
        help="Nav2 地图分辨率（米/格）",
    )
    parser.add_argument(
        "--obstacle-height-m", type=float, default=0.4,
        help="单元内高度差超过该值视为障碍",
    )
    parser.add_argument(
        "--inflate-radius-m", type=float, default=0.0,
        help="离线障碍膨胀半径（米；默认 0，由 Nav2 inflation_layer 统一处理）",
    )
    parser.add_argument(
        "--source", default="LIO-SAM", help="建图来源说明"
    )
    parser.add_argument(
        "--min-points-per-cell", type=int, default=1,
        help="单元内最少点数才标记（默认 1，稀疏地图更宽容）",
    )
    for key, desc in (("x_min", "裁剪范围 X 最小"), ("x_max", "裁剪范围 X 最大"),
                      ("y_min", "裁剪范围 Y 最小"), ("y_max", "裁剪范围 Y 最大")):
        parser.add_argument("--%s" % key.replace("_", "-"),
                            type=float, default=None, help=desc)
    args = parser.parse_args()

    map_dir = os.path.abspath(os.path.expanduser(args.map_dir))
    pcd_path = os.path.join(map_dir, "GlobalMap.pcd")
    if not os.path.isfile(pcd_path):
        sys.stderr.write("错误：未找到 %s，请先建图并执行 save_Map.sh。\n" % pcd_path)
        return 1

    os.makedirs(map_dir, exist_ok=True)
    if args.inflate_radius_m < 0.0:
        parser.error("--inflate-radius-m 不能小于 0")
    if args.inflate_radius_m > 0.0:
        print("警告：离线膨胀会与 Nav2 inflation_layer 叠加，仅在明确需要时使用。")
    bounds = {key: getattr(args, key) for key in
              ("x_min", "x_max", "y_min", "y_max")}
    # 先在 map_dir 同一文件系统内生成全部新文件；只有全部成功才备份并替换。
    with tempfile.TemporaryDirectory(prefix=".go2_map_build_", dir=map_dir) as tmp_dir:
        run_upstream_map_generation(
            pcd_path, tmp_dir, args.map_name,
            args.resolution, args.obstacle_height_m, args.inflate_radius_m,
            min_points_per_cell=args.min_points_per_cell,
            bounds=bounds,
        )
        generated_files = {
            "resolution_m": args.resolution,
            "obstacle_height_m": args.obstacle_height_m,
            "offline_inflate_radius_m": args.inflate_radius_m,
            "min_points_per_cell": args.min_points_per_cell,
            "bounds": bounds,
        }
        for ext in ("yaml", "pgm", "stats.json"):
            upstream_name = (args.map_name + "_stats.json"
                             if ext == "stats.json" else args.map_name + "." + ext)
            src = os.path.join(tmp_dir, upstream_name)
            if not os.path.isfile(src):
                sys.stderr.write("错误：上游脚本未产出 %s\n" % src)
                return 1
            dst_name = "map_" + ext if ext == "stats.json" else "map." + ext
            dst = os.path.join(tmp_dir, dst_name)
            if os.path.abspath(src) != os.path.abspath(dst):
                shutil.copy2(src, dst)

        staged_yaml = os.path.join(tmp_dir, "map.yaml")
        enrich_map_yaml(staged_yaml, args.source)
        staged_dir = os.path.join(tmp_dir, "bundle")
        os.makedirs(staged_dir)
        staged_files = ["GlobalMap.pcd", "map.yaml", "map.pgm", "map_stats.json"]
        for name in staged_files:
            source_path = (pcd_path if name == "GlobalMap.pcd"
                           else os.path.join(tmp_dir, name))
            if name == "map_stats.json":
                source_path = os.path.join(tmp_dir, "map_stats.json")
            shutil.copy2(source_path, os.path.join(staged_dir, name))
        write_bundle(staged_dir, args.map_name, args.source, generated_files)

        backup_dir = backup_existing_map_files(map_dir)
        try:
            for name in ("map.yaml", "map.pgm", "map_stats.json", "map_bundle.yaml"):
                os.replace(os.path.join(staged_dir, name), os.path.join(map_dir, name))
        except OSError:
            if backup_dir:
                for name in ("map.yaml", "map.pgm", "map_stats.json", "map_bundle.yaml"):
                    backup_path = os.path.join(backup_dir, name)
                    if os.path.isfile(backup_path):
                        shutil.copy2(backup_path, os.path.join(map_dir, name))
            raise

    if backup_dir:
        print("已备份旧地图派生产物：%s" % backup_dir)

    return 0


if __name__ == "__main__":
    sys.exit(main())
