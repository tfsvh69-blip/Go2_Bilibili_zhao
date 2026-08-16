#!/usr/bin/env python3
"""启动前校验同源导航地图包。"""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path
import sys

from go2_navigation.map_utils import (
    MAP_BUNDLE_SCHEMA_VERSION,
    MAP_FILES,
    MapValidationError,
    default_map_dir,
    load_bundle_metadata,
    load_static_map,
    safe_child_path,
)


def sha256(path: Path) -> str:
    """计算文件 SHA-256，用于完整性校验。"""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate(map_dir: str) -> tuple[bool, list[str]]:
    """校验新版本地图包，返回 ``(是否通过, 问题列表)``。"""
    problems: list[str] = []
    map_root = Path(map_dir).expanduser().resolve()
    try:
        bundle = load_bundle_metadata(map_root)
    except MapValidationError as exc:
        return False, [str(exc)]

    if bundle.get("schema_version") != MAP_BUNDLE_SCHEMA_VERSION:
        return False, [
            "地图包不是当前格式（需要 schema_version: %d）；"
            "请重新执行 build_map_bundle，避免使用旧的重复膨胀地图。"
            % MAP_BUNDLE_SCHEMA_VERSION
        ]
    if bundle.get("frame_id") != "map":
        problems.append("frame_id 不是 map：%r" % bundle.get("frame_id"))
    if not isinstance(bundle.get("generation"), dict):
        problems.append("缺少 generation 段，无法追溯建图参数")

    files = bundle.get("files")
    if not isinstance(files, dict):
        return False, ["map_bundle.yaml 格式错误：files 段必须是对象"]
    if set(files) != set(MAP_FILES):
        problems.append("files 段必须且只能包含：%s" % ", ".join(MAP_FILES))

    for name, role in MAP_FILES.items():
        metadata = files.get(name)
        if not isinstance(metadata, dict):
            problems.append("缺少文件元数据：%s" % name)
            continue
        if metadata.get("role") != role:
            problems.append("文件角色错误：%s" % name)
        try:
            path = safe_child_path(map_root, str(metadata.get("path", "")))
        except MapValidationError as exc:
            problems.append("%s：%s" % (name, exc))
            continue
        if path.name != name:
            problems.append("文件路径必须指向 %s" % name)
            continue
        if not path.is_file():
            problems.append("文件缺失：%s" % path)
            continue
        expected = metadata.get("sha256")
        if not isinstance(expected, str) or len(expected) != 64:
            problems.append("SHA-256 格式错误：%s" % name)
        elif sha256(path) != expected:
            problems.append("SHA-256 不匹配：%s（请重新执行 build_map_bundle）" % path)

    generation = bundle.get("generation") or {}
    if generation.get("offline_inflate_radius_m") != 0.0:
        problems.append(
            "地图含离线膨胀；阶段一导航要求 offline_inflate_radius_m: 0.0"
        )
    try:
        static_map = load_static_map(map_root)
        if static_map.width < 2 or static_map.height < 2:
            problems.append("map.pgm 尺寸过小，无法导航")
    except MapValidationError as exc:
        problems.append(str(exc))

    return not problems, problems


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--map-dir",
        default=default_map_dir(),
        help="地图包目录",
    )
    args = parser.parse_args()
    map_dir = str(Path(args.map_dir).expanduser().resolve())
    ok, problems = validate(map_dir)
    if ok:
        print("地图包校验通过：%s" % map_dir)
        return 0
    for problem in problems:
        sys.stderr.write("校验失败：%s\n" % problem)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
