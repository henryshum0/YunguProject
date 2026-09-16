#!/usr/bin/env python3
"""完整搜索技能演示：搜索 + 导航 + 检测（Mock）闭环。

规划覆盖航线 → 交给导航执行 → 飞行途中开启检测 → 多帧聚类确认目标 →
返回结构化结果（是否搜到 / 目标坐标 / 航线完成度 / 终止原因）。

前置：
    1) ./utils/start_all.sh          # 仿真 + 整套机载栈（传感器/导航/规划器/检测），一条命令
    2) ros2 service call /offboard/takeoff std_srvs/srv/Trigger "{}"   # 起飞并悬停
    （也可以直接用 GUI 的 Search mission 标签页，不用跑这个脚本）

运行（在新终端）：
    cd /home/zhugb/Projects/GCI_projects/YunguProject
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export PYTHONPATH=$PWD:$PYTHONPATH
    python3 run_search_detection_demo.py            # 默认搜车辆，找到即停
    python3 run_search_detection_demo.py person     # 搜行人
    python3 run_search_detection_demo.py vehicle --full     # 飞完全程，报告全部目标
"""
import argparse
import sys

import rclpy
from rclpy.node import Node
from skills import SearchMissionSkill, SkillRuntimeConfig

ROOT = "/home/zhugb/Projects/GCI_projects/YunguProject"

# 搜索区域：4 个 ENU 角点 (x=东, y=北)。默认 40x40m 无建筑区域，
# src/detection/config/targets.yaml 里的目标都摆在这个区域内，
# 且都在起飞点视野之外（>50m），所以必须真飞过去才找得到。
SEARCH_AREA = ((20.0, -20.0), (60.0, -20.0), (60.0, 20.0), (20.0, 20.0))


def _load_truth(path: str):
    """Load the simulated ground truth, or return None when it is unavailable.

    Simulation-only: the search result itself is what was estimated, and nothing
    in the skill path knows the truth. This is here so the estimate can be read
    against it.
    """
    if not path.strip():
        return None
    try:
        from detection.config import DetectionConfig, load_targets

        detection = DetectionConfig.load(f"{ROOT}/src/detection/config/detection.yaml")
        return load_targets(path, ground_z_m=detection.world.ground_z_m)
    except Exception as exc:  # noqa: BLE001 - the truth is a convenience, never required
        print(f"(未能加载真值目标 {path}: {exc})")
        return None


def _match_truth(targets, truth):
    if truth is None:
        return [None] * len(targets)
    from detection.truth import match_to_truth

    return match_to_truth([target.position for target in targets], truth)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("classes", nargs="*", default=["vehicle"],
                        help="要搜索的对象：vehicle / person，或具体类别 car、pedestrian ...")
    parser.add_argument("--full", action="store_true",
                        help="飞完整条航线再返回（默认发现第一个目标就中止航线）")
    parser.add_argument("--confirm-within", type=float, default=20.0,
                        help="只有目标落在无人机 N 米内才算确认 [m]，避免远处少帧的粗定位就停；0 表示不限制")
    parser.add_argument("--settle", type=float, default=2.0,
                        help="确认后再多观测 N 秒取更多近距离帧求平均再停 [s]")
    parser.add_argument("--timeout", type=float, default=900.0, help="任务超时 [s]")
    parser.add_argument("--targets", default=f"{ROOT}/src/detection/config/targets.yaml",
                        help="仿真真值目标 YAML，用于在结果里对照真实坐标；置空则不对照")
    arguments = parser.parse_args()

    rclpy.init()
    node = Node("search_detection_demo")

    print(">>> 加载技能配置（含检测）...")
    config = SkillRuntimeConfig.load(
        navigation_config_dir=f"{ROOT}/src/navigation/config/offboard",
        planner_config_file=f"{ROOT}/src/search/config/yungu_planner.json",
        detection_config_file=f"{ROOT}/src/detection/config/detection.yaml",
    )

    classes = tuple(arguments.classes)
    print(f">>> 搜索区域 {SEARCH_AREA}，目标类别 {classes}，"
          f"{'飞完全程' if arguments.full else '找到即停'}")
    print(">>> 任务进行中（无人机会逐点飞行，检测同时运行）...")
    search = SearchMissionSkill(node, config=config)
    try:
        result = search.call(
            SEARCH_AREA,
            classes=classes,
            stop_on_first_detection=not arguments.full,
            mission_timeout_sec=arguments.timeout,
            confirm_within_m=(arguments.confirm_within if arguments.confirm_within > 0 else None),
            settle_sec=arguments.settle,
        )
    except Exception as exc:
        print(f"!!! 任务失败: {type(exc).__name__} -> {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print()
    print("=" * 70)
    print(f"success            : {result.success}")
    print(f"message            : {result.message}")
    print(f"found              : {result.found}")
    print(f"termination_reason : {result.termination_reason}")
    print(f"route progress     : {result.waypoints_completed}/{result.waypoints_total} "
          f"({result.route_completion * 100:.0f}%)")
    print(f"detector frames    : {result.detector_frames}")
    if result.targets:
        truth = _load_truth(arguments.targets)
        print("targets（estimate = 检测推算，truth = 仿真真值）:")
        for target, match in zip(result.targets, _match_truth(result.targets, truth)):
            x, y, _z = target.position
            line = (f"  - {target.class_id:10s} estimate ({x:7.2f}, {y:7.2f})  "
                    f"hits={target.hits:3d}  best_score={target.best_score:.2f}")
            if match is not None:
                line += (f"\n    {'':12s} truth    ({match.position[0]:7.2f}, "
                         f"{match.position[1]:7.2f})  {match.target_id} "
                         f"[{match.class_id}]  误差 {match.error_m:5.2f} m")
            elif truth is not None:
                line += "\n    " + " " * 12 + "truth    附近没有真值目标（可能是误检）"
            print(line)
    print("=" * 70)

    node.destroy_node()
    rclpy.shutdown()
    return 0 if result.success else 1


if __name__ == "__main__":
    sys.exit(main())
