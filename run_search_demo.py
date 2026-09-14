#!/usr/bin/env python3
"""搜索技能演示（命令级）：规划一个覆盖区域并交给导航执行，不等待飞行结束。

想要"边飞边检测、返回是否搜到目标"的完整搜索任务，用 run_search_detection_demo.py。

前置：./utils/start_all.sh 已启动（仿真 + 机载栈），且已发过 /takeoff_cmd，
无人机处于 IDLE 悬停。

运行（在新终端）：
    cd /home/zhugb/Projects/GCI_projects/YunguProject
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    export PYTHONPATH=$PWD:$PYTHONPATH
    python3 run_search_demo.py
"""
import rclpy
from rclpy.node import Node
from skills import SearchSkill, SkillRuntimeConfig

ROOT = "/home/zhugb/Projects/GCI_projects/YunguProject"

# 搜索区域：4 个 ENU 角点 (x=东, y=北)，单位米。改这里换区域。
# 默认是起飞点以东的 40x40m 无建筑区域，在 15m 规划高度下可完整覆盖。
SEARCH_AREA = ((20.0, -20.0), (60.0, -20.0), (60.0, 20.0), (20.0, 20.0))


def main():
    rclpy.init()
    node = Node("search_demo")

    print(">>> 加载技能配置 ...")
    config = SkillRuntimeConfig.load(
        navigation_config_dir=f"{ROOT}/src/navigation/config/offboard",
        planner_config_file=f"{ROOT}/src/search/config/yungu_planner.json",
    )

    print(f">>> 规划覆盖区域 {SEARCH_AREA} 并排队执行 ...")
    search = SearchSkill(node, config=config)
    try:
        path = search.plan_and_queue(SEARCH_AREA, timeout_sec=40.0)
    except Exception as exc:
        print(f"!!! 搜索失败: {type(exc).__name__} -> {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"=== 成功：{len(path.poses)} 个航点（frame={path.header.frame_id}，已排队）===")
    for i, ps in enumerate(path.poses):
        p = ps.pose.position
        print(f"  wp{i}: ({p.x:7.1f}, {p.y:7.1f}, {p.z:6.1f})")
    print(">>> 无人机应开始逐点飞行（看终端 3 的 offboard 日志：Waypoint #N promoted / reached）。")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
