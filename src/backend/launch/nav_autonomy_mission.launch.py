"""Launch short-segment autonomy stack for start→dig nav mission testing."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    supervisor = Node(
        package="backend",
        executable="autonomy_supervisor",
        output="screen",
        parameters=[{"allow_motion": False}],
    )
    terrain_grid = Node(
        package="backend",
        executable="local_terrain_grid",
        output="screen",
    )
    perception = Node(
        package="backend",
        executable="perception_health",
        output="screen",
    )
    nav = Node(
        package="backend",
        executable="navigation_controller",
        output="screen",
        parameters=[
            {
                "claim_cmd_vel": True,
                "use_zone_goal": True,
                "zone_goal_id": "dig",
                "goal_preference": "zone",
                "use_flag_bearing": True,
            }
        ],
    )
    mission = Node(
        package="backend",
        executable="nav_mission_executor",
        output="screen",
        parameters=[
            {
                "explore_duration_sec": 4.0,
                "head_sweep_deg": [-30, 0, 30, 0],
            }
        ],
    )

    return LaunchDescription([supervisor, perception, terrain_grid, nav, mission])
