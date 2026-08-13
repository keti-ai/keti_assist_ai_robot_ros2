from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("kaair", package_name="kaair_moveit_config")
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner"],
            default_planning_pipeline="pilz_industrial_motion_planner",
        )
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)
