"""
moveit_pipeline_compat.py
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
kaair_moveit_config/config/*_planning.yaml (예: pilz_industrial_motion_
planner_planning.yaml, ompl_planning.yaml, chomp_planning.yaml) 는 Humble
시절 PlanningPipeline 파라미터 스키마로 작성되어 있다:

    planning_plugin: <str>            (단수, 문자열 1개)
    request_adapters: "a b c"         (공백으로 구분된 문자열 1개)

Jazzy 의 PlanningPipeline(generate_parameter_library 로 재작성됨, 참고:
/opt/ros/jazzy/include/moveit_ros_planning/planning_pipeline_parameters.hpp)
은 다음을 요구한다:

    planning_plugins: [<str>, ...]    (복수, string_array)
    request_adapters: [<str>, ...]    (string_array)
    response_adapters: [<str>, ...]   (string_array)

타입이 다르면 move_group/servo_node 기동 시
"rclcpp::ParameterTypeException: expected [string_array] got [string]"
로 즉시 abort 된다.

YAML 파일 자체는 Humble 호환을 위해 그대로 두고, MoveItConfigsBuilder 가
로드한 뒤의 dict(moveit_config.planning_pipelines)를 Jazzy 에서만 런타임에
보정한다 — 호출부에서 is_humble() 로 걸러서 Jazzy(및 이후 배포판)에서만
fix_planning_pipelines_for_jazzy() 를 호출한다.
"""

import os


def is_humble() -> bool:
    return os.environ.get('ROS_DISTRO') == 'humble'


def fix_planning_pipelines_for_jazzy(moveit_config):
    """moveit_config.planning_pipelines 딕셔너리를 Jazzy 스키마에 맞게 제자리에서 보정한다.

    Humble 스타일 키(planning_plugin, 문자열 request_adapters)는 그대로 두고
    Jazzy 가 실제로 읽는 키(planning_plugins, 리스트 request_adapters/
    response_adapters)를 추가/변환한다.
    """
    pipelines = moveit_config.planning_pipelines
    for name in pipelines.get('planning_pipelines', []):
        cfg = pipelines.get(name)
        if not isinstance(cfg, dict):
            continue
        if 'planning_plugin' in cfg and 'planning_plugins' not in cfg:
            cfg['planning_plugins'] = [cfg['planning_plugin']]
        for key in ('request_adapters', 'response_adapters'):
            if isinstance(cfg.get(key), str):
                split = cfg[key].split()
                if split:
                    cfg[key] = split
                else:
                    # launch_ros 는 원소 타입을 알 수 없는 빈 리스트를 파라미터
                    # 값으로 넘기지 못한다("got '()' of type 'tuple'"). 키를
                    # 아예 빼면 Jazzy PlanningPipeline 의 기본값(빈 배열)이
                    # 그대로 적용되므로 결과는 동일하다.
                    del cfg[key]
    return moveit_config
