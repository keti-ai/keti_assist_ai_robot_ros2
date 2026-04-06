#!/bin/bash

LOOP_COUNT=${1:-3}

# 목표 데이터 정의
TARGET1='{trajectory: {joint_names: ["head_joint1", "head_joint2"], points: [{positions: [1.4, -0.7], time_from_start: {sec: 1, nanosec: 0}}]}}'
TARGET2='{trajectory: {joint_names: ["head_joint1", "head_joint2"], points: [{positions: [-1.4, 0.3], time_from_start: {sec: 1, nanosec: 0}}]}}'

echo "=== 총 $LOOP_COUNT 회 헤드 왕복 테스트 시작 (Humble CLI) ==="

for ((i=1; i<=LOOP_COUNT; i++))
do
    echo "--- [회차 $i/$LOOP_COUNT] Target 1 이동 시작 ---"
    # Humble에서는 기본적으로 결과를 출력할 때까지 대기합니다.
    ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "$TARGET1"
    
    echo "Target 1 완료 대기 중..."

    echo "--- [회차 $i/$LOOP_COUNT] Target 2 이동 시작 ---"
    ros2 action send_goal /head_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "$TARGET2"

    echo "Target 2 완료 대기 중..."
done

echo "🎉 모든 테스트 완료!"