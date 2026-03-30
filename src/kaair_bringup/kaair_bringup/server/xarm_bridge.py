#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from xarm_msgs.srv import SetInt16, SetInt16ById, Call
from rclpy.executors import MultiThreadedExecutor

class XArmBridge(Node):
    def __init__(self):
        super().__init__('xarm_bridge')
        
        # xArm 전용 서비스 클라이언트 정의
        self.cli_clean_error = self.create_client(Call, '/xarm/clean_error')
        self.cli_motion_enable = self.create_client(SetInt16ById, '/xarm/motion_enable')
        self.cli_set_mode = self.create_client(SetInt16, '/xarm/set_mode')
        self.cli_set_state = self.create_client(SetInt16, '/xarm/set_state')

        # 통합 서비스 서버 생성
        self.srv_ready = self.create_service(Trigger, '/xarm/init_set', self.set_ready_callback)
        
        self.get_logger().info("🚀 xArm Bridge Node 시작: /xarm_set_ready")

    async def set_ready_callback(self, request, response):
        """
        로봇 팔 준비 시퀀스: 에러클리어 -> 토크온 -> 모드설정 -> 스테이트설정
        """
        try:
            # 1. Clean Error
            await self.cli_clean_error.call_async(Call.Request())
            self.get_logger().info("✅ 1/4: 에러 클리어 완료")

            # 2. Motion Enable (ID 8: All, Data 1: Enable)
            await self.cli_motion_enable.call_async(SetInt16ById.Request(id=8, data=1))
            self.get_logger().info("✅ 2/4: 모션 활성화 완료")

            # 3. Set Mode (1: External trajectory planner servoj (position) mode)
            await self.cli_set_mode.call_async(SetInt16.Request(data=1))
            self.get_logger().info("✅ 3/4: 컨트롤 모드 설정 완료")

            # 4. Set State (0: Motion State)
            await self.cli_set_state.call_async(SetInt16.Request(data=0))
            self.get_logger().info("✅ 4/4: 로봇 스테이트 설정 완료")

            response.success = True
            response.message = "xArm Ready Sequence 성공"

        except Exception as e:
            response.success = False
            response.message = f"시퀀스 오류: {str(e)}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    
    # 서비스 내 비동기 호출을 처리하기 위해 MultiThreadedExecutor 사용
    node = XArmBridge()
    executor = MultiThreadedExecutor()
    
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()