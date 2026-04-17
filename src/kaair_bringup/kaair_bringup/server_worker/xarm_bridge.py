#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger
from xarm_msgs.srv import SetInt16, SetInt16ById, Call


class XArmBridge(Node):
    def __init__(self):
        super().__init__('xarm_bridge')

        self.cli_clean_error   = self.create_client(Call,        '/xarm/clean_error')
        self.cli_motion_enable = self.create_client(SetInt16ById, '/xarm/motion_enable')
        self.cli_set_mode      = self.create_client(SetInt16,    '/xarm/set_mode')
        self.cli_set_state     = self.create_client(SetInt16,    '/xarm/set_state')

        for cli in [self.cli_clean_error, self.cli_motion_enable,
                    self.cli_set_mode, self.cli_set_state]:
            cli.wait_for_service()
            self.get_logger().info(f"{cli.srv_name} 서비스 준비 완료")

        # 콜백은 플래그만 세우고 즉시 반환 — 블로킹 없음
        self.create_service(Trigger, '/xarm/init_set', self._on_init_set)
        self.get_logger().info("xArm Bridge Node 시작: /xarm/init_set")

        self._init_requested = False

    # ── 서비스 서버 콜백 ──────────────────────────────────────────────────────
    # request, response 시그니처 필수
    def _on_init_set(self, request, response):
        self._init_requested = True
        response.success = True
        response.message = "init_set 요청 수신"
        return response

    # ── xArm 서비스 호출 헬퍼 (future 반환) ──────────────────────────────────
    def call_clean_error(self):
        return self.cli_clean_error.call_async(Call.Request())

    def call_motion_enable(self):
        return self.cli_motion_enable.call_async(SetInt16ById.Request(id=8, data=1))

    def call_set_mode(self):
        return self.cli_set_mode.call_async(SetInt16.Request(data=1))

    def call_set_state(self):
        return self.cli_set_state.call_async(SetInt16.Request(data=0))


def main(args=None):
    rclpy.init(args=args)
    node = XArmBridge()

    # executor를 명시적으로 관리: spin_once + spin_until_future_complete를
    # 같은 executor 인스턴스로 호출해야 서비스 응답이 올바르게 처리된다.
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    steps = [
        ("1/4 Clean Error",   node.call_clean_error),
        ("2/4 Motion Enable", node.call_motion_enable),
        ("3/4 Set Mode",      node.call_set_mode),
        ("4/4 Set State",     node.call_set_state),
    ]

    try:
        while rclpy.ok():
            # ← 반드시 spin_once를 호출해야 Trigger 서비스 요청이 콜백으로 전달된다
            executor.spin_once(timeout_sec=0.1)

            if not node._init_requested:
                continue

            node._init_requested = False
            node.get_logger().info("init_set 시퀀스 시작")
            success = True

            for label, call_fn in steps:
                future = call_fn()
                # 같은 executor로 spinning하면서 응답 대기
                executor.spin_until_future_complete(future, timeout_sec=5.0)

                if not future.done() or future.result() is None:
                    node.get_logger().error(f"{label}: 실패 또는 타임아웃")
                    success = False
                    break

                node.get_logger().info(f"{label}: 완료")

            level = node.get_logger().info if success else node.get_logger().error
            level("init_set 시퀀스 " + ("완료" if success else "실패"))

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()