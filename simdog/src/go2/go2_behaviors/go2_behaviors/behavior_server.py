#!/usr/bin/env python3

"""把现有 Go2 仿真动作暴露为可复用的 ROS 2 服务。"""

import fcntl
import threading
from typing import Dict, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .behavior_runner import BehaviorRunner


SERVICE_TO_BEHAVIOR: Dict[str, str] = {
    "stand": "stand",
    "lie": "lie",
    "hello": "hello",
    "stretch": "stretch",
    "dance": "dance",
}


class BehaviorServer(Node):
    """串行调度动作，并允许独立的 stop 服务取消当前轨迹。"""

    def __init__(self) -> None:
        super().__init__("go2_behavior_server")
        self._state_lock = threading.Lock()
        self._callback_group = ReentrantCallbackGroup()
        self._active_behavior: Optional[str] = None
        self._runner: Optional[BehaviorRunner] = None
        self._status_publisher = self.create_publisher(
            String, "/go2_behaviors/status", 10
        )
        self._services = []
        for service_name, behavior in SERVICE_TO_BEHAVIOR.items():
            service = self.create_service(
                Trigger,
                f"/go2_behaviors/{service_name}",
                lambda request, response, selected=behavior: self._execute(
                    selected, request, response
                ),
                callback_group=self._callback_group,
            )
            self._services.append(service)
        self._stop_service = self.create_service(
            Trigger,
            "/go2_behaviors/stop",
            self._stop,
            callback_group=self._callback_group,
        )
        self._publish_status("idle")

    def _publish_status(self, status: str) -> None:
        message = String()
        message.data = status
        self._status_publisher.publish(message)

    def _execute(
        self,
        behavior: str,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        with self._state_lock:
            if self._active_behavior is not None:
                response.success = False
                response.message = f"动作忙：{self._active_behavior}"
                return response
            self._active_behavior = behavior

        lock_file = open("/tmp/go2_behavior.lock", "w", encoding="utf-8")
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            with self._state_lock:
                self._active_behavior = None
            response.success = False
            response.message = "已有独立 go2_behavior 命令正在执行"
            lock_file.close()
            return response

        runner = BehaviorRunner()
        with self._state_lock:
            self._runner = runner
        self._publish_status(f"running:{behavior}")

        try:
            runner.execute(behavior)
            response.success = True
            response.message = f"{behavior} 执行完成"
            self._publish_status("idle")
        except (KeyboardInterrupt, RuntimeError) as error:
            response.success = False
            response.message = str(error) or "动作执行失败"
            self._publish_status(f"failed:{behavior}")
        finally:
            if runner.mode_acquired and not runner.keep_mode_on_exit:
                try:
                    runner.set_behavior_mode(False)
                except RuntimeError as error:
                    self.get_logger().error(f"恢复 CHAMP 失败：{error}")
            runner.destroy_node()
            with self._state_lock:
                self._runner = None
                self._active_behavior = None
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
            lock_file.close()
        return response

    def _stop(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        with self._state_lock:
            runner = self._runner
            behavior = self._active_behavior
        if runner is None:
            response.success = True
            response.message = "当前没有正在执行的动作"
            return response

        runner.request_cancel()
        response.success = True
        response.message = f"已请求取消 {behavior}"
        return response


def main() -> None:
    rclpy.init()
    server = BehaviorServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        server.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
