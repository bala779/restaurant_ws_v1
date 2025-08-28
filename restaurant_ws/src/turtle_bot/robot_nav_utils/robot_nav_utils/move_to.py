#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from butler_bot_interfaces.srv import MoveHome, MovePose
from nav2_msgs.action import NavigateToPose
import yaml
from collections import deque


class MoveService(Node):
    def __init__(self):
        super().__init__('waypoint_service_node')

        self.callback_group = ReentrantCallbackGroup()

        with open('/home/balaji/restaurant_ws/src/turtle_bot/robot_nav_utils/config/pose.yaml', 'r') as f:
            self.waypoints = yaml.safe_load(f)

        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group
        )

        self.home_srv = self.create_service(
            MoveHome, 'move_to_home', self.move_to_home_callback, callback_group=self.callback_group
        )
        self.goto_srv = self.create_service(
            MovePose, 'move_to_pose', self.move_to_pose_callback, callback_group=self.callback_group
        )

        self.goal_queue = deque()
        self.is_goal_running = False

        print(f"MoveService: Waypoint service ready!", flush=True)

    # ---------------------------
    # Async service callbacks
    # ---------------------------
    async def move_to_home_callback(self, request, response):
        await self.enqueue_goal('home')
        response.success = True
        response.status = "Goal enqueued: home"
        return response

    async def move_to_pose_callback(self, request, response):
        await self.enqueue_goal(request.destination)
        response.success = True
        response.status = f"Goal enqueued: {request.destination}"
        return response

    # ---------------------------
    # Async goal handling
    # ---------------------------
    async def enqueue_goal(self, waypoint_name: str):
        if waypoint_name not in self.waypoints:
            print(f"MoveService: Waypoint '{waypoint_name}' not found!")
            return

        self.goal_queue.append(waypoint_name)
        print(f"MoveService: # Enqueued: {waypoint_name}",end=" | ", flush=True)
        if not self.is_goal_running:
            await self.send_next_goal()

    async def send_next_goal(self):
        if not self.goal_queue:
            self.is_goal_running = False
            return

        waypoint_name = self.goal_queue.popleft()
        self.is_goal_running = True

        wp = self.waypoints[waypoint_name]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp['position']['x']
        goal_msg.pose.pose.position.y = wp['position']['y']
        goal_msg.pose.pose.position.z = wp['position']['z']
        goal_msg.pose.pose.orientation.x = wp['orientation']['x']
        goal_msg.pose.pose.orientation.y = wp['orientation']['y']
        goal_msg.pose.pose.orientation.z = wp['orientation']['z']
        goal_msg.pose.pose.orientation.w = wp['orientation']['w']

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            print(f"MoveService: Action server not available!")
            return False

        print(f"# Sending : {waypoint_name}", end=" | ", flush=True)
        goal_handle = await self._action_client.send_goal_async(goal_msg)
        if not goal_handle.accepted:
            print(f"MoveService: Goal rejected: {waypoint_name}")
            self.is_goal_running = False
            await self.send_next_goal()
            return

        print(f"# Goal accepted: {waypoint_name}", flush=True)
        result = await goal_handle.get_result_async()
        if result.result:
            print(f"MoveService: Reached {waypoint_name}", flush=True)
        else:
            print(f"MoveService: Failed to reach {waypoint_name}")

        self.is_goal_running = False
        await self.send_next_goal()

    # Feedback
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        print(f"MoveService: Feedback: {getattr(feedback, 'distance_remaining', 'N/A')} remaining", flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = MoveService()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
