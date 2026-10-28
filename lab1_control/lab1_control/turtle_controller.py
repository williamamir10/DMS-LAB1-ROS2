
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty
from turtlesim.srv import TeleportAbsolute
from lab1_interfaces.action import GoToPose
from lab1_interfaces.srv import SetSpeedLimits

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class TurtleController(Node):

    def __init__(self):
        super().__init__('turtle_controller')

        self.pose = None
        self.busy = False
        self.declare_parameter('max_linear_speed', 2.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.max_linear_speed = self.get_parameter(
            'max_linear_speed'
        ).value

        self.max_angular_speed = self.get_parameter(
            'max_angular_speed'
        ).value
        self.callback_group = ReentrantCallbackGroup()

        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.action_server = ActionServer(
            self,
            GoToPose,
            'go_to_pose',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )
        self.clear_client = self.create_client(
            Empty,
            '/clear'
        )

        self.teleport_client = self.create_client(
            TeleportAbsolute,
            '/turtle1/teleport_absolute'
        )

        self.reset_service = self.create_service(
            Empty,
            '/reset_turtle',
            self.reset_callback,
            callback_group=self.callback_group
        )
        self.speed_service = self.create_service(
            SetSpeedLimits,
            '/set_speed_limits',
            self.set_speed_limits_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info('GoToPose Action Server is ready!')

    def pose_callback(self, msg):
        self.pose = msg
    def reset_callback(self, request, response):
        self.get_logger().info('Resetting turtle...')

        if self.busy:
            self.get_logger().warning(
                'Cannot reset while a goal is active.'
            )
            return response

        if not self.clear_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Clear service unavailable.')
            return response

        if not self.teleport_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Teleport service unavailable.')
            return response

        self.cmd_pub.publish(Twist())

        teleport_request = TeleportAbsolute.Request()
        teleport_request.x = 5.544
        teleport_request.y = 5.544
        teleport_request.theta = 0.0

        teleport_future = self.teleport_client.call_async(
            teleport_request
        )

        while rclpy.ok() and not teleport_future.done():
            time.sleep(0.01)

        if teleport_future.exception() is not None:
            self.get_logger().error('Teleport failed.')
            return response

        clear_future = self.clear_client.call_async(
            Empty.Request()
        )

        while rclpy.ok() and not clear_future.done():
            time.sleep(0.01)

        if clear_future.exception() is not None:
            self.get_logger().error('Clear failed.')
            return response

        self.get_logger().info('Turtle reset completed!')
        return response
    def goal_callback(self, goal_request):
        self.get_logger().info(f'Goal received. busy={self.busy}')
        if self.busy:
           self.get_logger().warning('Another goal is active.')
           return GoalResponse.REJECT

        target = goal_request.target

        values = (target.x, target.y, target.yaw)

        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning('Goal rejected: invalid values.')
            return GoalResponse.REJECT

        if not (0.0 <= target.x <= 11.0 and
                0.0 <= target.y <= 11.0):
            self.get_logger().warning('Goal rejected: target out of bounds.')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.busy = True
        result = GoToPose.Result()
        target = goal_handle.request.target
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.cmd_pub.publish(Twist())
                    goal_handle.canceled()
                    result.success = False
                    result.message = 'Goal canceled.'
                    result.distance_error = 0.0
                    result.yaw_error = 0.0
                    return result
                if self.pose is None:
                    time.sleep(0.05)
                    continue
                dx = target.x - self.pose.x
                dy = target.y - self.pose.y
                distance = math.hypot(dx, dy)
                yaw_error = normalize_angle(
                    target.yaw - self.pose.theta
                )
                cmd = Twist()
                # First, move toward the target position
                if distance > 0.05:
                    desired_yaw = math.atan2(dy, dx)
                    heading_error = normalize_angle(
                        desired_yaw - self.pose.theta
                    )
                    if abs(heading_error) > 0.05:
                        cmd.linear.x = 0.0
                        cmd.angular.z = max(
                            -self.max_angular_speed,
                            min(
                                self.max_angular_speed,
                                4.0 * heading_error
                            )
                        )
                    else:
                        cmd.linear.x = min(
                            self.max_linear_speed,
                            1.5 * distance
                        )
                        cmd.angular.z = 0.0
                # Position reached: align the final orientation
                else:
                    cmd.linear.x = 0.0
                    if abs(yaw_error) > 0.05:
                        cmd.angular.z = max(
                            -self.max_angular_speed,
                            min(
                                self.max_angular_speed,
                                4.0 * yaw_error
                            )
                        )
                    else:
                        self.cmd_pub.publish(Twist())
                        goal_handle.succeed()
                        result.success = True
                        result.message = 'Target reached.'
                        result.distance_error = distance
                        result.yaw_error = yaw_error
                        return result
                self.cmd_pub.publish(cmd)
                feedback = GoToPose.Feedback()
                feedback.distance_remaining = distance
                feedback.yaw_error = yaw_error
                goal_handle.publish_feedback(feedback)
                time.sleep(0.05)
            result.success = False
            result.message = 'ROS 2 stopped.'
            result.distance_error = 0.0
            result.yaw_error = 0.0
            return result
        finally:
            try:
                self.cmd_pub.publish(Twist())
            except Exception:
                pass
            self.busy = False

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        self.action_server.destroy()
        super().destroy_node()

    def set_speed_limits_callback(self, request, response):
        if (not math.isfinite(request.max_linear_speed) or
                not math.isfinite(request.max_angular_speed) or
                request.max_linear_speed <= 0.0 or
                request.max_angular_speed <= 0.0):
            response.success = False
            response.message = 'Speed limits must be positive numbers.'
            return response

        self.max_linear_speed = request.max_linear_speed
        self.max_angular_speed = request.max_angular_speed

        response.success = True
        response.message = 'Speed limits updated successfully.'

        self.get_logger().info(
            f"New limits: linear={self.max_linear_speed}, "
            f"angular={self.max_angular_speed}"
        )
        return response
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
