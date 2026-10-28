import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_srvs.srv import Empty
from lab1_interfaces.action import GoToPose, DrawSquare


class SquareServer(Node):

    def __init__(self):
        super().__init__('square_server')

        self.callback_group = ReentrantCallbackGroup()

        self.reset_client = self.create_client(
            Empty,
            '/reset_turtle',
            callback_group=self.callback_group
        )

        self.go_to_pose_client = ActionClient(
            self,
            GoToPose,
            '/go_to_pose',
            callback_group=self.callback_group
        )

        self.square_server = ActionServer(
            self,
            DrawSquare,
            '/draw_square',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info('DrawSquare Action Server is ready!')

    def goal_callback(self, goal_request):
        side = goal_request.side_length

        if not math.isfinite(side) or not (0.1 <= side <= 5.0):
            self.get_logger().warning(
                'Rejected: side_length must be between 0.1 and 5.0.'
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def wait_for_future(self, future, timeout=30.0):
        start = time.monotonic()

        while rclpy.ok() and not future.done():
            if time.monotonic() - start > timeout:
                return False
            time.sleep(0.02)

        return future.done()

    def execute_callback(self, goal_handle):
        result = DrawSquare.Result()
        side = goal_handle.request.side_length

        result.success = False
        result.message = 'Square drawing did not complete.'
        result.sides_completed = 0

        try:
            # Step 1: Reset the turtle.
            if not self.reset_client.wait_for_service(timeout_sec=5.0):
                result.message = 'Reset service is unavailable.'
                goal_handle.abort()
                return result

            reset_future = self.reset_client.call_async(Empty.Request())

            if not self.wait_for_future(reset_future):
                result.message = 'Reset service timed out.'
                goal_handle.abort()
                return result

            if reset_future.exception() is not None:
                result.message = 'Reset service call failed.'
                goal_handle.abort()
                return result

            # Step 2: Wait for GoToPose action server.
            if not self.go_to_pose_client.wait_for_server(timeout_sec=5.0):
                result.message = 'GoToPose action server is unavailable.'
                goal_handle.abort()
                return result

            # The turtle resets to approximately the center of the screen.
            center_x = 5.544
            center_y = 5.544

            targets = [
                (center_x + side, center_y, 0.0),
                (center_x + side, center_y + side, math.pi / 2.0),
                (center_x, center_y + side, math.pi),
                (center_x, center_y, -math.pi / 2.0),
            ]

            # Exactly four GoToPose goals: one for each side.
            for index, (x, y, yaw) in enumerate(targets, start=1):

                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.message = 'Square drawing canceled.'
                    return result

                goal = GoToPose.Goal()
                goal.target.x = x
                goal.target.y = y
                goal.target.yaw = yaw

                send_future = self.go_to_pose_client.send_goal_async(goal)

                if not self.wait_for_future(send_future):
                    result.message = f'Timeout sending side {index}.'
                    goal_handle.abort()
                    return result

                pose_goal_handle = send_future.result()

                if pose_goal_handle is None or not pose_goal_handle.accepted:
                    result.message = f'GoToPose rejected side {index}.'
                    goal_handle.abort()
                    return result

                result_future = pose_goal_handle.get_result_async()

                if not self.wait_for_future(result_future):
                    result.message = f'Timeout completing side {index}.'
                    goal_handle.abort()
                    return result

                action_result = result_future.result().result

                if not action_result.success:
                    result.message = (
                        f'GoToPose failed on side {index}: '
                        f'{action_result.message}'
                    )
                    goal_handle.abort()
                    return result

                result.sides_completed = index

                feedback = DrawSquare.Feedback()
                feedback.current_side = index
                feedback.total_sides = 4
                goal_handle.publish_feedback(feedback)

            result.success = True
            result.message = 'Square completed successfully.'
            result.sides_completed = 4

            goal_handle.succeed()
            return result

        except Exception as exc:
            result.success = False
            result.message = f'Unexpected error: {exc}'
            goal_handle.abort()
            self.get_logger().error(result.message)
            return result


def main(args=None):
    rclpy.init(args=args)

    node = SquareServer()
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
