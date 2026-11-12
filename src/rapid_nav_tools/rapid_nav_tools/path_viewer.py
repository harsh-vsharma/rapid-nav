"""
path_viewer: on each goal (RViz's 2D Goal Pose), ask every planner for a path and publish it.

The robot's current pose is the start (use_start false), so the planner server reads it from the TF
tree. Each planner's path goes to /planned_paths/<planner id>, one topic per planner, and every
answer is logged with its time and length, or its error. The planner ids come from the parameter
planner_ids.
"""

import sys

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rapid_nav_tools.checks import path_length
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

DEFAULT_PLANNER_IDS = ['RapidAStar', 'RapidAStarSmooth', 'RapidDijkstra', 'RapidThetaStar',
                       'NavFn', 'Smac2D', 'NavThetaStar']


class PathViewer(Node):
    """Forward each goal to every planner and publish the answers."""

    def __init__(self):
        """Declare the planner ids; create the publishers, the action client and the subscriber."""
        super().__init__('path_viewer')
        self.declare_parameter('planner_ids', DEFAULT_PLANNER_IDS)
        self.planner_ids = list(self.get_parameter('planner_ids').value)
        self.client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.path_publishers = {
            planner_id: self.create_publisher(Path, '/planned_paths/' + planner_id, 1)
            for planner_id in self.planner_ids
        }
        self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.goal = None
        self.queue = []
        self.busy = False
        self.get_logger().info('path_viewer: planners {}'.format(', '.join(self.planner_ids)))

    def on_goal(self, goal):
        """Start asking every planner in turn for this goal; a new goal restarts the round."""
        self.get_logger().info('goal at ({:.2f}, {:.2f})'.format(
            goal.pose.position.x, goal.pose.position.y))
        self.goal = goal
        self.queue = list(self.planner_ids)
        if not self.busy:
            self._send_next()

    def _send_next(self):
        """Send the goal to the next planner in the queue (the server runs one goal at a time)."""
        if not self.queue:
            self.busy = False
            return
        self.busy = True
        planner_id = self.queue.pop(0)
        request = ComputePathToPose.Goal()
        request.goal = self.goal
        request.use_start = False
        request.planner_id = planner_id
        future = self.client.send_goal_async(request)
        future.add_done_callback(self._accepted_callback(planner_id))

    def _accepted_callback(self, planner_id):
        """Return the callback that asks for the result once the goal is accepted."""
        def accepted(future):
            handle = future.result()
            if not handle.accepted:
                self.get_logger().warn('{}: goal rejected'.format(planner_id))
                self._send_next()
                return
            result_future = handle.get_result_async()
            result_future.add_done_callback(self._result_callback(planner_id))
        return accepted

    def _result_callback(self, planner_id):
        """Return the callback that publishes or logs one planner's answer, then asks the next."""
        def result(future):
            answer = future.result().result
            seconds = answer.planning_time.sec + answer.planning_time.nanosec * 1e-9
            if answer.error_code != 0:
                self.get_logger().warn('{}: error {} {} after {:.1f} ms'.format(
                    planner_id, answer.error_code, answer.error_msg, seconds * 1000.0))
            else:
                self.path_publishers[planner_id].publish(answer.path)
                points = [(p.pose.position.x, p.pose.position.y) for p in answer.path.poses]
                self.get_logger().info('{}: {:.2f} m in {:.1f} ms, {} poses'.format(
                    planner_id, path_length(points), seconds * 1000.0, len(points)))
            self._send_next()
        return result


def main(argv=None):
    """Run the path viewer until it is interrupted."""
    rclpy.init(args=sys.argv if argv is None else argv)
    node = PathViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
