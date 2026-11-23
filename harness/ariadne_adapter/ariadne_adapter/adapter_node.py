"""Thin adapter between ARiADNE-ROS-Planner (rl_planner) and our TB4 + slam_toolbox + Nav2 stack.

instructions.md Module 7, Step C. The adapter only converts interfaces; it never decides which
goal to pursue:

  ARiADNE needs                       we have                           adapter does
  /projected_map (OccupancyGrid)      /map from slam_toolbox            republish, optionally downsampled
  /state_estimation (Odometry, map)   TF map -> base_link               look up TF, publish as Odometry
  publishes /way_point (PointStamped) Nav2 navigate_to_pose action      forward each new waypoint as a goal

Map downsampling (`projected_map_resolution`, 0 disables it): ARiADNE was trained on a 0.4 m grid
and the authors' ROS launch feeds it a 0.4 m OctoMap projection. slam_toolbox gives 0.05 m. A coarse
cell is occupied if any fine cell is occupied, else free if any fine cell is free, else unknown.
This changes ARiADNE's input, so the setting is a recorded team decision (docs/baseline_provenance.md).
"""

import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

FREE = 0
OCCUPIED = 100
UNKNOWN = -1


def downsample_grid(data: np.ndarray, factor: int, occ_min: int) -> np.ndarray:
    """Downsample an int8 occupancy grid (rows x cols) by an integer factor.

    Pads with unknown so both sizes divide by `factor`. Returns int8 with values FREE/OCCUPIED/UNKNOWN.
    """
    if factor <= 1:
        return data
    rows, cols = data.shape
    pad_r = (-rows) % factor
    pad_c = (-cols) % factor
    padded = np.pad(data, ((0, pad_r), (0, pad_c)), constant_values=UNKNOWN)
    blocks = padded.reshape(padded.shape[0] // factor, factor, padded.shape[1] // factor, factor)
    occupied = (blocks >= occ_min).any(axis=(1, 3))
    free = (blocks == FREE).any(axis=(1, 3))
    out = np.full(occupied.shape, UNKNOWN, dtype=np.int8)
    out[free] = FREE
    out[occupied] = OCCUPIED
    return out


class AriadneAdapter(Node):
    def __init__(self):
        super().__init__('ariadne_adapter')
        self.global_frame = self.declare_parameter('global_frame', 'map').value
        self.base_frame = self.declare_parameter('robot_base_frame', 'base_link').value
        pose_rate_hz = float(self.declare_parameter('pose_rate_hz', 10.0).value)
        self.target_res = float(self.declare_parameter('projected_map_resolution', 0.4).value)
        self.occ_min = int(self.declare_parameter('occupied_threshold', 65).value)
        map_topic = self.declare_parameter('map_topic', '/map').value
        projected_topic = self.declare_parameter('projected_map_topic', '/projected_map').value
        odom_topic = self.declare_parameter('state_estimation_topic', '/state_estimation').value
        waypoint_topic = self.declare_parameter('waypoint_topic', '/way_point').value
        action_name = self.declare_parameter('navigate_to_pose_action', 'navigate_to_pose').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # slam_toolbox publishes /map reliable + transient_local.
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, map_topic, self.on_map, map_qos)
        self.projected_pub = self.create_publisher(OccupancyGrid, projected_topic, map_qos)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.create_subscription(PointStamped, waypoint_topic, self.on_waypoint, 10)
        self.nav_client = ActionClient(self, NavigateToPose, action_name)
        self.create_timer(1.0 / pose_rate_hz, self.publish_state_estimation)

        self.current_goal_xy = None
        self.warned_res = False
        self.get_logger().info(
            f'ariadne_adapter: {map_topic} -> {projected_topic} '
            f'(resolution {self.target_res or "unchanged"}), TF {self.global_frame}->{self.base_frame} '
            f'-> {odom_topic}, {waypoint_topic} -> {action_name}')

    # ---- map -----------------------------------------------------------------------------
    def on_map(self, msg: OccupancyGrid):
        if self.target_res <= 0.0:
            self.projected_pub.publish(msg)
            return
        res = msg.info.resolution
        factor = int(round(self.target_res / res))
        if factor < 1 or not math.isclose(factor * res, self.target_res, rel_tol=1e-3):
            if not self.warned_res:
                self.get_logger().warn(
                    f'projected_map_resolution {self.target_res} is not an integer multiple of '
                    f'/map resolution {res}; publishing /map unchanged')
                self.warned_res = True
            self.projected_pub.publish(msg)
            return
        grid = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        coarse = downsample_grid(grid, factor, self.occ_min)
        out = OccupancyGrid()
        out.header = msg.header
        out.info = msg.info
        out.info.resolution = res * factor
        out.info.height, out.info.width = coarse.shape
        out.data = coarse.ravel().tolist()
        self.projected_pub.publish(out)

    # ---- pose ----------------------------------------------------------------------------
    def publish_state_estimation(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.global_frame, self.base_frame, Time())
        except TransformException:
            return
        odom = Odometry()
        odom.header.stamp = tf.header.stamp
        odom.header.frame_id = self.global_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = tf.transform.translation.x
        odom.pose.pose.position.y = tf.transform.translation.y
        odom.pose.pose.position.z = tf.transform.translation.z
        odom.pose.pose.orientation = tf.transform.rotation
        self.odom_pub.publish(odom)

    # ---- waypoint -> Nav2 ----------------------------------------------------------------
    def on_waypoint(self, msg: PointStamped):
        if msg.header.frame_id and msg.header.frame_id != self.global_frame:
            self.get_logger().warn(
                f'waypoint frame {msg.header.frame_id!r} != {self.global_frame!r}; dropping it')
            return
        xy = (msg.point.x, msg.point.y)
        # ARiADNE re-publishes the same waypoint; re-sending it would restart Nav2 for no reason.
        if xy == self.current_goal_xy:
            return
        if not self.nav_client.server_is_ready():
            self.get_logger().warn('navigate_to_pose server not ready; dropping waypoint',
                                   throttle_duration_sec=5.0)
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.global_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = xy[0]
        goal.pose.pose.position.y = xy[1]
        goal.pose.pose.orientation.w = 1.0
        self.current_goal_xy = xy
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f, sent=xy: self.on_goal_response(f, sent))

    def on_goal_response(self, future, sent_xy):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'Nav2 rejected waypoint {sent_xy}')
            if self.current_goal_xy == sent_xy:
                self.current_goal_xy = None
            return
        handle.get_result_async().add_done_callback(
            lambda f, sent=sent_xy: self.on_goal_result(f, sent))

    def on_goal_result(self, future, sent_xy):
        status = future.result().status
        # If Nav2 gave up, forget the goal so the same waypoint can be sent again.
        # (Preempted goals end as ABORTED or CANCELED too; by then current_goal_xy is the new one.)
        if status != GoalStatus.STATUS_SUCCEEDED and self.current_goal_xy == sent_xy:
            self.current_goal_xy = None


def main(args=None):
    rclpy.init(args=args)
    node = AriadneAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
