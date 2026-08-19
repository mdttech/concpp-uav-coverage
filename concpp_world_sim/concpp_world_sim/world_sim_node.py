import random
import threading

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import UInt64
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster

from concpp_msgs.msg import CellState, ExecutePath
from concpp_msgs.srv import Sense, GetInitialState

TAU = 1.0              # seconds per motion primitive, matches the paper's tau
FLIGHT_ALTITUDE = 2.0  # cosmetic z-height for the quadcopter marker/TF


class WorldSim(Node):
    def __init__(self):
        super().__init__('world_sim')

        self.declare_parameter('map_file', '')
        self.declare_parameter('seed', 42)
        self.declare_parameter('comm_range', 5.0)
        self.declare_parameter('base_xs', [0])
        self.declare_parameter('base_ys', [0])
        self.declare_parameter('robots_per_base', [4])

        map_file = self.get_parameter('map_file').get_parameter_value().string_value
        seed = self.get_parameter('seed').get_parameter_value().integer_value
        self.comm_range = self.get_parameter('comm_range').get_parameter_value().double_value
        base_xs = list(self.get_parameter('base_xs').get_parameter_value().integer_array_value)
        base_ys = list(self.get_parameter('base_ys').get_parameter_value().integer_array_value)
        robots_per_base = list(
            self.get_parameter('robots_per_base').get_parameter_value().integer_array_value)

        if not (len(base_xs) == len(base_ys) == len(robots_per_base)):
            raise RuntimeError(
                f"base_xs ({len(base_xs)}), base_ys ({len(base_ys)}), and "
                f"robots_per_base ({len(robots_per_base)}) must all list the same "
                f"number of values -- one entry per base station")

        self.bases = list(zip(base_xs, base_ys))   # static -- fixed for the whole run

        if not map_file:
            raise RuntimeError("world_sim requires the 'map_file' parameter to be set")

        self.free_mask = self._load_map(map_file)      # shape (H, W), True = free
        self.height, self.width = self.free_mask.shape
        self.get_logger().info(
            f'Loaded map {map_file}: {self.width}x{self.height}, '
            f'{int(self.free_mask.sum())} free cells')

        self.clk = 0
        self.robot_true_pose = {}     # id -> (x, y)
        self.robot_queued_path = {}   # id -> (path: list[(x, y)], t_start: int)
        self.lock = threading.Lock()

        self._deploy_robots(robots_per_base, seed)
        self.num_robots = len(self.robot_true_pose)

        # --- publishers ---
        self.clk_pub = self.create_publisher(UInt64, '/concpp/clk', 10)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        self.marker_pub = self.create_publisher(MarkerArray, '/world/robot_markers', 10)
        self.network_pub = self.create_publisher(MarkerArray, '/network_status', 10)

        self.tf_broadcaster = TransformBroadcaster(self)

        cbg = ReentrantCallbackGroup()

        # --- services (world_sim is the server; robot_node will be the client) ---
        self.create_service(Sense, '/world/sense', self._handle_sense,
                             callback_group=cbg)
        self.create_service(GetInitialState, '/world/get_initial_state',
                             self._handle_get_initial_state, callback_group=cbg)

        # --- subscriber: robot_node tells us to move a robot ---
        self.create_subscription(ExecutePath, '/world/execute_path',
                                  self._handle_execute_path, 10, callback_group=cbg)

        self._publish_map()
        self.create_timer(5.0, self._publish_map)
        self._publish_network_status()   # draws every base marker/circle immediately
        self.create_timer(5.0, self._publish_network_status)
        self.create_timer(TAU, self._tick)

    # ---------------------------------------------------------------- setup

    def _load_map(self, path):
        """Parse a Moving AI Lab .map file into a boolean free/obstacle grid."""
        with open(path) as f:
            lines = f.read().splitlines()
        h = int(lines[1].split()[1])
        w = int(lines[2].split()[1])
        grid = lines[4:4 + h]
        free = np.array([[c not in '@OTW' for c in row] for row in grid], dtype=bool)
        return free

    def _deploy_robots(self, robots_per_base, seed):
        """Deploys robots_per_base[i] robots within comm_range of bases[i], for
        every base. Distinct positions are guaranteed across the WHOLE
        deployment, not just within one base's own allocation -- two
        different bases' ranges can legitimately overlap, and a cell picked
        by one base's draw must not also be picked by another's."""
        rc_pairs = list(zip(*np.where(self.free_mask)))     # (row, col) = (y, x)
        all_free_cells = set((int(x), int(y)) for (y, x) in rc_pairs)
        rng = random.Random(seed)
        chosen = set()
        rid = 0
        for (bx, by), n in zip(self.bases, robots_per_base):
            candidates = [
                c for c in all_free_cells
                if c not in chosen
                and (c[0] - bx) ** 2 + (c[1] - by) ** 2 <= self.comm_range ** 2
            ]
            if n > len(candidates):
                raise RuntimeError(
                    f'base ({bx},{by}) needs {n} robots but only {len(candidates)} '
                    f'free cells are available within comm_range -- increase '
                    f'comm_range or reduce robots_per_base for this base')
            picked = rng.sample(candidates, n)
            for cell in picked:
                self.robot_true_pose[rid] = cell
                chosen.add(cell)
                rid += 1
        self.get_logger().info(
            f'Deployed {rid} robots across {len(self.bases)} base(s): '
            f'{self.robot_true_pose}')

    # ----------------------------------------------------------------- tick

    def _tick(self):
        self.clk += 1
        self.clk_pub.publish(UInt64(data=self.clk))

        with self.lock:
            finished = []
            for rid, (path, t_start) in self.robot_queued_path.items():
                idx = self.clk - t_start
                if idx < 0:
                    continue                        # hasn't started yet
                idx = min(idx, len(path) - 1)        # clamp: hold at goal once arrived
                self.robot_true_pose[rid] = path[idx]
                if idx >= len(path) - 1:
                    finished.append(rid)
            for rid in finished:
                del self.robot_queued_path[rid]
            pose_snapshot = dict(self.robot_true_pose)

        for rid, (x, y) in pose_snapshot.items():
            self._broadcast_tf(rid, x, y)
        self._publish_robot_markers(pose_snapshot)

    def _broadcast_tf(self, rid, x, y):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = f'robot_{rid}/base_link'
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = FLIGHT_ALTITUDE
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

    def _publish_robot_markers(self, pose_snapshot):
        arr = MarkerArray()
        for rid, (x, y) in pose_snapshot.items():
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'robots'
            m.id = rid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(x)
            m.pose.position.y = float(y)
            m.pose.position.z = FLIGHT_ALTITUDE
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.6
            m.color.a = 1.0
            m.color.r = (rid * 47 % 255) / 255.0
            m.color.g = (rid * 97 % 255) / 255.0
            m.color.b = (rid * 151 % 255) / 255.0
            arr.markers.append(m)
        self.marker_pub.publish(arr)

    # --------------------------------------------------------------- map I/O

    def _publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = 1.0
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.orientation.w = 1.0
        data = np.zeros((self.height, self.width), dtype=np.int8)
        data[~self.free_mask] = 100
        grid.data = data.flatten().tolist()
        self.map_pub.publish(grid)

    def _publish_network_status(self):
        """Draws every base station as a colored marker plus a LINE_STRIP
        circle showing its comm_range -- static, so this doesn't change
        over the course of a run, but is still republished periodically so
        a late-joining RViz subscriber always sees it (same QoS reasoning
        as /map)."""
        arr = MarkerArray()
        for idx, (x, y) in enumerate(self.bases):
            node_m = Marker()
            node_m.header.frame_id = 'map'
            node_m.header.stamp = self.get_clock().now().to_msg()
            node_m.ns = 'network_nodes'
            node_m.id = idx
            node_m.type = Marker.CYLINDER
            node_m.action = Marker.ADD
            node_m.pose.position.x = float(x)
            node_m.pose.position.y = float(y)
            node_m.pose.position.z = FLIGHT_ALTITUDE
            node_m.pose.orientation.w = 1.0
            node_m.scale.x = node_m.scale.y = node_m.scale.z = 1.2
            node_m.color.a = 1.0
            node_m.color.r = node_m.color.g = node_m.color.b = 1.0
            arr.markers.append(node_m)

            circle_m = Marker()
            circle_m.header.frame_id = 'map'
            circle_m.header.stamp = self.get_clock().now().to_msg()
            circle_m.ns = 'network_range'
            circle_m.id = idx
            circle_m.type = Marker.LINE_STRIP
            circle_m.action = Marker.ADD
            circle_m.pose.orientation.w = 1.0
            circle_m.scale.x = 0.3    # widened from 0.15 -- thin lines were
                                       # getting lost against the grid's own lines
            circle_m.color.a = 1.0    # fully opaque -- was 0.6, blending into
                                       # the map's light background
            circle_m.color.r, circle_m.color.g, circle_m.color.b = 0.15, 0.55, 1.0  # bright blue,
                                       # verified distinct from every other color already on screen
            num_points = 36
            for i in range(num_points + 1):
                theta = 2 * np.pi * i / num_points
                p = Point()
                p.x = float(x + self.comm_range * np.cos(theta))
                p.y = float(y + self.comm_range * np.sin(theta))
                p.z = 0.3   # raised from 0.02 -- was nearly overlapping the
                            # coverage-status cubes' own z-range (roughly
                            # -0.025 to 0.025), causing the two independently
                            # published MarkerArrays to z-fight for which
                            # renders on top -- not uniformly across the map,
                            # which is why only one of two bases' circles
                            # was ever visible at a time
                circle_m.points.append(p)
            arr.markers.append(circle_m)

        self.network_pub.publish(arr)

    # ------------------------------------------------------------- services

    def _handle_sense(self, request, response):
        """Emulate the paper's four rangefinders: report ground truth for the
        current cell's N/E/S/W neighbors only — never long-range, never the
        robot's own cell (it already knows that one)."""
        rid = request.robot_id
        with self.lock:
            if rid not in self.robot_true_pose:
                return response
            x, y = self.robot_true_pose[rid]
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx, ny = x + dx, y + dy
            cell = CellState()
            cell.x, cell.y = nx, ny
            in_bounds = 0 <= nx < self.width and 0 <= ny < self.height
            if in_bounds and self.free_mask[ny, nx]:
                cell.status = CellState.GOAL
            else:
                cell.status = CellState.OBSTACLE
            response.neighbors.append(cell)
        return response

    def _handle_get_initial_state(self, request, response):
        rid = request.robot_id
        with self.lock:
            if rid in self.robot_true_pose:
                x, y = self.robot_true_pose[rid]
                response.success = True
                response.state.id = rid
                response.state.x = float(x)
                response.state.y = float(y)
            else:
                response.success = False
        return response

    # ------------------------------------------------------------ commands

    def _handle_execute_path(self, msg):
        cells = [(int(round(p.pose.position.x)), int(round(p.pose.position.y)))
                 for p in msg.path.poses]
        with self.lock:
            self.robot_queued_path[msg.robot_id] = (cells, msg.t_start)
        self.get_logger().debug(
            f'robot {msg.robot_id}: queued {len(cells)}-cell path starting at t={msg.t_start}')


def main(args=None):
    rclpy.init(args=args)
    node = WorldSim()
    executor = MultiThreadedExecutor(num_threads=node.num_robots + 4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
