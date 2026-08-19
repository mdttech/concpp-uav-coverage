import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import UInt64, ColorRGBA
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray

from concpp_msgs.action import PlanPath
from concpp_msgs.msg import CellState

from .algorithms.concpp_for_par import concpp_for_par

_COVERAGE_COLORS = {
    CellState.UNEXPLORED: (0.0, 0.0, 0.0),
    CellState.OBSTACLE:   (0.85, 0.1, 0.1),
    CellState.GOAL:       (0.9, 0.85, 0.1),
    CellState.COVERED:    (0.1, 0.75, 0.2),
}


class CPNode(Node):
    def __init__(self):
        super().__init__('cp_node')

        self.declare_parameter('num_robots', 4)
        self.declare_parameter('tau', 1.0)
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)
        self.declare_parameter('comm_range', 5.0)
        self.declare_parameter('base_xs', [0])
        self.declare_parameter('base_ys', [0])
        self.R = self.get_parameter('num_robots').get_parameter_value().integer_value
        self.tau = self.get_parameter('tau').get_parameter_value().double_value
        self.width = self.get_parameter('width').get_parameter_value().integer_value
        self.height = self.get_parameter('height').get_parameter_value().integer_value
        self.comm_range = self.get_parameter('comm_range').get_parameter_value().double_value
        base_xs = list(self.get_parameter('base_xs').get_parameter_value().integer_array_value)
        base_ys = list(self.get_parameter('base_ys').get_parameter_value().integer_array_value)

        if len(base_xs) != len(base_ys):
            raise RuntimeError(
                f"base_xs and base_ys must be the same length -- got "
                f"{len(base_xs)} and {len(base_ys)}")

        self.network = list(zip(base_xs, base_ys))    # static -- fixed for the whole run,
                                                        # never grows (no relay dispatch)

        self.clk = 0
        self.I_par = set()
        self.S = {}                                             # id -> (x, y)
        self.W = {}                                              # (x,y) -> status
        self.eta = self.R
        self.T_stop = {i: 0 for i in range(self.R)}
        self.robot_path = {i: ([], 0) for i in range(self.R)}    # id -> (path, t_start)

        self.lock = threading.Lock()
        self._pending_events = {}     # id -> threading.Event
        self._pending_results = {}    # id -> PlanPath.Result
        self._round_in_progress = False
        self._coverage_complete = False

        self.create_subscription(UInt64, '/concpp/clk', self._on_clk, 10)
        self.coverage_pub = self.create_publisher(MarkerArray, '/coverage_status', 10)
        self._action_server = ActionServer(
            self, PlanPath, 'plan_path', self._on_goal,
            callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f'cp_node started, waiting for {self.R} robots across '
            f'{len(self.network)} static base(s) {self.network}')

    def _on_clk(self, msg):
        self.clk = msg.data

    # ---------------------------------------------------- receive_localview

    def _on_goal(self, goal_handle):
        """This is Algorithm 1's M_req handler AND rclpy's action execute
        callback in one. It registers the robot as a participant, then
        BLOCKS this goal's own thread (MultiThreadedExecutor gives each
        accepted goal its own thread) until ConCPP_Round has an answer for
        it -- which may come from a later round, not necessarily this one."""
        req = goal_handle.request
        rid = req.state.id
        event = threading.Event()

        with self.lock:
            if self._coverage_complete:
                result = PlanPath.Result()
                result.active = False
                goal_handle.succeed()
                return result
            self.I_par.add(rid)
            self.S[rid] = (req.state.x, req.state.y)
            self._merge_view(req.local_view)
            self._pending_events[rid] = event

        self._publish_coverage_status()
        self._check_cpp_criteria()

        event.wait()

        with self.lock:
            result = self._pending_results.pop(rid)
            self._pending_events.pop(rid, None)

        self._publish_coverage_status()
        goal_handle.succeed()
        return result

    def _merge_view(self, local_view_msg):
        for cell in local_view_msg.cells:
            key = (cell.x, cell.y)
            if self.W.get(key) == CellState.COVERED:
                continue    # COVERED is permanent -- never let a stale report downgrade it
            self.W[key] = cell.status

    # ------------------------------------------------- check_CPP_criteria

    def _goal_cells(self):
        return {cell for cell, status in self.W.items() if status == CellState.GOAL}

    def _reserved_goals(self):
        non_participants = set(range(self.R)) - self.I_par
        reserved = set()
        for rid in non_participants:
            path, _ = self.robot_path.get(rid, ([], 0))
            if path:
                reserved.add(path[-1])
        return reserved

    def _in_range_cells(self, candidate_cells):
        """Subset of candidate_cells within comm_range (Euclidean) of any
        static base in self.network. Unlike the dynamic-relay version,
        self.network never changes after startup -- so a cell that's out
        of range now stays out of range for the whole run. That's exactly
        the 'cover whatever you can' semantics: those cells stay GOAL
        forever in W, correctly never assigned, and the run still reaches
        a clean stop once nothing reachable is left."""
        in_range = set()
        for cx, cy in candidate_cells:
            for nx, ny in self.network:
                if (cx - nx) ** 2 + (cy - ny) ** 2 <= self.comm_range ** 2:
                    in_range.add((cx, cy))
                    break
        return in_range

    def _check_cpp_criteria(self):
        with self.lock:
            if self._coverage_complete or self._round_in_progress:
                return
            if len(self.I_par) < self.eta:
                return

            reserved = self._reserved_goals()
            known_goals = self._goal_cells() - reserved
            unassigned = self._in_range_cells(known_goals)

            if unassigned:
                W_snap = dict(self.W)
                I_snap, S_snap = set(self.I_par), dict(self.S)
                goals_snap = list(unassigned)
                non_par_paths_snap = {rid: self.robot_path[rid]
                                       for rid in set(range(self.R)) - I_snap}
                network_snap = list(self.network)
                self.get_logger().info(
                    f'round starting: {len(I_snap)} participants {sorted(I_snap)}, '
                    f'{len(goals_snap)} unassigned goals available')
                self.I_par.clear()
                self.S.clear()
                self.eta = 0
                self._round_in_progress = True
                threading.Thread(
                    target=self._concpp_round,
                    args=(W_snap, I_snap, S_snap, goals_snap, non_par_paths_snap, network_snap),
                    daemon=True).start()
            elif len(self.I_par) == self.R:
                # Every robot is simultaneously idle with nothing reachable
                # left to assign -- exactly "coverage complete within every
                # base's range, including overlaps" once out-of-range goals
                # (if any remain) can never become reachable, since nothing
                # in this version ever grows self.network.
                self._coverage_complete = True
                self._shutdown_complete_coverage()
            else:
                self._increment_eta(self.I_par)

    def _increment_eta(self, ids):
        candidates = [i for i in ids if self.T_stop[i] > self.clk]
        if not candidates:
            return
        t_min = min(self.T_stop[i] for i in candidates)
        self.eta += sum(1 for i in candidates if self.T_stop[i] == t_min)

    # ------------------------------------------------------- ConCPP_Round

    def _concpp_round(self, W_snap, I_snap, S_snap, goals_snap, non_par_paths_snap, network_snap):
        participant_positions = {rid: (int(round(x)), int(round(y)))
                                  for rid, (x, y) in S_snap.items()}

        sigma, t_start = concpp_for_par(
            W_snap, participant_positions, goals_snap, non_par_paths_snap,
            self.clk, self.tau, network=network_snap, comm_range=self.comm_range)

        active_summary = {rid: sigma[rid][-1] for rid in sigma}
        inactive_ids = [rid for rid in I_snap if rid not in sigma or len(sigma[rid]) <= 1]
        self.get_logger().info(f'round full paths (t_start={t_start}): {sigma}')
        self.get_logger().info(
            f'round result @ t_start={t_start}: active -> goal {active_summary}, '
            f'inactive {inactive_ids}')

        with self.lock:
            for rid in I_snap:
                path_cells = sigma.get(rid)
                is_active = path_cells is not None and len(path_cells) > 1

                result = PlanPath.Result()
                if is_active:
                    result.active = True
                    result.path = self._cells_to_path_msg(path_cells)
                    result.t_start = t_start
                    self.T_stop[rid] = t_start + len(path_cells) - 1
                    self.robot_path[rid] = (path_cells, t_start)
                else:
                    result.active = False
                    # NOTE: deliberately NOT re-adding rid to I_par/S here.
                    # The robot itself immediately re-requests once it
                    # receives this inactive result -- and THAT physical
                    # request is what correctly re-registers it via
                    # _on_goal, with a real, live event attached.

                self._pending_results[rid] = result
                event = self._pending_events.get(rid)
                if event:
                    event.set()

            for rid in set(range(self.R)) - I_snap:
                if self.T_stop[rid] <= self.clk and rid not in self.I_par:
                    self.eta += 1

            if self.eta == 0:
                self._increment_eta(list(range(self.R)))

            self._round_in_progress = False

        self._publish_coverage_status()
        self._check_cpp_criteria()

    @staticmethod
    def _cells_to_path_msg(cells):
        path = Path()
        path.header.frame_id = 'map'
        for (x, y) in cells:
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            path.poses.append(p)
        return path

    def _publish_coverage_status(self):
        """Renders the CP's OWN merged belief -- never ground truth -- color
        coded exactly like the paper's Fig. 2/3: black=unexplored,
        red=obstacle, yellow=goal, green=covered. A single CUBE_LIST marker
        is far cheaper for RViz to render than one Marker per cell, which
        matters once this scales past a handful of robots on a larger map."""
        if self.width <= 0 or self.height <= 0:
            return   # map_width/map_height weren't set -- nothing to draw

        with self.lock:
            w_snapshot = dict(self.W)

        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'coverage_status'
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = 0.95
        m.scale.z = 0.05
        m.pose.orientation.w = 1.0

        for y in range(self.height):
            for x in range(self.width):
                status = w_snapshot.get((x, y), CellState.UNEXPLORED)
                p = Point()
                p.x, p.y, p.z = float(x), float(y), 0.0
                m.points.append(p)
                r, g, b = _COVERAGE_COLORS[status]
                c = ColorRGBA()
                c.r, c.g, c.b, c.a = r, g, b, 0.9
                m.colors.append(c)

        self.coverage_pub.publish(MarkerArray(markers=[m]))

    def _shutdown_complete_coverage(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('COVERAGE COMPLETE -- all reachable free cells covered')
        self.get_logger().info('=' * 50)
        for rid, event in list(self._pending_events.items()):
            result = PlanPath.Result()
            result.active = False
            self._pending_results[rid] = result
            event.set()


def main(args=None):
    rclpy.init(args=args)
    node = CPNode()
    # MultiThreadedExecutor's default thread count comes from the CPU core
    # count, NOT from how many robots we're serving. With R robots each
    # potentially blocked simultaneously in _on_goal (one thread per
    # in-flight goal, each waiting on its own threading.Event), a
    # core-count-sized pool can be exhausted well before R robots --
    # silently starving any NEW incoming request of a thread to run on,
    # with no error, no log, nothing. Size it to the actual workload instead.
    executor = MultiThreadedExecutor(num_threads=node.R + 4)
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
