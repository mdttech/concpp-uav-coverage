import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import UInt64
from nav_msgs.msg import Path

from concpp_msgs.action import PlanPath
from concpp_msgs.msg import CellState, RobotState, LocalView, ExecutePath
from concpp_msgs.srv import Sense, GetInitialState


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        self.declare_parameter('robot_id', 0)
        self.id = self.get_parameter('robot_id').get_parameter_value().integer_value

        self.local_view = {}     # (x, y) -> status : full accumulated belief
        self.pending_diff = {}   # (x, y) -> CellState msg : changed since last request

        self.x = None
        self.y = None
        self.current_path = []
        self.path_t_start = None
        self.following = False

        self._sense_client = self.create_client(Sense, '/world/sense')
        self._init_client = self.create_client(GetInitialState, '/world/get_initial_state')
        self._execute_pub = self.create_publisher(ExecutePath, '/world/execute_path', 10)
        self._path_pub = self.create_publisher(Path, f'/robot_{self.id}/current_path', 10)
        self._plan_client = ActionClient(self, PlanPath, 'plan_path')

        self.create_subscription(UInt64, '/concpp/clk', self._on_clk, 10)

        self._fetch_initial_state()

    # ------------------------------------------------------ retry helper

    def _call_with_retry(self, client, request_factory, on_success, timeout=3.0, what=''):
        """Sends a service request and keeps resending every `timeout`
        seconds until a response actually arrives. Needed because a service
        call can fail silently under load: the server's response times out
        server-side and the client's future simply never completes -- no
        exception, no callback firing, nothing a try/except could catch.
        Without this, a single dropped response at startup permanently
        hangs one robot -- and since cp_node starts with eta=R (needing
        every robot to check in before round 1 can ever happen), one
        silently-stuck robot freezes the entire fleet, not just itself."""
        state = {'done': False}

        def _send():
            client.call_async(request_factory()).add_done_callback(_on_response)

        def _on_response(future):
            if state['done']:
                return   # a retry's response arriving after the original, or vice versa
            state['done'] = True
            watchdog.cancel()
            on_success(future.result())

        def _watchdog():
            if state['done']:
                watchdog.cancel()
                return
            self.get_logger().warn(f'robot {self.id}: no response for {what}, retrying')
            _send()

        client.wait_for_service()
        _send()
        watchdog = self.create_timer(timeout, _watchdog)

    # ---------------------------------------------------------------- startup

    def _fetch_initial_state(self):
        self._call_with_retry(
            self._init_client,
            lambda: GetInitialState.Request(robot_id=self.id),
            self._on_initial_state,
            what='get_initial_state')

    def _on_initial_state(self, resp):
        if not resp.success:
            self.get_logger().error(f'robot {self.id}: no initial state from world_sim')
            return
        self.x = int(round(resp.state.x))
        self.y = int(round(resp.state.y))
        self._mark_covered(self.x, self.y)
        self.get_logger().info(f'robot {self.id}: starting at ({self.x}, {self.y})')
        # Sense once BEFORE the first request -- otherwise cp_node sees zero
        # GOAL cells on round 1 and wrongly concludes coverage is complete.
        self._call_with_retry(
            self._sense_client,
            lambda: Sense.Request(robot_id=self.id),
            self._on_first_sense,
            what='initial sense')

    def _on_first_sense(self, resp):
        self._merge_sense_response(resp)
        self._request_path()

    # -------------------------------------------------------- Algorithm 1 loop

    def _request_path(self):
        goal = PlanPath.Goal()
        goal.state = RobotState(id=self.id, x=float(self.x), y=float(self.y))
        goal.local_view = LocalView(cells=list(self.pending_diff.values()))
        self.pending_diff = {}

        self.get_logger().info(f'robot {self.id}: requesting path from ({self.x},{self.y})')
        self._plan_client.wait_for_server()
        self._plan_client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'robot {self.id}: goal rejected, retrying')
            self._request_path()
            return
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        result = future.result().result
        if result.active and len(result.path.poses) > 0:
            self._start_following(result.path, result.t_start)
        else:
            self._request_path()   # inactive this round -- ask again

    def _start_following(self, path_msg, t_start):
        self.current_path = [(int(round(p.pose.position.x)), int(round(p.pose.position.y)))
                              for p in path_msg.poses]
        self.path_t_start = t_start
        self.following = True

        exec_msg = ExecutePath()
        exec_msg.robot_id = self.id
        exec_msg.path = path_msg
        exec_msg.t_start = t_start
        self._execute_pub.publish(exec_msg)
        self._path_pub.publish(path_msg)
        self.get_logger().info(
            f'robot {self.id}: following {len(self.current_path)}-cell path from t={t_start}')

    # ------------------------------------------------------ clock-driven step

    def _on_clk(self, msg):
        if not self.following:
            return
        clk = msg.data
        idx = clk - self.path_t_start
        if idx < 0:
            return
        idx = min(idx, len(self.current_path) - 1)
        self.x, self.y = self.current_path[idx]
        self._mark_covered(self.x, self.y)

        reached_goal = idx >= len(self.current_path) - 1
        if reached_goal:
            self.following = False
            self.get_logger().info(f'robot {self.id}: reached ({self.x},{self.y}), re-requesting')
        # Sensing is async -- request_path must wait for THIS position's sense
        # response to actually be merged before firing, or the goal cell's
        # newly-discovered neighbors get silently dropped: pending_diff would
        # get read and cleared by _request_path() before the sense response
        # for this exact tick has had any chance to arrive and populate it.
        self._sense_and_merge(then_request=reached_goal)

    def _sense_and_merge(self, then_request=False):
        if not self._sense_client.service_is_ready():
            if then_request:
                self._request_path()   # can't sense right now -- request anyway
                                        # rather than stall forever waiting
            return
        self._call_with_retry(
            self._sense_client,
            lambda: Sense.Request(robot_id=self.id),
            lambda resp: self._on_sense_merged(resp, then_request),
            what='sense')

    def _on_sense_merged(self, resp, then_request):
        self._merge_sense_response(resp)
        if then_request:
            self._request_path()

    def _merge_sense_response(self, resp):
        changed = 0
        for cell in resp.neighbors:
            key = (cell.x, cell.y)
            known = self.local_view.get(key, CellState.UNEXPLORED)
            if known == CellState.UNEXPLORED:
                self.local_view[key] = cell.status
                self.pending_diff[key] = self._cell_msg(cell.x, cell.y, cell.status)
                changed += 1
        if changed:
            self.get_logger().debug(
                f'robot {self.id}: learned {changed} new cells, '
                f'{len(self.local_view)} known total')
        return changed

    # ---------------------------------------------------------------- helpers

    def _mark_covered(self, x, y):
        key = (x, y)
        self.local_view[key] = CellState.COVERED
        self.pending_diff[key] = self._cell_msg(x, y, CellState.COVERED)

    @staticmethod
    def _cell_msg(x, y, status):
        c = CellState()
        c.x, c.y, c.status = x, y, status
        return c


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
