import math
import subprocess
import time

import rclpy
from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Pose
from lotusim_msgs.action import MASCmd, MASCmdArray
from lotusim_msgs.msg import MASCmd as MASCmdMsg
from rclpy.action import ActionClient
from rclpy.node import Node


# Test-scene geographic centre.
SPAWN_LATITUDE = 1.2605794416293148
SPAWN_LONGITUDE = 103.7516212463379
SPAWN_ALTITUDE = 0.0

# LOTUSim's current geographic-to-local-XY reference.
LOTUSIM_ORIGIN_LATITUDE = 1.2421
LOTUSIM_ORIGIN_LONGITUDE = 103.7198
EARTH_RADIUS_M = 6378137.0

# Four stationary commandos around the centre.
STATIC_COMMANDO_COUNT = 4
STATIC_COMMANDO_RADIUS_M = 60.0

# commando_0 moves outside the stationary ring.
MOVING_COMMANDO_NAME = "commando_0"
MOVING_COMMANDO_RADIUS_M = 80.0

# At 1 m/s, one 80 m-radius orbit takes about 8.4 minutes.
ORBIT_SPEED_MPS = 4.0
MOVE_UPDATE_PERIOD_SECONDS = 0.5

PPI_DISPLAY_RANGE_M = 100.0


def offset_geo_point(
    centre_latitude: float,
    centre_longitude: float,
    north_m: float,
    east_m: float,
) -> GeoPoint:
    """Create an approximate geographic point at a metre offset."""

    point = GeoPoint()

    point.latitude = (
        centre_latitude
        + north_m / 111111.0
    )

    point.longitude = (
        centre_longitude
        + east_m
        / (
            111111.0
            * math.cos(math.radians(centre_latitude))
        )
    )

    point.altitude = SPAWN_ALTITUDE
    return point


def lat_lon_to_xy(
    latitude: float,
    longitude: float,
) -> tuple[float, float]:
    """
    Convert latitude/longitude to LOTUSim local XY coordinates.

    X points east.
    Y points north.
    """

    d_latitude = math.radians(
        latitude - LOTUSIM_ORIGIN_LATITUDE
    )

    d_longitude = math.radians(
        longitude - LOTUSIM_ORIGIN_LONGITUDE
    )

    y = d_latitude * EARTH_RADIUS_M

    x = (
        d_longitude
        * EARTH_RADIUS_M
        * math.cos(
            math.radians(LOTUSIM_ORIGIN_LATITUDE)
        )
    )

    return x, y


def yaw_to_quaternion(pose: Pose, yaw_rad: float) -> None:
    """Set a Pose orientation from a yaw angle."""

    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(yaw_rad * 0.5)
    pose.orientation.w = math.cos(yaw_rad * 0.5)


class RadarTestScene(Node):
    def __init__(self):
        super().__init__("radar_test_scene")

        self.mas_action_client = ActionClient(
            self,
            MASCmd,
            "/lotusim/mas_cmd",
        )

        self.mas_array_action_client = ActionClient(
            self,
            MASCmdArray,
            "/lotusim/mas_cmd_array",
        )

        self.ppi_process = None
        self.orbit_timer = None
        self.orbit_start_time = None

        # Prevent multiple MOVE_CMD goals from overlapping.
        self.move_goal_in_flight = False

    def spawn_center_bluerov(self):
        command = MASCmdMsg()
        command.cmd_type = MASCmdMsg.CREATE_CMD
        command.model_name = "bluerov2_heavy"
        command.vessel_name = "bluerov2_heavy_0"

        command.geo_point = offset_geo_point(
            SPAWN_LATITUDE,
            SPAWN_LONGITUDE,
            north_m=0.0,
            east_m=0.0,
        )

        command.sdf_string = """
        <lotus_param>
            <waypoint_follower>
                <follower>
                    <loop>false</loop>
                </follower>
            </waypoint_follower>
        </lotus_param>
        """

        goal = MASCmd.Goal()
        goal.cmd = command

        self.get_logger().info(
            "Waiting for /lotusim/mas_cmd..."
        )

        self.mas_action_client.wait_for_server()

        self.get_logger().info(
            "Spawning bluerov2_heavy_0 in the centre"
        )

        return self.mas_action_client.send_goal_async(goal)

    def spawn_commandos(self):
        goal = MASCmdArray.Goal()

        # ---------------------------------------------------------
        # Moving commando: begins north of the centre at 80 metres.
        # ---------------------------------------------------------
        moving_command = MASCmdMsg()
        moving_command.cmd_type = MASCmdMsg.CREATE_CMD
        moving_command.model_name = "commando"
        moving_command.vessel_name = MOVING_COMMANDO_NAME

        moving_command.geo_point = offset_geo_point(
            SPAWN_LATITUDE,
            SPAWN_LONGITUDE,
            north_m=MOVING_COMMANDO_RADIUS_M,
            east_m=0.0,
        )

        moving_command.sdf_string = """
        <lotus_param>
            <waypoint_follower>
                <follower>
                    <loop>false</loop>
                </follower>
            </waypoint_follower>
        </lotus_param>
        """

        goal.cmd.append(moving_command)

        self.get_logger().info(
            f"Prepared moving {MOVING_COMMANDO_NAME} "
            f"at {MOVING_COMMANDO_RADIUS_M:.1f} m"
        )

        # ---------------------------------------------------------
        # Four stationary commandos on the inner 60-metre ring.
        # ---------------------------------------------------------
        for index in range(STATIC_COMMANDO_COUNT):
            angle_rad = (
                2.0
                * math.pi
                * index
                / STATIC_COMMANDO_COUNT
            )

            north_m = (
                STATIC_COMMANDO_RADIUS_M
                * math.cos(angle_rad)
            )

            east_m = (
                STATIC_COMMANDO_RADIUS_M
                * math.sin(angle_rad)
            )

            command = MASCmdMsg()
            command.cmd_type = MASCmdMsg.CREATE_CMD
            command.model_name = "commando"

            # Names: commando_1 through commando_4.
            command.vessel_name = f"commando_{index + 1}"

            command.geo_point = offset_geo_point(
                SPAWN_LATITUDE,
                SPAWN_LONGITUDE,
                north_m=north_m,
                east_m=east_m,
            )

            command.sdf_string = """
            <lotus_param>
                <waypoint_follower>
                    <follower>
                        <loop>false</loop>
                    </follower>
                </waypoint_follower>
            </lotus_param>
            """

            goal.cmd.append(command)

            self.get_logger().info(
                f"Prepared stationary commando_{index + 1} "
                f"at {STATIC_COMMANDO_RADIUS_M:.1f} m"
            )

        self.get_logger().info(
            "Waiting for /lotusim/mas_cmd_array..."
        )

        self.mas_array_action_client.wait_for_server()

        return self.mas_array_action_client.send_goal_async(
            goal
        )

    def start_orbit(self):
        self.orbit_start_time = time.monotonic()

        self.orbit_timer = self.create_timer(
            MOVE_UPDATE_PERIOD_SECONDS,
            self.update_moving_commando,
        )

        orbit_period_seconds = (
            2.0
            * math.pi
            * MOVING_COMMANDO_RADIUS_M
            / ORBIT_SPEED_MPS
        )

        self.get_logger().info(
            f"{MOVING_COMMANDO_NAME} orbit started: "
            f"radius={MOVING_COMMANDO_RADIUS_M:.1f} m, "
            f"speed={ORBIT_SPEED_MPS:.1f} m/s, "
            f"period={orbit_period_seconds:.1f} s"
        )

    def update_moving_commando(self):
        if self.orbit_start_time is None:
            return

        if self.move_goal_in_flight:
            return

        elapsed_seconds = (
            time.monotonic() - self.orbit_start_time
        )

        angular_speed_rad_s = (
            ORBIT_SPEED_MPS
            / MOVING_COMMANDO_RADIUS_M
        )

        angle_rad = (
            elapsed_seconds
            * angular_speed_rad_s
        )

        # Start north and move clockwise.
        east_m = (
            MOVING_COMMANDO_RADIUS_M
            * math.sin(angle_rad)
        )

        north_m = (
            MOVING_COMMANDO_RADIUS_M
            * math.cos(angle_rad)
        )

        geographic_position = offset_geo_point(
            SPAWN_LATITUDE,
            SPAWN_LONGITUDE,
            north_m=north_m,
            east_m=east_m,
        )

        x, y = lat_lon_to_xy(
            geographic_position.latitude,
            geographic_position.longitude,
        )

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = SPAWN_ALTITUDE

        # Tangent direction for clockwise travel:
        #
        # east velocity  =  cos(angle)
        # north velocity = -sin(angle)
        yaw_rad = -angle_rad

        yaw_to_quaternion(
            pose,
            yaw_rad,
        )

        command = MASCmdMsg()
        command.cmd_type = MASCmdMsg.MOVE_CMD
        command.vessel_name = MOVING_COMMANDO_NAME
        command.vessel_position = pose
        command.geo_point = geographic_position

        # Also populate heading for components that read it.
        command.heading = float(yaw_rad)

        goal = MASCmd.Goal()
        goal.cmd = command

        self.move_goal_in_flight = True

        future = self.mas_action_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.move_goal_response_callback
        )

    def move_goal_response_callback(self, future):
        try:
            goal_handle = future.result()

            if goal_handle is None:
                self.get_logger().error(
                    "MOVE_CMD returned no goal handle"
                )
                self.move_goal_in_flight = False
                return

            if not goal_handle.accepted:
                self.get_logger().warning(
                    f"MOVE_CMD rejected for "
                    f"{MOVING_COMMANDO_NAME}"
                )
                self.move_goal_in_flight = False
                return

            result_future = (
                goal_handle.get_result_async()
            )

            result_future.add_done_callback(
                self.move_result_callback
            )

        except Exception as exception:
            self.get_logger().error(
                f"MOVE_CMD failed: {exception}"
            )
            self.move_goal_in_flight = False

    def move_result_callback(self, future):
        try:
            wrapped_result = future.result()

            if (
                wrapped_result is not None
                and wrapped_result.status
                != GoalStatus.STATUS_SUCCEEDED
            ):
                self.get_logger().warning(
                    f"MOVE_CMD finished with status "
                    f"{wrapped_result.status}"
                )

        except Exception as exception:
            self.get_logger().error(
                f"MOVE_CMD result failed: {exception}"
            )

        finally:
            self.move_goal_in_flight = False

    def launch_ppi(self):
        command = [
            "ros2",
            "run",
            "fmcw_radar_sensor",
            "fmcw_radar_marine_ppi",
            "--ros-args",
            "-p",
            f"display_range_m:={PPI_DISPLAY_RANGE_M}",
        ]

        self.get_logger().info(
            "Launching marine PPI with "
            f"{PPI_DISPLAY_RANGE_M:.1f} m range"
        )

        self.ppi_process = subprocess.Popen(command)

    def stop_ppi(self):
        if self.ppi_process is None:
            return

        if self.ppi_process.poll() is None:
            self.get_logger().info("Stopping PPI")
            self.ppi_process.terminate()

            try:
                self.ppi_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.ppi_process.kill()
                self.ppi_process.wait()

        self.ppi_process = None


def wait_for_action(
    node: Node,
    future,
    description: str,
) -> bool:
    """Wait for acceptance and completion of a ROS action."""

    rclpy.spin_until_future_complete(
        node,
        future,
    )

    goal_handle = future.result()

    if goal_handle is None:
        node.get_logger().error(
            f"No goal handle returned for {description}"
        )
        return False

    if not goal_handle.accepted:
        node.get_logger().error(
            f"Goal rejected for {description}"
        )
        return False

    result_future = goal_handle.get_result_async()

    rclpy.spin_until_future_complete(
        node,
        result_future,
    )

    wrapped_result = result_future.result()

    if wrapped_result is None:
        node.get_logger().error(
            f"No action result returned for {description}"
        )
        return False

    if wrapped_result.status != GoalStatus.STATUS_SUCCEEDED:
        node.get_logger().error(
            f"Action failed for {description}; "
            f"status={wrapped_result.status}"
        )
        return False

    node.get_logger().info(
        f"Completed: {description}"
    )

    return True


def main(args=None):
    rclpy.init(args=args)
    node = RadarTestScene()

    try:
        centre_future = node.spawn_center_bluerov()

        if not wait_for_action(
            node,
            centre_future,
            "bluerov2_heavy_0",
        ):
            return

        commandos_future = node.spawn_commandos()

        if not wait_for_action(
            node,
            commandos_future,
            "five commandos",
        ):
            return

        node.launch_ppi()
        node.start_orbit()

        node.get_logger().info(
            "Radar scene running. "
            "commando_0 is moving around the other commandos. "
            "Press Ctrl+C to stop."
        )

        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info(
            "Stopping radar test scene"
        )

    except Exception as exception:
        node.get_logger().error(
            f"Radar test scene failed: {exception}"
        )

    finally:
        if node.orbit_timer is not None:
            node.destroy_timer(node.orbit_timer)

        node.stop_ppi()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
