import math
import subprocess
import time

import rclpy

from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import Pose
from lotusim_msgs.action import MASCmd
from lotusim_msgs.msg import MASCmd as MASCmdMsg
from rclpy.action import ActionClient
from rclpy.node import Node


# ============================================================
# SCENE
# ============================================================

SPAWN_LATITUDE = 1.2605794416293148
SPAWN_LONGITUDE = 103.7516212463379
SPAWN_ALTITUDE = 0.0

LOTUSIM_ORIGIN_LATITUDE = 1.2421
LOTUSIM_ORIGIN_LONGITUDE = 103.7198
EARTH_RADIUS_M = 6378137.0


# ============================================================
# CROSSING SHIP
# ============================================================

CROSSING_SHIP_NAME = "commando_0"

# Ship travels west -> east, passing 20 m north of the radar.
CROSSING_START_EAST_M = -80.0
CROSSING_END_EAST_M = 80.0
CROSSING_NORTH_M = 20.0

CROSSING_SPEED_MPS = 4.0
MOVE_UPDATE_PERIOD_SECONDS = 0.5

PPI_DISPLAY_RANGE_M = 100.0


# ============================================================
# COORDINATE HELPERS
# ============================================================

def offset_geo_point(
    centre_latitude: float,
    centre_longitude: float,
    north_m: float,
    east_m: float,
) -> GeoPoint:

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


def yaw_to_quaternion(
    pose: Pose,
    yaw_rad: float,
) -> None:

    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(yaw_rad * 0.5)
    pose.orientation.w = math.cos(yaw_rad * 0.5)


# ============================================================
# DEMO NODE
# ============================================================

class ShipCrossingScene(Node):

    def __init__(self):
        super().__init__("ship_crossing_scene")

        self.mas_action_client = ActionClient(
            self,
            MASCmd,
            "/lotusim/mas_cmd",
        )

        self.ppi_process = None

        self.crossing_timer = None
        self.crossing_start_time = None

        self.move_goal_in_flight = False


    # ========================================================
    # RADAR VESSEL
    # ========================================================

    def spawn_radar_vessel(self):

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
            "Spawning radar vessel at centre"
        )

        return self.mas_action_client.send_goal_async(goal)


    # ========================================================
    # CROSSING VESSEL
    # ========================================================

    def spawn_crossing_ship(self):

        command = MASCmdMsg()

        command.cmd_type = MASCmdMsg.CREATE_CMD
        command.model_name = "commando"
        command.vessel_name = CROSSING_SHIP_NAME

        command.geo_point = offset_geo_point(
            SPAWN_LATITUDE,
            SPAWN_LONGITUDE,
            north_m=CROSSING_NORTH_M,
            east_m=CROSSING_START_EAST_M,
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
            f"Spawning {CROSSING_SHIP_NAME} "
            f"{abs(CROSSING_START_EAST_M):.0f} m west, "
            f"{CROSSING_NORTH_M:.0f} m north"
        )

        return self.mas_action_client.send_goal_async(goal)


    # ========================================================
    # CROSSING MOTION
    # ========================================================

    def start_crossing(self):

        self.crossing_start_time = time.monotonic()

        self.crossing_timer = self.create_timer(
            MOVE_UPDATE_PERIOD_SECONDS,
            self.update_crossing_ship,
        )

        distance_m = (
            CROSSING_END_EAST_M
            - CROSSING_START_EAST_M
        )

        crossing_time_s = (
            distance_m
            / CROSSING_SPEED_MPS
        )

        self.get_logger().info(
            f"Crossing started: "
            f"{distance_m:.1f} m at "
            f"{CROSSING_SPEED_MPS:.1f} m/s "
            f"({crossing_time_s:.1f} s)"
        )


    def update_crossing_ship(self):

        if self.crossing_start_time is None:
            return

        if self.move_goal_in_flight:
            return

        elapsed_seconds = (
            time.monotonic()
            - self.crossing_start_time
        )

        east_m = (
            CROSSING_START_EAST_M
            + CROSSING_SPEED_MPS
            * elapsed_seconds
        )

        if east_m >= CROSSING_END_EAST_M:

            east_m = CROSSING_END_EAST_M

            if self.crossing_timer is not None:
                self.crossing_timer.cancel()

            self.get_logger().info(
                "Crossing complete"
            )

        geographic_position = offset_geo_point(
            SPAWN_LATITUDE,
            SPAWN_LONGITUDE,
            north_m=CROSSING_NORTH_M,
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

        # Travelling east.
        yaw_rad = 0.0

        yaw_to_quaternion(
            pose,
            yaw_rad,
        )

        command = MASCmdMsg()

        command.cmd_type = MASCmdMsg.MOVE_CMD
        command.vessel_name = CROSSING_SHIP_NAME
        command.vessel_position = pose
        command.geo_point = geographic_position
        command.heading = float(yaw_rad)

        goal = MASCmd.Goal()
        goal.cmd = command

        self.move_goal_in_flight = True

        future = (
            self.mas_action_client
            .send_goal_async(goal)
        )

        future.add_done_callback(
            self.move_goal_response_callback
        )


    # ========================================================
    # MOVE CALLBACKS
    # ========================================================

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
                    f"{CROSSING_SHIP_NAME}"
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


    # ========================================================
    # PPI
    # ========================================================

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
            f"Launching PPI at "
            f"{PPI_DISPLAY_RANGE_M:.0f} m"
        )

        self.ppi_process = subprocess.Popen(command)


    def stop_ppi(self):

        if self.ppi_process is None:
            return

        if self.ppi_process.poll() is None:

            self.ppi_process.terminate()

            try:
                self.ppi_process.wait(timeout=5.0)

            except subprocess.TimeoutExpired:
                self.ppi_process.kill()
                self.ppi_process.wait()

        self.ppi_process = None


# ============================================================
# ACTION HELPER
# ============================================================

def wait_for_action(
    node: Node,
    future,
    description: str,
) -> bool:

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

    result_future = (
        goal_handle.get_result_async()
    )

    rclpy.spin_until_future_complete(
        node,
        result_future,
    )

    wrapped_result = result_future.result()

    if (
        wrapped_result is None
        or wrapped_result.status
        != GoalStatus.STATUS_SUCCEEDED
    ):
        node.get_logger().error(
            f"Action failed for {description}"
        )
        return False

    node.get_logger().info(
        f"Completed: {description}"
    )

    return True


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = ShipCrossingScene()

    try:

        radar_future = (
            node.spawn_radar_vessel()
        )

        if not wait_for_action(
            node,
            radar_future,
            "radar vessel",
        ):
            return

        crossing_future = (
            node.spawn_crossing_ship()
        )

        if not wait_for_action(
            node,
            crossing_future,
            "crossing ship",
        ):
            return

        node.launch_ppi()

        # Small pause before motion so the initial return is visible.
        time.sleep(2.0)

        node.start_crossing()

        node.get_logger().info(
            "Ship crossing demo running. "
            "Press Ctrl+C to stop."
        )

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Stopping ship crossing demo"
        )

    finally:

        if node.crossing_timer is not None:
            node.destroy_timer(
                node.crossing_timer
            )

        node.stop_ppi()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
