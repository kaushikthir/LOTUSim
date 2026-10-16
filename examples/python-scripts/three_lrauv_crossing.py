import math
import subprocess
import time

import rclpy

from geographic_msgs.msg import GeoPoint
from lotusim_msgs.action import MASCmdArray
from lotusim_msgs.msg import MASCmd as MASCmdMsg
from lotusim_msgs.srv import SetWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node


# ============================================================
# SCENE
# ============================================================

LAT = 1.2605794416293148
LON = 103.7516212463379
ALT = 0.0

RADAR_NAME = "demo_radar"

LRAUV_NAMES = [
    "demo_lrauv_1",
    "demo_lrauv_2",
    "demo_lrauv_3",
]

PPI_RANGE_M = 50.0


# ============================================================
# CROSSING GEOMETRY
# ============================================================

# Every LRAUV begins and ends at exactly this radius.
OUTER_RADIUS_M = 30.0

# Every route passes this far from the radar.
CLOSEST_APPROACH_M = 15.0

# Routes are rotated copies of one another.
ROUTE_ANGLES_DEG = [
    0.0,
    120.0,
    240.0,
]


def geo_offset(north_m, east_m):

    p = GeoPoint()

    p.latitude = (
        LAT
        + north_m / 111111.0
    )

    p.longitude = (
        LON
        + east_m
        / (
            111111.0
            * math.cos(math.radians(LAT))
        )
    )

    p.altitude = ALT

    return p


def make_route(angle_deg):

    angle = math.radians(angle_deg)

    half_chord = math.sqrt(
        OUTER_RADIUS_M ** 2
        - CLOSEST_APPROACH_M ** 2
    )

    # Direction of travel
    east_dir = math.cos(angle)
    north_dir = math.sin(angle)

    # Perpendicular offset
    east_normal = -math.sin(angle)
    north_normal = math.cos(angle)

    start_east = (
        CLOSEST_APPROACH_M * east_normal
        - half_chord * east_dir
    )

    start_north = (
        CLOSEST_APPROACH_M * north_normal
        - half_chord * north_dir
    )

    end_east = (
        CLOSEST_APPROACH_M * east_normal
        + half_chord * east_dir
    )

    end_north = (
        CLOSEST_APPROACH_M * north_normal
        + half_chord * north_dir
    )

    return (
        (start_north, start_east),
        (end_north, end_east),
    )


class Demo(Node):

    def __init__(self):

        super().__init__(
            "three_lrauv_crossing",
            namespace="/lotusim",
        )

        self.mas_client = ActionClient(
            self,
            MASCmdArray,
            "/lotusim/mas_cmd_array",
        )

        self.ppi = None


    # ========================================================
    # SPAWN
    # ========================================================

    def spawn_everything(self):

        goal = MASCmdArray.Goal()

        # ----------------------------------------------------
        # Stationary radar platform
        # ----------------------------------------------------

        radar = MASCmdMsg()

        radar.cmd_type = MASCmdMsg.CREATE_CMD
        radar.model_name = "bluerov2_heavy"
        radar.vessel_name = RADAR_NAME

        radar.geo_point = geo_offset(
            0.0,
            0.0,
        )

        goal.cmd.append(radar)


        # ----------------------------------------------------
        # Three identical LRAUVs
        # ----------------------------------------------------

        for name, angle in zip(
            LRAUV_NAMES,
            ROUTE_ANGLES_DEG,
        ):

            start, _ = make_route(angle)

            start_north, start_east = start

            cmd = MASCmdMsg()

            cmd.cmd_type = MASCmdMsg.CREATE_CMD
            cmd.model_name = "lrauv"
            cmd.vessel_name = name

            cmd.geo_point = geo_offset(
                start_north,
                start_east,
            )

            cmd.sdf_string = """
            <lotus_param>

                <waypoint_follower>
                    <follower>

                        <loop>false</loop>

                        <linear_accel_limit>
                            0.10
                        </linear_accel_limit>

                        <angular_accel_limit>
                            0.02
                        </angular_accel_limit>

                        <angular_velocities_limits>
                            0.12
                        </angular_velocities_limits>

                        <range_tolerance>
                            0.8
                        </range_tolerance>

                    </follower>
                </waypoint_follower>

            </lotus_param>
            """

            goal.cmd.append(cmd)

            self.get_logger().info(
                f"{name}: "
                f"start N={start_north:.1f}, "
                f"E={start_east:.1f}"
            )

        self.mas_client.wait_for_server()

        future = self.mas_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        handle = future.result()

        if handle is None or not handle.accepted:
            raise RuntimeError(
                "Spawn request rejected"
            )

        result_future = (
            handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        self.get_logger().info(
            "Radar + 3 LRAUVs spawned."
        )


    # ========================================================
    # ROUTES
    # ========================================================

    def send_destination(
        self,
        name,
        north,
        east,
    ):

        service = self.create_client(
            SetWaypoints,
            f"{name}/waypoints",
        )

        self.get_logger().info(
            f"Waiting for {name}/waypoints"
        )

        if not service.wait_for_service(
            timeout_sec=10.0
        ):
            raise RuntimeError(
                f"No waypoint service for {name}"
            )

        request = SetWaypoints.Request()

        request.path = [
            geo_offset(
                north,
                east,
            )
        ]

        # IMPORTANT:
        # cross once and STOP.
        request.loop = False

        future = service.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        response = future.result()

        if response is None:
            raise RuntimeError(
                f"No response from {name}"
            )

        if not response.success:
            raise RuntimeError(
                f"Route rejected for {name}"
            )

        self.get_logger().info(
            f"{name}: GO"
        )


    def start_crossings(self):

        for name, angle in zip(
            LRAUV_NAMES,
            ROUTE_ANGLES_DEG,
        ):

            _, end = make_route(angle)

            end_north, end_east = end

            self.send_destination(
                name,
                end_north,
                end_east,
            )

        self.get_logger().info(
            "All three LRAUVs crossing once."
        )


    # ========================================================
    # PPI
    # ========================================================

    def launch_ppi(self):

        topic = (
            f"/{RADAR_NAME}/"
            "mas10_fmcw_radar/returns"
        )

        self.get_logger().info(
            f"PPI topic: {topic}"
        )

        self.ppi = subprocess.Popen(
            [
                "ros2",
                "run",
                "fmcw_radar_sensor",
                "fmcw_radar_marine_ppi",
                "--ros-args",
                "-p",
                f"display_range_m:={PPI_RANGE_M}",
                "-p",
                f"topic:={topic}",
            ]
        )


    def stop_ppi(self):

        if self.ppi is None:
            return

        if self.ppi.poll() is None:

            self.ppi.terminate()

            try:
                self.ppi.wait(
                    timeout=3.0
                )

            except subprocess.TimeoutExpired:
                self.ppi.kill()


def main():

    rclpy.init()

    node = Demo()

    try:

        print()
        print("==============================")
        print("     3 LRAUV CROSSING DEMO")
        print("==============================")
        print()
        print("Start distance : 30 m")
        print("Closest pass   : 15 m")
        print("End distance   : 30 m")
        print("Loop           : NO")
        print()

        node.spawn_everything()

        # Give waypoint follower services time to appear.
        time.sleep(3.0)

        node.launch_ppi()

        time.sleep(2.0)

        node.start_crossings()

        print()
        print("3 LRAUVs crossing.")
        print("They cross once and stop.")
        print("Ctrl+C to end demo.")
        print()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        node.get_logger().error(
            f"DEMO ERROR: {e}"
        )

    finally:

        node.stop_ppi()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
