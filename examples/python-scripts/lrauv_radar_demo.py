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
    "demo_lrauv_4",
    "demo_lrauv_5",
    "demo_lrauv_6",
    "demo_lrauv_7",
    "demo_lrauv_8",
]

PPI_RANGE_M = 100.0


# ============================================================
# ORBIT
# ============================================================

ORBIT_RADIUS_M = 70.0

# 120 points = one point every 3 degrees.
# At 60 m radius, each waypoint is only ~3.1 m apart.
WAYPOINT_COUNT = 360

START_ANGLES_DEG = [
    0.0,
    45.0,
    90.0,
    135.0,
    180.0,
    225.0,
    270.0,
    315.0,
]


# ============================================================
# COORDINATES
# ============================================================

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


def position_on_circle(angle_deg):

    a = math.radians(angle_deg)

    north = ORBIT_RADIUS_M * math.cos(a)
    east = ORBIT_RADIUS_M * math.sin(a)

    return north, east


def make_orbit(start_angle_deg):

    path = []

    step_deg = 360.0 / WAYPOINT_COUNT

    # Begin at the NEXT small step because the LRAUV is
    # already spawned at start_angle_deg.
    for i in range(1, WAYPOINT_COUNT + 1):

        angle = (
            start_angle_deg
            + i * step_deg
        )

        north, east = position_on_circle(
            angle
        )

        path.append(
            geo_offset(
                north,
                east,
            )
        )

    return path


# ============================================================
# NODE
# ============================================================

class RadarDemo(Node):

    def __init__(self):

        super().__init__(
            "lrauv_radar_demo",
            namespace="/lotusim",
        )

        self.mas_client = ActionClient(
            self,
            MASCmdArray,
            "/lotusim/mas_cmd_array",
        )

        self.ppi_process = None


    # ========================================================
    # SPAWN
    # ========================================================

    def spawn_scene(self):

        goal = MASCmdArray.Goal()

        # ----------------------------------------------------
        # Stationary BlueROV radar platform
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
        # Three LRAUVs
        # ----------------------------------------------------

        for name, angle in zip(
            LRAUV_NAMES,
            START_ANGLES_DEG,
        ):

            north, east = position_on_circle(
                angle
            )

            cmd = MASCmdMsg()

            cmd.cmd_type = MASCmdMsg.CREATE_CMD
            cmd.model_name = "lrauv"
            cmd.vessel_name = name

            cmd.geo_point = geo_offset(
                north,
                east,
            )

            cmd.sdf_string = """
            <lotus_param>

                <waypoint_follower>

                    <follower>

                        <loop>false</loop>

                        <linear_accel_limit>
                            0.60
                        </linear_accel_limit>

                        <angular_accel_limit>
                            0.08
                        </angular_accel_limit>

                        <angular_velocities_limits>
                            0.30
                        </angular_velocities_limits>

                        <range_tolerance>
                            0.30
                        </range_tolerance>

                    </follower>

                </waypoint_follower>

            </lotus_param>
            """

            goal.cmd.append(cmd)

            self.get_logger().info(
                f"{name}: "
                f"radius={ORBIT_RADIUS_M:.0f} m, "
                f"angle={angle:.0f} deg"
            )

        self.get_logger().info(
            "Waiting for MAS server..."
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
                "Spawn rejected"
            )

        result_future = (
            handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future,
        )

        self.get_logger().info(
            "BlueROV + 3 LRAUVs spawned."
        )


    # ========================================================
    # SMOOTH WAYPOINT ORBITS
    # ========================================================

    def start_orbits(self):

        clients = []
        futures = []

        # Prepare every client first.
        for name in LRAUV_NAMES:

            client = self.create_client(
                SetWaypoints,
                f"{name}/waypoints",
            )

            self.get_logger().info(
                f"Waiting for "
                f"{name}/waypoints..."
            )

            if not client.wait_for_service(
                timeout_sec=10.0
            ):

                raise RuntimeError(
                    f"No waypoint service: {name}"
                )

            clients.append(client)


        # Send all three routes before waiting,
        # so they start almost simultaneously.
        for (
            client,
            name,
            start_angle
        ) in zip(
            clients,
            LRAUV_NAMES,
            START_ANGLES_DEG,
        ):

            request = SetWaypoints.Request()

            request.path = make_orbit(
                start_angle
            )

            # ONE lap only.
            request.loop = False

            futures.append(
                (
                    name,
                    client.call_async(
                        request
                    )
                )
            )


        # Check responses.
        for name, future in futures:

            rclpy.spin_until_future_complete(
                self,
                future,
            )

            response = future.result()

            if (
                response is None
                or not response.success
            ):

                raise RuntimeError(
                    f"Orbit rejected for {name}"
                )

            self.get_logger().info(
                f"{name}: orbit started"
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
            f"PPI topic = {topic}"
        )

        self.ppi_process = subprocess.Popen(
            [
                "ros2",
                "run",
                "fmcw_radar_sensor",
                "fmcw_radar_marine_ppi",
                "--ros-args",
                "-p",
                f"display_range_m:="
                f"{PPI_RANGE_M}",
                "-p",
                f"topic:={topic}",
            ]
        )


    def stop_ppi(self):

        if self.ppi_process is None:
            return

        if self.ppi_process.poll() is None:

            self.ppi_process.terminate()

            try:
                self.ppi_process.wait(
                    timeout=3.0
                )

            except subprocess.TimeoutExpired:
                self.ppi_process.kill()


# ============================================================
# MAIN
# ============================================================

def main():

    rclpy.init()

    node = RadarDemo()

    try:

        print()
        print("==============================")
        print("      MAS10 RADAR DEMO")
        print("==============================")
        print()
        print("PPI range      : 100 m")
        print("Radar          : stationary")
        print("LRAUV radius   : 60 m")
        print("LRAUV spacing  : 120 deg")
        print("Orbit          : 1 lap")
        print("Waypoints      : 120")
        print()

        node.spawn_scene()

        # Let waypoint services initialise.
        time.sleep(3.0)

        node.launch_ppi()

        time.sleep(2.0)

        node.start_orbits()

        print()
        print("Demo running.")
        print("Three LRAUVs are smoothly orbiting")
        print("the stationary BlueROV.")
        print()
        print("They will complete ONE lap and stop.")
        print("Ctrl+C to exit.")
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
