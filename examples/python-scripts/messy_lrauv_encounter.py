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
PPI_RANGE_M = 100.0

LRAUVS = [
    "enc_lrauv_1",
    "enc_lrauv_2",
    "enc_lrauv_3",
    "enc_lrauv_4",
    "enc_lrauv_5",
    "enc_lrauv_6",
]


# ============================================================
# HELPERS
# ============================================================

def geo_offset(north_m, east_m):
    p = GeoPoint()
    p.latitude = LAT + north_m / 111111.0
    p.longitude = LON + east_m / (111111.0 * math.cos(math.radians(LAT)))
    p.altitude = ALT
    return p


def bezier(p0, p1, p2, t):
    a = (1.0 - t) * (1.0 - t)
    b = 2.0 * (1.0 - t) * t
    c = t * t
    north = a * p0[0] + b * p1[0] + c * p2[0]
    east = a * p0[1] + b * p1[1] + c * p2[1]
    return north, east


def make_curve(start, control, end, num_points=90):
    pts = []
    for i in range(1, num_points + 1):
        t = i / num_points
        north, east = bezier(start, control, end, t)
        pts.append(geo_offset(north, east))
    return pts


# ============================================================
# ENCOUNTER ROUTES
# ============================================================
# Format:
#   (start_north, start_east),
#   (control_north, control_east),
#   (end_north, end_east)
#
# Chosen to look messy / natural, but still spaced out.
# No one passes too close to the radar.

ROUTES = {
    "enc_lrauv_1": (
        ( 82.0, -70.0),
        ( 35.0, -18.0),
        (-78.0,  48.0),
    ),
    "enc_lrauv_2": (
        (-88.0, -42.0),
        (-18.0, -12.0),
        ( 74.0,  66.0),
    ),
    "enc_lrauv_3": (
        (-76.0,  78.0),
        (-20.0,  28.0),
        ( 66.0, -72.0),
    ),
    "enc_lrauv_4": (
        ( 58.0,  86.0),
        ( 16.0,  44.0),
        (-84.0, -14.0),
    ),
    "enc_lrauv_5": (
        (  8.0, -92.0),
        (  6.0, -34.0),
        (-18.0,  86.0),
    ),
    "enc_lrauv_6": (
        (-26.0,  92.0),
        (-10.0,  36.0),
        ( 34.0, -86.0),
    ),
}


# ============================================================
# NODE
# ============================================================

class MessyEncounter(Node):

    def __init__(self):
        super().__init__(
            "messy_lrauv_encounter",
            namespace="/lotusim",
        )

        self.mas_client = ActionClient(
            self,
            MASCmdArray,
            "/lotusim/mas_cmd_array",
        )

        self.ppi_process = None

    # --------------------------------------------------------
    # spawn
    # --------------------------------------------------------

    def spawn_scene(self):
        goal = MASCmdArray.Goal()

        # Radar platform
        radar = MASCmdMsg()
        radar.cmd_type = MASCmdMsg.CREATE_CMD
        radar.model_name = "bluerov2_heavy"
        radar.vessel_name = RADAR_NAME
        radar.geo_point = geo_offset(0.0, 0.0)
        goal.cmd.append(radar)

        # LRAUVs
        for name in LRAUVS:
            start, _, _ = ROUTES[name]
            start_north, start_east = start

            cmd = MASCmdMsg()
            cmd.cmd_type = MASCmdMsg.CREATE_CMD
            cmd.model_name = "lrauv"
            cmd.vessel_name = name
            cmd.geo_point = geo_offset(start_north, start_east)

            # Push harder so they move much faster.
            cmd.sdf_string = """
            <lotus_param>
                <waypoint_follower>
                    <follower>
                        <loop>false</loop>

                        <linear_accel_limit>
                            1.50
                        </linear_accel_limit>

                        <angular_accel_limit>
                            0.25
                        </angular_accel_limit>

                        <angular_velocities_limits>
                            0.60
                        </angular_velocities_limits>

                        <range_tolerance>
                            0.35
                        </range_tolerance>
                    </follower>
                </waypoint_follower>
            </lotus_param>
            """

            goal.cmd.append(cmd)

            self.get_logger().info(
                f"{name}: start N={start_north:.1f}, E={start_east:.1f}"
            )

        self.get_logger().info("Waiting for MAS server...")
        self.mas_client.wait_for_server()

        future = self.mas_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("Spawn request rejected")

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        self.get_logger().info("Radar + 6 LRAUVs spawned.")

    # --------------------------------------------------------
    # routes
    # --------------------------------------------------------

    def start_routes(self):
        clients = []
        futures = []

        for name in LRAUVS:
            client = self.create_client(SetWaypoints, f"{name}/waypoints")

            self.get_logger().info(f"Waiting for {name}/waypoints...")
            if not client.wait_for_service(timeout_sec=10.0):
                raise RuntimeError(f"No waypoint service for {name}")

            clients.append((name, client))

        for name, client in clients:
            start, control, end = ROUTES[name]

            req = SetWaypoints.Request()
            req.path = make_curve(start, control, end, num_points=90)
            req.loop = False

            futures.append((name, client.call_async(req)))

        for name, future in futures:
            rclpy.spin_until_future_complete(self, future)

            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(f"Route rejected for {name}")

            self.get_logger().info(f"{name}: underway")

    # --------------------------------------------------------
    # ppi
    # --------------------------------------------------------

    def launch_ppi(self):
        topic = f"/{RADAR_NAME}/mas10_fmcw_radar/returns"

        self.get_logger().info(f"PPI topic = {topic}")

        self.ppi_process = subprocess.Popen([
            "ros2",
            "run",
            "fmcw_radar_sensor",
            "fmcw_radar_marine_ppi",
            "--ros-args",
            "-p", f"display_range_m:={PPI_RANGE_M}",
            "-p", f"topic:={topic}",
        ])

    def stop_ppi(self):
        if self.ppi_process is None:
            return

        if self.ppi_process.poll() is None:
            self.ppi_process.terminate()
            try:
                self.ppi_process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.ppi_process.kill()


# ============================================================
# MAIN
# ============================================================

def main():
    rclpy.init()
    node = MessyEncounter()

    try:
        print()
        print("===================================")
        print("     MESSY LRAUV ENCOUNTER DEMO")
        print("===================================")
        print()
        print("Radar: BlueROV fixed at centre")
        print("Targets: 6 LRAUVs")
        print("PPI range: 100 m")
        print("Motion: fast, smooth, one-shot")
        print()

        node.spawn_scene()

        time.sleep(3.0)

        node.launch_ppi()

        time.sleep(2.0)

        node.start_routes()

        print()
        print("Encounter running.")
        print("Ctrl+C to stop.")
        print()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        node.get_logger().error(f"DEMO ERROR: {e}")

    finally:
        node.stop_ppi()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
