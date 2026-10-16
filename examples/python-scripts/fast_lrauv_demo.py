import math
import time
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geographic_msgs.msg import GeoPoint
from lotusim_msgs.msg import MASCmd as MASCmdMsg
from lotusim_msgs.action import MASCmdArray
from lotusim_msgs.srv import SetWaypoints


LAT = 1.2605794416293148
LON = 103.7516212463379
ALT = 0.0

RADAR_NAME = "demo_radar"

PPI_RANGE_M = 100.0


def geo(north_m, east_m):
    p = GeoPoint()

    p.latitude = LAT + north_m / 111111.0

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


# ============================================================
# Predictable straight tracks
#
# start -> end
#
# Nothing random.
# Nothing loops.
# Different headings.
# ============================================================

SHIPS = [
    # ~30 m group
    (
        "lrauv_30_a",
        ( 30.0,   0.0),
        ( 30.0,  92.0),
    ),

    (
        "lrauv_30_b",
        (-30.0,   0.0),
        (-30.0, -92.0),
    ),

    # ~50 m group
    (
        "lrauv_50_a",
        (  0.0,  50.0),
        ( 88.0,  50.0),
    ),

    (
        "lrauv_50_b",
        (  0.0, -50.0),
        (-88.0, -50.0),
    ),

    # ~80 m group
    (
        "lrauv_80_a",
        ( 56.5,  56.5),
        (-65.0, -55.0),
    ),

    (
        "lrauv_80_b",
        (-56.5,  56.5),
        ( 65.0, -55.0),
    ),
]


class Demo(Node):

    def __init__(self):
        super().__init__(
            "fast_lrauv_demo",
            namespace="/lotusim",
        )

        self.mas = ActionClient(
            self,
            MASCmdArray,
            "/lotusim/mas_cmd_array",
        )

        self.ppi = None


    def spawn(self):

        goal = MASCmdArray.Goal()

        # ================================================
        # Stationary BlueROV radar
        # ================================================

        radar = MASCmdMsg()

        radar.cmd_type = MASCmdMsg.CREATE_CMD
        radar.model_name = "bluerov2_heavy"
        radar.vessel_name = RADAR_NAME

        radar.geo_point = geo(
            0.0,
            0.0,
        )

        goal.cmd.append(radar)


        # ================================================
        # LRAUV traffic
        # ================================================

        for name, start, end in SHIPS:

            cmd = MASCmdMsg()

            cmd.cmd_type = MASCmdMsg.CREATE_CMD
            cmd.model_name = "lrauv"
            cmd.vessel_name = name

            cmd.geo_point = geo(
                start[0],
                start[1],
            )

            cmd.sdf_string = """
            <lotus_param>
                <waypoint_follower>
                    <follower>

                        <loop>false</loop>

                        <linear_accel_limit>
                            5.0
                        </linear_accel_limit>

                        <angular_accel_limit>
                            1.0
                        </angular_accel_limit>

                        <angular_velocities_limits>
                            1.2
                        </angular_velocities_limits>

                        <range_tolerance>
                            0.5
                        </range_tolerance>

                    </follower>
                </waypoint_follower>
            </lotus_param>
            """

            goal.cmd.append(cmd)

        self.get_logger().info(
            "Spawning radar + 6 LRAUVs..."
        )

        self.mas.wait_for_server()

        f = self.mas.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            f,
        )

        handle = f.result()

        if not handle or not handle.accepted:
            raise RuntimeError("Spawn rejected")

        rf = handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            rf,
        )


    def start_traffic(self):

        requests = []

        # Create all clients first
        for name, start, end in SHIPS:

            client = self.create_client(
                SetWaypoints,
                f"{name}/waypoints",
            )

            if not client.wait_for_service(
                timeout_sec=10.0
            ):
                raise RuntimeError(
                    f"No waypoint service for {name}"
                )

            req = SetWaypoints.Request()

            # ONE destination.
            # This gives an actual straight trajectory.
            req.path = [
                geo(
                    end[0],
                    end[1],
                )
            ]

            req.loop = False

            requests.append(
                (
                    name,
                    client,
                    req,
                )
            )


        # Send them almost simultaneously
        futures = []

        for name, client, req in requests:

            futures.append(
                (
                    name,
                    client.call_async(req),
                )
            )


        for name, f in futures:

            rclpy.spin_until_future_complete(
                self,
                f,
            )

            result = f.result()

            if result is None or not result.success:
                raise RuntimeError(
                    f"Waypoint failed: {name}"
                )

            self.get_logger().info(
                f"{name} GO"
            )


    def launch_ppi(self):

        topic = (
            f"/{RADAR_NAME}/"
            "mas10_fmcw_radar/returns"
        )

        self.ppi = subprocess.Popen([
            "ros2",
            "run",
            "fmcw_radar_sensor",
            "fmcw_radar_marine_ppi",

            "--ros-args",

            "-p",
            "display_range_m:=100.0",

            "-p",
            f"topic:={topic}",
        ])


    def stop_ppi(self):

        if (
            self.ppi
            and self.ppi.poll() is None
        ):

            self.ppi.terminate()


def main():

    rclpy.init()

    node = Demo()

    try:

        print()
        print("============================")
        print(" FAST LRAUV RADAR DEMO")
        print("============================")
        print()
        print("BlueROV : centre")
        print("PPI     : 100 m")
        print("LRAUVs  : 6")
        print("Tracks  : straight")
        print("Loop    : NO")
        print()

        node.spawn()

        time.sleep(3.0)

        node.launch_ppi()

        time.sleep(2.0)

        node.start_traffic()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_ppi()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
