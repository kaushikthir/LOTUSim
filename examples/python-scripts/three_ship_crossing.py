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

SPAWN_LATITUDE = 1.2605794416293148
SPAWN_LONGITUDE = 103.7516212463379
SPAWN_ALTITUDE = 0.0

PPI_DISPLAY_RANGE_M = 50.0


# ============================================================
# CROSSING GEOMETRY
# ============================================================

# Every vessel starts and ends exactly this far from scene centre.
START_END_RADIUS_M = 25.0

# Every vessel passes this far from the centre.
# Prevents all three from colliding in the middle.
CLOSEST_APPROACH_M = 12.0


# ============================================================
# VESSELS
# ============================================================

BLUE_NAME = "cross_b1"
COMMANDO_1_NAME = "cross_c1"
COMMANDO_2_NAME = "cross_c2"

SHIPS = [
    ("bluerov2_heavy", BLUE_NAME, 0.0),
    ("commando", COMMANDO_1_NAME, 60.0),
    ("commando", COMMANDO_2_NAME, 120.0),
]


# ============================================================
# COORDINATE HELPERS
# ============================================================

def offset_geo(
    north_m: float,
    east_m: float,
) -> GeoPoint:

    point = GeoPoint()

    point.latitude = (
        SPAWN_LATITUDE
        + north_m / 111111.0
    )

    point.longitude = (
        SPAWN_LONGITUDE
        + east_m
        / (
            111111.0
            * math.cos(
                math.radians(SPAWN_LATITUDE)
            )
        )
    )

    point.altitude = SPAWN_ALTITUDE

    return point


def make_crossing_route(angle_deg: float):
    """
    Construct a straight chord across a 25 m radius circle.

    Every route:
      - starts 25 m from centre
      - finishes 25 m from centre
      - passes 12 m from centre
    """

    phi = math.radians(angle_deg)

    half_chord = math.sqrt(
        START_END_RADIUS_M ** 2
        - CLOSEST_APPROACH_M ** 2
    )

    # Direction along route.
    east_direction = math.cos(phi)
    north_direction = math.sin(phi)

    # Perpendicular direction.
    east_normal = -math.sin(phi)
    north_normal = math.cos(phi)

    start_east = (
        CLOSEST_APPROACH_M * east_normal
        - half_chord * east_direction
    )

    start_north = (
        CLOSEST_APPROACH_M * north_normal
        - half_chord * north_direction
    )

    end_east = (
        CLOSEST_APPROACH_M * east_normal
        + half_chord * east_direction
    )

    end_north = (
        CLOSEST_APPROACH_M * north_normal
        + half_chord * north_direction
    )

    return (
        (start_north, start_east),
        (end_north, end_east),
    )


# ============================================================
# NODE
# ============================================================

class ThreeShipCrossing(Node):

    def __init__(self):

        super().__init__(
            "three_ship_crossing",
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

    def spawn_ships(self):

        goal = MASCmdArray.Goal()

        for model, name, angle_deg in SHIPS:

            start, _ = make_crossing_route(
                angle_deg
            )

            start_north, start_east = start

            command = MASCmdMsg()

            command.cmd_type = (
                MASCmdMsg.CREATE_CMD
            )

            command.model_name = model
            command.vessel_name = name

            command.geo_point = offset_geo(
                start_north,
                start_east,
            )

            # Gentle autonomous waypoint follower.
            # No circle. No loop.
            command.sdf_string = """
            <lotus_param>
                <waypoint_follower>
                    <follower>

                        <loop>false</loop>

                        <linear_accel_limit>
                            0.20
                        </linear_accel_limit>

                        <angular_accel_limit>
                            0.03
                        </angular_accel_limit>

                        <angular_velocities_limits>
                            0.20
                        </angular_velocities_limits>

                        <range_tolerance>
                            0.8
                        </range_tolerance>

                    </follower>
                </waypoint_follower>
            </lotus_param>
            """

            goal.cmd.append(command)

            self.get_logger().info(
                f"{name}: start "
                f"N={start_north:.1f} m, "
                f"E={start_east:.1f} m"
            )

        self.get_logger().info(
            "Waiting for MAS action server..."
        )

        self.mas_client.wait_for_server()

        future = (
            self.mas_client.send_goal_async(
                goal
            )
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
            "All three ships spawned."
        )


    # ========================================================
    # ONE-SHOT ROUTE
    # ========================================================

    def send_route(
        self,
        vessel_name: str,
        end_north: float,
        end_east: float,
    ):

        service_name = (
            f"{vessel_name}/waypoints"
        )

        client = self.create_client(
            SetWaypoints,
            service_name,
        )

        self.get_logger().info(
            f"Waiting for {service_name}"
        )

        if not client.wait_for_service(
            timeout_sec=10.0
        ):
            raise RuntimeError(
                f"No waypoint service for "
                f"{vessel_name}"
            )

        request = SetWaypoints.Request()

        # Only one destination.
        # Ship travels there once and stops.
        request.path = [
            offset_geo(
                end_north,
                end_east,
            )
        ]

        request.loop = False

        future = client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
        )

        response = future.result()

        if response is None:
            raise RuntimeError(
                f"No waypoint response from "
                f"{vessel_name}"
            )

        if not response.success:
            raise RuntimeError(
                f"Waypoint rejected for "
                f"{vessel_name}"
            )

        self.get_logger().info(
            f"{vessel_name}: crossing started"
        )


    def start_crossing(self):

        for _, name, angle_deg in SHIPS:

            _, end = make_crossing_route(
                angle_deg
            )

            end_north, end_east = end

            self.send_route(
                name,
                end_north,
                end_east,
            )

        self.get_logger().info(
            "All three one-shot crossings active."
        )


    # ========================================================
    # PPI
    # ========================================================

    def launch_ppi(self):

        radar_topic = (
            f"/{BLUE_NAME}/"
            "mas10_fmcw_radar/returns"
        )

        command = [
            "ros2",
            "run",
            "fmcw_radar_sensor",
            "fmcw_radar_marine_ppi",
            "--ros-args",
            "-p",
            f"display_range_m:="
            f"{PPI_DISPLAY_RANGE_M}",
            "-p",
            f"topic:={radar_topic}",
        ]

        self.get_logger().info(
            f"PPI radar topic: "
            f"{radar_topic}"
        )

        self.ppi_process = (
            subprocess.Popen(command)
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

        self.ppi_process = None


# ============================================================
# MAIN
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = ThreeShipCrossing()

    try:

        print()
        print("==============================")
        print("   THREE SHIP CROSSING DEMO")
        print("==============================")
        print()
        print("3 vessels")
        print("start radius : 25 m")
        print("end radius   : 25 m")
        print("centre miss  : 12 m")
        print("loop          : OFF")
        print()

        node.spawn_ships()

        # Allow waypoint services to come online.
        time.sleep(2.0)

        node.launch_ppi()

        time.sleep(1.5)

        node.start_crossing()

        print()
        print("All ships are crossing once.")
        print("They will stop at the far side.")
        print("Press Ctrl+C when finished.")
        print()

        rclpy.spin(node)

    except KeyboardInterrupt:
        print("\nStopping demo.")

    except Exception as exception:

        node.get_logger().error(
            f"Demo failed: {exception}"
        )

    finally:

        node.stop_ppi()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
