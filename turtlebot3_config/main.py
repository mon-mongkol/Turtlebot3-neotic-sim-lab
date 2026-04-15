#!/usr/bin/env python3
"""
Subscribe to /map (OccupancyGrid) and /amcl_pose (PoseWithCovarianceStamped)
Display the map with OpenCV and draw the robot position + orientation arrow.
"""

import rospy
import cv2
import numpy as np
import math
from threading import Lock

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped


class MapVisualizer:
    # Minimum display size — the image will be scaled up so the shortest
    # side is at least this many pixels.
    DISPLAY_MIN_SIZE = 900

    def __init__(self):
        rospy.init_node("map_visualizer", anonymous=True)

        # ── Map data ──
        self.map_image = None          # BGR image of the map
        self.map_resolution = 0.0      # meters per pixel
        self.map_origin_x = 0.0        # map origin in world coords (meters)
        self.map_origin_y = 0.0
        self.map_width = 0
        self.map_height = 0
        self.map_lock = Lock()

        # ── Robot pose ──
        self.robot_x = None   # world metres
        self.robot_y = None
        self.robot_yaw = None
        self.pose_lock = Lock()

        # ── Subscribers ──
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)

        rospy.loginfo("map_visualizer started – waiting for /map and /amcl_pose ...")

    # ─────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────
    def map_callback(self, msg):
        """Convert OccupancyGrid → OpenCV BGR image."""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        # OccupancyGrid data: -1 = unknown, 0 = free, 100 = occupied
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Map values → grayscale image
        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 128    # unknown → gray
        img[data == 0] = 255     # free    → white
        img[data == 100] = 0     # occupied→ black

        # OccupancyGrid row 0 = bottom of map, so flip vertically for display
        img = cv2.flip(img, 0)

        # Convert to BGR so we can draw coloured overlays
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        with self.map_lock:
            self.map_image = img_bgr
            self.map_resolution = resolution
            self.map_origin_x = origin_x
            self.map_origin_y = origin_y
            self.map_width = width
            self.map_height = height

        rospy.loginfo_once(
            f"Map received: {width}x{height}, resolution={resolution} m/px, "
            f"origin=({origin_x:.2f}, {origin_y:.2f})"
        )

    def pose_callback(self, msg):
        """Extract x, y, yaw from AMCL pose."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

        with self.pose_lock:
            self.robot_x = pos.x
            self.robot_y = pos.y
            self.robot_yaw = yaw

        rospy.loginfo_once(
            f"AMCL pose received: x={pos.x:.2f}, y={pos.y:.2f}, yaw={math.degrees(yaw):.1f}°"
        )

    # ─────────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────────
    @staticmethod
    def quaternion_to_yaw(x, y, z, w):
        """Convert quaternion to yaw (rotation around Z)."""
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def world_to_pixel(self, wx, wy):
        """
        Convert world coordinates (metres) → pixel coordinates.
        OccupancyGrid pixel (0,0) is at the map origin.
        We flipped the image vertically, so py must be inverted.
        """
        px = int((wx - self.map_origin_x) / self.map_resolution)
        py = int((wy - self.map_origin_y) / self.map_resolution)
        # Flip Y because the image was flipped
        py = self.map_height - 1 - py
        return px, py

    # def is_inside_map(self, px, py):
    #     """Return True if pixel (px, py) is within the map image bounds."""
    #     return 0 <= px < self.map_width and 0 <= py < self.map_height

    # ─────────────────────────────────────────────
    #  Main display loop
    # ─────────────────────────────────────────────
    def run(self):
        rate = rospy.Rate(15)  # 15 Hz display refresh

        while not rospy.is_shutdown():
            with self.map_lock:
                if self.map_image is None:
                    rate.sleep()
                    continue
                display = self.map_image.copy()
                resolution = self.map_resolution

            # ── Draw robot on map ──
            with self.pose_lock:
                rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

            if rx is not None and ry is not None:
                px, py = self.world_to_pixel(rx, ry)

                # Robot radius in pixels (0.15 m ≈ TurtleBot3 burger radius)
                radius_px = max(int(0.15 / resolution), 4)

                # Draw filled circle for robot body
                cv2.circle(display, (px, py), radius_px, (0, 0, 255), -1)

                # Draw border
                cv2.circle(display, (px, py), radius_px, (0, 0, 180), 2)

                # Draw arrow showing heading direction
                if ryaw is not None:
                    arrow_len = radius_px * 2.5
                    # Note: in image coords, Y is inverted
                    ex = int(px + arrow_len * math.cos(ryaw))
                    ey = int(py - arrow_len * math.sin(ryaw))
                    cv2.arrowedLine(display, (px, py), (ex, ey), (0, 255, 0), 2, tipLength=0.35)

                # Draw coordinate text
                label = f"({rx:.2f}, {ry:.2f}) {math.degrees(ryaw):.0f} deg"
                cv2.putText(display, label, (px + radius_px + 5, py - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 100, 0), 1, cv2.LINE_AA)

            # ── Info overlay ──
            h, w = display.shape[:2]
            info = f"Map: {w}x{h}  Res: {resolution:.3f} m/px"
            cv2.putText(display, info, (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 0), 1, cv2.LINE_AA)

            if rx is not None:
                pose_txt = f"Robot: x={rx:.2f} y={ry:.2f} yaw={math.degrees(ryaw):.1f} deg"
                cv2.putText(display, pose_txt, (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1, cv2.LINE_AA)
            else:
                cv2.putText(display, "Waiting for /amcl_pose ...", (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

            # ── Show ──
            cv2.imshow("ROS Map + Robot Position", display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                rospy.loginfo("Quit key pressed – shutting down.")
                break

        cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        vis = MapVisualizer()
        vis.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
