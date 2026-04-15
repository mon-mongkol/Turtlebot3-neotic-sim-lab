#!/usr/bin/env python3
"""
Wall Manager GUI — OpenCV-based tool for managing prohibition walls & zones.

Data source:
  • Map comes from the /map topic (nav_msgs/OccupancyGrid)
  • CRUD operations go through the ist_layouts_srv ROS service
    (web_interface_msgs/Layout)

Service commands used:
  add    — push new walls/zones
  edit   — update existing walls/zones
  erase  — delete a wall/zone by uuid+type
  list   — fetch all existing walls/zones
  clear  — remove everything

After every push the script also triggers dynamic_reconfigure to
set dynamic_polygons=true on both costmap layers.

Controls (see on-screen menu):
  Key / Button      Action
  ─────────────     ──────
  1                 Mode: Add Wall  (click two points)
  2                 Mode: Add Zone  (click ≥3 points, press Enter to finish)
  3                 Mode: Edit      (drag existing points)
  4                 Mode: Delete    (click a wall/zone to select → Del key)
  Enter             Finish current polygon (Add Zone mode)
  Delete / d        Delete selected wall/zone (from GUI + ROS)
  r                 Refresh — pull walls from ROS service
  p                 Push ALL walls to ROS (clear + re-add)
  u                 Undo last action
  Esc / q           Quit

Mouse:
  Left-click        Add point / select / start drag
  Left-release      End drag (Edit mode)
  Right-click       Cancel current drawing
"""

import cv2
import numpy as np
import math
import uuid as _uuid
import copy
from threading import Lock

# ── ROS imports ──
try:
    import rospy
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import Point as ROSPoint
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from web_interface_msgs.msg import UILayout
    from web_interface_msgs.srv import Layout, LayoutRequest
    from dynamic_reconfigure.msg import BoolParameter
    from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False
    print("[WARN] ROS packages not found — running in offline/demo mode.")


# ═══════════════════════════════════════════════════════
#  Data structures
# ═══════════════════════════════════════════════════════
WALL_TYPE = 13
ZONE_TYPE = 9


class WallItem:
    """Represents a single wall (line) or zone (polygon)."""
    def __init__(self, item_type, points, name="", uid=None):
        self.type = item_type          # WALL_TYPE or ZONE_TYPE
        self.points = list(points)     # list of (x_world, y_world)
        self.name = name or ("wall" if item_type == WALL_TYPE else "zone")
        self.uuid = uid or str(_uuid.uuid4())[:12]
        self.selected = False

    def label(self):
        kind = "Wall" if self.type == WALL_TYPE else "Zone"
        return f"{kind} [{self.uuid}]"

    def to_uilayout(self):
        """Convert to a UILayout message (requires ROS)."""
        layout = UILayout()
        layout.uuid = self.uuid
        layout.name = self.name
        layout.type = self.type
        layout.text = ""
        layout.startHandle = 0
        layout.rotateHandle = 0
        for pt in self.points:
            p = ROSPoint()
            p.x = pt[0]
            p.y = pt[1]
            p.z = 0.0
            layout.points.append(p)
        return layout


# ═══════════════════════════════════════════════════════
#  Wall Manager GUI
# ═══════════════════════════════════════════════════════
class WallManagerGUI:
    WINDOW = "Wall Manager"
    MENU_WIDTH = 340

    # Colours (BGR)
    COL_WALL         = (0, 0, 255)
    COL_WALL_SEL     = (0, 100, 255)
    COL_ZONE         = (255, 100, 0)
    COL_ZONE_SEL     = (0, 255, 255)
    COL_POINT        = (0, 255, 0)
    COL_DRAWING      = (0, 200, 200)
    COL_MENU_BG      = (30, 30, 35)
    COL_MENU_TXT     = (230, 230, 230)
    COL_MENU_HL      = (80, 180, 255)
    COL_BTN          = (55, 55, 60)
    COL_BTN_HOVER    = (75, 75, 85)
    COL_BTN_ACTIVE   = (70, 150, 240)
    COL_BTN_ACTIVE_H = (90, 170, 255)
    COL_BTN_BORDER   = (90, 90, 100)
    COL_BTN_BORDER_A = (100, 180, 255)
    COL_SECTION_BG   = (38, 38, 45)
    COL_STATUS_OK    = (0, 200, 0)
    COL_STATUS_ERR   = (0, 0, 255)
    COL_ACCENT       = (80, 180, 255)
    COL_DANGER       = (80, 80, 255)
    COL_DANGER_HOVER = (100, 100, 255)

    MODE_NONE     = 0
    MODE_ADD_WALL = 1
    MODE_ADD_ZONE = 2
    MODE_EDIT     = 3
    MODE_DELETE   = 4

    MODE_NAMES = {
        MODE_NONE:     "View",
        MODE_ADD_WALL: "Add Wall",
        MODE_ADD_ZONE: "Add Zone",
        MODE_EDIT:     "Edit",
        MODE_DELETE:   "Delete",
    }

    MIN_DISPLAY_H  = 900
    MIN_DISPLAY_W  = 1100
    SNAP_RADIUS_PX = 10
    BTN_HEIGHT     = 42
    BTN_MARGIN     = 6
    BTN_PAD_X      = 14
    BTN_RADIUS     = 8

    # ─────────────────────────────────────────────
    #  Init
    # ─────────────────────────────────────────────
    def __init__(self):
        # ── Map data (from /map topic) ──
        self.map_image = None          # BGR image (flipped for display)
        self.map_resolution = 0.05     # m/px (default until received)
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_w = 0
        self.map_h = 0
        self.map_lock = Lock()
        self.map_received = False

        # ── Walls data ──
        self.walls: list = []          # list[WallItem]
        self.walls_lock = Lock()

        # ── Drawing state ──
        self.mode = self.MODE_NONE
        self.drawing_pts = []
        self.selected_idx = -1
        self.dragging = False
        self.drag_wall_idx = -1
        self.drag_pt_idx = -1
        self.mouse_pos = (0, 0)

        # ── Display state ──
        self.scale = 1.0
        self.status_msg = "Waiting for /map topic..."
        self.status_col = self.COL_ACCENT
        self.status_time = 0
        self.hover_btn_idx = -1
        self.buttons = []
        self.history = []
        self._window_sized = False

        # ── Robot pose (from /amcl_pose) ──
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.robot_pose_received = False
        self.robot_pose_lock = Lock()

        # ── ROS init ──
        if ROS_AVAILABLE:
            rospy.init_node("wall_manager_gui", anonymous=False, disable_signals=True)
            rospy.Subscriber("/map", OccupancyGrid, self._map_callback)
            rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_pose_callback)
            rospy.loginfo("wall_manager_gui: Waiting for /map topic ...")
        else:
            print("[INFO] No ROS — showing empty canvas. Use inside ROS environment.")

    # ─────────────────────────────────────────────
    #  /map topic callback
    # ─────────────────────────────────────────────
    def _map_callback(self, msg):
        """Convert OccupancyGrid -> OpenCV BGR image."""
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y

        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        img = np.zeros((height, width), dtype=np.uint8)
        img[data == -1] = 128     # unknown -> gray
        img[data == 0]  = 255     # free    -> white
        img[data == 100] = 0      # occupied-> black

        # Row 0 = bottom of map -> flip vertically for display
        img = cv2.flip(img, 0)
        img_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        with self.map_lock:
            first_time = not self.map_received
            self.map_image = img_bgr
            self.map_resolution = resolution
            self.map_origin_x = origin_x
            self.map_origin_y = origin_y
            self.map_w = width
            self.map_h = height
            self.map_received = True

        if first_time:
            self._compute_scale()
            self._set_status(
                f"Map received: {width}x{height}, res={resolution:.3f} m/px",
                self.COL_STATUS_OK)
            # Auto-pull existing walls from ROS on first map
            self._pull_from_ros()

    # ─────────────────────────────────────────────
    #  /amcl_pose topic callback
    # ─────────────────────────────────────────────
    def _amcl_pose_callback(self, msg):
        """Extract robot position and yaw from PoseWithCovarianceStamped."""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        with self.robot_pose_lock:
            self.robot_x = pos.x
            self.robot_y = pos.y
            self.robot_yaw = yaw
            self.robot_pose_received = True

    # ─────────────────────────────────────────────
    #  Scale computation
    # ─────────────────────────────────────────────
    def _compute_scale(self):
        if self.map_h == 0 or self.map_w == 0:
            self.scale = 1.0
            return
        target_h = max(self.MIN_DISPLAY_H, 900)
        self.scale = target_h / self.map_h
        if self.map_w * self.scale < self.MIN_DISPLAY_W:
            self.scale = max(self.scale, self.MIN_DISPLAY_W / self.map_w)
        max_map_w = 1920 - self.MENU_WIDTH
        if self.map_w * self.scale > max_map_w:
            self.scale = max_map_w / self.map_w
        self.scale = max(self.scale, 1.0)

    # ─────────────────────────────────────────────
    #  Coordinate conversions
    # ─────────────────────────────────────────────
    def world_to_pixel(self, wx, wy):
        px = (wx - self.map_origin_x) / self.map_resolution
        py = (wy - self.map_origin_y) / self.map_resolution
        py = self.map_h - 1 - py
        return int(round(px)), int(round(py))

    def pixel_to_world(self, px, py):
        wx = px * self.map_resolution + self.map_origin_x
        wy = (self.map_h - 1 - py) * self.map_resolution + self.map_origin_y
        return wx, wy

    def display_to_pixel(self, dx, dy):
        px = (dx - self.MENU_WIDTH) / self.scale
        py = dy / self.scale
        return int(round(px)), int(round(py))

    def pixel_to_display(self, px, py):
        dx = int(round(px * self.scale)) + self.MENU_WIDTH
        dy = int(round(py * self.scale))
        return dx, dy

    # ─────────────────────────────────────────────
    #  Status helper
    # ─────────────────────────────────────────────
    def _set_status(self, msg, col=None):
        self.status_msg = msg
        self.status_col = col or self.COL_STATUS_OK
        self.status_time = cv2.getTickCount()

    # ─────────────────────────────────────────────
    #  Drawing helpers
    # ─────────────────────────────────────────────
    @staticmethod
    def _rounded_rect(img, pt1, pt2, color, radius, thickness=-1):
        x1, y1 = pt1
        x2, y2 = pt2
        r = min(radius, (x2 - x1) // 2, (y2 - y1) // 2)
        if thickness == -1:
            cv2.rectangle(img, (x1 + r, y1), (x2 - r, y2), color, -1)
            cv2.rectangle(img, (x1, y1 + r), (x2, y2 - r), color, -1)
            cv2.circle(img, (x1 + r, y1 + r), r, color, -1)
            cv2.circle(img, (x2 - r, y1 + r), r, color, -1)
            cv2.circle(img, (x1 + r, y2 - r), r, color, -1)
            cv2.circle(img, (x2 - r, y2 - r), r, color, -1)
        else:
            cv2.line(img, (x1 + r, y1), (x2 - r, y1), color, thickness)
            cv2.line(img, (x1 + r, y2), (x2 - r, y2), color, thickness)
            cv2.line(img, (x1, y1 + r), (x1, y2 - r), color, thickness)
            cv2.line(img, (x2, y1 + r), (x2, y2 - r), color, thickness)
            cv2.ellipse(img, (x1 + r, y1 + r), (r, r), 180, 0, 90, color, thickness)
            cv2.ellipse(img, (x2 - r, y1 + r), (r, r), 270, 0, 90, color, thickness)
            cv2.ellipse(img, (x1 + r, y2 - r), (r, r), 90, 0, 90, color, thickness)
            cv2.ellipse(img, (x2 - r, y2 - r), (r, r), 0, 0, 90, color, thickness)

    def _draw_btn(self, frame, x1, y1, x2, y2, label, icon, is_active, is_hovered,
                  col_normal, col_hover, col_active, col_active_hover,
                  border_normal, border_active, text_col=None):
        if is_active:
            bg = col_active_hover if is_hovered else col_active
            border = border_active
        else:
            bg = col_hover if is_hovered else col_normal
            border = border_normal

        r = self.BTN_RADIUS
        self._rounded_rect(frame, (x1, y1), (x2, y2), bg, r, -1)
        self._rounded_rect(frame, (x1, y1), (x2, y2), border, r, 1)

        icon_cx = x1 + 24
        icon_cy = (y1 + y2) // 2
        if is_active:
            cv2.circle(frame, (icon_cx, icon_cy), 12, (255, 255, 255), -1)
            icon_col = col_active
        else:
            cv2.circle(frame, (icon_cx, icon_cy), 12, border, 1)
            icon_col = self.COL_ACCENT if is_hovered else (180, 180, 180)

        tsz = cv2.getTextSize(icon, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)[0]
        cv2.putText(frame, icon, (icon_cx - tsz[0] // 2, icon_cy + tsz[1] // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, icon_col, 1, cv2.LINE_AA)

        tcol = text_col or ((255, 255, 255) if is_active else self.COL_MENU_TXT)
        cv2.putText(frame, label, (x1 + 44, (y1 + y2) // 2 + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, tcol, 1, cv2.LINE_AA)

    def _draw_section_header(self, frame, y, title, mw):
        cv2.putText(frame, title, (self.BTN_PAD_X, y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (130, 140, 160), 1, cv2.LINE_AA)
        tw = cv2.getTextSize(title, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)[0][0]
        cv2.line(frame, (self.BTN_PAD_X + tw + 8, y + 9),
                 (mw - self.BTN_PAD_X, y + 9), (60, 60, 70), 1)
        return y + 26

    def _scale_pt(self, pt):
        return (int(round(pt[0] * self.scale)), int(round(pt[1] * self.scale)))

    # ─────────────────────────────────────────────
    #  Draw a wall/zone on the map
    # ─────────────────────────────────────────────
    def _draw_wall_on(self, img, wall, idx):
        if not wall.points:
            return
        pts_px = [self.world_to_pixel(*p) for p in wall.points]
        pts_disp = [self._scale_pt(p) for p in pts_px]
        is_sel = wall.selected or idx == self.selected_idx

        if wall.type == WALL_TYPE:
            col = self.COL_WALL_SEL if is_sel else self.COL_WALL
            thickness = 3 if is_sel else 2
            if len(pts_disp) >= 2:
                cv2.line(img, pts_disp[0], pts_disp[1], col, thickness, cv2.LINE_AA)
        else:
            col = self.COL_ZONE_SEL if is_sel else self.COL_ZONE
            thickness = 3 if is_sel else 2
            np_pts = np.array(pts_disp, dtype=np.int32)
            if is_sel:
                overlay = img.copy()
                cv2.fillPoly(overlay, [np_pts], (*col[:3],))
                cv2.addWeighted(overlay, 0.25, img, 0.75, 0, img)
            cv2.polylines(img, [np_pts], isClosed=True, color=col,
                          thickness=thickness, lineType=cv2.LINE_AA)

        for dp in pts_disp:
            cv2.circle(img, dp, 4, self.COL_POINT, -1)
            if is_sel:
                cv2.circle(img, dp, 6, (255, 255, 255), 1)

        cx = int(np.mean([p[0] for p in pts_disp]))
        cy = int(np.mean([p[1] for p in pts_disp])) - 10
        cv2.putText(img, wall.label(), (cx - 30, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1, cv2.LINE_AA)

    # ─────────────────────────────────────────────
    #  Draw robot position & heading on the map
    # ─────────────────────────────────────────────
    def _draw_robot_on(self, img):
        """Draw an arrow at the robot's AMCL pose on the scaled map image."""
        with self.robot_pose_lock:
            if not self.robot_pose_received:
                return
            rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw

        # World → pixel → scaled display
        ppx, ppy = self.world_to_pixel(rx, ry)
        dx, dy = self._scale_pt((ppx, ppy))

        # Arrow length in display pixels
        arrow_len = int(20 * self.scale)
        arrow_len = max(arrow_len, 18)

        # Tip of the arrow (direction robot is facing)
        # Note: pixel Y is inverted compared to world Y
        tip_x = int(dx + arrow_len * math.cos(-ryaw))
        tip_y = int(dy + arrow_len * math.sin(-ryaw))

        # Robot colour: cyan
        col_robot = (255, 200, 0)       # bright cyan-ish
        col_robot_ring = (255, 255, 0)

        # Draw filled circle at robot centre
        cv2.circle(img, (dx, dy), 8, col_robot, -1, cv2.LINE_AA)
        cv2.circle(img, (dx, dy), 10, col_robot_ring, 2, cv2.LINE_AA)

        # Draw heading arrow
        cv2.arrowedLine(img, (dx, dy), (tip_x, tip_y),
                        col_robot_ring, 2, cv2.LINE_AA, tipLength=0.35)

    # ─────────────────────────────────────────────
    #  Build full display frame
    # ─────────────────────────────────────────────
    def _draw_frame(self):
        with self.map_lock:
            if not self.map_received or self.map_image is None:
                # Show a placeholder waiting screen
                frame = np.zeros((700, self.MENU_WIDTH + 800, 3), dtype=np.uint8)
                frame[:, :self.MENU_WIDTH] = self.COL_MENU_BG
                self._draw_menu(frame, 700)
                cv2.putText(frame, "Waiting for /map topic...",
                            (self.MENU_WIDTH + 200, 350),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, self.COL_ACCENT, 2, cv2.LINE_AA)
                cv2.putText(frame, "Make sure roscore + navigation are running",
                            (self.MENU_WIDTH + 140, 400),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (150, 150, 160), 1, cv2.LINE_AA)
                return frame

            display_map = self.map_image.copy()
            resolution = self.map_resolution

        # Scale map
        disp_w = int(self.map_w * self.scale)
        disp_h = int(self.map_h * self.scale)
        map_scaled = cv2.resize(display_map, (disp_w, disp_h),
                                interpolation=cv2.INTER_NEAREST)

        # Draw walls/zones on the scaled map
        with self.walls_lock:
            for idx, wall in enumerate(self.walls):
                self._draw_wall_on(map_scaled, wall, idx)

        # Draw robot position & heading from /amcl_pose
        self._draw_robot_on(map_scaled)

        # Draw in-progress points
        if self.drawing_pts:
            pts_px = [self.world_to_pixel(*p) for p in self.drawing_pts]
            pts_disp = [self._scale_pt(p) for p in pts_px]
            for i, dp in enumerate(pts_disp):
                cv2.circle(map_scaled, dp, 5, self.COL_DRAWING, -1)
                if i > 0:
                    cv2.line(map_scaled, pts_disp[i - 1], dp, self.COL_DRAWING, 2)
            # Rubber-band line to cursor
            if self.mouse_pos:
                mx, my = self.mouse_pos
                mx -= self.MENU_WIDTH
                if mx >= 0:
                    cv2.line(map_scaled, pts_disp[-1], (mx, my),
                             self.COL_DRAWING, 1, cv2.LINE_AA)

        # Compose frame (menu | map)
        frame_h = disp_h
        frame_w = self.MENU_WIDTH + disp_w
        frame = np.zeros((frame_h, frame_w, 3), dtype=np.uint8)
        frame[:, self.MENU_WIDTH:] = map_scaled

        # Menu panel
        self._draw_menu(frame, frame_h)

        # Status bar
        if self.status_msg:
            elapsed = (cv2.getTickCount() - self.status_time) / cv2.getTickFrequency()
            if elapsed < 6.0:
                bar_h = 32
                bar_y = frame_h - bar_h
                overlay = frame.copy()
                cv2.rectangle(overlay, (self.MENU_WIDTH, bar_y),
                              (frame_w, frame_h), (30, 30, 35), -1)
                cv2.addWeighted(overlay, 0.85, frame, 0.15, 0, frame)
                cv2.putText(frame, self.status_msg,
                            (self.MENU_WIDTH + 14, frame_h - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                            self.status_col, 1, cv2.LINE_AA)

        return frame

    # ─────────────────────────────────────────────
    #  Menu panel
    # ─────────────────────────────────────────────
    def _draw_menu(self, frame, h):
        mw = self.MENU_WIDTH
        px = self.BTN_PAD_X
        bh = self.BTN_HEIGHT
        gap = self.BTN_MARGIN

        frame[:, :mw] = self.COL_MENU_BG
        cv2.line(frame, (mw - 1, 0), (mw - 1, h), (60, 60, 70), 1)

        y = 12
        self.buttons = []
        btn_idx = 0

        # ═══ Title ═══
        cv2.putText(frame, "WALL MANAGER", (px, y + 24),
                    cv2.FONT_HERSHEY_DUPLEX, 0.75, self.COL_ACCENT, 1, cv2.LINE_AA)
        y += 32
        mode_label = self.MODE_NAMES[self.mode]
        cv2.putText(frame, f"Mode: {mode_label}", (px, y + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (160, 170, 190), 1, cv2.LINE_AA)
        y += 20

        # ROS connection indicator
        if ROS_AVAILABLE and self.map_received:
            cv2.circle(frame, (mw - px - 8, 22), 5, (0, 200, 0), -1)
            cv2.putText(frame, "ROS", (mw - px - 38, 27),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 200, 0), 1, cv2.LINE_AA)
        else:
            cv2.circle(frame, (mw - px - 8, 22), 5, (0, 0, 200), -1)
            cv2.putText(frame, "NO MAP", (mw - px - 55, 27),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.33, (0, 0, 200), 1, cv2.LINE_AA)

        y += 6
        cv2.line(frame, (px, y), (mw - px, y), (55, 55, 65), 1)
        y += 14

        # ═══ TOOLS section ═══
        y = self._draw_section_header(frame, y, "TOOLS", mw)

        mode_btns = [
            ("Add Wall",  "W", self.MODE_ADD_WALL, lambda: self._set_mode(self.MODE_ADD_WALL)),
            ("Add Zone",  "Z", self.MODE_ADD_ZONE, lambda: self._set_mode(self.MODE_ADD_ZONE)),
            ("Edit",      "E", self.MODE_EDIT,     lambda: self._set_mode(self.MODE_EDIT)),
            ("Delete",    "X", self.MODE_DELETE,    lambda: self._set_mode(self.MODE_DELETE)),
        ]
        for label, icon, mode_id, cb in mode_btns:
            by1, by2 = y, y + bh
            is_active = self.mode == mode_id
            is_hov = self.hover_btn_idx == btn_idx
            self._draw_btn(frame, px, by1, mw - px, by2, label, icon,
                           is_active, is_hov,
                           self.COL_BTN, self.COL_BTN_HOVER,
                           self.COL_BTN_ACTIVE, self.COL_BTN_ACTIVE_H,
                           self.COL_BTN_BORDER, self.COL_BTN_BORDER_A)
            self.buttons.append((by1, by2, cb))
            btn_idx += 1
            y = by2 + gap

        y += 10

        # ═══ ROS ACTIONS section ═══
        y = self._draw_section_header(frame, y, "ROS ACTIONS", mw)

        ros_btns = [
            ("Push All to ROS",  "P", self._push_all_to_ros, False),
            ("Pull from ROS",    "R", self._pull_from_ros,   False),
            ("Clear ROS Walls",  "C", self._clear_ros,       True),
        ]
        for label, icon, cb, is_danger in ros_btns:
            by1, by2 = y, y + bh - 4
            is_hov = self.hover_btn_idx == btn_idx

            if is_danger:
                self._draw_btn(frame, px, by1, mw - px, by2, label, icon,
                               False, is_hov,
                               self.COL_DANGER, self.COL_DANGER_HOVER,
                               self.COL_DANGER, self.COL_DANGER_HOVER,
                               (120, 80, 80), (150, 100, 100))
            else:
                self._draw_btn(frame, px, by1, mw - px, by2, label, icon,
                               False, is_hov,
                               (50, 50, 55), (65, 65, 72),
                               (50, 50, 55), (65, 65, 72),
                               (75, 75, 85), (100, 100, 110))
            self.buttons.append((by1, by2, cb))
            btn_idx += 1
            y = by2 + gap

        y += 6

        # ═══ LOCAL ACTIONS section ═══
        y = self._draw_section_header(frame, y, "LOCAL", mw)

        local_btns = [
            ("Undo",  "U", self._undo),
        ]
        for label, icon, cb in local_btns:
            by1, by2 = y, y + bh - 4
            is_hov = self.hover_btn_idx == btn_idx
            self._draw_btn(frame, px, by1, mw - px, by2, label, icon,
                           False, is_hov,
                           (50, 50, 55), (65, 65, 72),
                           (50, 50, 55), (65, 65, 72),
                           (75, 75, 85), (100, 100, 110))
            self.buttons.append((by1, by2, cb))
            btn_idx += 1
            y = by2 + gap

        y += 12

        # ═══ ITEMS LIST section ═══
        cv2.line(frame, (px, y), (mw - px, y), (55, 55, 65), 1)
        y += 6
        with self.walls_lock:
            n_walls = len(self.walls)
        y = self._draw_section_header(frame, y, f"ITEMS  ({n_walls})", mw)

        max_list_y = h - 90
        with self.walls_lock:
            walls_snapshot = list(self.walls)

        for idx, wall in enumerate(walls_snapshot):
            if y > max_list_y:
                remaining = len(walls_snapshot) - idx
                cv2.putText(frame, f"  ... +{remaining} more", (px + 4, y + 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.36, (120, 120, 130), 1, cv2.LINE_AA)
                break

            is_sel = idx == self.selected_idx
            is_hov = self.hover_btn_idx == btn_idx
            item_h = 28
            by1, by2 = y, y + item_h

            if is_sel:
                self._rounded_rect(frame, (px, by1), (mw - px, by2),
                                   (60, 100, 160), 4, -1)
            elif is_hov:
                self._rounded_rect(frame, (px, by1), (mw - px, by2),
                                   (50, 50, 58), 4, -1)

            kind = "WALL" if wall.type == WALL_TYPE else "ZONE"
            badge_col = self.COL_WALL if wall.type == WALL_TYPE else self.COL_ZONE
            badge_w = 42
            badge_x = px + 6
            badge_y1 = by1 + 5
            badge_y2 = by2 - 5
            self._rounded_rect(frame, (badge_x, badge_y1),
                               (badge_x + badge_w, badge_y2), badge_col, 3, -1)
            tsz = cv2.getTextSize(kind, cv2.FONT_HERSHEY_SIMPLEX, 0.30, 1)[0]
            cv2.putText(frame, kind,
                        (badge_x + (badge_w - tsz[0]) // 2, badge_y1 + tsz[1] + 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.30, (255, 255, 255), 1, cv2.LINE_AA)

            npts = len(wall.points)
            txt_col = (255, 255, 255) if is_sel else (190, 190, 200)
            cv2.putText(frame, f"{wall.uuid}  ({npts} pts)",
                        (badge_x + badge_w + 8, (by1 + by2) // 2 + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, txt_col, 1, cv2.LINE_AA)

            if is_sel:
                cv2.circle(frame, (mw - px - 10, (by1 + by2) // 2), 4,
                           (255, 255, 255), -1)

            self.buttons.append((by1, by2, lambda i=idx: self._select_item(i)))
            btn_idx += 1
            y = by2 + 3

        # ═══ Help section ═══
        help_y = h - 110
        cv2.line(frame, (px, help_y), (mw - px, help_y), (55, 55, 65), 1)
        help_y += 6

        # ── Robot Pose from /amcl_pose ──
        with self.robot_pose_lock:
            rpose_ok = self.robot_pose_received
            r_x, r_y, r_yaw = self.robot_x, self.robot_y, self.robot_yaw

        if rpose_ok:
            yaw_deg = math.degrees(r_yaw)
            cv2.putText(frame, "ROBOT POSE (AMCL)", (px, help_y + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 200, 0), 1, cv2.LINE_AA)
            help_y += 18
            cv2.putText(frame, f"  x={r_x:.3f}  y={r_y:.3f}  yaw={yaw_deg:.1f} deg",
                        (px, help_y + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.34, (200, 210, 230), 1, cv2.LINE_AA)
            help_y += 18
        else:
            cv2.putText(frame, "ROBOT: waiting for /amcl_pose", (px, help_y + 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.33, (100, 100, 120), 1, cv2.LINE_AA)
            help_y += 18

        cv2.line(frame, (px, help_y), (mw - px, help_y), (55, 55, 65), 1)
        help_y += 14
        helps = [
            "Enter = finish zone    Del/d = delete",
            "Right-click = cancel   q/Esc = quit",
            "r = pull from ROS  p = push to ROS",
        ]
        for txt in helps:
            cv2.putText(frame, txt, (px, help_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.33, (110, 110, 125), 1, cv2.LINE_AA)
            help_y += 16

        # Mouse world coordinates
        if self.map_received:
            mx, my = self.mouse_pos
            mpx, mpy = self.display_to_pixel(mx, my)
            if 0 <= mpx < self.map_w and 0 <= mpy < self.map_h:
                wx, wy = self.pixel_to_world(mpx, mpy)
                cv2.putText(frame, f"Cursor: ({wx:.2f}, {wy:.2f})",
                            (px, h - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 150, 170), 1, cv2.LINE_AA)

    # ─────────────────────────────────────────────
    #  Mode & selection helpers
    # ─────────────────────────────────────────────
    def _set_mode(self, mode):
        self.mode = mode
        self.drawing_pts = []
        self._deselect_all()
        self._set_status(f"Mode: {self.MODE_NAMES[mode]}", self.COL_MENU_HL)

    def _select_item(self, idx):
        self._deselect_all()
        with self.walls_lock:
            if 0 <= idx < len(self.walls):
                self.selected_idx = idx
                self.walls[idx].selected = True

    def _deselect_all(self):
        self.selected_idx = -1
        with self.walls_lock:
            for w in self.walls:
                w.selected = False

    # ─────────────────────────────────────────────
    #  Mouse callback
    # ─────────────────────────────────────────────
    def _mouse_cb(self, event, x, y, flags, param):
        self.mouse_pos = (x, y)

        # Hover tracking for menu buttons
        if x < self.MENU_WIDTH:
            self.hover_btn_idx = -1
            for i, (by1, by2, _cb) in enumerate(self.buttons):
                if by1 <= y <= by2:
                    self.hover_btn_idx = i
                    break
        else:
            self.hover_btn_idx = -1

        # Menu clicks
        if event == cv2.EVENT_LBUTTONDOWN and x < self.MENU_WIDTH:
            for by1, by2, cb in self.buttons:
                if by1 <= y <= by2:
                    cb()
                    return
            return

        if not self.map_received:
            return

        # Convert display -> map pixel -> world
        mpx, mpy = self.display_to_pixel(x, y)
        if not (0 <= mpx < self.map_w and 0 <= mpy < self.map_h):
            return
        wx, wy = self.pixel_to_world(mpx, mpy)

        # ── ADD WALL ──
        if self.mode == self.MODE_ADD_WALL:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drawing_pts.append((wx, wy))
                if len(self.drawing_pts) == 2:
                    self._save_history()
                    new_wall = WallItem(WALL_TYPE, self.drawing_pts, name="wall")
                    with self.walls_lock:
                        self.walls.append(new_wall)
                    self._ros_add_item(new_wall)
                    self._set_status(f"Wall added: {new_wall.uuid}", self.COL_STATUS_OK)
                    self.drawing_pts = []
            elif event == cv2.EVENT_RBUTTONDOWN:
                self.drawing_pts = []
                self._set_status("Cancelled", (150, 150, 150))

        # ── ADD ZONE ──
        elif self.mode == self.MODE_ADD_ZONE:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drawing_pts.append((wx, wy))
            elif event == cv2.EVENT_RBUTTONDOWN:
                self.drawing_pts = []
                self._set_status("Cancelled", (150, 150, 150))

        # ── EDIT ──
        elif self.mode == self.MODE_EDIT:
            if event == cv2.EVENT_LBUTTONDOWN:
                best_d = float("inf")
                best_wi, best_pi = -1, -1
                with self.walls_lock:
                    for wi, wall in enumerate(self.walls):
                        for pi, pt in enumerate(wall.points):
                            ppx, ppy = self.world_to_pixel(*pt)
                            dpx, dpy = self.pixel_to_display(ppx, ppy)
                            d = math.hypot(dpx - x, dpy - y)
                            if d < best_d:
                                best_d = d
                                best_wi, best_pi = wi, pi
                if best_d < self.SNAP_RADIUS_PX / self.scale + 15:
                    self._save_history()
                    self.dragging = True
                    self.drag_wall_idx = best_wi
                    self.drag_pt_idx = best_pi
                    self._select_item(best_wi)

            elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
                with self.walls_lock:
                    self.walls[self.drag_wall_idx].points[self.drag_pt_idx] = (wx, wy)

            elif event == cv2.EVENT_LBUTTONUP:
                if self.dragging:
                    self.dragging = False
                    # Push edit to ROS
                    with self.walls_lock:
                        edited_wall = self.walls[self.drag_wall_idx]
                    self._ros_edit_item(edited_wall)
                    self._set_status(f"Edited: {edited_wall.uuid}", self.COL_STATUS_OK)

        # ── DELETE ──
        elif self.mode == self.MODE_DELETE:
            if event == cv2.EVENT_LBUTTONDOWN:
                idx = self._find_nearest_wall(x, y)
                if idx >= 0:
                    self._select_item(idx)
                    with self.walls_lock:
                        lbl = self.walls[idx].label()
                    self._set_status(f"Selected {lbl} -- press Del/d", (200, 200, 0))

    # ─────────────────────────────────────────────
    #  Nearest-wall helper
    # ─────────────────────────────────────────────
    def _find_nearest_wall(self, dx, dy):
        best_d = float("inf")
        best_idx = -1
        with self.walls_lock:
            for idx, wall in enumerate(self.walls):
                pts_disp = []
                for pt in wall.points:
                    ppx, ppy = self.world_to_pixel(*pt)
                    pts_disp.append(self.pixel_to_display(ppx, ppy))
                for i in range(len(pts_disp)):
                    vd = math.hypot(pts_disp[i][0] - dx, pts_disp[i][1] - dy)
                    if vd < best_d:
                        best_d = vd
                        best_idx = idx
                    j = (i + 1) % len(pts_disp)
                    ed = self._point_line_dist(dx, dy, pts_disp[i], pts_disp[j])
                    if ed < best_d:
                        best_d = ed
                        best_idx = idx
        if best_d < 20:
            return best_idx
        return -1

    @staticmethod
    def _point_line_dist(px, py, a, b):
        ax, ay = a
        bx, by = b
        ddx, ddy = bx - ax, by - ay
        if ddx == 0 and ddy == 0:
            return math.hypot(px - ax, py - ay)
        t = max(0, min(1, ((px - ax) * ddx + (py - ay) * ddy) / (ddx * ddx + ddy * ddy)))
        proj_x = ax + t * ddx
        proj_y = ay + t * ddy
        return math.hypot(px - proj_x, py - proj_y)

    # ─────────────────────────────────────────────
    #  Delete selected (local + ROS erase)
    # ─────────────────────────────────────────────
    def _delete_selected(self):
        with self.walls_lock:
            if not (0 <= self.selected_idx < len(self.walls)):
                return
            self._save_history()
            removed = self.walls.pop(self.selected_idx)
        self._ros_erase_item(removed)
        self._set_status(f"Deleted {removed.label()}", self.COL_STATUS_OK)
        self.selected_idx = -1

    # ─────────────────────────────────────────────
    #  Undo
    # ─────────────────────────────────────────────
    def _save_history(self):
        with self.walls_lock:
            self.history.append(copy.deepcopy(self.walls))
        if len(self.history) > 50:
            self.history.pop(0)

    def _undo(self):
        if self.history:
            with self.walls_lock:
                self.walls = self.history.pop()
            self._set_status("Undo -- remember to Push to sync ROS", (200, 200, 0))
        else:
            self._set_status("Nothing to undo", self.COL_STATUS_ERR)

    # ═══════════════════════════════════════════════
    #  ROS Service helpers  (ist_layouts_srv CRUD)
    # ═══════════════════════════════════════════════
    def _call_layout_srv(self, cmd, layouts_list=None):
        """Call ist_layouts_srv with the given command. Returns response or None."""
        if not ROS_AVAILABLE:
            self._set_status("ROS not available", self.COL_STATUS_ERR)
            return None
        try:
            rospy.wait_for_service("ist_layouts_srv", timeout=3.0)
            proxy = rospy.ServiceProxy("ist_layouts_srv", Layout)
            req = LayoutRequest()
            req.cmd = cmd
            if layouts_list:
                req.layouts = layouts_list
            resp = proxy(req)
            return resp
        except Exception as e:
            self._set_status(f"Service error: {e}", self.COL_STATUS_ERR)
            return None

    def _trigger_dynamic_reconfigure(self):
        """Set dynamic_polygons=true on both global and local costmap layers."""
        if not ROS_AVAILABLE:
            return
        for ns in ["/move_base/global_costmap/ist_costmap_prohibition_layer/set_parameters",
                    "/move_base/local_costmap/ist_costmap_prohibition_layer/set_parameters"]:
            try:
                rospy.wait_for_service(ns, timeout=2.0)
                proxy = rospy.ServiceProxy(ns, Reconfigure)
                req = ReconfigureRequest()
                bp = BoolParameter()
                bp.name = "dynamic_polygons"
                bp.value = True
                req.config.bools = [bp]
                req.config.ints = []
                req.config.strs = []
                req.config.doubles = []
                req.config.groups = []
                proxy(req)
            except Exception as e:
                rospy.logwarn(f"dynamic_reconfigure ({ns}): {e}")

    # ── Add single item to ROS ──
    def _ros_add_item(self, wall_item):
        if not ROS_AVAILABLE:
            return
        layout = wall_item.to_uilayout()
        resp = self._call_layout_srv("add", [layout])
        if resp:
            self._trigger_dynamic_reconfigure()

    # ── Edit single item on ROS ──
    def _ros_edit_item(self, wall_item):
        if not ROS_AVAILABLE:
            return
        layout = wall_item.to_uilayout()
        resp = self._call_layout_srv("edit", [layout])
        if resp:
            self._trigger_dynamic_reconfigure()

    # ── Erase single item from ROS ──
    def _ros_erase_item(self, wall_item):
        if not ROS_AVAILABLE:
            return
        layout = wall_item.to_uilayout()
        resp = self._call_layout_srv("erase", [layout])
        if resp:
            self._trigger_dynamic_reconfigure()

    # ── Push ALL walls (clear + re-add) ──
    def _push_all_to_ros(self):
        if not ROS_AVAILABLE:
            self._set_status("ROS not available", self.COL_STATUS_ERR)
            return
        # Clear everything first
        self._call_layout_srv("clear")

        # Add all current walls
        with self.walls_lock:
            layouts = [w.to_uilayout() for w in self.walls]
        if layouts:
            resp = self._call_layout_srv("add", layouts)
            if resp:
                self._trigger_dynamic_reconfigure()
                self._set_status(f"Pushed {len(layouts)} items to ROS", self.COL_STATUS_OK)
            else:
                self._set_status("Failed to push to ROS", self.COL_STATUS_ERR)
        else:
            self._trigger_dynamic_reconfigure()
            self._set_status("Cleared all walls on ROS (none to push)", (200, 200, 0))

    # ── Pull walls from ROS (list all) ──
    def _pull_from_ros(self):
        if not ROS_AVAILABLE:
            self._set_status("ROS not available", self.COL_STATUS_ERR)
            return
        try:
            query = UILayout()
            query.uuid = ""
            query.type = 0
            resp = self._call_layout_srv("list", [query])
            if resp is None:
                return
            self._save_history()
            with self.walls_lock:
                self.walls.clear()
                for layout in resp.layouts:
                    pts = [(p.x, p.y) for p in layout.points]
                    self.walls.append(WallItem(
                        item_type=layout.type,
                        points=pts,
                        name=layout.name,
                        uid=layout.uuid,
                    ))
                n = len(self.walls)
            self._set_status(f"Pulled {n} items from ROS", self.COL_STATUS_OK)
        except Exception as e:
            self._set_status(f"Pull error: {e}", self.COL_STATUS_ERR)

    # ── Clear all on ROS ──
    def _clear_ros(self):
        resp = self._call_layout_srv("clear")
        if resp:
            self._save_history()
            with self.walls_lock:
                self.walls.clear()
            self._trigger_dynamic_reconfigure()
            self._set_status("Cleared all walls on ROS + local", self.COL_STATUS_OK)

    # ─────────────────────────────────────────────
    #  Main loop
    # ─────────────────────────────────────────────
    def run(self):
        cv2.namedWindow(self.WINDOW, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.WINDOW, self._mouse_cb)

        # Initial window size (before map arrives)
        init_w = self.MENU_WIDTH + 800
        init_h = 700
        cv2.resizeWindow(self.WINDOW, init_w, init_h)

        print("=" * 50)
        print("  Wall Manager GUI  (ROS /map + ist_layouts_srv)")
        print("  Keys: 1=AddWall  2=AddZone  3=Edit  4=Delete")
        print("  Enter=finish zone  Del/d=delete")
        print("  r=pull from ROS  p=push to ROS  u=undo")
        print("  q/Esc=quit")
        print("=" * 50)

        while True:
            # Check ROS shutdown
            if ROS_AVAILABLE and rospy.is_shutdown():
                break

            frame = self._draw_frame()

            # Resize window once to fit map when first received
            if self.map_received and not self._window_sized:
                disp_w = int(self.map_w * self.scale) + self.MENU_WIDTH
                disp_h = int(self.map_h * self.scale)
                cv2.resizeWindow(self.WINDOW, disp_w, disp_h)
                self._window_sized = True

            cv2.imshow(self.WINDOW, frame)
            key = cv2.waitKey(30) & 0xFF

            if key == ord('q') or key == 27:
                break
            elif key == ord('1'):
                self._set_mode(self.MODE_ADD_WALL)
            elif key == ord('2'):
                self._set_mode(self.MODE_ADD_ZONE)
            elif key == ord('3'):
                self._set_mode(self.MODE_EDIT)
            elif key == ord('4'):
                self._set_mode(self.MODE_DELETE)
            elif key == 13:  # Enter -- finish zone
                if self.mode == self.MODE_ADD_ZONE and len(self.drawing_pts) >= 3:
                    self._save_history()
                    new_zone = WallItem(ZONE_TYPE, self.drawing_pts, name="zone")
                    with self.walls_lock:
                        self.walls.append(new_zone)
                    self._ros_add_item(new_zone)
                    self._set_status(f"Zone added: {new_zone.uuid}", self.COL_STATUS_OK)
                    self.drawing_pts = []
                elif self.mode == self.MODE_ADD_ZONE:
                    self._set_status("Need at least 3 points for a zone", self.COL_STATUS_ERR)
            elif key == 255 or key == ord('d'):
                self._delete_selected()
            elif key == ord('r'):
                self._pull_from_ros()
            elif key == ord('p'):
                self._push_all_to_ros()
            elif key == ord('u'):
                self._undo()

        cv2.destroyAllWindows()


# ═══════════════════════════════════════════════════════
#  Entry point
# ═══════════════════════════════════════════════════════
if __name__ == "__main__":
    gui = WallManagerGUI()
    gui.run()
