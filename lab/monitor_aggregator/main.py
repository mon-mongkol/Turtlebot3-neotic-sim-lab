#!/usr/bin/env python3
try:
    import rospy
    import psutil
    import actionlib
    import uuid as _uuid
    import math
    from geometry_msgs.msg import PoseWithCovarianceStamped
    import sys , os , json ,csv
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from actionlib_msgs.msg import GoalStatus
    from sensor_msgs.msg import LaserScan, Imu  # เพิ่ม Imu
    from actionlib_msgs.msg import GoalStatus, GoalStatusArray
    from typing import Any
    from typing import Optional
    from web_interface_msgs.srv import Layout, LayoutRequest
    from web_interface_msgs.msg import UILayout
    from dynamic_reconfigure.msg import BoolParameter
    from dynamic_reconfigure.srv import Reconfigure, ReconfigureRequest
    from geometry_msgs.msg import Point as ROSPoint
    from threading import Lock
    import datetime
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

    def as_dict(self):
        return {
            "type": self.type,
            "points": self.points,
            "name": self.name,
            "uuid": self.uuid,
            "selected": self.selected,
        }
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

class RobotMonitor:
    CATEGORY_ID = {
        "Safety_Compliance": 0,
        "Localization_Mapping": 1,
        "Navigation_Path_Control": 2,
        "Motion_Drive_System": 3,
        "Battery_Charging": 4,
        "Sensor_Vision": 5,
        "IO_Electrical_Alarm": 6,
        "Task_Mission": 7,
        "Software_Controller": 8,
        "Maintenance_Warning": 9
    }
    def __init__(self):
        rospy.init_node('robot_monitor_node')
                # ── Walls data ──
        self.walls: list = []          # list[WallItem]
        self.walls_lock = Lock()

        # Maintenance Thresholds (Example: hours or cycles)
        self.battery_max_cycles = 500
        self.motor_max_hours = 2000
        
        # In a real app, load these from a database or file
        self.current_battery_cycles = 480 
        self.current_motor_hours = 1950
        
        # Configuration parameters
        self.cpu_threshold = rospy.get_param('~cpu_threshold', 10.0)
        self.disk_threshold = rospy.get_param('~disk_threshold', 90.0)
        
        # --- Per-component Diagnostic Publishers (separate topics) ---
        self.pub_cpu = rospy.Publisher('/diagnostics/cpu', DiagnosticArray, queue_size=10)
        self.pub_disk = rospy.Publisher('/diagnostics/disk', DiagnosticArray, queue_size=10)
        self.pub_amcl = rospy.Publisher('/diagnostics/amcl', DiagnosticArray, queue_size=10)
        self.pub_maintenance = rospy.Publisher('/diagnostics/maintenance', DiagnosticArray, queue_size=10)
        self.pub_laser = rospy.Publisher('/diagnostics/laser', DiagnosticArray, queue_size=10)
        self.pub_imu = rospy.Publisher('/diagnostics/imu', DiagnosticArray, queue_size=10)
        self.pub_nav = rospy.Publisher('/diagnostics/navigation', DiagnosticArray, queue_size=10)

        # Timer to run diagnostics at 1Hz (publishes per-component topics)
        self.diag_timer = rospy.Timer(rospy.Duration(1.0), self.publish_diagnostics)


        # --- Laser Scan Variables ---
        self.last_scan_time = rospy.Time(0)
        self.scan_timeout = 0.5  # Seconds before we consider it "lost"
        self.has_scan_data = False
        
        # Subscribe to /scan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # --- IMU Variables (init to avoid AttributeError) ---
        self.last_imu_time = rospy.Time(0)
        self.imu_timeout = rospy.get_param('~imu_timeout', 0.5)
        self.imu_data_valid = False
        # Subscribe to IMU topic (ปรับชื่อ topic ถ้าจำเป็น เช่น '/imu/data')
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)



        self.last_move_base_status_code = None
        self.last_move_base_status_text = ""
        self.server_status = "Offline"
        self.current_nav_state = "IDLE / NO GOAL"
        self.nav_level = DiagnosticStatus.OK
        # subscribe to move_base status
        self.mb_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_cb)


        

        self.amcl_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.amcl_pose_callback)
        rospy.loginfo("Monitor Node Initialized...")


    ID_TO_CATEGORY = {v: k for k, v in CATEGORY_ID.items()}

    def get_category_type(self, category_id: int) -> Optional[str]:
        return self.ID_TO_CATEGORY.get(category_id)

    def create_status(self, name, level, message, values):
        """Helper to build a DiagnosticStatus message."""
        status = DiagnosticStatus()
        status.name = name
        status.level = level
        status.message = message
        status.values = [KeyValue(key=k, value=str(v)) for k, v in values.items()]
        return status


    def check_cpu(self):
        usage = psutil.cpu_percent()
        level = DiagnosticStatus.OK
        msg = "CPU Usage OK"
        
        if usage > self.cpu_threshold:
            get_type_cpu = self.get_category_type(8) or "Unknown"
            base_dir = rospy.get_param('~log_base_dir', os.path.join(os.path.dirname(__file__), 'LOG'))
            log_dir = os.path.abspath(os.path.join(base_dir, get_type_cpu))
            try:
                monitor.write_log_entry(
                    {"CPU_Percent": usage},
                    log_dir=log_dir,
                    header=["Timestamp_ISO", "Date_TH", "Time_HM", "CPU_Percent"]
                )

                # rospy.loginfo(f"CPU log appended: {log_dir}")
            except Exception as e:
                rospy.logerr(f"Failed to write CPU log: {e}")
            level = DiagnosticStatus.WARN
            msg = "CPU Overload detected"

            
        return self.create_status("System: CPU", level, msg, {"Usage (%)": usage})

    def check_disk(self):
        disk = psutil.disk_usage('/')
        level = DiagnosticStatus.OK
        msg = "Disk Space OK"
        
        if disk.percent > self.disk_threshold:
            level = DiagnosticStatus.ERROR
            msg = "Disk Full or Critical"
            
        return self.create_status("System: Disk", level, msg, {
            "Used (%)": disk.percent,
            "Free (GB)": round(disk.free / (1024**3), 2)
        })

    def check_amcl(self):
        """
        Example check: verifies if the /amcl_pose topic has publishers.
        """
        # Get list of topics and their types
        topics = rospy.get_published_topics()
        amcl_active = any('/amcl_pose' in t[0] for t in topics)
        
        level = DiagnosticStatus.OK if amcl_active else DiagnosticStatus.ERROR
        msg = "AMCL Running" if amcl_active else "AMCL Not Found"
        
        return self.create_status("Software: AMCL", level, msg, {"Active": amcl_active})

    def publish_diagnostics(self, event):
        # Publish each component diagnostic to its own topic
        cpu_status = self.check_cpu()
        disk_status = self.check_disk()
        amcl_status = self.check_amcl()
        maintenance_status = self.check_maintenance()
        laser_status = self.check_laser_scanner()
        imu_status = self.check_imu()

        def publish(pub, status):
            arr = DiagnosticArray()
            arr.header.stamp = rospy.Time.now()
            arr.status.append(status)
            pub.publish(arr)

        publish(self.pub_cpu, cpu_status)
        publish(self.pub_disk, disk_status)
        publish(self.pub_amcl, amcl_status)
        publish(self.pub_maintenance, maintenance_status)
        publish(self.pub_laser, laser_status)
        publish(self.pub_imu, imu_status)


    def check_maintenance(self):
        """
        Calculates if hardware components are nearing their end-of-life.
        """
        level = DiagnosticStatus.OK
        messages = []
        vals = {
            "Battery Cycles": self.current_battery_cycles,
            "Motor Hours": self.current_motor_hours
        }

        # 1. Battery Lifetime Logic
        if self.current_battery_cycles >= self.battery_max_cycles:
            level = DiagnosticStatus.ERROR
            messages.append("REPLACE BATTERY NOW")
        elif self.current_battery_cycles >= (self.battery_max_cycles * 0.9):
            level = max(level, DiagnosticStatus.WARN)
            messages.append("Battery Lifetime Warning")

        # 2. Motor Lifetime Logic
        if self.current_motor_hours >= self.motor_max_hours:
            level = DiagnosticStatus.ERROR
            messages.append("MOTOR PM OVERDUE")
        elif self.current_motor_hours >= (self.motor_max_hours * 0.9):
            level = max(level, DiagnosticStatus.WARN)
            messages.append("Motor Lifetime Warning")

        # Final Status Summary
        summary = " / ".join(messages) if messages else "Maintenance: All Systems Nominal"
        
        return self.create_status("Hardware: Maintenance", level, summary, vals)

    def scan_callback(self, msg):
        """Update heartbeat and check if there are valid readings."""
        self.last_scan_time = rospy.Time.now()
        # Check if there is at least one valid measurement in the scan array
        # This filters out "ghost" scans where the topic is alive but data is empty
        self.has_scan_data = any(msg.ranges)

    def imu_callback(self, msg):
        """Update IMU heartbeat and simple data-valid check."""
        self.last_imu_time = rospy.Time.now()

        def vec_nonzero_and_finite(vals):
            return any((math.isfinite(v) and abs(v) > 1e-9) for v in vals)

        orient = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        ang_vel = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        lin_acc = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)

        # Consider IMU valid if any of the orientation/angular/linear vectors contain finite non-zero values
        self.imu_data_valid = vec_nonzero_and_finite(orient) or vec_nonzero_and_finite(ang_vel) or vec_nonzero_and_finite(lin_acc)

    def check_laser_scanner(self):
        """Evaluates the health of the Lidar sensor."""
        level = DiagnosticStatus.OK
        msg = "Laser Scanner OK"
        time_since_last = (rospy.Time.now() - self.last_scan_time).to_sec()

        # 1. Heartbeat Check
        if self.last_scan_time == rospy.Time(0):
            level = DiagnosticStatus.ERROR
            msg = "Lidar Not Started (No Data Received)"
        elif time_since_last > self.scan_timeout:
            level = DiagnosticStatus.ERROR
            msg = f"Lidar Timeout: {time_since_last:.2f}s since last message"
        
        # 2. Data Integrity Check
        elif not self.has_scan_data:
            level = DiagnosticStatus.WARN
            msg = "Lidar publishing, but all range data is empty/zero"

        return self.create_status("Hardware: Laser Scanner", level, msg, {
            "Last Update (s ago)": round(time_since_last, 2),
            "Topic": "/scan"
        })

    def check_imu(self):
        """Evaluates the health of the IMU sensor."""
        level = DiagnosticStatus.OK
        msg = "IMU OK"
        time_since_last = (rospy.Time.now() - self.last_imu_time).to_sec()

        # 1. Heartbeat Check
        if self.last_imu_time == rospy.Time(0):
            level = DiagnosticStatus.ERROR
            msg = "IMU Not Started"
        elif time_since_last > self.imu_timeout:
            level = DiagnosticStatus.ERROR
            msg = f"IMU Timeout: {time_since_last:.2f}s since last message"
        
        # 2. Data Quality Check
        elif not self.imu_data_valid:
            level = DiagnosticStatus.WARN
            msg = "IMU Data Invalid (NaN or All Zeros)"

        return self.create_status("Hardware: IMU", level, msg, {
            "Update Latency (s)": round(time_since_last, 4),
            "Status": "Valid" if self.imu_data_valid else "Invalid"
        })

    def publish_report(self):
        diag_array = DiagnosticArray()
        diag_array.header.stamp = rospy.Time.now()

        status = DiagnosticStatus()
        status.name = "Software: Navigation Control"
        status.level = self.nav_level
        status.message = self.current_nav_state

        status.values.append(KeyValue(key="Action Server", value=self.server_status))

        # Safely include action client state if available
        if hasattr(self, 'client') and self.client is not None:
            try:
                status.values.append(KeyValue(key="Current State ID", value=str(self.client.get_state())))
            except Exception:
                status.values.append(KeyValue(key="Current State ID", value="Unknown"))
        else:
            status.values.append(KeyValue(key="Current State ID", value="N/A"))

        status.values.append(KeyValue(key="Last Status Code", value=str(self.last_move_base_status_code)))
        status.values.append(KeyValue(key="Last Status Text", value=self.last_move_base_status_text or ""))

        diag_array.status.append(status)
        self.pub_nav.publish(diag_array)

    def move_base_status_cb(self, msg): #https://github.com/ros/common_msgs/blob/noetic-devel/actionlib_msgs/msg/GoalStatus.msg

        #uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                    #    to some failure (Terminal State)
        #uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                    #    because the goal was unattainable or invalid (Terminal State)
        #uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                    #    and has not yet completed execution
        #uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                    #    but the action server has not yet confirmed that the goal is canceled
        #uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                    #    and was successfully cancelled (Terminal State)
        """Parse /move_base/status (GoalStatusArray) to detect path/plan issues."""
        if not msg.status_list:
            self.server_status = "Offline"
            self.current_nav_state = "IDLE / NO GOAL"
            self.nav_level = DiagnosticStatus.OK
            self.last_move_base_status_code = None
            self.last_move_base_status_text = ""
            self.publish_report()
            return

        self.server_status = "Online"
        st = msg.status_list[-1]  # most recent
        code = st.status
        text = (st.text or "").lower()
        self.last_move_base_status_code = code
        self.last_move_base_status_text = st.text or ""

        # keyword-based detection (fallback to status codes)
        if any(k in text for k in ("no valid", "no plan", "invalid plan", "no path")):
            self.current_nav_state = "No Valid Path"
            self.nav_level = DiagnosticStatus.ERROR
        elif any(k in text for k in ("blocked", "path blocked", "obstacle")):
            self.current_nav_state = "Path Blocked"
            self.nav_level = DiagnosticStatus.ERROR
        elif any(k in text for k in ("deadlock", "stuck", "oscillating")):
            self.current_nav_state = "Deadlock Detected"
            self.nav_level = DiagnosticStatus.ERROR
        else:
            # fallback by GoalStatus code
            if code == GoalStatus.PENDING:
                self.current_nav_state = "Goal Pending"
                self.nav_level = DiagnosticStatus.OK
            elif code == GoalStatus.ACTIVE:
                self.current_nav_state = "Robot Moving"
                self.nav_level = DiagnosticStatus.OK
            elif code == GoalStatus.SUCCEEDED:
                self.current_nav_state = "Goal Reached Successfully"
                self.nav_level = DiagnosticStatus.OK
            elif code == GoalStatus.ABORTED:
                self.current_nav_state = "Navigation Aborted"
                self.nav_level = DiagnosticStatus.ERROR
            elif code == GoalStatus.REJECTED:
                self.current_nav_state = "Invalid Goal Sent"
                self.nav_level = DiagnosticStatus.ERROR
            elif code == GoalStatus.PREEMPTED:
                self.current_nav_state = "Goal Preempted"
                self.nav_level = DiagnosticStatus.WARN
            else:
                self.current_nav_state = f"Status {code}"
                self.nav_level = DiagnosticStatus.WARN

        # publish updated navigation diagnostic
        self.publish_report()

    def write_log_entry(self, data, log_dir: str, filename: str = "cpu_log.json", header: Optional[list] = None) -> str:
        """
        Append a JSON record file in this structure:
        { "records": [ { "header": [...], "linedata": [...] }, ... ] }

        - data can be dict, list/tuple, or scalar.
        - If data is dict and header is None, header is derived from dict.keys().
        - If data is list/tuple, header must be provided.
        - If data is scalar and header is None, uses default columns:
          ["Timestamp_ISO","Date_TH","Time_HM","CPU_Percent"]
        """
        path = os.path.join(log_dir, filename)
        try:
            # ensure variables used later are defined early (avoids NameError if mkdir fails)
            
            csv_path = os.path.splitext(path)[0] + ".csv"

            os.makedirs(log_dir, exist_ok=True)

            # Thailand timezone (UTC+7)
            tz_th = datetime.timezone(datetime.timedelta(hours=7))
            now = datetime.datetime.now(datetime.timezone.utc).astimezone(tz_th)
            defaults = {
                "Timestamp_ISO": now.isoformat(),
                "Date_TH": f"{now.day:02d}/{now.month:02d}/{now.year + 543}",
                "Time_HM": f"{now.hour:02d}:{now.minute:02d}"
            }

            # Normalize header and linedata
            if isinstance(data, dict):
                if header:
                    linedata = [str(data.get(col, defaults.get(col, ""))) for col in header]
                else:
                    header = list(data.keys())
                    linedata = [str(data[k]) for k in header]
            elif isinstance(data, (list, tuple)):
                if not header:
                    raise ValueError("header is required when data is a list/tuple")
                linedata = [str(v) for v in data]
                linedata = [v if v != "" else defaults.get(col, "") for col, v in zip(header, linedata)]
            # else:
            #     if not header:
            #         header = ["Timestamp_ISO", "Date_TH", "Time_HM", "DATA"]
            #     linedata = [
            #         defaults["Timestamp_ISO"],
            #         defaults["Date_TH"],
            #         defaults["Time_HM"],
            #         str(data)
            #     ]

            record = {"header": header, "linedata": linedata}

            # Read existing file (if any), append record, and write back
            records_obj = {"records": []}
            if os.path.exists(path):
                try:
                    with open(path, "r", encoding="utf-8") as f:
                        existing = json.load(f)
                        if isinstance(existing, dict) and "records" in existing and isinstance(existing["records"], list):
                            records_obj = existing
                        elif isinstance(existing, list):
                            records_obj = {"records": existing}
                except Exception:
                    records_obj = {"records": []}

            records_obj["records"].append(record)

            with open(path, "w", encoding="utf-8") as f:
                json.dump(records_obj, f, ensure_ascii=False, indent=2)

            # --- Also write a CSV version of the log ---
            try:
                # collect a stable ordered union of headers (preserve first-seen order)
                ordered_headers = []
                for rec in records_obj.get("records", []):
                    for h in rec.get("header", []):
                        if h not in ordered_headers:
                            ordered_headers.append(h)

                # build rows as dicts keyed by ordered_headers
                rows = []
                for rec in records_obj.get("records", []):
                    hdr = rec.get("header", [])
                    lined = rec.get("linedata", [])
                    row = {h: "" for h in ordered_headers}
                    for i, h in enumerate(hdr):
                        if i < len(lined):
                            row[h] = lined[i]
                    rows.append(row)

                # write CSV
                with open(csv_path, "w", encoding="utf-8", newline='') as cf:
                    writer = csv.DictWriter(cf, fieldnames=ordered_headers)
                    writer.writeheader()
                    for r in rows:
                        writer.writerow(r)
            except Exception as e:
                rospy.logwarn(f"write_log_entry: failed to write CSV for {path}: {e}")

            return os.path.abspath(path)
        except Exception as e:
            rospy.logerr(f"write_log_entry error writing to {log_dir}/{filename}: {e}")
            raise

    def list_wall_and_zone_layouts(self):
        rospy.wait_for_service('/ist_layouts_srv')
        try:
            list_srv = rospy.ServiceProxy('/ist_layouts_srv', Layout)
            req = LayoutRequest()
            req.cmd = "list"
            # To list all, send an empty UILayout in the layouts array
            req.layouts.append(UILayout())
            resp = list_srv(req)
            # Define your type values for wall and zone
            WALL_TYPE = 1
            ZONE_TYPE = 2
            wall_zone_layouts = [l for l in resp.layouts if l.type in (WALL_TYPE, ZONE_TYPE)]
            for layout in wall_zone_layouts:
                print(f"uuid={layout.uuid}, name={layout.name}, type={layout.type}")
            return wall_zone_layouts
        except rospy.ServiceException as e:
            print("Service call failed:", e)
            return []
        
    def _call_layout_srv(self, cmd, layouts_list=None):
        """Call ist_layouts_srv with the given command. Returns response or None."""
        if not ROS_AVAILABLE:
            pass
            # self._set_status("ROS not available", self.COL_STATUS_ERR)
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
        
            # self._set_status(f"Service error: {e}", self.COL_STATUS_ERR)
            return None
        
    def _pull_from_ros(self):
        if not ROS_AVAILABLE:
            rospy.loginfo('ros module error')
            # self._set_status("ROS not available", self.COL_STATUS_ERR)
            return
        try:
            query = UILayout()
            query.uuid = ""
            query.type = 0
            resp = self._call_layout_srv("list", [query])
            if resp is None:
                rospy.loginfo('ros sercice error')
                return
            # self._save_history()
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
            # print(f"Pulled {n} items from ROS:") 
            # rospy.loginfo(f"Pulled {n} items from ROS: {self.walls}")  
            walls_data = [w.as_dict() for w in self.walls]
            rospy.loginfo(f"Pulled {n} items from ROS: {walls_data}")
            # self._set_status(f"Pulled {n} items from ROS", self.COL_STATUS_OK)
        except Exception as e:
            rospy.loginfo(f"Error pulling from ROS: {e}")
            #pass
            # self._set_status(f"Pull error: {e}", self.COL_STATUS_ERR)

    def point_in_polygon(self,point, polygon):
        """Ray casting algorithm for checking if point is inside polygon."""
        x, y = point
        inside = False
        n = len(polygon)
        if n < 3:
            return False
        px1, py1 = polygon[0]
        for i in range(n + 1):
            px2, py2 = polygon[i % n]
            if min(py1, py2) < y <= max(py1, py2) and x <= max(px1, px2):
                if py1 != py2:
                    xinters = (y - py1) * (px2 - px1) / (py2 - py1 + 1e-9) + px1
                if px1 == px2 or x <= xinters:
                    inside = not inside
            px1, py1 = px2, py2
        return inside
    
    def amcl_pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        robot_pos = (x, y)
        with self.walls_lock:
            for wall in self.walls:
                # ถ้าเป็นโซน (polygon)
                if wall.type == ZONE_TYPE and self.point_in_polygon(robot_pos, wall.points):
                    rospy.logwarn(f"Robot entered ZONE: {wall.name} ({wall.uuid})")
                # ถ้าเป็น wall (เส้นตรง 2 จุด) ให้เช็คระยะ
                elif wall.type == WALL_TYPE and len(wall.points) == 2:
                    p1, p2 = wall.points
                    px, py = robot_pos
                    x1, y1 = p1
                    x2, y2 = p2
                    dx, dy = x2 - x1, y2 - y1
                    if dx == dy == 0:
                        dist = ((px - x1)**2 + (py - y1)**2)**0.5
                    else:
                        t = max(0, min(1, ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)))
                        proj_x = x1 + t * dx
                        proj_y = y1 + t * dy
                        dist = ((px - proj_x)**2 + (py - proj_y)**2)**0.5
                    if dist < 0.1:  # ปรับ threshold ระยะใกล้ wall (เมตร)
                        rospy.logwarn(f"Robot is near WALL: {wall.name} ({wall.uuid}) dist={dist:.2f}")

if __name__ == '__main__':
    try:
        monitor = RobotMonitor()
        # monitor.list_wall_and_zone_layouts()
        monitor._pull_from_ros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass