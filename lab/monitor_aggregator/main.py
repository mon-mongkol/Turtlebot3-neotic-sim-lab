#!/usr/bin/env python3
import rospy
import psutil
import actionlib
import math
import sys
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan, Imu  # เพิ่ม Imu
from actionlib_msgs.msg import GoalStatus, GoalStatusArray

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

class RobotMonitor:
    def __init__(self):
        rospy.init_node('robot_monitor_node')

        # Maintenance Thresholds (Example: hours or cycles)
        self.battery_max_cycles = 500
        self.motor_max_hours = 2000
        
        # In a real app, load these from a database or file
        self.current_battery_cycles = 480 
        self.current_motor_hours = 1950
        
        # Configuration parameters
        self.cpu_threshold = rospy.get_param('~cpu_threshold', 85.0)
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


        


        rospy.loginfo("Monitor Node Initialized...")

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
            level = DiagnosticStatus.WARN
            msg = "CPU Overload detected"
            write_log_json(self, data, prefix="robot_monitor" ):
            # cat_id_cpu_check = CATEGORY_ID["Software_Controller"]
            # print(cat_id_cpu_check)  # 2
            
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

    def write_log_json(data: Any, log_dir: str = "lab/monitor_aggregator/LOG", filename: str = "data.json") -> str:
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, filename)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return path


if __name__ == '__main__':
    try:
        monitor = RobotMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass