import rclpy
import time
import tf_transformations
import subprocess
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, BatteryState
from mavros_msgs.msg import State
from rich.console import Console
from rich.panel import Panel
from rich.layout import Layout
from rich.live import Live
from rich.table import Table
from rich.columns import Columns

class UAVStatus(Node):
    def __init__(self):
        super().__init__('uav_status_node')
        self.console = Console()
        self.layout = Layout()
        self.pose_info = {
            "Position": {"x": "N/A", "y": "N/A", "z": "N/A"},
            "Orientation (Euler)": {"roll": "N/A", "pitch": "N/A", "yaw": "N/A"},
            "Orientation (Quaternion)": {"x": "N/A", "y": "N/A", "z": "N/A", "w": "N/A"}
        }
        self.battery_info = {"Voltage": "N/A", "Current": "N/A", "Percentage": "N/A"}
        self.flight_mode = "N/A"
        self.connection_status = "N/A"

        self.odom_last_time = None
        self.imu_last_time = None
        self.odom_frequency = 0
        self.imu_frequency = 0
        
        self.odom_timeout = 1  # Timeout in seconds
        self.imu_timeout = 1   # Timeout in seconds
        
        self.subscription_odom = self.create_subscription(
            PoseStamped,
            '/mavros/vision_pose/pose',
            self.odometry_callback,
            10
        )
        self.subscription_imu = self.create_subscription(
            Imu,
            '/zed/zed_node/imu/data',
            self.imu_callback,
            10
        )
        self.subscription_battery = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            10
        )
        self.subscription_state = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )

    def quaternion_to_euler(self, orientation):
        quaternion = (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        euler = tf_transformations.euler_from_quaternion(quaternion)
        return euler  # roll, pitch, yaw

    def odometry_callback(self, msg):
        self.odom_last_time = self.get_clock().now().seconds_nanoseconds()

        pose = msg.pose
        position = pose.position
        orientation = pose.orientation
        roll, pitch, yaw = self.quaternion_to_euler(orientation)

        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        self.pose_info = {
            "Position": {
                "x": f"{position.x:.2f}",
                "y": f"{position.y:.2f}",
                "z": f"{position.z:.2f}",
            },
            "Orientation (Euler)": {
                "roll": f"{roll_deg:.2f}",   
                "pitch": f"{pitch_deg:.2f}", 
                "yaw": f"{yaw_deg:.2f}",    
            },
            "Orientation (Quaternion)": {
                "x": f"{orientation.x:.2f}",
                "y": f"{orientation.y:.2f}",
                "z": f"{orientation.z:.2f}",
                "w": f"{orientation.w:.2f}",
            }
        }

    def imu_callback(self, msg):
        self.imu_last_time = self.get_clock().now().seconds_nanoseconds()

    def battery_callback(self, msg):
        self.battery_info = {
            "Voltage": f"{msg.voltage:.2f}V",
            "Current": f"{msg.current:.2f}A",
            "Percentage": f"{msg.percentage:.2f}%"
        }

    def state_callback(self, msg):
        self.flight_mode = msg.mode
        self.connection_status = "Connected" if msg.connected else "Disconnected"

    def check_program_status(self):
        programs = ["mavros_node", "mavlink-routerd"]
        program_status = {}
        for program in programs:
            try:
                result = subprocess.run(["pgrep", "-x", program], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                program_status[program] = "✔️" if result.returncode == 0 else "❌"
            except Exception as e:
                program_status[program] = f"Error: {str(e)}"
        return program_status

    def create_pose_table(self, pose_info):
        table = Table(title="Pose Information", title_style="bold magenta")
        table.add_column("Type", justify="center", style="cyan", no_wrap=True)
        table.add_column("x / roll", justify="center", style="green")
        table.add_column("y / pitch", justify="center", style="green")
        table.add_column("z / yaw", justify="center", style="green")
        table.add_column("w", justify="center", style="green")

        table.add_row(
            "Position",
            pose_info["Position"]["x"],
            pose_info["Position"]["y"],
            pose_info["Position"]["z"]
        )
   
        table.add_row(
            "Orientation (Euler)",
            pose_info["Orientation (Euler)"]["roll"],
            pose_info["Orientation (Euler)"]["pitch"],
            pose_info["Orientation (Euler)"]["yaw"]
        )
     
        table.add_row(
            "Orientation (Quaternion)",
            pose_info["Orientation (Quaternion)"]["x"],
            pose_info["Orientation (Quaternion)"]["y"],
            pose_info["Orientation (Quaternion)"]["z"],
            pose_info["Orientation (Quaternion)"]["w"]
        )

        return table

    def create_odometry_table(self, odometry_frequency, odometry_health, imu_frequency):
        table = Table(title="Info", title_style="bold magenta")
        table.add_column("Name", justify="center", style="cyan", no_wrap=True)
        table.add_column("Value", justify="center", style="green")

        table.add_row("Odom. Freq. (Hz)", f"{odometry_frequency:.2f}")
        table.add_row("Odom. Health", odometry_health)
        table.add_row("IMU Freq. (Hz)", f"{imu_frequency:.2f}")
        
        return table

    def create_status_table(self):
        table = Table(title="UAV Info", title_style="bold magenta")
        table.add_column("Name", justify="center", style="cyan", no_wrap=True)
        table.add_column("Value", justify="center", style="green")

        table.add_row("Flight Mode", self.flight_mode)
        table.add_row("Connection Status", self.connection_status)
        table.add_row("Battery Voltage", self.battery_info["Voltage"])
        table.add_row("Battery Current", self.battery_info["Current"])
        table.add_row("Battery Percentage", self.battery_info["Percentage"])
        
        return table

    def create_program_status_table(self, program_status):
        table = Table(title="Program Status", title_style="bold magenta")
        table.add_column("Program", justify="center", style="cyan", no_wrap=True)
        table.add_column("Status", justify="center", style="green")

        for program, status in program_status.items():
            table.add_row(program, status)
        
        return table

    def update_data(self):
        now = self.get_clock().now().seconds_nanoseconds()
        
        # Calculate time since last odometry and IMU messages
        odom_time_diff = (now[0] - self.odom_last_time[0] + (now[1] - self.odom_last_time[1]) * 1e-9) if self.odom_last_time else float('inf')
        imu_time_diff = (now[0] - self.imu_last_time[0] + (now[1] - self.imu_last_time[1]) * 1e-9) if self.imu_last_time else float('inf')
        
        if odom_time_diff > self.odom_timeout:
            self.pose_info = {
                "Position": {"x": "N/A", "y": "N/A", "z": "N/A"},
                "Orientation (Euler)": {"roll": "N/A", "pitch": "N/A", "yaw": "N/A"},
                "Orientation (Quaternion)": {"x": "N/A", "y": "N/A", "z": "N/A", "w": "N/A"}
            }
        
        if imu_time_diff > self.imu_timeout:
            self.imu_frequency = 0
        
        uav_status = "Operational"
        odometry_health = "Good"  

        program_status = self.check_program_status()

        return uav_status, self.odom_frequency, odometry_health, self.pose_info, self.imu_frequency, program_status

    def run(self):
        with Live(self.layout, console=self.console, refresh_per_second=1) as live:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.1)
                uav_status, odometry_frequency, odometry_health, pose_info, imu_frequency, program_status = self.update_data()

                pose_table = self.create_pose_table(pose_info)
                odometry_table = self.create_odometry_table(odometry_frequency, odometry_health, imu_frequency)
                status_table = self.create_status_table()
                program_status_table = self.create_program_status_table(program_status)

                status_panel_content = Columns([status_table, program_status_table], align="center")
                odometry_panel_content = Columns([pose_table, odometry_table], align="center")

                panel1 = Panel(status_panel_content, title="UAV Status", border_style="green")
                panel2 = Panel(odometry_panel_content, title="Odometry", border_style="red")

                self.layout.split(
                    Layout(panel1, size=14),
                    Layout(panel2, size=12)
                )
                live.update(self.layout)
                #time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    monitor = UAVStatus()
    monitor.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
