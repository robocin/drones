#!/usr/bin/env python3
import sys
import time
import rclpy
import argparse
import signal
import os
from rclpy.node import Node
from subprocess import Popen, PIPE
from gazebo_msgs.srv import SpawnEntity

stop_flag = False


def signal_handler(signum, frame):
    global stop_flag
    print("Received signal to stop the program.")
    stop_flag = True
    tmux_cleanup()

def tmux_cleanup():
    """
    Cleans up the tmux session and ensures that the `mavlink-routerd` process is stopped so the ports become available.
    """


    command = 'pkill mavlink-routerd'
    Popen(command.split(), preexec_fn=os.setsid)
    
    command = 'tmux kill-server'
    Popen(command.split(), preexec_fn=os.setsid)

class SimulationModule(Node):
    def __init__(self, params):
        super().__init__('simulation_module')

        # Simulation module parameters
        self.params = params

        # Register signal handler for graceful shutdown
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        # Create a new tmux session
        self.tmux_session_name = "simulation"
        self.tmux_create_session()

        # Initialize the simulator
        self.start_simulator()
        time.sleep(2)

        # Initialize MAVLink router
        self.start_mavlink_router()
        time.sleep(2)

        # Spawn the drone model in the simulator
        self.spawn_drone()
        time.sleep(2)

        # Initialize the drone SITL
        self.start_sitl()
        time.sleep(2)

        # Initialize the ground station
        self.start_ground_station()
        time.sleep(2)

        # Initialize MAVROS
        self.start_mavros()

    def tmux_create_session(self):
        """
        Creates a new tmux session.
        """
        command = f'tmux new-session -d -s {self.tmux_session_name}'
        Popen(command.split(), preexec_fn=os.setsid)

    def tmux_new_window(self, name):
        """
        Creates a new window in the tmux session.
        """
        command = f'tmux new-window -t {self.tmux_session_name} -n {name}'
        Popen(command.split(), preexec_fn=os.setsid)


    def tmux_new_pane(self, command):
        """
        Splits the current tmux window and runs a command in the new pane.
        """
        tmux_command = f'tmux split-window -t {self.tmux_session_name} -v "{command}"'
        Popen(tmux_command, shell=True, preexec_fn=os.setsid)
 
        layout_command = f'tmux select-layout -t {self.tmux_session_name} tiled'
        Popen(layout_command, shell=True, preexec_fn=os.setsid)


    def start_simulator(self):
        """
        Starts the Gazebo simulator with the specified world.
        """
        command = 'ros2 launch gazebo_ros gazebo.launch.py'
        if self.params['world']:
            command += f" world:={self.params['world']}"
        self.get_logger().info(f'Starting simulator with command: {command}')
        self.tmux_new_window('gazebo')
        self.tmux_new_pane(command)

    def spawn_drone(self):
        """
        Spawns the drone model in the Gazebo simulator.
        """
        client = self.create_client(SpawnEntity, '/spawn_entity')
        client.wait_for_service()

        request = SpawnEntity.Request()
        request.name = "iris_demo"
        request.xml = "<?xml version=\"1.0\" ?><sdf version=\"1.6\"><model name=\"iris_demo\"> \
                       <include><uri>model://iris_with_ardupilot</uri></include></model></sdf>"
        request.initial_pose.position.x = 0.0
        request.initial_pose.position.y = 0.0
        request.initial_pose.position.z = 0.5

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info('Drone model spawned in Gazebo.')

    def start_sitl(self):
        """
        Starts the SITL for the drone.
        """
        command = 'sim_vehicle.py -v ArduCopter -f gazebo-iris --no-mavproxy'
        self.get_logger().info(f'Starting SITL with command: {command}')
        self.tmux_new_pane(command)

    def start_ground_station(self):
        """
        Starts the ground station for the drone.
        """
        protocol = self.params['mavproxy_protocol']

        command = f'mavproxy.py --master {protocol}:127.0.0.1:{self.params["mavproxy_in_port"]}'
        self.get_logger().info(f'Starting ground station with command: {command}')
        self.tmux_new_pane(command)

    def start_mavlink_router(self):
        """
        Starts the MAVLink router.
        """
        command = f'mavlink-routerd -e {self.params["router_external_ip"]}:{self.params["router_external_port"]} \
                   -e 127.0.0.1:14562 \
                   -e 127.0.0.1:{self.params["router_internal_port"]} \
                   127.0.0.1:{self.params["router_master_port"]} \
                   -t 0'
        self.get_logger().info(f'Starting MAVLink router with command: {command}')
        self.tmux_new_pane(command)

    def start_mavros(self):
        """
        Starts MAVROS with the specified parameters.
        """
        command = f'ros2 launch mavros apm.launch fcu_url:={self.params["fcu_url"]} gcs_url:={self.params["gcs_url"]}'
        self.get_logger().info(f'Starting MAVROS with command: {command}')
        self.tmux_new_pane(command)

def main():
    parser = argparse.ArgumentParser(description='Simulation Module Parameters')
    parser.add_argument('--world', default='', help='Gazebo world file')
    parser.add_argument('--mavproxy_protocol', default='udp', help='MAVProxy protocol')
    parser.add_argument('--mavproxy_in_port', default='14551', help='MAVProxy master port')
    parser.add_argument('--router_master_port', default='14550', help='MAVLink router input port')
    parser.add_argument('--router_external_ip', default='127.0.0.1', help='MAVLink router external IP')
    parser.add_argument('--router_external_port', default='14553', help='MAVLink router external port')
    parser.add_argument('--router_internal_port', default='14551', help='MAVLink router internal port')
    parser.add_argument('--fcu_url', default='tcp://127.0.0.1:5760', help='FCU URL for MAVROS')
    parser.add_argument('--gcs_url', default='udp://@127.0.0.1:14550', help='GCS URL for MAVROS')

    args = parser.parse_args()
    params = vars(args)

    rclpy.init()
    simulation = SimulationModule(params)

    terminal_command = f'gnome-terminal -- bash -c "tmux attach -t {simulation.tmux_session_name}"'
    Popen(terminal_command, shell=True, preexec_fn=os.setsid)


    try:
        while not stop_flag:
            time.sleep(1)
    finally:
        tmux_cleanup()
        simulation.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
