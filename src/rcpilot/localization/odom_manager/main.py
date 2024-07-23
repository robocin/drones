import argparse
import signal
import os
import time
from odom.odom_interface import Odometry
from utils.tmux import create_tmux_session, cleanup_tmux_session, create_tmux_pane
from utils.config import load_config
from utils.logger_config import setup_logger

stop_flag = False
config = load_config()
logger = setup_logger()

def signal_handler(signum, frame):
    global stop_flag
    logger.info("Received signal to stop the program.")
    stop_flag = True
    cleanup_tmux_session(config["tmux"]["session_name"])

def main():
    parser = argparse.ArgumentParser(description='Odometry management system.')

    parser.add_argument("--odom", type=str, help="Odometry system to use [zed, open_vins, vins_fusion]",
                        default="zed")
    parser.add_argument("--type", type=str, help="Type of operation [manual, autonomous]", default="autonomous")
    parser.add_argument("--rviz", type=bool, help="Enable rviz visualization", default=False)
    parser.add_argument("--rqt", type=bool, help="Enable rqt visualization", default=False)
    parser.add_argument("--report", type=bool, help="Enable resource and odom delay monitoring", default=False)
    parser.add_argument("--topics", type=str, nargs='+', help="List of topics to record", default=[])

    args = parser.parse_args()

    session_name = config["tmux"]["session_name"]
    window_name = config["tmux"]["window_name"]
    session = create_tmux_session(session_name, window_name)

    # Run UAVStatus in a tmux pane and pass the shared_data
    uav_status_script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "odom_status/odom_status.py")
    uav_status_cmd = f"python3 {uav_status_script}"
    create_tmux_pane(session_name, window_name, uav_status_cmd, True, "80%")

    odom = Odometry(config, args.odom)
    odom.start()

    mavros_cmd = f"ros2 run mavros mavros_node --ros-args --param fcu_url:={config['mavros']['fcu_url']} --param gcs_url:={config['mavros']['gcs_url']}"
    create_tmux_pane(session_name, window_name, mavros_cmd, False)

    # odom_topic_cmd = f"ros2 topic echo {config['mavros']['odom_topic']}"
    # create_tmux_pane(session_name, window_name, odom_topic_cmd, True)

    if args.topics:
        rosbag_cmd = "ros2 bag record " + " ".join(args.topics)
        logger.info(f"Recording topics: {args.topics}")
        create_tmux_pane(session_name, window_name, rosbag_cmd, False)

    if args.type == "manual":
        pass
    elif args.type == "autonomous":
        mavlink_router_cmd = f"mavlink-routerd {config['mavlink_router']['master_ip']}:{config['mavlink_router']['master_port']} \
        -e {config['mavlink_router']['out1_ip']}:{config['mavlink_router']['out1_port']} \
        -e {config['mavlink_router']['out2_ip']}:{config['mavlink_router']['out2_port']}"
        create_tmux_pane(session_name, window_name, mavlink_router_cmd, False)
    else:
        logger.error("Invalid type of operation")

    logger.info(f"Created tmux session, use 'tmux attach-session -t {session_name}:{window_name}' to view the session.")

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    try:
        while not stop_flag:
            time.sleep(1)
    finally:
        cleanup_tmux_session(session_name)

if __name__ == '__main__':
    main()
