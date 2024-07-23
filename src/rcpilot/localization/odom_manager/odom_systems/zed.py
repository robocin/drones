from utils.tmux import create_tmux_pane

class Zed:
    def __init__(self, config):
        self.config = config

    def start_vio(self):
        create_tmux_pane(self.config["tmux"]["session_name"], self.config["tmux"]["window_name"],
                f"ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i", True, "10%")

    def stop_vio(self):
        pass

    def get_current_pose(self):
        pass
