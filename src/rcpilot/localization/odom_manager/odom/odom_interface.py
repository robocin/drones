from odom_systems.zed import Zed
from odom_systems.open_vins import OpenVins

class Odometry():
    def __init__(self, config, vio_name):
        self.config = config
        self.vio_name = vio_name
        self.vio_system = None

        if self.vio_name == 'zed':
            self.vio_system = Zed(self.config)
        elif self.vio_name == 'open_vins':
            self.vio_system = OpenVins(self.config)
        else:
            raise ValueError("Unsupported VIO type")

    def start(self):
        if self.vio_system:
            self.vio_system.start_vio()

    def stop(self):
        if self.vio_system:
            self.vio_system.stop_vio()

    def get_pose(self):
        pass
