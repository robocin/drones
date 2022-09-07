import os
import sys
import subprocess
from typing import List

class ColorPrint:
    def __init__(self, color: str):
        self.color = color

    def __call__(self, *args, **kwargs):
        print(self.color, end='')
        print(*args, **kwargs)
        print('\033[0m', end='')


def cprint(color: str) -> ColorPrint:
    return ColorPrint(color=color)


def check_is_sudo():
    cprint('\x1B[01;94m')('checking is sudo...')
    if is_sudo():
        cprint('\x1B[32m')('[OK] sudo privileges are allowed.')
    else:
        cprint('\x1B[31m')('[ERROR] this script should run with sudo privileges.')
        exit(-1)


def is_sudo():
    return os.geteuid() == 0


def run(command):
    subprocess.check_call(command, shell=True, stdout=sys.stdout, stderr=subprocess.STDOUT)


def check_dependencies():
    raise NotImplementedError

def install_px4_autopilot():
    cprint('\x1B[34m')('installing PX4-Autopilot...')
    try:
        run('git clone https://github.com/PX4/PX4-Autopilot.git --recursive')
    except:
        cprint('\x1B[31m')('[ERROR] could not install PX4-Autopilot')
    else:
        cprint('\x1B[32m')('[OK] PX4-Autopilot installed')


def build_px4_autopilot():
    cprint('\x1B[34m')('building PX4-Autopilot...')
    try:
        run('bash ./PX4-Autopilot/Tools/setup/ubuntu.sh')
    except:
        cprint('\x1B[31m')('[ERROR] could not install PX4-Autopilot')
    else:
        cprint('\x1B[32m')('[OK] PX4-Autopilot built')


def download_qgroundcontrol():
    cprint('\x1B[34m')('installing QGroundControl...')
    try:
        run('wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage')
    except:
        cprint('\x1B[31m')('[ERROR] could not get QGroundControl')
    else:
        cprint('\x1B[32m')('[OK] QGroundControl downloaded')

def update_qgroundcontrol_permissions():
    cprint('\x1B[34m')('updating QGroundControl permissions...')
    try:
        run('chmod +x ./QGroundControl.AppImage')
    except:
        cprint('\x1B[31m')('[ERROR] could not update QGroundControl permissions')
    else:
        cprint('\x1B[32m')('[OK] QGroundControl permissions updated')


def install_comm_mavlink():
    cprint('\x1B[34m')('installing mavlink...')
    try:
        run('pip3 install mavlink')
    except:
        cprint('\x1B[31m')('[ERROR] could not install mavlink')
    else:
        cprint('\x1B[32m')('[OK] mavlink installed')


def install_comm_mavsdk():
    cprint('\x1B[34m')('installing mavsdk...')
    try:
        run('pip3 install mavsdk')
    except:
        cprint('\x1B[31m')('[ERROR] could not install mavsdk')
    else:
        cprint('\x1B[32m')('[OK] mavsdk installed')


def install_sequence():
    # Check permissions
    check_is_sudo()

    # Flight stack
    install_px4_autopilot()
    build_px4_autopilot()
    download_qgroundcontrol()
    update_qgroundcontrol_permissions()

    # Communication libraries
    install_comm_mavlink()
    install_comm_mavsdk


if __name__ == '__main__':
    check_is_sudo()