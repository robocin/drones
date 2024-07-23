import libtmux
import subprocess
from utils.logger_config import setup_logger
import time
logger = setup_logger()

def create_tmux_session(session_name, window_name):
    server = libtmux.Server()
    session = server.new_session(session_name, kill_session=True, attach=False)

    window = session.new_window(attach=False, window_name=window_name)

    logger.info(f"Created tmux session {session_name} with window {window_name}")
    
    return session

def create_tmux_pane(session_name, window_name, command, direction, size="15%"):
    server = libtmux.Server()
    session = server.find_where({"session_name": session_name})
    if session:
        window = session.find_where({"window_name": window_name})
        if window:
            pane = window.split_window(attach=False, start_directory=None,
                          vertical=direction, size=size)
            pane.send_keys(command)

            logger.info(f"Created tmux pane with command {command}")
        else:
            logger.error(f"Window {window_name} not found in session {session_name}")
    else:
        logger.error(f"Session {session_name} not found")

   
def launch_session(session_name, window_name):
    terminal_command = f"tmux attach-session -t {session_name}:{window_name}"
    subprocess.Popen(['xterm', '-e', terminal_command])

def cleanup_tmux_session(session_name):
    server = libtmux.Server()
    session = server.find_where({"session_name": session_name})
    if session:
        session.kill()