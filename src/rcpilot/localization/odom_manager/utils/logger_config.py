import logging

def setup_logger(name='odom_manager', log_file='odom_manager.log'):
    """Set up and return a logger.

    Args:
        name (str): Name of the logger.
        log_file (str): File path for the log file.

    Returns:
        logging.Logger: Configured logger.
    """
    logger = logging.getLogger(name)
    logger.setLevel(logging.DEBUG) 

    # Check if the logger already has handlers. If not, add handlers.
    if not logger.handlers:
        c_handler = logging.StreamHandler() 
        f_handler = logging.FileHandler(log_file) 

        c_format = logging.Formatter('%(name)s - %(levelname)s - %(message)s')
        f_format = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        c_handler.setFormatter(c_format)
        f_handler.setFormatter(f_format)

        logger.addHandler(c_handler)
        logger.addHandler(f_handler)

    return logger
