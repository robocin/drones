import logging
import time 

class _IntervalFilter(logging.Filter):
    last_log_time = {}

    def __init__(self, min_interval):
        super().__init__()
        self.min_interval = min_interval

    def filter(self, record):
        current_time = time.time()
        if (current_time - _IntervalFilter.last_log_time.get(record.module, 0)) < self.min_interval:
            return False
        _IntervalFilter.last_log_time[record.module] = current_time
        return True

class Logger:
    """
        A class that handles the creation of loggers to write messages on files

        Methods
        -------
        get_logger(logger_name: str, log_file: str, min_interval: float):
            returns a logger with the given logger_name, create one if it doesn't exist
    """
    _loggers = {}

    @staticmethod
    def get_logger(logger_name, log_file=None, min_interval=None):
        """
        static method that returns a logger with the given logger_name, create one if it doesn't exist

        Parâmetros
        ----------
            logger_name : str
                logger name
            log_file : str | None
                name of the log file that is going to be created if the logger does not exist, if None, the logger file name will be the logger_name
            min_interval : float | None
                if not None, is the minimum interval in seconds between writing messages to logger
        """
        if logger_name not in Logger._loggers:
            logger = logging.getLogger(logger_name)
            logger.setLevel(logging.DEBUG)

            formatter = logging.Formatter('%(asctime)s - %(message)s')

            # if file name not specified, use {logger_name}.log
            if not log_file:
                log_file = f"{logger_name}.log"

            file_handler = logging.FileHandler(log_file)
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(formatter)
            logger.addHandler(file_handler)

            # don't show on terminal
            logger.propagate = False 

            # Defining a filter to control the minimal interval between messages
            if min_interval:
                interval_filter = _IntervalFilter(min_interval)
                logger.addFilter(interval_filter)

            Logger._loggers[logger_name] = logger

        return Logger._loggers[logger_name]