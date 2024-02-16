from .generic_logger import GenericLogger, LoggingLevel

class ROS2Logger:
    '''
    Logger class compatible with ROS 2 using rclpy
    '''

    def __init__(self) -> None:
        self._logger = None
        # generic logger as fallback
        self._generic_logger = GenericLogger()

    def setNode(self, node) -> None:
        self._logger = node.get_logger()

    def log(self, level: LoggingLevel, message: str) -> None:
        # use generic logger if invalid ROS 2 logger
        if self._logger is None:
            self._logger = self._generic_logger

        if level == LoggingLevel.DEBUG:
            self._logger.debug(f'{message}')
        if level == LoggingLevel.INFO:
            self._logger.info(f'{message}')
        if level == LoggingLevel.WARN:
            self._logger.warn(f'{message}')
        if level == LoggingLevel.ERROR:
            self._logger.error(f'{message}')
        if level == LoggingLevel.FATAL:
            self._logger.fatal(f'{message}')
