import csv
import logging
from abc import ABC, abstractmethod
from logging import getLogger
from logging.handlers import RotatingFileHandler


def _init_root_logger() -> logging.Logger:
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)
    format = "[%(asctime)s][%(name)s][%(levelname)s] %(message)s"
    dateformat = "%I:%M:%S"
    _std_formatter = logging.Formatter(fmt=format, datefmt=dateformat)

    _stream_handler = logging.StreamHandler()
    _stream_handler.setLevel(logging.INFO)
    _stream_handler.setFormatter(_std_formatter)

    root_logger.addHandler(_stream_handler)
    return root_logger


class OSLDeviceLogger:
    _root_logger = _init_root_logger()

    def init_logger(self, parent_logger: logging.Logger = None):
        if parent_logger:
            self._log = parent_logger.getChild(self.name)
        else:
            self._log = self._root_logger.getChild("Debuglogger")

    @property
    @abstractmethod
    def name(self) -> str:
        pass


if __name__ == "__main__":
    pass
