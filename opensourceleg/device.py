from abc import ABC, abstractmethod
from logging import Logger

from opensourceleg.log import OSLDeviceLogger
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition


class OSLDevice(ABC, OSLDeviceLogger):
    """_summary_

    Args:
        ABC (_type_): _description_
        OSLDeviceLogger (_type_): _description_
    """

    def __init__(
        self,
        name: str = "Device",
        units: UnitsDefinition = DEFAULT_UNITS,
        parent_logger: Logger = None,
        debug_level: int = 0,
    ) -> None:
        """Base class for devices in the OSL device tree

        Args:
            name (str): _description_. Defaults to "Device".
            units (UnitsDefinition): _description_. Defaults to DEFAULT_UNITS.
            parent_logger (Logger): _description_. Defaults to None.
            debug_level (int): _description_. Defaults to 0.
        """
        self._debug_level = debug_level
        self._data: dict = None
        self._running = False
        self._units = units if units else DEFAULT_UNITS
        self._log = None
        self.name = name

        self.init_logger(parent_logger=parent_logger)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.stop()

    def start(self):
        if not self._running:
            self._start()
            self._running = True
            self._log.info("[START]")
        else:
            raise RuntimeError("Trying to start device when already running")

    def stop(self):
        if self._running:
            self._stop()
            self._log.info("[STOP]")
            self._running = False
        else:
            raise RuntimeError("Trying to stop device that is not running")

    def update(self):
        if self._running:
            self._update()
            self._log.debug("[UPDATE]")
        else:
            raise RuntimeError("Device is not running")
        pass

    @abstractmethod
    def _start(self):
        pass

    @abstractmethod
    def _stop(self):
        pass

    @abstractmethod
    def _update(self):
        pass

    @property
    def name(self) -> str:
        return self._name

    @name.setter
    def name(self, name: str):
        self._name = name


class DeviceDriver(OSLDevice):
    def __init__(self, name: str = "DeviceDriver", **kwargs) -> None:
        super().__init__(name=name, **kwargs)


class TestDevice(OSLDevice):
    def __init__(self, name: str = "TestDevice", **kwargs) -> None:
        super().__init__(name=name, **kwargs)

    def _start(self):
        self._log.info("Start Test device")

    def _stop(self):
        self._log.info("Stop Test device")

    def _update(self):
        self._log.info("Update Test device")


if __name__ == "__main__":
    with TestDevice(name="ParentDevice") as parent, TestDevice(
        name="ChildDevice", parent_logger=parent._log
    ) as child:
        parent.update()
        child.update()
        pass
