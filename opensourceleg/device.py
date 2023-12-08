from types import TracebackType
from typing import Any, Final, Self, TypedDict, TypeVar, Union

import traceback
from abc import ABC, abstractmethod
from collections.abc import KeysView
from contextlib import ExitStack
from dataclasses import dataclass
from functools import wraps
from logging import Logger
from pathlib import PurePosixPath

from log import getLogger

from opensourceleg.timer import OSLTimer
from opensourceleg.units import DEFAULT_UNITS, UnitsDefinition
from opensourceleg.utilities import SoftRealtimeLoop


class DevicePath(PurePosixPath):
    pass


class Interface(ABC):
    _units: UnitsDefinition

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    class State(TypedDict):
        pass

    @abstractmethod
    def apply_state(self, state: "Interface.State") -> None:
        pass


DeviceInterface = TypeVar("DeviceInterface", bound="Interface")


class DeviceManager:
    """
    A class that handles the device tree and and allows devices to access
    other devices by path-like strings. A device doesn't need to know it's
    absolute path in the tree structure, it can access child devices by
    their relative path.

    It implements PurePath  so that devices can be accessed
    by path-like strings (e.g. /device1/device2/device3).

    The device is stored in a dictionary with the device path as the key and a
    dictionary of device attributes as the value. The device attributes are

    - "device": the device object
    - "interfaces": a list of interfaces that the device implements

    The device tree dictionary is a class attribute so that it is shared between
    all instances of the class. This allows devices to be added to the tree from
    anywhere in the code.

    The OSLDevice class inherits from this class and adds itself to the tree when
    it is instantiated. This allows devices to be added to the tree by simply.
    When a new device is added from within an instance of OSLDevice, the new device
    is added as a child of the device that is adding it.

    A string representation of the device tree can be obtained by calling the
    __str__ method of the DeviceTree class. This will print the tree in a format
    that is similar to the output of the Linux tree command.

    Class Attributes:
        _device_tree: A dictionary that represents the device tree

    """

    _clock: SoftRealtimeLoop = SoftRealtimeLoop()
    _device_tree: dict[DevicePath, "OSLDevice"] = {}
    ROOT = DevicePath("/")
    _log: Logger = None
    _exitStack: ExitStack = None
    _lock = False
    _timer = OSLTimer("DeviceManager(Update)", logger=None)

    def __init__(
        self, path: Union[DevicePath, str] = ROOT, device: "OSLDevice" = None, **kwargs
    ) -> None:
        """Initialise a device manager instance

        Devices are added to the tree when they initialise a DeviceManager instance
        as part of the OSLDevice __init__ method. This allows devices to be added
        to the tree by simply instantiating a DeviceManager instance.
        """
        if self.lock:
            raise RuntimeError("Device tree is locked and cannot be modified")
        super().__init__(**kwargs)
        self._cwd: DevicePath = DevicePath(path)

        ## Initialise the root device manager object.
        if self._cwd == DeviceManager.ROOT:
            if device:
                raise ValueError("Device cannot be specified for root device manager")

            ## Init logger for the device manager root object
            DeviceManager._log = getLogger("/")
            DeviceManager._log.info("Device manager initialised")

        ## Check that the parent path exists in the tree
        elif (
            self._cwd.parent not in DeviceManager._device_tree
            and self._cwd.parent != DeviceManager.ROOT
        ):
            print(f"Class device_tree: {DeviceManager._device_tree}")
            print(f"Instance device_tree: {self._device_tree}")
            raise KeyError(f"Parent device {self._cwd.parent} not found in device tree")
        ## Check that a device does not already exist in the path
        elif self._cwd in DeviceManager._device_tree:
            raise KeyError(f"Device {self._cwd} already exists in device tree")
        ## Check that device is valid
        elif not isinstance(device, OSLDevice):
            raise ValueError(
                "Valid OSLDevice must be specified for non-root device manager"
            )
        ## Initialise the device manager object for a device and add it to the device tree
        else:
            DeviceManager._device_tree[device.path] = device
            device._log.info(f"Added {device} to device tree")

    def __enter__(self):
        """Enter a context manager that locks the device tree and enters all subdevices

        If an exception is raised during the enter process, the current exitStack is
        unwound and the exception is re-raised. This ensures that the all previously
        entered devices are exited before the exception is raised.

        Returns:
            self: The DeviceManager instance

        Raises:
            RuntimeError: If the device tree is already locked
            Exception: Any exception raised during the __enter__ of subdevices in the tree
        """
        if DeviceManager._lock:
            raise RuntimeError("Device tree is locked and cannot be modified")
        try:
            DeviceManager._lock = True
            DeviceManager._exitStack = ExitStack()
            self._enter_subdevices_recursive(self._cwd)
        except Exception as e:
            DeviceManager._lock = False
            DeviceManager._exitStack.close()
            raise e
        else:
            return self

    def __exit__(self, exc_type, exc_value, traceback) -> bool:
        """Exit a context manager that unlocks the device tree and exits all subdevices

        Any exception causing the current level of the context manager to exit will be
        re-raised after the exitStack is unwound. This ensures that all previously entered
        devices are exited before the exception is raised.
        """
        DeviceManager._lock = False
        return DeviceManager._exitStack.__exit__(exc_type, exc_value, traceback)

    def _enter_subdevices_recursive(self, device_path: DevicePath) -> None:
        """Enter all subdevices of a device recursively in a post-order traversal

        Args:
            device_path (DevicePath): The path which will be searched for subdevices
            which will be entered in a post-order traversal to ensure all subdevices are
            entered before their parent device is entered.

        Raises:
            Exception: Any exception raised during the __enter__ of subdevices in the tree

        """
        for subdevice in DeviceManager._get_subdevices(device_path):
            self._enter_subdevices_recursive(subdevice.path)
            DeviceManager._exitStack.enter_context(subdevice)

    @classmethod
    def _get_subdevices(cls, device_path: DevicePath) -> list["OSLDevice"]:
        return cls.match(str(device_path / "./*/"))

    def update(self) -> None:
        """Update all devices in the device tree

        This method calls the update method of all devices in the device tree.

        TODO: No care is taken to the order in which devices are updated. There might
        be a good reason to update devices in a specific order. For example, if a device
        is a child of another device, it might be necessary to update the parent device
        before the child device.
        """
        if not self.lock:
            raise RuntimeError("Device tree is not locked and cannot be updated")
        with self._timer:
            for device in self._device_tree.values():
                device.update()

    @property
    def lock(self) -> bool:
        return DeviceManager._lock

    @property
    def started(self) -> bool:
        return DeviceManager._lock

    @property
    def cwd(self) -> DevicePath:
        return self._cwd

    @property
    def frequency(self) -> float:
        """Frequency of DeviceManager clock

        Returns:
            float: Frequency (Hz) of DeviceManager clock
        """
        return 1 / DeviceManager._clock.dt

    @frequency.setter
    def frequency(self, value: float) -> None:
        """Set frequency of DeviceManager clock

        Args:
            value (float): Frequency (Hz) of DeviceManager clock
        """
        DeviceManager._clock.dt = 1 / value

    @property
    def validPaths(self) -> KeysView[DevicePath]:
        return DeviceManager._device_tree.keys()

    @property
    def clock(self) -> SoftRealtimeLoop:
        return DeviceManager._clock

    def get(self, path: Union[DevicePath, str]) -> "OSLDevice":
        devicepath = self._cwd / path
        try:
            return DeviceManager._device_tree[devicepath]
        except KeyError:
            if path not in DeviceManager._device_tree:
                raise KeyError(f"Device {devicepath} not found in device tree")
        except Exception as e:
            raise e

    def query(
        self, interface: type[DeviceInterface], pattern: str
    ) -> list[DeviceInterface]:
        """Query the device tree for a device that implements an interface

        Args:
            interface (type[DeviceInterface]): Interface to be queried
            pattern (str): Glob-style pattern to match against device paths

        Raises:
            KeyError: If no device is found that implements the interface

        Returns:
            list[DeviceInterface]: Device that implements the interface
        """
        res: list[DeviceInterface] = []
        for key, value in DeviceManager._device_tree.items():
            if key.match(pattern) and interface in value._implemented_interfaces:
                res.append(value)
        if res:
            return res
        else:
            raise KeyError(
                f"No device with pattern '{pattern}' found that implements {interface}"
            )

    def __del__(self) -> None:
        if self._cwd == DeviceManager.ROOT:
            return
        try:
            del DeviceManager._device_tree[self._cwd]
        except KeyError:
            DeviceManager._log.debug(
                f"During deletion of {self}, {self._cwd} not found in device tree"
            )
        else:
            DeviceManager._log.info(f"Removed {self._cwd} from device tree")

    @classmethod
    def match(cls, path: DevicePath) -> list["OSLDevice"]:
        res: list["OSLDevice"] = []
        for key in cls._device_tree.keys():
            if key.match(path):
                cls._log.debug(f"Matched {key} to {path}")
                res.append(cls._device_tree[key])
        return res

    @classmethod
    def getLogger(cls, path: Union[DevicePath, str]) -> Logger:
        return getLogger(str(path))

    @classmethod
    def print_tree(cls) -> None:
        for key, value in cls._device_tree.items():
            cls.print_entry(key, value)

    @staticmethod
    def print_entry(path: DevicePath, value: dict) -> None:
        indent = "| " * (len(path.parts) - 2)
        print(f"{indent}+--{path.name} ({value.__class__.__name__}))")

    def __call__(
        self, key: type[DeviceInterface], device_path: Union[DevicePath, str]
    ) -> DeviceInterface:
        """Device call method to access interfaces implemented by the device

        Args:
            key (type[DeviceInterface]): Interface to be accessed
            device_path (Union[DevicePath, str], optional): Path of the device to be accessed. Defaults to ".".

        Raises:
            AttributeError: If the device does not implement the interface

        Returns:
            DeviceInterface: Interface object
        """
        device = self.get(self.cwd / device_path)
        if key in device._implemented_interfaces:
            return device
        print(f"Interfaces implemented by device is {device._implemented_interfaces}")
        raise AttributeError(f"{self} does not implement {key}")


class OSLDevice(ABC):
    """
    Base class for devices in the OSL device tree

    Attributes:
        _name (str): Name of the device
        _parent_path (DevicePath): Path of the parent device
        _log (Logger): Logger for the device
        _units (UnitsDefinition): Units used by the device
        _implemented_interfaces (list[DeviceInterface]): List of interfaces implemented by the device
    """

    def __init__(
        self,
        name: str = "OSLDevice",
        basepath: Union[DevicePath, str] = DeviceManager.ROOT,
        units: UnitsDefinition = DEFAULT_UNITS,
        log_level: str | int | None = None,
        **kwargs,
    ) -> None:
        super().__init__(**kwargs)
        self._name: str = name
        self._parent_path: Final[DevicePath] = DevicePath(basepath)
        self._log: Logger = DeviceManager.getLogger(self.path)
        self._units: UnitsDefinition = units
        self.devmgr = DeviceManager(path=self.path, device=self)
        self._init_interfaces()
        self._timer: OSLTimer = OSLTimer(self._name, logger=None)

        if log_level is not None:
            self._log.setLevel(log_level)

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type: type, exc_value, exc_traceback):
        if exc_type is KeyboardInterrupt:
            self._log.warning(f"[EXIT] KeyboardInterrupt")
            self.stop()
            return (True, None, None)
        elif exc_type is not None:
            self._log.error(f" Exception in {self} with {exc_type}, {exc_value}")
            print(f"Exiting {self} with {exc_type}, {exc_value}")
            traceback.print_tb(exc_traceback)
        self.stop()
        return (True, None, None)

    def _init_interfaces(self) -> None:
        self._implemented_interfaces: list[DeviceInterface] = []
        for baseclass in self.__class__.__bases__:
            if issubclass(baseclass, Interface):
                self._log.info(
                    f"Adding Interface {baseclass.__name__} to implemented interfaces"
                )
                self._implemented_interfaces.append(baseclass)
            else:
                self._log.info(
                    f"Skipping {baseclass} because it is not a subclass of Interface"
                )

    def start(self) -> None:
        self._log.info(f"START")
        self._start()

    def stop(self) -> None:
        self._log.info(f"STOP")
        self._stop()
        self._log.info(self._timer)

    def update(self) -> None:
        with self._timer:
            self._update()

    @abstractmethod
    def _start(self) -> None:
        pass

    @abstractmethod
    def _stop(self) -> None:
        pass

    @abstractmethod
    def _update(self) -> None:
        pass

    @property
    def name(self) -> str:
        return self._name

    @property
    def path(self) -> DevicePath:
        return self._parent_path / self.name

    def __str__(self) -> str:
        return f"{self.path} ({self.__class__.__name__})"


class Units:
    def to_defaults(attribute: str):
        """Decorator to convert a value to the default units of the attribute

        Example usage:
            @to_defaults("time")
            def get_time(self):
                return self._data["time"]

            equivalent to:
            def get_time(self):
                return self._units.convert_to_default_units(self._data["time"], "time")

        Args:
            attribute (str): Attribute to convert

        Returns:
            float: Converted value
        """

        def decorator(func):
            @wraps(func)
            def wrapper(self: OSLDevice):
                self._log.debug(
                    f"Converting from {self._units[attribute]} to {DEFAULT_UNITS[attribute]} (default)"
                )
                return self._units.convert_to_default_units(func(self), attribute)

            return wrapper

        return decorator


DeviceType = TypeVar("DeviceType", bound=OSLDevice)


if __name__ == "__main__":
    pass
