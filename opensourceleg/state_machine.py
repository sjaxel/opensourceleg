#!/usr/bin/python3
# A simple and scalable Finite State Machine module

from typing import Dict, Any, Callable, ClassVar, List, Optional

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from logging import Logger
from time import monotonic

from transitions import Machine, State

from opensourceleg.actpack import ActpackMode, Gains, IdleMode
from opensourceleg.device import DeviceManager, DevicePath, OSLDevice
from opensourceleg.joints import Joint

class DeviceSettings(ABC):
    @abstractmethod
    def apply_to(self, device: OSLDevice) -> None:
        pass


@dataclass
class JointState(DeviceSettings):

    mode: ActpackMode = None
    angle: float = None
    impedance: Gains = None
    velocity: float = None
    velocity_gains: Gains = None
    torque: float = None

    def apply_to(self, joint: Joint) -> None:
        if self.mode:
            joint.mode = self.mode
        if self.velocity_gains:
            joint.gains = self.velocity_gains
        if self.impedance:
            joint.impedance = self.impedance
        if self.angle is not None:
            joint.position = self.angle
        if self.velocity is not None:
            joint.velocity = self.velocity
        if self.torque is not None:
            joint.torque = self.torque

class StateSettings:

    def __init__(self, keys: List[DevicePath]):
        self._settings: dict[DevicePath, dict] = dict.fromkeys(keys, {})

    def __getitem__(self, path: DevicePath | str) -> dict:
        if isinstance(path, str):
            path = DevicePath(path)
        for device, settings in self._settings.items():
            if device.match(path):
                return {device, settings}
        raise KeyError(f"No settings found for pattern {path}")
    
    def __setitem__(self, path: DevicePath | str, settings: DeviceSettings) -> None:
        if isinstance(path, str):
            path = DevicePath(path)

        self._settings[path] = settings

    def __delitem__(self, path: DevicePath) -> None:
        del self._settings[path]

    def items(self) -> list[tuple[DevicePath, DeviceSettings]]:
        return list(self._settings.items())

    def __str__(self) -> str:
        return str(self._settings)

class OSLState(State, ABC):
    NAME: str

    def __init__(self, devmgr: DeviceManager,  **kwargs) -> None:
        super().__init__(name=self.NAME, 
                         on_enter=[self.apply_settings, self._entry_timestamp],
                         **kwargs)
        self._devmgr = devmgr
        self._entry_time: float = 0
        self._log: Logger = self._devmgr.getLogger("STATE:" + self.name)
        self._init_settings_keys()
        self.init_state_settings()
        self._log.info(f"Initialized")


    def _init_settings_keys(self) -> None:
        loaded_devices = self._devmgr._device_tree.keys()
        self._stateSettings: StateSettings = StateSettings(loaded_devices)

    def _entry_timestamp(self) -> None:
        self._entry_time = monotonic()
    
    def timeout(self, timeout: float) -> Callable[[], bool]:
        t = timeout
        def _check_timeout() -> bool:
            return monotonic() - self._entry_time > t
        return _check_timeout
    
    @property
    def settings(self) -> StateSettings:
        return self._stateSettings
    
    def apply_settings(self) -> None:
        for path, settings in self._stateSettings.items():
            if settings:
                device = self._devmgr(Joint, path)
                self._log.info(f"Applying {settings} to {device}")
                settings.apply_to(device)

    @abstractmethod
    def init_transitions(self) -> list[dict]:
        pass

    @abstractmethod
    def init_state_settings(self) -> None:
        pass



class OSLMachine(Machine):
    state_cls = OSLState

class OffState(OSLState):
    NAME = "off"
    JOINT_STATES: dict[str, JointState] = {}

    def init_transitions(self) -> list[dict]:
        state_transitions = [
            {
                "trigger": "started",
                "source": "off",
                "dest": "idle",
            },
            {
                "trigger": "stopped",
                "source": "*",
                "dest": "off",
            },
        ]
        return state_transitions
    
    def init_state_settings(self) -> None:
        pass   

if __name__ == "__main__":
    js1 = JointState(mode=IdleMode, angle=0.5)
    js1_dict = dir(js1)
    js2 = JointState(**js1_dict)
    print(js2)