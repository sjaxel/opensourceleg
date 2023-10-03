#!/usr/bin/python3
# A simple and scalable Finite State Machine module

from typing import Any, Callable, ClassVar, List, Optional

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from logging import Logger

from transitions import Machine, State

from opensourceleg.actpack import ActpackMode, Gains
from opensourceleg.device import DeviceManager
from opensourceleg.joints import Joint


@dataclass
class JointState:

    joint: str
    mode: ActpackMode = None
    angle: float = None
    impedance: Gains = None
    velocity: float = None
    velocity_gains: Gains = None
    torque: float = None
    devmgr: ClassVar[DeviceManager]

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        assert self.joint, "Joint must be set"
        joint = self.devmgr(Joint, self.joint)

        if self.mode:
            joint.mode = self.mode
        if self.velocity_gains:
            joint.gains = self.velocity_gains
        if self.impedance:
            joint.impedance = self.impedance
        if self.angle:
            joint.position = self.angle
        if self.velocity:
            joint.velocity = self.velocity
        if self.torque:
            joint.torque = self.torque

    @classmethod
    def set_devmgr(cls, devmgr: DeviceManager) -> None:
        JointState.devmgr = devmgr


class OSLState(ABC):
    NAME: str
    _devmgr: DeviceManager
    JOINT_STATES: list[JointState]

    def __init__(self, devmgr: DeviceManager, sm: Machine) -> None:
        self._state = State(name=self.NAME)
        self._devmgr = devmgr
        ## Add all joint states setting on enter
        for js in self.JOINT_STATES:
            self._state.add_callback("enter", js)

        sm.add_state(self._state)

        sm.add_transitions(self.init_transitions())

    @abstractmethod
    def init_transitions(self) -> list[dict]:
        pass


if __name__ == "__main__":
    print("This is a module.")
