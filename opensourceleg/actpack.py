from typing import TypeVar

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from logging import Logger

from opensourceleg.device import Interface, OSLDevice, Units


@dataclass
class Gains:
    """
    Dataclass for controller gains

    Attributes:
        kp (int): Proportional gain
        ki (int): Integral gain
        kd (int): Derivative gain
        K (int): Stiffness of the impedance controller
        B (int): Damping of the impedance controller
        ff (int): Feedforward gain
    """

    kp: int = 0
    ki: int = 0
    kd: int = 0
    K: int = 0
    B: int = 0
    ff: int = 0

    def __repr__(self) -> str:
        return f"kp={self.kp}, ki={self.ki}, kd={self.kd}, K={self.K}, B={self.B}, ff={self.ff}"

    def __ge__(self, other: "Gains") -> bool:
        return all(
            [
                getattr(self, attr) >= getattr(other, attr)
                for attr in Gains.__dataclass_fields__.keys()
            ]
        )

    def applyTransmission(self, gear_ratio: float) -> "Gains":
        """
        Apply a transmission to the gains

        Args:
            gear_ratio (float): The gear ratio to apply
        """
        return Gains(
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            K=self.K / (gear_ratio**2),
            B=self.B / (gear_ratio**2),
            ff=self.ff,
        )


# class ControlMode(Enum):
#     IDLE = 0
#     IMPEDANCE = 1
#     POSITION = 2
#     FULL_STATE = 3

ControlMode = TypeVar("ControlMode")


class Actuator(Interface):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    @property
    @abstractmethod
    def mode(self) -> ControlMode:
        pass

    @mode.setter
    @abstractmethod
    def mode(self, mode: ControlMode) -> None:
        pass

    @property
    @abstractmethod
    def gains(self) -> Gains:
        pass

    @gains.setter
    @abstractmethod
    def gains(self, gains: Gains) -> None:
        pass

    @property
    @abstractmethod
    def position(self) -> float:
        pass

    @position.setter
    @abstractmethod
    def position(self, position: float) -> None:
        pass

    @property
    @abstractmethod
    def velocity(self) -> float:
        pass

    @velocity.setter
    @abstractmethod
    def velocity(self, velocity: float) -> None:
        pass

    @property
    @abstractmethod
    def torque(self) -> float:
        pass

    @torque.setter
    @abstractmethod
    def torque(self, velocity: float) -> None:
        pass


class ActpackMode(ABC):
    _driver: "Actpack"

    def __init__(self, driver: "Actpack", **kwargs):
        super().__init__(**kwargs)
        self._driver: Actpack = driver
        self._gains: Gains = None

    def __str__(self) -> str:
        return f"{self.__class__.__name__}"

    def enter(self):
        self._driver._log.info(f"Entering {self}")
        if self._driver._mode == None:
            self._driver._mode = self
            self._enter()
        elif self._driver._mode != self:
            self._driver._mode.exit()
            self._driver._mode = self
            self._enter()
        else:
            self._driver._log.warn(f"Already in {self}, not not run _enter()")

    def exit(self):
        self._driver._mode = None
        self._current_gains = None
        self._exit()

    @abstractmethod
    def _enter(self):
        pass

    @abstractmethod
    def _exit(self):
        pass


class IdleMode(ActpackMode):
    # MODE: ControlMode = ControlMode.IDLE
    pass


class ImpedenceMode(ActpackMode):
    DEFAULT_GAINS: Gains
    _current_gains: Gains

    def _enter(self):
        self.gains = self.DEFAULT_GAINS

    def _exit(self):
        self._current_gains = None

    @property
    @abstractmethod
    def gains(self) -> Gains:
        pass

    @gains.setter
    @abstractmethod
    def gains(self, gains: Gains) -> None:
        pass

    @property
    @abstractmethod
    def position(self) -> float:
        pass

    @position.setter
    @abstractmethod
    def position(self, position: float) -> None:
        pass


class SpeedMode(ActpackMode):
    DEFAULT_GAINS: Gains
    DEFAULT_VELOCITY: float

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._gains = self.DEFAULT_GAINS
        self._velocity_setpoint = self.DEFAULT_VELOCITY

    def _enter(self):
        self.gains = self._gains
        self.velocity = self._velocity_setpoint

    def _exit(self):
        pass

    @property
    @abstractmethod
    def gains(self) -> Gains:
        pass

    @gains.setter
    @abstractmethod
    def gains(self, gains: Gains) -> None:
        pass

    @property
    @abstractmethod
    def position(self) -> float:
        pass

    @property
    @abstractmethod
    def velocity(self) -> float:
        pass

    @velocity.setter
    @abstractmethod
    def velocity(self, position: float) -> None:
        pass


class FullStateMode(ActpackMode):
    DEFAULT_GAINS: Gains
    DEFAULT_POSITION: float
    DEFAULT_TORQUE: float
    DEFAULT_VELOCITY: float
    _current_gains: Gains
    _position_setpoint: float

    def _enter(self):
        self.gains = self.DEFAULT_GAINS
        self.position = self.DEFAULT_POSITION
        self.torque = self.DEFAULT_TORQUE
        self.velocity = self.DEFAULT_VELOCITY

    def _exit(self):
        self._current_gains = None

    @property
    @abstractmethod
    def gains(self) -> Gains:
        pass

    @gains.setter
    @abstractmethod
    def gains(self, gains: Gains) -> None:
        pass

    @property
    @abstractmethod
    def position(self) -> float:
        pass

    @position.setter
    @abstractmethod
    def position(self, position: float) -> None:
        pass

    @abstractmethod
    def velocity(self, velocity: float) -> None:
        pass

    @property
    @abstractmethod
    def torque(self) -> float:
        pass

    @torque.setter
    @abstractmethod
    def torque(self, position: float) -> None:
        pass


class Actpack(OSLDevice, Actuator):
    CONTROL_MODES: set[ControlMode]
    _mode: ActpackMode

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        self._modes: dict[ControlMode, ActpackMode] = {}
        self._mode: ActpackMode = None
        for mode in self.CONTROL_MODES:
            self._modes[mode] = mode(driver=self)

    @property
    def mode(self) -> ControlMode:
        return self._mode

    @mode.setter
    def mode(self, mode: ControlMode) -> ControlMode:
        for m in self._modes.values():
            if isinstance(m, mode):
                m.enter()
                return
        raise KeyError(f"Mode {mode} not found")

    @property
    def gains(self) -> Gains:
        try:
            return self.mode.gains
        except AttributeError:
            raise RuntimeError(f"{self.mode} does not support reading velocity")

    @gains.setter
    def gains(self, gains: Gains) -> None:
        try:
            self.mode.gains = gains
            self._log.debug(f"Set gains to {gains}")
        except AttributeError:
            raise RuntimeError(f"{self.mode} does not support setting gains")

    @property
    @Units.to_defaults("position")
    def position(self) -> float:
        return self.mode.position

    @position.setter
    def position(self, position: float) -> None:
        try:
            self.mode.position = position
            self._log.debug(f"Set position to {position:.3f} rad")
        except AttributeError:
            raise RuntimeError(f"{self.mode} does not support setting position")

    @property
    def velocity(self) -> float:
        try:
            return self.mode.velocity
        except AttributeError:
            raise NotImplementedError(f"{self.mode} does not support reading velocity")

    @velocity.setter
    def velocity(self, velocity: float) -> None:
        try:
            self.mode.velocity = velocity
            self._log.debug(f"Set velocity to {velocity:.3f} rad/s")
        except AttributeError:
            raise RuntimeError(f"{self.mode} does not support velocity control")

    @property
    def torque(self) -> float:
        try:
            return self.mode.torque
        except AttributeError:
            raise RuntimeError(f"{self.mode} does not support reading torque")

    @torque.setter
    def torque(self, torque: float) -> None:
        try:
            self.mode.torque = torque
            self._log.info(f"Set torque to {torque} Nm")
        except AttributeError:
            raise RuntimeError(f"{self.mode} does not support torque control")


if __name__ == "__main__":
    pass
