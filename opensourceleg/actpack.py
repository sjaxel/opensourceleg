import logging
import time
from abc import ABC, abstractmethod, abstractproperty
from dataclasses import dataclass, field

import numpy as np
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from TMotorCANControl.mit_can import TMotorManager_mit_can, _TMotorManState

from opensourceleg.device import DeviceDriver, OSLDevice
from opensourceleg.encoder import AS5048A_Encoder, Encoder


class Actpack(OSLDevice):
    """_summary_

    Args:
        OSLDevice (_type_): _description_
    """

    def __init__(
        self,
        name: str = "Actpack",
        driver_type=type[DeviceDriver],
        driver_args: dict = {},
        **kwargs,
    ) -> None:
        """_summary_

        Args:
            name (str): _description_. Defaults to "Actpack".
            driver_type (type): _description_. Defaults to type[DeviceDriver].
            driver_args (dict): _description_. Defaults to {}.
            **kwargs (_type_): _description_.
        """
        super().__init__(name=name, **kwargs)

        self._data: dict = None
        self._mode = None

        self._init_driver(driver_type, driver_args)

        self._modes: dict[str, ActpackMode] = {}

        self._init_modes(driver_type)

    def set_mode(self, mode: str):
        if mode in self._modes:
            if self._mode:
                self._mode.transition(self._modes[mode])
                self._mode = self._modes[mode]
            else:
                self._modes[mode].enter()
                self._mode = self._modes[mode]

        else:
            raise KeyError(f"Mode {mode} not found")

    def _init_driver(self, driver_type: type[DeviceDriver], driver_args: dict) -> None:
        if not issubclass(driver_type, DeviceDriver):
            raise TypeError(f"driver_type must be a subclass of DeviceDriver")
        self._driver = driver_type({**driver_args, **{"parent_logger": self._log}})

    def _init_modes(self, driver_type) -> None:

        for mode in Actpack.subclasses_recursive(ActpackMode):
            if hasattr(mode, "DRIVER_TYPE"):
                if mode.DRIVER_TYPE == driver_type:
                    self._modes[mode.NAME] = mode(self._driver)
                    self._log.info(f"{mode.__name__} added to driver")

    def subclasses_recursive(cls):
        direct = cls.__subclasses__()
        indirect = []
        for subclass in direct:
            indirect.extend(Actpack.subclasses_recursive(subclass))
        return direct + indirect

    def _start(self) -> None:
        self._driver.start()
        self.set_mode("idle")

    def _stop(self) -> None:
        self._driver.stop()

    def _update(self) -> None:
        self._driver.update()


class ActpackMode(ABC):
    def __init__(self, control_mode, driver) -> None:
        self._control_mode = control_mode
        self._driver = driver

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, ActpackMode):
            return self._control_mode == __o._control_mode
        return False

    def __str__(self) -> str:
        return f"Mode: {self.NAME} - {self._control_mode}"

    @property
    def mode(self):
        return self._control_mode

    def enter(self):
        self._driver._log.info(f"[ENTER MODE] {self.NAME}")
        self._enter()

    def exit(self):
        self._driver._log.info(f"[EXIT MODE] {self.NAME}")
        self._exit()

    def transition(self, to_state: "ActpackMode"):
        if self != to_state:
            self.exit()
            to_state.enter()
        else:
            raise ValueError(f"Transition from {self} -> {to_state}")

    @abstractmethod
    def _enter(self):
        pass

    @abstractmethod
    def _exit(self):
        pass


class IdleMode(ActpackMode):
    NAME = "idle"


class ImpedenceMode(ActpackMode):
    NAME = "impedence"


class PositionMode(ActpackMode):
    NAME = "position"


@dataclass()
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


class ModeGains:
    """A class that contains the min, max and default gains for a given control mode"""

    def __init__(
        self, min: Gains = Gains(), max: Gains = Gains(), default: Gains = Gains()
    ) -> None:
        """Initialize the ModeGains class instance

        Args:
            min (Gains): Lower bound of the gains allowed
            max (Gains): Upper bound of the gains allowed
            default (Gains): Default gains

        Raises:
            ValueError: If the default gains are not within the min and max gains
        """
        if not (min <= max):
            raise ValueError("Min gains must be less than max gains")
        if not (min <= default <= max):
            raise ValueError("Default gains must be within the min and max gains")
        self._min = min
        self._max = max
        self._default = default
        self._gains = self._default

    @property
    def gains(self) -> Gains:
        """The current gains for the control mode

        Setter:
        Raises:
            ValueError: If the gains are not within the min and max gains
        """
        return self._gains

    @gains.setter
    def gains(self, gains: Gains):
        if self._min <= gains <= self._max:
            self._gains = gains
        else:
            raise ValueError(
                f"Gains={gains} must be within the min={self._min} and max={self._max} gains"
            )

    def _gain_string(self, attr: str) -> str:
        gain = getattr(self._gains, attr)
        min_gain = getattr(self._min, attr)
        max_gain = getattr(self._max, attr)
        default_gain = getattr(self._default, attr)

        if gain == min_gain == max_gain == default_gain == 0:
            return f"{attr: >2} = 0 (Not used)"

        return f"{attr: >2} = {gain: >3} [{min_gain},{max_gain}] default={default_gain}"

    def __repr__(self) -> str:
        return f"min={self._min}, max={self._max}, default={self._default}, gains={self._gains}"

    def __str__(self) -> str:
        str = ""
        for (key, _) in Gains.__dataclass_fields__.items():
            str += self._gain_string(key) + "\n"
        return str[0:-1]


if __name__ == "__main__":
    # tmotor_actpack = Actpack(name="TMotorActpack1")

    # with tmotor_actpack as actpack:
    #     time.sleep(1)
    #     actpack.update()

    #     loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)

    #     i = 0
    #     toggle = True
    #     for t in loop:
    #         actpack.update()

    mode_gains = ModeGains(
        min=Gains(), max=Gains(kp=200, ki=300), default=Gains(kp=100, ki=200)
    )
    newgains = Gains(kp=200, ki=500)
    mode_gains.gains = newgains
    print(mode_gains)

    pass
