from typing import TypeVar

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum

from TMotorCANControl.mit_can import TMotorManager_mit_can, _TMotorManState

from opensourceleg.actpack import (
    Actpack,
    ActpackMode,
    Actuator,
    FullStateMode,
    Gains,
    IdleMode,
    ImpedenceMode,
    SpeedMode,
)
from opensourceleg.device import Interface, OSLDevice, Units
from opensourceleg.encoder import Encoder


class TMotorMode(ActpackMode):
    _driver: "TMotorActpack"

    @property
    def gains(self) -> Gains:
        return self._gains

    @property
    def driver(self) -> TMotorManager_mit_can:
        return self._driver._driver

    @property
    def position(self) -> float:
        return self.driver.get_output_angle_radians()

    @property
    def velocity(self) -> float:
        return self.driver.get_output_velocity_radians_per_second()

    @property
    def acceleration(self) -> float:
        return self.driver.get_output_acceleration_radians_per_second_squared()

    @property
    def torque(self) -> float:
        return self.driver.get_output_torque_newton_meters()


class TMotorIdleMode(TMotorMode, IdleMode):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    def _enter(self):
        self.driver._control_state = _TMotorManState.IDLE

    def _exit(self):
        pass


class TMotorImpedenceMode(TMotorMode, ImpedenceMode):
    DEFAULT_GAINS = Gains(K=1, B=0.5)
    DEFAULT_POSITION = 0

    @TMotorMode.gains.setter
    def gains(self, gains: Gains) -> None:
        self.driver.set_impedance_gains_real_unit(K=gains.K, B=gains.B)
        self._gains = gains

    @TMotorMode.position.setter
    def position(self, position: float) -> None:
        self.driver.set_output_angle_radians(position)


class TMotorSpeedeMode(TMotorMode, SpeedMode):
    DEFAULT_GAINS = Gains(kd=1)
    DEFAULT_VELOCITY = 0.5

    @TMotorMode.gains.setter
    def gains(self, gains: Gains) -> None:
        self.driver.set_speed_gains(kd=gains.kd)
        self._gains = gains

    @TMotorMode.velocity.setter
    def velocity(self, velocity: float) -> None:
        self.driver.set_output_velocity_radians_per_second(velocity)


class TMotorFullStateMode(TMotorMode, FullStateMode):
    DEFAULT_GAINS = Gains(K=0.5, B=0.1)
    DEFAULT_POSITION = 0
    DEFAULT_VELOCITY = 0.5
    DEFAULT_TORQUE = 0.5

    @TMotorMode.gains.setter
    def gains(self, gains) -> None:
        self.driver.set_impedance_gains_real_unit_full_state_feedback(gains)
        self._current_gains = gains

    @TMotorMode.position.setter
    def position(self, position) -> None:
        self._driver._driver.set_output_angle_radians(position)

    @TMotorMode.velocity.setter
    def velocity(self, velocity: float) -> None:
        self.driver.set_output_velocity_radians_per_second(velocity)

    @TMotorMode.torque.setter
    def torque(self, torque: float) -> None:
        self._driver._driver.set_output_torque_newton_meters(torque)


class TMotorActpack(Actpack, Actuator, Encoder):
    CONTROL_MODES = {
        TMotorIdleMode,
        TMotorImpedenceMode,
        TMotorSpeedeMode,
        TMotorFullStateMode,
    }

    def __init__(self, name: str = "TMotorDriver", motor_id: int = 0, **kwargs) -> None:
        super().__init__(name=name, **kwargs)
        self._driver: TMotorManager_mit_can = TMotorManager_mit_can(motor_ID=motor_id)

    def _start(self):
        self._driver.power_on()
        self._driver._send_command()
        self._driver._entered = True
        self.mode = TMotorIdleMode
        if not self._driver.check_can_connection():
            raise RuntimeError("Device not connected")

    def _stop(self):
        self._driver.power_off()
        self._driver._entered = False

    def _update(self):
        self._driver.update()

    @property
    def encoder_output(self) -> int:
        raise NotImplementedError("TMotor does not report raw encoder output")

    def set_zero(self) -> None:
        """
        Set the current position as the zero position

        User must wait for 1 second after calling this function before issuing any other commands.
        """
        self._driver.set_zero_position()

    @property
    def zero_position(self) -> int:
        """
        Will always return 0 as the TMotor does not have an absolute encoder
        """
        return 0

    @zero_position.setter
    def zero_position(self, value: int) -> None:
        """
        Todo: Should move the motor to the desired zero position and set the zero position to 0
        """
        self._driver.set_zero_position()
