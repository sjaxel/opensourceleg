import logging
import time
from abc import ABC, abstractmethod, abstractproperty

import numpy as np
from NeuroLocoMiddleware.SoftRealtimeLoop import SoftRealtimeLoop
from TMotorCANControl.mit_can import TMotorManager_mit_can, _TMotorManState

from opensourceleg.encoder import AS5048A_Encoder, Encoder
from opensourceleg.hardware import (
    CONTROL_MODE,
    DEFAULT_CURRENT_GAINS,
    DEFAULT_IMPEDANCE_GAINS,
    DEFAULT_POSITION_GAINS,
    DEFAULT_UNITS,
    NM_PER_AMP,
    NM_PER_MILLIAMP,
    RAD_PER_COUNT,
    RAD_PER_DEG,
    Gains,
    UnitsDefinition,
)


class Actpack(ABC):
    """Abstract base class implementing the Actpack interface

    Attributes:
        DEFAULT_IMPEDANCE_GAINS (Gains)

    Raises:
        KeyError: _description_
        ValueError: _description_
        KeyError: _description_

    Returns:
        _type_: _description_
    """

    @property
    def DEFAULT_IMPEDANCE_GAINS(self) -> Gains:
        """
        Gains: Default impedence gains for the Actpack
        """
        pass

    @property
    def DEFAULT_CURRENT_GAINS(self) -> Gains:
        """
        Gains: Default current gains for the Actpack
        """
        pass

    @property
    def DEFAULT_POSITION_GAINS(self) -> Gains:
        """
        Gains: Default position gains for the Actpack
        """
        pass

    def __init__(
        self,
        name: str = "Actpack",
        units: UnitsDefinition = DEFAULT_UNITS,
        logger: logging.Logger = None,
        debug_level: int = 0,
    ) -> None:
        """
        Initializes the Actpack class

        Args:

        """
        self._debug_level = debug_level
        self._data: dict = None
        if logger:
            self._log = logger.getChild(name)
        else:
            self._log = logging.getLogger(name)

        self._state = None
        self._units = units if units else DEFAULT_UNITS
        self._mode = None

        self._motor_zero_position = 0.0
        self._joint_zero_position = 0.0

        self._modes: dict[str, ActpackMode] = {}

    @abstractmethod
    def start(self):
        pass

    @abstractmethod
    def stop(self):
        pass

    @abstractmethod
    def update(self):
        pass

    def set_mode(self, mode: str):
        if mode in self._modes:
            if self._mode:
                self._mode.transition(self._modes[mode])
                self._mode = self._modes[mode]
            else:
                self._modes[mode].enter()
                self._mode = self._modes[mode]

        else:
            self._log.warning(f"Mode {mode} not found")
            return

    def set_motor_zero_position(self, position: float):
        self._motor_zero_position = position

    def set_joint_zero_position(self, position: float):
        self._joint_zero_position = position

    def set_position_gains(
        self,
        gains: Gains = DEFAULT_POSITION_GAINS,
        force: bool = True,
    ):
        """
        Sets the position gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            kd (int): The derivative gain
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["position"]:
            self._log.warning(f"Cannot set position gains in mode {self._mode}")
            return

        self._mode._set_gains(gains)

    def set_current_gains(
        self,
        gains: Gains = DEFAULT_CURRENT_GAINS,
        force: bool = True,
    ):

        """
        Sets the current gains

        Args:
            kp (int): The proportional gain
            ki (int): The integral gain
            ff (int): The feedforward gain
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["current"]:
            self._log.warning(f"Cannot set current gains in mode {self._mode}")
            return

        self._mode._set_gains(gains)

    def set_impedance_gains(
        self,
        gains: Gains = DEFAULT_IMPEDANCE_GAINS,
        force: bool = True,
    ):
        """
        Sets the impedance gains

        Args:
            gains (Gains): _description_
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["impedance"]:
            self._log.warning(f"Cannot set impedance gains in mode {self._mode}")
            return

        self._mode._set_gains(gains)

    def set_voltage(self, value: float, force: bool = False):
        """
        Sets the q axis voltage

        Args:
            value (float): The voltage to set
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["voltage"]:
            self._log.warning(f"Cannot set voltage in mode {self._mode}")
            return

        self._mode._set_voltage(
            int(self._units.convert_to_default_units(value, "voltage")),
        )

    def set_current(self, value: float, force: bool = False):
        """
        Sets the q axis current

        Args:
            value (float): The current to set
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["current"]:
            self._log.warning(f"Cannot set current in mode {self._mode}")
            return

        self._mode._set_current(
            int(self._units.convert_to_default_units(value, "current")),
        )

    def set_motor_torque(self, torque: float, force: bool = False):
        """
        Sets the motor torque

        Args:
            torque (float): The torque to set
            force (bool): Force the mode transition. Defaults to False.
        """
        if self._mode != self._modes["current"]:
            self._log.warning(f"Cannot set motor_torque in mode {self._mode}")
            return

        self._mode._set_current(
            int(
                self._units.convert_to_default_units(torque, "torque") / NM_PER_MILLIAMP
            ),
        )

    def set_motor_position(self, position: float):
        """
        Sets the motor position

        Args:
            position (float): The position to set
        """
        if self._mode not in [self._modes["position"], self._modes["impedance"]]:
            self._log.warning(f"Cannot set motor position in mode {self._mode}")
            return

        self._mode._set_motor_position(
            int(
                self._units.convert_to_default_units(position, "position")
                / RAD_PER_COUNT
            ),
        )

    # Read only properties from the actpack

    @property
    def units(self):
        return self._units

    @property
    def frequency(self):
        return self._frequency

    @property
    def mode(self):
        return self._mode

    @property
    def motor_zero_position(self):
        return self._motor_zero_position

    @property
    def joint_zero_position(self):
        return self._joint_zero_position

    @property
    @abstractmethod
    def battery_voltage(self) -> float:
        pass

    @property
    @abstractmethod
    def batter_current(self) -> float:
        pass

    @property
    @abstractmethod
    def motor_voltage(self) -> float:
        pass

    @property
    @abstractmethod
    def motor_current(self) -> float:
        pass

    @property
    @abstractmethod
    def motor_torque(self) -> float:
        pass

    @property
    @abstractmethod
    def motor_position(self) -> float:
        pass

    @property
    @abstractmethod
    def motor_velocity(self) -> float:
        pass

    @property
    @abstractmethod
    def motor_acceleration(self) -> float:
        pass

    @property
    @abstractmethod
    def motor_torque(self) -> float:
        pass

    @property
    @abstractmethod
    def joint_position(self) -> float:
        pass

    @property
    @abstractmethod
    def joint_velocity(self) -> float:
        pass

    @property
    @abstractmethod
    def genvars(self):
        pass

    @property
    @abstractmethod
    def acc_x(self) -> float:
        pass

    @property
    @abstractmethod
    def acc_y(self) -> float:
        pass

    @property
    @abstractmethod
    def acc_z(self) -> float:
        pass

    @property
    @abstractmethod
    def gyro_x(self):
        pass

    @property
    @abstractmethod
    def gyro_y(self):
        pass

    @property
    @abstractmethod
    def gyro_z(self):
        pass


class TMotorActpack(TMotorManager_mit_can, Actpack):
    DEFAULT_IMPEDANCE_GAINS = Gains(K=2, B=0.1)
    """
    Default impedence gains for the TMotorActpack
    
    Attributes
        K: The stiffness in Nm/rad
        B: The damping in Nm/(rad/s)
    """

    DEFAULT_CURRENT_GAINS = Gains(kp=40, ki=400, ff=128)
    """
    Default current gains for the TMotorActpack
    
    Attributes
        K: The stiffness in Nm/rad
        B: The damping in Nm/(rad/s)
    """

    def __init__(
        self,
        motor_ID=None,
        encoder: Encoder = None,
        name: str = "Actpack",
        units: UnitsDefinition = DEFAULT_UNITS,
        logger: logging.Logger = None,
        debug_level: int = 0,
    ):
        assert motor_ID
        assert encoder
        assert issubclass(type(encoder), Encoder)
        self.encoder = encoder

        TMotorManager_mit_can.__init__(self, motor_ID=motor_ID)
        Actpack.__init__(
            self, name=name, units=units, logger=logger, debug_level=debug_level
        )

        self._log.info(self.DEFAULT_IMPEDANCE_GAINS)

        self._modes: dict[str, ActpackMode] = {
            "impedance": TMotorImpedanceMode(self),
            "position": TMotorPositionMode(self),
        }

    def __enter__(self):
        """
        Start the actpack and the joint encoder
        """

        # Overridden TMotor __enter__ to avoid incopatible logging module
        self._log.info("Activate: " + self.device_info_string())
        self.power_on()  # TODO: How to control this?
        self._send_command()
        self._entered = True
        if not self.check_can_connection():
            raise RuntimeError(
                "Device not connected: " + str(self.device_info_string())
            )

        # Start the encoder
        self.encoder.open()
        self.set_mode("impedance")
        return self

    def __exit__(self, etype, value, tb):
        """
        Stop the actpack and the joint encoder
        """
        self._log.info("Deactivate: " + self.device_info_string())
        self.power_off()  # TODO: How to control this

        # Stop the encoder
        self.encoder.close()

        # if not (etype is None):
        #     traceback.print_exception(etype, value, tb)

    def start(self):
        raise NotImplementedError()

    def stop(self):
        raise NotImplementedError()

    def update(self):
        Encoder.update(self.encoder)
        TMotorManager_mit_can.update(self)

    @property
    def battery_voltage(self) -> float:
        raise NotImplementedError()

    @property
    def batter_current(self) -> float:
        raise NotImplementedError()

    @property
    def battery_voltage(self) -> float:
        raise NotImplementedError()

    @property
    def batter_current(self) -> float:
        raise NotImplementedError()

    @property
    def motor_voltage(self) -> float:
        raise NotImplementedError()

    @property
    def motor_current(self) -> float:
        return self.get_current_qaxis_amps()

    @property
    def motor_torque(self) -> float:
        return self.get_output_torque_newton_meters()

    @property
    def motor_position(self) -> float:
        return float(self.position)

    @property
    def motor_velocity(self) -> float:
        return self.get_motor_velocity_radians_per_second()

    @property
    def motor_acceleration(self) -> float:
        return self.get_motor_acceleration_radians_per_second_squared()

    @property
    def joint_position(self) -> float:
        return self.encoder.encoder_position

    @property
    def joint_velocity(self) -> float:
        return self.encoder.encoder_velocity

    @property
    def genvars(self):
        raise NotImplementedError()

    @property
    def acc_x(self) -> float:
        raise NotImplementedError()

    @property
    def acc_y(self) -> float:
        raise NotImplementedError()

    @property
    def acc_z(self) -> float:
        raise NotImplementedError()

    @property
    def gyro_x(self):
        raise NotImplementedError()

    @property
    def gyro_y(self):
        raise NotImplementedError()

    @property
    def gyro_z(self):
        raise NotImplementedError()


class ActpackMode:
    def __init__(self, control_mode, device: Actpack) -> None:
        self._control_mode = control_mode
        self._device = device
        self._entry_callback: callable = lambda: None
        self._exit_callback: callable = lambda: None

        self._has_gains = False

    def __eq__(self, __o: object) -> bool:
        if isinstance(__o, ActpackMode):
            return self._control_mode == __o._control_mode
        return False

    def __str__(self) -> str:
        return str(self._control_mode)

    @property
    def mode(self):
        return self._control_mode

    @property
    def has_gains(self) -> bool:
        return self._has_gains

    def enter(self):
        self._entry_callback()

    def exit(self):
        self._exit_callback()

    def transition(self, to_state: "ActpackMode"):
        self.exit()
        to_state.enter()


class TMotorPositionMode(ActpackMode):
    def __init__(self, device: TMotorActpack):
        super().__init__(_TMotorManState.FULL_STATE, device)
        self._device = device
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        self._device._log.info(f"[ENTER MODE] {__class__.__name__}")
        if not self.has_gains:
            self._set_gains()

        self._set_motor_position(
            int(
                self._device._units.convert_to_default_units(
                    self._device.motor_position, "position"
                )
            )
        )

    def _exit(self):
        self._has_gains = False
        self._device._log.info(f"[EXIT MODE] {__class__.__name__}")
        pass

    def _set_motor_position(self, counts: int):
        """Sets the motor position

        Args:
            counts (int): position in counts
        """
        self._device.position = counts * RAD_PER_COUNT

    def _set_gains(self, gains: Gains = TMotorActpack.DEFAULT_IMPEDANCE_GAINS):
        self._device.set_impedance_gains_real_unit_full_state_feedback(
            K=gains.K, B=gains.B
        )
        self._has_gains = True


class TMotorImpedanceMode(ActpackMode):
    def __init__(self, device: TMotorActpack):
        super().__init__(_TMotorManState.IMPEDANCE, device)
        self._device = device
        self._entry_callback = self._entry
        self._exit_callback = self._exit

    def _entry(self):
        self._device._log.info(f"[ENTER MODE] {__class__.__name__}")
        if not self.has_gains:
            self._set_gains()

        self._set_motor_position(
            int(
                self._device._units.convert_to_default_units(
                    self._device.motor_position, "position"
                )
            )
            / RAD_PER_COUNT
        )

    def _exit(self):
        self._has_gains = False
        self._device._log.info(f"[EXIT MODE] {__class__.__name__}")
        pass

    def _set_motor_position(self, counts: int):
        """Sets the motor position

        Args:
            counts (int): position in counts
        """
        self._device.position * RAD_PER_COUNT

    def _set_gains(self, gains: Gains = TMotorActpack.DEFAULT_IMPEDANCE_GAINS):
        self._device.set_impedance_gains_real_unit(K=gains.K, B=gains.B)
        self._has_gains = True


if __name__ == "__main__":
    DEG2RAD = 2 * np.pi / 360
    enc = AS5048A_Encoder(bus="/dev/i2c-1")

    logging.basicConfig(
        format="[%(asctime)s][%(name)s][%(levelname)s] %(message)s",
        datefmt="%I:%M:%S",
        level=logging.INFO,
    )

    tmotor_actpack = TMotorActpack(logger=logging.getLogger(), motor_ID=10, encoder=enc)

    with tmotor_actpack as actpack:

        actpack.set_zero_position()
        time.sleep(1)
        actpack.update()
        enc_offset = actpack.encoder.encoder_position

        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)

        actpack.set_impedance_gains(Gains(K=0.1, B=0.1))
        i = 0
        toggle = True
        for t in loop:
            actpack.update()
            i += 1
            if not (i % 100):
                actpack._log.info(f"Motor position: {(actpack.position / 5):.3f}")
                actpack._log.info(
                    f"Encoder position: {((-1) * (actpack.encoder.encoder_position - enc_offset)):.3f}"
                )
            if not (i % 500):
                if toggle:
                    actpack.set_mode("position")
                    actpack.velocity = 0
                    actpack.position = -25 * DEG2RAD * 5
                    toggle = not toggle
                else:
                    actpack.set_mode("position")
                    actpack.velocity = 0
                    actpack.position = 25 * DEG2RAD * 5
                    toggle = not toggle

    pass
