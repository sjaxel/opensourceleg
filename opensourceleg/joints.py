from abc import abstractmethod
from dataclasses import dataclass
from math import isclose
from time import sleep

from opensourceleg.actpack import (
    ActpackMode,
    Actuator,
    FullStateMode,
    Gains,
    IdleMode,
    ImpedenceMode,
    OSLDevice,
    SpeedMode,
    Units,
)
from opensourceleg.encoder import Encoder


class Transmission:
    """Transmission class for converting between driver and driven values

    TODO: This is confusing and causes extremly obtuse code. Should
    be replaced with a more intuitive system.
    """

    def __init__(self, ratio: tuple[int, int]) -> None:
        """
        Args:
            ratio (tuple[int, int]): (driven, driver) gear ratio
        """
        self._ratio = ratio

    def get(self, value: float) -> float:
        """Get the driven value from the driver value

        Args:
            value (float): driver value
        """
        return value * self._ratio[0] / self._ratio[1]

    def set(self, value: float) -> float:
        """Get the driver value from the driven value

        Args:
            value (float): driven value
        """
        return value * self._ratio[1] / self._ratio[0]


class Joint(Actuator):
    """Device class for joints

    Can be subclassed to support different joint types and versions
    """

    GEAR_RATIO: tuple[int, int]

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)


    @property
    @abstractmethod
    def is_homed(self) -> bool:
        pass

    @property
    def impedance(self) -> Gains:
        pass

    @impedance.setter
    def impedance(self, gains: Gains) -> None:
        pass

    @abstractmethod
    def home(self):
        pass


class OSLv2Joint(OSLDevice, Joint):
    GEAR_RATIO = (18, 83)
    HOMING_TOLERANCE = 0.002

    def __init__(
        self,
        name: str = "joint",
        gear_ratio: tuple[int, int] = GEAR_RATIO,
        homing_speed: float = 0.1,
        direction = 1,
        **kwargs,
    ) -> None:
        super().__init__(name=name, **kwargs)
        self._transmission = Transmission(gear_ratio)
        self._current_mode = None
        self._homing_speed = homing_speed
        self._direction = direction
        self._actpack_offset: float = 0

    def _start(self):
        pass

    def _stop(self):
        pass

    def _update(self):
        ## Note: Since the joint isn't performing any IO, this is a no-op. This might
        ## change in the future.
        pass

    def home(self):
        self._log.info(f"Homing joint {self.name}")
        # try:
        self.mode = SpeedMode
        self.devmgr(Actuator, "./actpack").gains = Gains(kd=5)
        cycles = 0
        while True:
            ##Unsure about how wise it is for devices to update the device tree
            self.devmgr.update()
            err = self.position
            if 0 < err < 0.01:
                velocity = -self._homing_speed
            elif -0.01 < err < 0:
                velocity = self._homing_speed
            else:
                velocity = -(err * 10)
            self.devmgr(Actuator, "./actpack").velocity = velocity
            if isclose(err, 0, abs_tol=0.001):
                self._log.info(f"Zero position found with error {err:.3f} rad")
                break
            cycles += 1
            if cycles > 2000:
                raise Exception(f"Homing timeout after {cycles} cycles")
            sleep(0.001)
        self._log.info(f"Setting Actpack zero, wait 1 second")
        self.devmgr(Encoder, "./actpack").set_zero()
        sleep(1)
        self._log.info(f"Joint homing completed")
        self.mode = IdleMode

    @property
    def is_homed(self) -> bool:
        return isclose(self.position, 0, abs_tol=self.HOMING_TOLERANCE)

    def calculate_actpack_offset(self) -> None:
        """
        The Actpack encoder might not be aligned to the joint zero. This function
        calculates the offset between the actpack zero and the joint zero. 

        This function should be called when the joint is in the zero position.
        
        This offset is then used when setting the joint position.
        """
        actpack_output = self.devmgr(Encoder, "./actpack").position * self._direction
        actpack_pos = self._transmission.get(actpack_output)
        diff = actpack_pos - self.position
        self._actpack_offset = diff
        self._log.info(f"SET Actpack offset: {self._actpack_offset:.3f} rad")


    @property
    def mode(self) -> ActpackMode:
        """Get joint control mode

        Returns:
            ActpackMode: Joint control mode

        Raises:
            RuntimeError: If the control mode does not support position control

        """
        return self.devmgr(Actuator, device_path="./actpack").mode

    @mode.setter
    def mode(self, controlmode: ActpackMode):
        """Set joint control mode

        Args:
            controlmode (ActpackMode): Joint control mode

        Raises:
            KeyError: If the control mode is not supported by the driver
        """
        self._log.info(f"Setting joint {self.name} mode to {controlmode}")
        self.devmgr(Actuator, device_path="./actpack").mode = controlmode

    @property
    def gains(self) -> Gains:
        return self.devmgr(Actuator, device_path="./actpack").gains

    @gains.setter
    def gains(self, gains: Gains):
        self.devmgr(Actuator, device_path="./actpack").gains = gains

    @property
    def impedance(self) -> Gains:
        # TODO: Return motor impedence recaulcated for joint impedence (gear ratio)
        pass

    @impedance.setter
    def impedance(self, gains: Gains):
        """Set joint impedance gains

        Args:
            gains (Gains): Joint impedance gains K and B

        Raises:
            RuntimeError: If the control mode does not support impedance control
        """
        self.devmgr(Actuator, "./actpack").gains = Gains(
            K=(gains.K / (self._transmission.set(1) ** 2)),
            B=(gains.B / (self._transmission.set(1) ** 2)),
        )

    @property
    @Units.to_defaults("position")
    def position(self) -> float:
        """Joint position in radians

        Returns:
            float: Joint position in radians

        """
        return self.devmgr(Encoder, device_path="./encoder").position

    @position.setter
    def position(self, value: float):
        """Set joint position command in radians

        This accounts for any offset between the actpack zero and the joint zero.
        The actpack_offset is in the gear context of the joint. This means that
        the offset is calculated after the transmission ratio is applied. This
        means that the actpack_offset is in the same units as the joint position.

        The actpack_offset is calculated when the joint is started.

        Args:
            value (float): Joint position command in radians

        Raises:
            RuntimeError: If the control mode does not support position control
        """        
        self.devmgr(Actuator, "./actpack").position =((self._transmission.set(value + self._actpack_offset)) * self._direction)

    @property
    @Units.to_defaults("velocity")
    def velocity(self) -> float:
        """Joint velocity in radians per second

        Returns:
            float: Joint velocity in radians per second
        """
        return self.devmgr(Encoder, "./encoder").velocity

    @velocity.setter
    def velocity(self, value: float) -> None:
        """Set joint velocity command in radians per second

        Args:
            value (float): Joint velocity command in radians per second

        Raises:
            RuntimeError: If the control mode does not support velocity control
        """
        self.devmgr(Actuator, "./actpack").velocity = (self._transmission.set(value) * self._direction)

    @property
    def torque(self) -> float:
        """Joint torque in Nm

        Returns:
            float: Joint torque in Nm
        """
        return self.devmgr(Actuator, "./actpack").torque

    @torque.setter
    def torque(self, torque: float):
        """Set joint torque command in Nm

        Args:
            torque (float): Joint torque command in Nm

        Raises:
            RuntimeError: If the control mode does not support torque control

        """
        self.devmgr(Actuator, "./actpack").torque = (torque * self._direction) / self._transmission.set(1)


if __name__ == "__main__":
    print("Module not executable")
