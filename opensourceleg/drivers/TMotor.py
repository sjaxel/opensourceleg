from actpack import IdleMode, ImpedenceMode, PositionMode
from device import DeviceDriver
from hardware import Gains
from TMotorCANControl.mit_can import TMotorManager_mit_can, _TMotorManState


class TMotorDriver(DeviceDriver):
    def __init__(self, name: str = "TMotorDriver", motor_id: int = 0, **kwargs) -> None:
        super().__init__(name=name, **kwargs)
        self._driver = TMotorManager_mit_can(motor_ID=motor_id)
        pass

    def _start(self):
        self._driver.power_on()
        self._driver._send_command()
        self._driver._entered = True
        if not self._driver.check_can_connection():
            raise RuntimeError("Device not connected")

    def _stop(self):
        self._driver.power_off()

    def _update(self):
        self._driver.update()


class TMotorIdleMode(IdleMode):
    DRIVER_TYPE = TMotorDriver

    def __init__(self, driver: TMotorDriver):
        super().__init__(control_mode=_TMotorManState.IDLE, driver=driver)

    def _enter(self):
        pass

    def _exit(self):
        pass


class TMotorImpedenceMode(ImpedenceMode):
    DRIVER_TYPE = TMotorDriver
    DEFAULT_POSITION_GAINS = Gains(K=0.0, B=0.0)

    def __init__(self, driver: TMotorDriver):
        """_summary_

        Args:
            driver (TMotorDriver): _description_
        """
        super().__init__(control_mode=_TMotorManState.IMPEDANCE, driver=driver)

    def _enter(self):
        self._set_gains()

    def _exit(self):
        pass

    def _set_gains(self, gains: Gains = DEFAULT_POSITION_GAINS):
        self._driver._driver.set_impedance_gains_real_unit(K=gains.K, B=gains.B)
        self._has_gains = True


class TMotorPositionMode(PositionMode):
    DRIVER_TYPE = TMotorDriver
    DEFAULT_POSITION_GAINS = Gains(K=0.0, B=0.0)

    def __init__(self, driver: TMotorDriver):
        super().__init__(control_mode=_TMotorManState.FULL_STATE, driver=driver)
        pass

    def _enter(self):
        pass

    def _exit(self):
        pass

    def _set_gains(self, gains: Gains = DEFAULT_POSITION_GAINS):
        self._driver._driver.set_position_gains_real_unit(Kp=gains.K, Kd=gains.B)
        self._has_gains = True
