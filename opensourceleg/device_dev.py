from typing import Any, ClassVar

from numpy import deg2rad

from opensourceleg.actpack import (
    ActpackMode,
    Actuator,
    FullStateMode,
    Gains,
    IdleMode,
    ImpedenceMode,
    SpeedMode,
)
from opensourceleg.com_msgserver import MsgServer
from opensourceleg.com_server import ComServer
from opensourceleg.device import DeviceManager
from opensourceleg.drivers.TMotor import TMotorActpack
from opensourceleg.encoder import AS5048A_Encoder, Encoder
from opensourceleg.joints import ImpedenceGains, Joint, OSLv2Joint, VelocityGains
from opensourceleg.loadcell import FlexSEAStrainAmp, Loadcell, LoadcellCalibration
from opensourceleg.locomotion.init import InitMode
from opensourceleg.locomotion.level import LevelMode
from opensourceleg.osl import OSL, Leg
from opensourceleg.state_machine import OSLState


class ZeroingState(OSLState):
    NAME = "zeroing"

    def init_state_settings(self) -> None:
        self.device_states["/leg/knee"] = Joint.State(mode=IdleMode)
        self.device_states["/leg/ankle"] = Joint.State(mode=IdleMode)

        self.add_callback("enter", self._zero_actpack)

    def init_transitions(self) -> list[dict]:
        state_transitions = [
            {
                "trigger": "new_data",
                "source": self.name,
                "dest": "idle",
                "conditions": self.timeout(1),
            },
        ]
        return state_transitions

    def _zero_actpack(self) -> None:
        self._devmgr(Encoder, "/leg/knee/actpack").set_zero()
        self._devmgr(Encoder, "/leg/ankle/actpack").set_zero()


class UserControlState(OSLState):
    NAME = "usercontrol"

    def init_state_settings(self) -> None:
        self.device_states["/leg/knee"] = self.device_states[
            "/leg/ankle"
        ] = Joint.State(mode=ImpedenceMode, gains=ImpedenceGains(K=40, B=5), angle=0.0)

    def init_transitions(self) -> list[dict]:
        state_transitions = [
            {"trigger": "usercontrol", "source": "idle", "dest": self.NAME},
            {
                "trigger": "stopped",
                "source": self.NAME,
                "dest": "off",
            },
            {
                "trigger": "new_data",
                "source": self.NAME,
                "dest": None,
            },
            {
                "trigger": "device_state_update",
                "source": self.NAME,
                "dest": None,
                "after": self.apply_device_states,
            },
        ]
        return state_transitions


class AnnoyedState(OSLState):
    NAME = "annoyed"
    TRIGGERED_ANGLE = 0.3
    RELAXING_ANGLE = 0.01

    def init_state_settings(self) -> None:
        self.device_states["/leg/knee"] = Joint.State(
            mode=ImpedenceMode, gains=ImpedenceGains(K=40, B=5), angle=0.0
        )
        self.device_states["/leg/ankle"] = Joint.State(
            mode=ImpedenceMode, gains=ImpedenceGains(K=80, B=10), angle=0.0
        )

    def init_transitions(self) -> list[dict]:
        state_transitions = [
            {
                "trigger": "new_data",
                "source": "idle",
                "dest": self.NAME,
                "conditions": self._becomes_annoyed,
            },
            {
                "trigger": "new_data",
                "source": self.NAME,
                "dest": "idle",
                "conditions": self._relaxes,
            },
            {
                "trigger": "new_data",
                "source": self.NAME,
                "dest": "idle",
                "conditions": self.timeout(3),
            },
            {
                "trigger": "stopped",
                "source": self.NAME,
                "dest": "off",
            },
        ]
        return state_transitions

    def _becomes_annoyed(self) -> bool:
        if abs(self._devmgr(Joint, "/leg/ankle").angle) > self.TRIGGERED_ANGLE:
            return True
        else:
            return False

    def _relaxes(self) -> None:
        if abs(self._devmgr(Joint, "/leg/ankle").angle) < self.RELAXING_ANGLE:
            return True
        else:
            return False


if __name__ == "__main__":

    ## Init DeviceManager, ComServer and MsgServer

    comserver = ComServer()

    devmgr = DeviceManager()
    devmgr.frequency = 80

    msgserver = MsgServer(devmgr, comserver, log_level="DEBUG")

    ## Init the hardware devices

    # Init OSL device
    osl = OSL(
        name="leg",
        basepath="/",
    )

    # Init loadcell with device decoupling matrix
    decoupling_matrix = [
        (8.50935, -1297.76172, 11.37597, 11.41524, 0.08508, 1279.14832),
        (-26.36361, 734.72449, -14.30058, -1468.19031, 20.55947, 768.81042),
        (-822.97290, -7.42868, -818.24860, -9.61839, -831.10327, 0.99515),
        (16.68220, 0.28728, -0.36670, -0.01215, -17.29246, -0.13635),
        (-9.87459, -0.81627, 19.66820, -0.12097, -9.77155, 0.49604),
        (-0.33606, -21.51688, 0.03178, -19.13444, -0.05028, -20.86477),
    ]

    sa_offset = [-10, -9, 33, 21, 9, 11]
    lc_yaw_correction = deg2rad(-30)
    lc_zero_offset = [0.710, 15.164, 32.149, 1.014, 0.376, 0.025]

    loadcell = FlexSEAStrainAmp(
        name="loadcell",
        basepath="/leg",
        bus="/dev/i2c-1",
        amplifier_input_offset=sa_offset,
        I2C_addr=0x66,
        calibration=LoadcellCalibration(
            loadcell_matrix=decoupling_matrix,
            yaw=lc_yaw_correction,
            loadcell_zero=lc_zero_offset,
        ),
        log_level="INFO",
    )

    # Init joints
    knee_joint = OSLv2Joint(
        name="knee", basepath="/leg", homing_speed=0.2, direction=-1
    )
    ankle_joint = OSLv2Joint(name="ankle", basepath="/leg", homing_speed=0.2)

    # Init device specific TMotorActpack
    knee_actpack = TMotorActpack(
        name="actpack", basepath="/leg/knee", motor_id=11, log_level="WARN"
    )
    ankle_actpack = TMotorActpack(
        name="actpack", basepath="/leg/ankle", motor_id=10, log_level="WARN"
    )

    # Init AS5048A encoder
    ankle_enc = AS5048A_Encoder(
        name="encoder",
        basepath="/leg/ankle",
        bus="/dev/i2c-1",
        A1_adr_pin=False,
        A2_adr_pin=False,
        zero_position=13342,
    )

    knee_enc = AS5048A_Encoder(
        name="encoder",
        basepath="/leg/knee",
        bus="/dev/i2c-1",
        A1_adr_pin=True,
        A2_adr_pin=False,
        zero_position=15900,
    )

    # Load the states into the OSL device
    osl.initStateMachine([InitMode, LevelMode])

    osl.config["user"]["height"] = 190
    osl.config["user"]["weight"] = 20

    def print_osl_state():
        osl._log.info(f"OSL State: {osl.state}")
        osl._log.info(f"Leg load fz: {osl.fz:.2f} N")
        osl._log.info(f"Knee angle: {osl.knee.angle:.2f} rad")
        osl._log.info(f"Ankle angle: {osl.ankle.angle:.2f} rad")

    MAINLOOP_TIMEOUT = 200 * 20  # sec
    STATE_REPORT = 1  # sec
    """Main program loop

    Starts the DeviceManager and MsgServer and runs the inner clock loop
    at the configured frequency


    """

    with devmgr, msgserver:
        # osl.home()
        last_report = 0
        for tick, time in devmgr.clock:
            if time > MAINLOOP_TIMEOUT:
                devmgr._log.info(f"Timeout: {time}")
                break
            # Call the update function of all devices
            devmgr.update()

            # Print the current state of the OSL device
            if time - last_report > STATE_REPORT:
                last_report = time
                # print_osl_state()

            # Process all messages recieved over ComServer interface
            if unhandled_msg := msgserver.process():
                devmgr._log.warning(f"Unhandled messages: {unhandled_msg}")
                ##Process manually

        # Print colleted timer data
        devmgr._log.info(devmgr._timer)
