from typing import Any, ClassVar

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
from opensourceleg.joints import Joint, OSLv2Joint
from opensourceleg.loadcell import FlexSEAStrainAmp, Loadcell, LoadcellCalibration
from opensourceleg.osl import OSL, Leg
from opensourceleg.state_machine import JointState, OSLState


class HomingState(OSLState):
    NAME = "homing"
    VELOCITY_GAINS = Gains(kd=5)
    HOMING_VELOCITY = 0.1

    JOINT_STATES: list[JointState] = [
        JointState(
            joint="/leg/knee",
            mode=SpeedMode,
            velocity_gains=VELOCITY_GAINS,
        )
    ]

    def init_transitions(self) -> list[dict]:
        state_transitions = [
            {
                "trigger": "start_home",
                "source": ["idle", "offset"],
                "dest": self.NAME,
            },
            {
                "trigger": "new_data",
                "source": self.NAME,
                "dest": "idle",
                "conditions": "is_homed",
            },
            {
                "trigger": "new_data",
                "source": self.NAME,
                "dest": None,
                "after": self._updateJointState,
            },
        ]

        return state_transitions

    def _updateJointState(self) -> None:
        err = self._devmgr(Joint, "/leg/knee").position
        if 0 < err < 0.03:
            velocity = -0.1
        elif -0.03 < err < 0:
            velocity = 0.1
        else:
            velocity = -(err * 3)

        JointState(joint="/leg/knee", velocity=velocity)()


class IdleState(OSLState):
    NAME = "idle"
    JOINT_STATES: list[JointState] = [JointState(joint="/leg/knee", mode=IdleMode)]

    def init_transitions(self) -> list[dict]:
        state_transitions = [
            {
                "trigger": "stopped",
                "source": "idle",
                "dest": "off",
            }
        ]
        return state_transitions


class OffsetState(OSLState):
    NAME = "offset"
    JOINT_STATES: list[JointState] = [
        JointState(
            joint="/leg/knee", mode=ImpedenceMode, impedance=Gains(K=25, B=1), angle=0.1
        )
    ]

    def init_transitions(self) -> list[dict]:
        state_transitions = [
            {"trigger": "offset", "source": "idle", "dest": self.NAME},
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
        ]
        return state_transitions


class OffState(OSLState):
    NAME = "off"
    JOINT_STATES: list[JointState] = []

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


class AnnoyedState(OSLState):
    NAME = "annoyed"
    TRIGGERED_ANGLE = 0.3
    RELAXING_ANGLE = 0.025

    JOINT_STATES: list[JointState] = [
        JointState(
            joint="/leg/knee", mode=ImpedenceMode, impedance=Gains(K=30, B=1), angle=0.0
        )
    ]

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
                "trigger": "stopped",
                "source": self.NAME,
                "dest": "off",
            },
        ]
        return state_transitions

    def _becomes_annoyed(self) -> bool:
        if abs(self._devmgr(Joint, "/leg/knee").position) > self.TRIGGERED_ANGLE:
            return True
        else:
            return False

    def _relaxes(self) -> None:
        if abs(self._devmgr(Joint, "/leg/knee").position) < self.RELAXING_ANGLE:
            return True
        else:
            return False


if __name__ == "__main__":

    ## Init DeviceManager, ComServer and MsgServer

    comserver = ComServer()

    devmgr = DeviceManager()
    devmgr.frequency = 100

    msgserver = MsgServer(devmgr, comserver)

    ## Init the hardware devices

    # Init OSL device with desired FSM states
    osl = OSL(
        name="leg",
        basepath="/",
        states=[HomingState, IdleState, OffState, AnnoyedState, OffsetState],
    )

    JointState.devmgr = devmgr

    # Init loadcell with device decoupling matrix
    decoupling_matrix = [
        (8.50935, -1297.76172, 11.37597, 11.41524, 0.08508, 1279.14832),
        (-26.36361, 734.72449, -14.30058, -1468.19031, 20.55947, 768.81042),
        (-822.97290, -7.42868, -818.24860, -9.61839, -831.10327, 0.99515),
        (16.68220, 0.28728, -0.36670, -0.01215, -17.29246, -0.13635),
        (-9.87459, -0.81627, 19.66820, -0.12097, -9.77155, 0.49604),
        (-0.33606, -21.51688, 0.03178, -19.13444, -0.05028, -20.86477),
    ]
    loadcell = FlexSEAStrainAmp(
        name="loadcell",
        basepath="/",
        bus="/dev/i2c-1",
        I2C_addr=0x66,
        calibration=LoadcellCalibration(loadcell_matrix=decoupling_matrix),
    )

    # Init joints
    knee_joint = OSLv2Joint(name="knee", basepath="/leg", homing_speed=0.2, home=False)

    # Init device specific TMotorActpack
    actpack = TMotorActpack(name="actpack", basepath="/leg/knee", motor_id=10)

    # Init AS5048A encoder
    encoder = AS5048A_Encoder(
        name="encoder",
        basepath="/leg/knee",
        bus="/dev/i2c-1",
        A1_adr_pin=False,
        A2_adr_pin=False,
        zero_position=13342,
    )

    MAINLOOP_TIMEOUT = 60 * 10  # sec
    """Main program loop

    Starts the DeviceManager and MsgServer and runs the inner clock loop
    at the configured frequency


    """

    with devmgr, msgserver:
        for tick, time in devmgr.clock:
            if time > MAINLOOP_TIMEOUT:
                devmgr._log.info(f"Timeout: {time}")
                break
            # Call the update function of all devices
            devmgr.update()

            # Process all messages recieved over ComServer interface
            if unhandled_msg := msgserver.process():
                devmgr._log.warning(f"Unhandled messages: {unhandled_msg}")
                ##Process manually

        # Print colleted timer data
        devmgr._log.info(devmgr._timer)
