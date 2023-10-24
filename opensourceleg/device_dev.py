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

    def init_state_settings(self) -> None:
        self.settings["/leg/knee"] = JointState(
            mode=SpeedMode,
            velocity_gains=HomingState.VELOCITY_GAINS,
        )
        self.settings["/leg/ankle"] = JointState(
            mode=SpeedMode,
            velocity_gains=HomingState.VELOCITY_GAINS,
        )

        self.add_callback("exit", self._set_actpack_offsets)

    def init_transitions(self) -> list[dict]:
        state_transitions = [
            {
                "trigger": "start_home",
                "source": ["idle", "annoyed"],
                "dest": self.name,
            },
            {
                "trigger": "new_data",
                "source": self.name,
                "dest": "idle",
                "conditions": "is_homed",
            },
            {
                "trigger": "new_data",
                "source": self.name,
                "dest": None,
                "after": self._updateHomingVelocity,
            },
        ]
        return state_transitions

    def _set_actpack_offsets(self) -> None:
        self._devmgr(Joint, "/leg/knee").calculate_actpack_offset()
        self._devmgr(Joint, "/leg/ankle").calculate_actpack_offset()

    def _updateHomingVelocity(self) -> None:
        err = self._devmgr(Joint, "/leg/knee").position
        if 0 < err < 0.03:
            velocity = -0.1
        elif -0.03 < err < 0:
            velocity = 0.1
        else:
            velocity = -(err * 3)

        self._devmgr(Joint, "/leg/knee").velocity = velocity

        err = self._devmgr(Joint, "/leg/ankle").position

        if 0 < err < 0.03:
            velocity = -0.1
        elif -0.03 < err < 0:
            velocity = 0.1
        else:
            velocity = -(err * 3)

        self._devmgr(Joint, "/leg/ankle").velocity = velocity


class ZeroingState(OSLState):
    NAME = "zeroing"

    def init_state_settings(self) -> None:
        self.settings["/leg/knee"] = JointState(mode=IdleMode)
        self.settings["/leg/ankle"] = JointState(mode=IdleMode)

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


class IdleState(OSLState):
    NAME = "idle"

    def init_transitions(self) -> list[dict]:
        state_transitions = [
            {
                "trigger": "idle",
                "source": ["Annoyed", "homing", "usercontrol"],
                "dest": self.name,
            },
            {
                "trigger": "stopped",
                "source": "idle",
                "dest": "off",
            },
        ]
        return state_transitions

    def init_state_settings(self) -> None:
        self.settings["/leg/knee"] = JointState(mode=IdleMode)
        self.settings["/leg/ankle"] = JointState(mode=IdleMode)


class UserControlState(OSLState):
    NAME = "usercontrol"

    def init_state_settings(self) -> None:
        self.settings["/leg/knee"] = JointState(
            mode=ImpedenceMode, impedance=Gains(K=40, B=5), angle=0.0
        )
        self.settings["/leg/ankle"] = JointState(
            mode=ImpedenceMode, impedance=Gains(K=40, B=5), angle=0.0
        )

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
                "trigger": "joint_state_update",
                "source": self.NAME,
                "dest": None,
                "after": "updateJointStates",
            },
        ]
        return state_transitions


class OffState(OSLState):
    NAME = "off"

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


class AnnoyedState(OSLState):
    NAME = "annoyed"
    TRIGGERED_ANGLE = 0.3
    RELAXING_ANGLE = 0.025

    def init_state_settings(self) -> None:
        self.settings["/leg/knee"] = JointState(
            mode=ImpedenceMode, impedance=Gains(K=30, B=10), angle=0.0
        )
        self.settings["/leg/ankle"] = JointState(
            mode=ImpedenceMode, impedance=Gains(K=30, B=10), angle=0.0
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
                "conditions": self.timeout(2),
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
    devmgr.frequency = 200

    msgserver = MsgServer(devmgr, comserver)

    ## Init the hardware devices

    # Init OSL device with desired FSM states
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
    loadcell = FlexSEAStrainAmp(
        name="loadcell",
        basepath="/",
        bus="/dev/i2c-1",
        I2C_addr=0x66,
        calibration=LoadcellCalibration(loadcell_matrix=decoupling_matrix),
    )

    # Init joints
    knee_joint = OSLv2Joint(
        name="knee", basepath="/leg", homing_speed=0.2, home=False, direction=-1
    )
    ankle_joint = OSLv2Joint(
        name="ankle", basepath="/leg", homing_speed=0.2, home=False
    )

    # Init device specific TMotorActpack
    knee_actpack = TMotorActpack(name="actpack", basepath="/leg/knee", motor_id=11)
    ankle_actpack = TMotorActpack(name="actpack", basepath="/leg/ankle", motor_id=10)

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
    osl.initStateMachine(
        [HomingState, ZeroingState, IdleState, OffState, AnnoyedState, UserControlState]
    )

    MAINLOOP_TIMEOUT = 200 * 20  # sec
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
