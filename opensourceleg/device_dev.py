from typing import Any, ClassVar

import signal
import traceback
from queue import Empty
from threading import Event, Thread

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
from opensourceleg.com_msgserver import MsgServer, RPCMsgServer
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


class DevMgrThread(Thread):
    def __init__(self, devmgr: DeviceManager, comsrv: ComServer):
        super().__init__()
        self._devmgr = devmgr
        self._msgsrv: RPCMsgServer = RPCMsgServer(
            devmgr, comsrv, subscription=("GET", "SET", "CALL")
        )
        self._stop_event = Event()
        self._stopped_event = Event()
        self._exit_event = Event()
        self._start_event = Event()
        self._started_event = Event()

    def run(self):
        while True:
            if self._start_event.wait(timeout=0.1):
                self._start_event.clear()
                ## We got the start event, enter the devmgr context manager
                try:
                    with self._devmgr as devmgr, self._msgsrv as msgsrv:
                        self._stopped_event.clear()
                        self._started_event.set()
                        for time, tick in devmgr.clock:
                            if self._stop_event.is_set():
                                break
                            devmgr.update()
                            msgsrv.process()
                except Exception as e:
                    traceback.print_exc()
                    self._stop_event.set()
                    self._exit_event.set()
                    self._stopped_event.set()
                    self._started_event.clear()

                self._stop_event.clear()
                self._stopped_event.set()
                self._started_event.clear()
            else:
                # Timeout, check if we should exit for some reason.
                if self._exit_event.is_set():
                    break

    def __enter__(self):
        self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.exit()

    def start_devmgr(self, timeout=1) -> bool:
        self._start_event.set()
        return self._started_event.wait(timeout)

    def stop_devmgr(self, timeout=1) -> bool:
        self._stop_event.set()
        return self._stopped_event.wait(timeout)

    def exit(self, timeout: float = 2):
        self._exit_event.set()
        self._stop_event.set()
        self.join(timeout)


def signal_handler(sig, frame):
    print(f"SIGINT received")
    raise KeyboardInterrupt


if __name__ == "__main__":

    signal.signal(signal.SIGTERM, signal_handler)
    ## Init DeviceManager, ComServer and MsgServer

    comserver = ComServer()

    devmgr = DeviceManager()
    devmgr.frequency = 80

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
    lc_zero_offset = [-11.7, 5.6, 145.7, -1, -0.15, -0.6]

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

    msgsrv = MsgServer(devmgr, comserver, subscription=("CMD"))
    t_osl: DevMgrThread = DevMgrThread(devmgr, comserver)

    with comserver, msgsrv, t_osl:
        while True:
            try:
                msg = msgsrv.get(timeout=1)
                match msg.data:
                    case "START":
                        if t_osl.start_devmgr():
                            msgsrv.ack(msg)
                        else:
                            err = RuntimeError("DevMgrThread did not respond to start")
                            msgsrv.nack(msg, err)
                    case "STOP":
                        if t_osl.stop_devmgr():
                            msgsrv.ack(msg)
                        else:
                            err = RuntimeError("DevMgrThread did not respond to stop")
                            msgsrv.nack(msg, err)
                    case "EXIT":
                        msgsrv.ack(msg)
                        break
                    case _:
                        err = ValueError("Unknown command")
                        msgsrv.nack(msg, err)
                        continue
            except Empty:
                continue
            except KeyboardInterrupt:
                print("Keyboard interrupt")
                break
            except Exception as e:
                traceback.print_exc()
                break
    print("Exited with block")
