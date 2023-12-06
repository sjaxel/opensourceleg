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
from opensourceleg.com.msgserver import MsgServer, RPCMsgServer
from opensourceleg.com.server import ComServer
from opensourceleg.device import DeviceManager
from opensourceleg.drivers.TMotor import TMotorActpack
from opensourceleg.encoder import AS5048A_Encoder
from opensourceleg.joints import OSLv2Joint
from opensourceleg.loadcell import FlexSEAStrainAmp, Loadcell, LoadcellCalibration
from opensourceleg.locomotion.init import InitMode
from opensourceleg.locomotion.level import LevelMode
from opensourceleg.osl import OSL, Leg
from opensourceleg.state_machine import OSLState


class DevMgrThread(Thread):
    def __init__(self, devmgr: DeviceManager, comsrv: ComServer):
        super().__init__()
        self.name = "DevMgrThread"

        self._devmgr = devmgr
        self._msgsrv: RPCMsgServer = RPCMsgServer(
            devmgr, comsrv, subscription=("GET", "SET", "CALL", "CMD")
        )
        self._log = self._devmgr.getLogger(self.name)
        self._log.info(f"Init {self.name}")
        self._devmgr.get("/leg").initStateMachine([InitMode, LevelMode])
        self._stop_event = Event()

    def run(self):
        self._log.info(f"Starting {self.name}")
        with self._msgsrv as msgsrv:
            while True:
                try:
                    msg = msgsrv.get(timeout=1)
                    match msg.type, msg.data:
                        case ["CMD", "START"]:
                            msgsrv.ack(msg)
                            break
                        case _:
                            err = RuntimeError("Unknown command")
                            msgsrv.nack(msg, err)

                except Empty:
                    pass
                except Exception as e:
                    traceback.print_exc()
                    return
                finally:
                    if self._stop_event.is_set():
                        break

            with self._devmgr as devmgr:
                for time, tick in devmgr.clock:
                    if self._stop_event.is_set():
                        break
                    devmgr.update()
                    for unhandled_msg in msgsrv.process():
                        match unhandled_msg.type, unhandled_msg.data:
                            case ["CMD", "STOP"]:
                                msgsrv.ack(unhandled_msg)
                                self._stop_event.set()
                            case ["CMD", "START"]:
                                msgsrv.nack(
                                    unhandled_msg,
                                    RuntimeError("OSL is already started"),
                                )
                            case _:
                                msgsrv.nack(
                                    unhandled_msg, RuntimeError("Unknown command")
                                )

    def stop(self):
        self._log.info(f"Recived stop signal")
        self._stop_event.set()
        self.join()


if __name__ == "__main__":

    def signal_handler(sig, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, signal_handler)
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

    with comserver:
        while True:
            try:
                devmgr_thread: DevMgrThread = DevMgrThread(devmgr, comserver)
                devmgr_thread.start()
                while devmgr_thread.is_alive():
                    devmgr_thread.join(timeout=1)
                print("I died")
            except KeyboardInterrupt:
                devmgr._log.warning("KeyboardInterrupt")
                devmgr_thread.stop()
                break
            except Exception as e:
                traceback.print_exc()
                break
