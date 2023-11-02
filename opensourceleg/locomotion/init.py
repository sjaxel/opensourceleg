from typing import Self

from enum import Enum, auto

from opensourceleg.actpack import IdleMode
from opensourceleg.joints import Joint, SpeedMode, VelocityGains
from opensourceleg.state_machine import OSLState, ParentOSLState


class InitMode(ParentOSLState):
    NAME = "init"

    def init_substates(self) -> None:
        self.SUBSTATES = [
            InitMode.Off,
            InitMode.Idle,
            InitMode.Homing,
        ]
        self.initial = "off"

    def init_transitions(self) -> list[dict]:
        return super().init_transitions() + [
            {
                "trigger": "new_data",
                "source": "init",
                "dest": None,
            }
        ]

    def init_state_settings(self) -> None:
        pass

    class Off(OSLState):
        NAME = "off"

        def init_transitions(self) -> list[dict]:
            state_transitions = [
                {
                    "trigger": "started",
                    "source": "init_off",
                    "dest": "init_idle",
                },
            ]
            return state_transitions

        def init_state_settings(self) -> None:
            pass

    class Idle(OSLState):
        NAME = "idle"

        def init_transitions(self) -> list[dict]:
            state_transitions = [
                {
                    "trigger": "idle",
                    "source": "*",
                    "dest": "init_idle",
                },
                {
                    "trigger": "stopped",
                    "source": "init_idle",
                    "dest": "init_off",
                },
                # {
                #     "trigger": "new_data",
                #     "source": "init_idle",
                #     "dest": "level",
                #     "conditions": self.timeout(5),
                # },
            ]
            return state_transitions

        def init_state_settings(self) -> None:
            self.device_states["/leg/knee"] = Joint.State(mode=IdleMode)
            self.device_states["/leg/ankle"] = Joint.State(mode=IdleMode)

    class Homing(OSLState):
        NAME = "homing"
        VELOCITY_GAINS = VelocityGains(kd=10)
        HOMING_VELOCITY = 0.1

        def init_state_settings(self) -> None:
            self.device_states["/leg/knee"] = self.device_states[
                "/leg/ankle"
            ] = Joint.State(
                mode=SpeedMode,
                gains=self.VELOCITY_GAINS,
            )

            self.add_callback("exit", self._set_actpack_offsets)

        def init_transitions(self) -> list[dict]:
            state_transitions = [
                {
                    "trigger": "start_home",
                    "source": "init_idle",
                    "dest": "init_homing",
                },
                {
                    "trigger": "new_data",
                    "source": "init_homing",
                    "dest": "init_idle",
                    "conditions": "is_homed",
                },
                {
                    "trigger": "new_data",
                    "source": "init_homing",
                    "dest": None,
                    "after": self._updateHomingVelocity,
                },
            ]
            return state_transitions

        def _set_actpack_offsets(self) -> None:
            self._devmgr(Joint, "/leg/knee").calculate_actpack_offset()
            self._devmgr(Joint, "/leg/ankle").calculate_actpack_offset()

        def _updateHomingVelocity(self) -> None:
            err = self._devmgr(Joint, "/leg/knee").angle
            if 0 < err < 0.03:
                velocity = -0.05
            elif -0.03 < err < 0:
                velocity = 0.05
            else:
                velocity = -(err * 3)

            self._devmgr(Joint, "/leg/knee").velocity = velocity

            err = self._devmgr(Joint, "/leg/ankle").angle

            if 0 < err < 0.03:
                velocity = -0.05
            elif -0.03 < err < 0:
                velocity = 0.05
            else:
                velocity = -(err * 3)

            self._devmgr(Joint, "/leg/ankle").velocity = velocity


if __name__ == "__main__":
    print("This is a module...")
