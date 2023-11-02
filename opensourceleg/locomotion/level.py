from typing import Callable

from enum import Enum, auto

from numpy import deg2rad

from opensourceleg.actpack import IdleMode, ImpedenceMode
from opensourceleg.joints import ImpedenceGains, Joint
from opensourceleg.osl import OSL, Leg, Loadcell
from opensourceleg.state_machine import OSLState, ParentOSLState


class LevelMode(ParentOSLState):
    NAME = "level"

    def init_substates(self) -> None:
        self.SUBSTATES = [
            LevelMode.EStance,
            LevelMode.LStance,
            LevelMode.ESwing,
            LevelMode.LSwing,
        ]

        self.initial = LevelMode.EStance.NAME

    def init_transitions(self) -> list[dict]:
        ## Mode-wide transitions (i.e. transitions that are valid in all substates)

        state_transitions = [
            {
                "trigger": "new_data",
                "source": "level",
                "dest": "init_idle",
                "conditions": self.timeout(10),
            },
            {
                "trigger": "stopped",
                "source": "level",
                "dest": "init_off",
            },
        ]

        return super().init_transitions().append(state_transitions)

    def init_state_settings(self) -> None:
        pass

    class LevelState(OSLState):
        TIMEOUT_TARGET = "init_idle"
        TIMEOUT = 10

        def __init__(self, *args, **kwargs) -> None:
            super().__init__(*args, **kwargs)
            self.osl: OSL = self._devmgr.get("/leg")
            self.add_callback("enter", self._log_attr_on_entry_cb)

        def _log_attr_on_entry_cb(self) -> None:
            self._log.info(f"Current load: {self.osl.load:.2f} BW")
            self._log.info(f"Current knee angle: {self.osl.knee.angle:.2f} rad")
            self._log.info(f"Current ankle angle: {self.osl.ankle.angle:.2f} rad")

    # --------------------------------------------- #
    # STATE 1: EARLY STANCE

    class EStance(LevelState):
        NAME = "estance"
        NEXT = "level_lstance"

        def init_state_settings(self) -> None:
            ## Default values
            KNEE_K = 99.372
            KNEE_B = 3.180
            KNEE_THETA = 5

            ANKLE_K = 19.874
            ANKLE_B = 0
            ANKLE_THETA = -2

            LOAD_TO_NEXT: float = 0.25
            ANKLE_THETA_ESTANCE_TO_LSTANCE = 6.0

            self.device_states["/leg/knee"] = Joint.State(
                mode=ImpedenceMode,
                gains=ImpedenceGains(K=KNEE_K, B=KNEE_B),
                angle=deg2rad(KNEE_THETA),
            )
            self.device_states["/leg/ankle"] = Joint.State(
                mode=ImpedenceMode,
                gains=ImpedenceGains(K=ANKLE_K, B=ANKLE_B),
                angle=deg2rad(ANKLE_THETA),
            )

            self.state_config["load_to_" + self.NEXT] = LOAD_TO_NEXT
            self.state_config["ankle_theta_to_" + self.NEXT] = deg2rad(
                ANKLE_THETA_ESTANCE_TO_LSTANCE
            )

        def init_transitions(self) -> list[dict]:
            state_transitions = [
                {
                    "trigger": "new_data",
                    "source": "level_estance",
                    "dest": self.NEXT,
                    "conditions": self._to_next,
                },
            ]
            return state_transitions

        def _to_next(self) -> bool:
            load_condition = self.osl.load > self.state_config["load_to_" + self.NEXT]
            angle_condition = (
                self.osl.ankle.angle > self.state_config["ankle_theta_to_" + self.NEXT]
            )
            return load_condition and angle_condition

    # --------------------------------------------- #
    # STATE 2: LATE STANCE

    class LStance(LevelState):
        NAME = "lstance"
        NEXT = "level_eswing"

        def init_state_settings(self) -> None:
            ## Default values
            KNEE_K = 99.372
            KNEE_B = 1.272
            KNEE_THETA = 8

            LOAD_TO_NEXT: float = 0.15

            ANKLE_K = 79.498
            ANKLE_B = 0.063
            ANKLE_THETA = -20

            self.device_states["/leg/knee"] = Joint.State(
                mode=ImpedenceMode,
                gains=ImpedenceGains(K=KNEE_K, B=KNEE_B),
                angle=deg2rad(KNEE_THETA),
            )
            self.device_states["/leg/ankle"] = Joint.State(
                mode=ImpedenceMode,
                gains=ImpedenceGains(K=ANKLE_K, B=ANKLE_B),
                angle=deg2rad(ANKLE_THETA),
            )

            self.state_config["load_to_" + self.NEXT] = LOAD_TO_NEXT

        def init_transitions(self) -> list[dict]:
            state_transitions = [
                {
                    "trigger": "new_data",
                    "source": "level_lstance",
                    "dest": self.NEXT,
                    "conditions": self._to_next,
                },
            ]
            return state_transitions

        def _to_next(self) -> bool:
            load_condition = self.osl.load < self.state_config["load_to_" + self.NEXT]
            angle_condition = True
            return load_condition and angle_condition

    # --------------------------------------------- #
    # STATE 3: EARLY SWING

    class ESwing(LevelState):
        NAME = "eswing"
        NEXT = "level_lswing"

        def init_state_settings(self) -> None:
            ## Default values
            KNEE_K = 39.749
            KNEE_B = 0.063
            KNEE_THETA = 60

            KNEE_THETA_TO_NEXT: float = 50
            KNEE_DTHETA_TO_NEXT: float = 3.0

            ANKLE_K = 7.949
            ANKLE_B = 0.0
            ANKLE_THETA = 25

            self.device_states["/leg/knee"] = Joint.State(
                mode=ImpedenceMode,
                gains=ImpedenceGains(K=KNEE_K, B=KNEE_B),
                angle=deg2rad(KNEE_THETA),
            )
            self.device_states["/leg/ankle"] = Joint.State(
                mode=ImpedenceMode,
                gains=ImpedenceGains(K=ANKLE_K, B=ANKLE_B),
                angle=deg2rad(ANKLE_THETA),
            )

            self.state_config["knee_theta_to_" + self.NEXT] = deg2rad(
                KNEE_THETA_TO_NEXT
            )
            self.state_config["knee_dtheta_to_" + self.NEXT] = deg2rad(
                KNEE_DTHETA_TO_NEXT
            )

        def init_transitions(self) -> list[dict]:
            state_transitions = [
                {
                    "trigger": "new_data",
                    "source": "level_eswing",
                    "dest": self.NEXT,
                    "conditions": self._to_next,
                },
            ]
            return state_transitions

        def _to_next(self) -> bool:
            load_condition = True
            angle_condition = (
                self.osl.knee.angle > self.state_config["knee_theta_to_" + self.NEXT]
            )
            velocity_condition = (
                self.osl.knee.velocity
                < self.state_config["knee_dtheta_to_" + self.NEXT]
            )
            return all([load_condition, angle_condition, velocity_condition])

    # --------------------------------------------- #
    # STATE 4: LATE SWING

    class LSwing(LevelState):
        NAME = "lswing"
        NEXT = "level_estance"

        def init_state_settings(self) -> None:
            ## Default values
            KNEE_K = 15.899
            KNEE_B = 3.816
            KNEE_THETA = 6

            LOAD_TO_NEXT: float = 0.4
            KNEE_THETA_TO_NEXT: float = 30

            ANKLE_K = 7.949
            ANKLE_B = 0.0
            ANKLE_THETA = 15

            self.device_states["/leg/knee"] = Joint.State(
                mode=ImpedenceMode,
                gains=ImpedenceGains(K=KNEE_K, B=KNEE_B),
                angle=deg2rad(KNEE_THETA),
            )
            self.device_states["/leg/ankle"] = Joint.State(
                mode=ImpedenceMode,
                gains=ImpedenceGains(K=ANKLE_K, B=ANKLE_B),
                angle=deg2rad(ANKLE_THETA),
            )

            self.state_config["knee_theta_to_" + self.NEXT] = deg2rad(
                KNEE_THETA_TO_NEXT
            )
            self.state_config["load_to_" + self.NEXT] = LOAD_TO_NEXT

        def init_transitions(self) -> list[dict]:
            state_transitions = [
                {
                    "trigger": "new_data",
                    "source": "level_lswing",
                    "dest": self.NEXT,
                    "conditions": self._to_next,
                },
            ]
            return state_transitions

        def _to_next(self) -> bool:
            load_condition = self.osl.load > self.state_config["load_to_" + self.NEXT]
            angle_condition = (
                self.osl.knee.angle < self.state_config["knee_theta_to_" + self.NEXT]
            )
            return all([load_condition, angle_condition])
