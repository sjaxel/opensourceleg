from math import isclose

from opensourceleg.state_machine import JointState, OSLState, OSLMachine, OffState

from opensourceleg.device import Interface, OSLDevice, DeviceManager
from opensourceleg.joints import Joint


class Leg(Interface):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)  



class OSL(OSLDevice, Leg):
    def __init__(
        self,
        name: str = "leg",
        **kwargs,
    ) -> None:
        super().__init__(name=name, **kwargs)
        self._sm: OSLMachine = None

    def _start(self):
        if self.sm:
            self.trigger("started")

    def _stop(self):
        if self.sm:
            self.trigger("stopped")

    def _update(self) -> None:
        if self.sm:
            self.trigger("new_data")

    def home(self) -> None:
        if self.sm:
            self.trigger("start_home")

    def initStateMachine(self, states: list[type[OSLState]]) -> None:
        """
        Initialize the OSL state machine with the provided states

        Args:
            states (list[OSLState]): The list of states to load into the OSL state machine
        """
        self._sm: OSLMachine = OSLMachine(model=self,
                                    initial=OffState(self.devmgr),
                                    queued=True)

        for state in states:
            s = state(self.devmgr)
            self.sm.add_state(s)
            for transition in s.init_transitions():
                self._log.info(f"Adding transition {transition} to state {s}")
                self.sm.add_transition(**transition)

    @property
    def is_homed(self) -> bool:
        knee_res = self.devmgr(Joint, "./knee").is_homed
        ankle_res = self.devmgr(Joint, "./ankle").is_homed
        
        return knee_res and ankle_res

    @property
    def sm(self) -> OSLMachine:
        """
        The state machine of the OSL
        
        Returns:
            Machine: The state machine of the OSL
        """
        return self._sm




if __name__ == "__main__":
    pass