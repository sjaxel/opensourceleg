from math import isclose

from state_machine import JointState, OSLState
from transitions import Machine

from opensourceleg.device import Interface, OSLDevice
from opensourceleg.joints import Joint


class Leg(Interface):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)


class OSL(OSLDevice, Leg):
    def __init__(
        self,
        name: str = "leg",
        states: list[OSLState] = [],
        **kwargs,
    ) -> None:
        super().__init__(name=name, **kwargs)
        JointState.set_devmgr(self.devmgr)
        self.state: str
        self._sm = Machine(model=self, initial="off", queued=True)
        for state in states:
            state(self.devmgr, self.sm)

    def _start(self):
        self.trigger("started")

    def _stop(self):
        self.trigger("stopped")

    def _update(self) -> None:
        self.trigger("new_data")

    def home(self) -> None:
        self.trigger("start_home")
        ##self.devmgr(Joint, "./knee").home()

    @property
    def is_homed(self) -> bool:
        err = self.devmgr(Joint, "./knee").position
        res = isclose(err, 0, abs_tol=0.002)
        return res

    @property
    def sm(self) -> Machine:
        return self._sm
