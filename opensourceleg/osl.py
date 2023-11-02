from abc import abstractmethod
from math import isclose

from opensourceleg.config import OSLConfig, StatesConfig, UserConfig
from opensourceleg.device import DeviceManager, Interface, OSLDevice
from opensourceleg.joints import Joint
from opensourceleg.loadcell import Loadcell
from opensourceleg.state_machine import OSLMachine, OSLState
from opensourceleg.utilities import nested_dict_update


class Leg(Interface):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    @property
    @abstractmethod
    def ankle(self) -> Joint:
        pass

    @property
    @abstractmethod
    def knee(self) -> Joint:
        pass

    @property
    @abstractmethod
    def load(self) -> float:
        pass


class DeviceConfig(Interface):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    @abstractmethod
    def set_device_state(self, device_path, state_name=None, **device_state_values):
        pass

    @property
    @abstractmethod
    def config(self) -> OSLConfig:
        pass

    @config.setter
    @abstractmethod
    def config(self, config: OSLConfig) -> None:
        pass


class OSL(OSLDevice, Leg, Loadcell, DeviceConfig):
    def __init__(
        self,
        name: str = "leg",
        **kwargs,
    ) -> None:
        """
        Initializes an OSL object.

        Args:
            name (str): The name of the OSL object.
            **kwargs: Additional keyword arguments.
        """
        super().__init__(name=name, **kwargs)
        self._sm: OSLMachine = None

        self._config = OSLConfig()
        self.config["user"] = UserConfig(
            height=1.0,
            weight=1.0,
        )
        self.config["states"] = StatesConfig()

    def _start(self):
        """
        Starts the OSL object.
        """
        if self.sm:
            self.trigger("started")

    def _stop(self):
        """
        Stops the OSL object.
        """
        if self.sm:
            self.trigger("stopped")

    def _update(self) -> None:
        """
        Updates the OSL object.
        """
        if self.sm:
            self.trigger("new_data")

    def home(self) -> None:
        """
        Homes the OSL object.
        """
        if self.sm:
            self.trigger("start_home")

    def apply_state(self, state: Leg.State) -> None:
        """
        Applies a state to the OSL object.

        Args:
            state (Leg.State): The state to apply.
        """
        raise NotImplementedError(f"apply_state not implemented for {self.__class__}")

    def initStateMachine(self, states: list[type[OSLState]]) -> None:
        """
        Initializes the OSL state machine with the provided states.

        Args:
            states (list[type[OSLState]]): The list of states to load into the OSL state machine.
        """

        res_states = []
        res_transitions = []
        for state in states:
            s = state(self.devmgr, self.config)
            res_states.append(s)

        self._sm: OSLMachine = OSLMachine(
            model=self,
            queued=True,
            transitions=res_transitions,
            states=res_states,
            initial="init",
        )

        loaded_states: list[OSLState] = self.sm.get_states(
            self.sm.get_nested_state_names()
        )

        self._log.info(f"State machine initialized with states: {loaded_states}")

        for state in loaded_states:
            transitions = state.init_transitions()
            self._log.info(f"Adding transitions for state {state.name}: {transitions}")
            self.sm.add_transitions(transitions)

    @property
    def is_homed(self) -> bool:
        """
        Returns whether the OSL object is homed.

        Returns:
            bool: Whether the OSL object is homed.
        """
        knee_res = self.devmgr(Joint, "./knee").is_homed
        ankle_res = self.devmgr(Joint, "./ankle").is_homed

        return knee_res and ankle_res

    @property
    def sm(self) -> OSLMachine:
        """
        Returns the state machine of the OSL object.

        Returns:
            OSLMachine: The state machine of the OSL object.
        """
        return self._sm

    @property
    def config(self) -> OSLConfig:
        """
        Returns the configuration of the OSL object.

        Returns:
            OSLConfig: The configuration of the OSL object.
        """
        return self._config

    @config.setter
    def config(self, config: dict) -> None:
        """
        Sets the configuration of the OSL object.

        Args:
            config (dict): A nested dict of the configuration to set.
        """
        nested_dict_update(self._config, config)

    @property
    def ankle(self) -> Joint:
        return self.devmgr(Joint, "./ankle")

    @property
    def knee(self) -> Joint:
        return self.devmgr(Joint, "./knee")

    def set_device_state(self, device_path, state_name=None, **device_state_values):
        """
        Sets the state of a device.

        Args:
            device_path: The path of the device.
            state_name: The name of the state.
            **device_state_values: The values of the device state.
        """
        if state_name is None:
            state_name = self.state
        self._log.info(
            f"Setting device state for {device_path} and state = {state_name} to {device_state_values}"
        )

        self.config["states"][state_name]["device_states"][
            device_path
        ] = device_state_values
        self.trigger("device_state_update")

    @property
    def fx(self) -> float:
        """
        Returns the force in the x direction.

        Returns:
            float: The force in the x direction.
        """
        return self.devmgr(Loadcell, "./loadcell").fx

    @property
    def fy(self) -> float:
        """
        Returns the force in the y direction.

        Returns:
            float: The force in the y direction.
        """
        return self.devmgr(Loadcell, "./loadcell").fy

    @property
    def fz(self) -> float:
        """
        Returns the force in the z direction.

        Returns:
            float: The force in the z direction.
        """
        return self.devmgr(Loadcell, "./loadcell").fz

    @property
    def mx(self) -> float:
        """
        Returns the moment in the x direction.

        Returns:
            float: The moment in the x direction.
        """
        return self.devmgr(Loadcell, "./loadcell").mx

    @property
    def my(self) -> float:
        """
        Returns the moment in the y direction.

        Returns:
            float: The moment in the y direction.
        """
        return self.devmgr(Loadcell, "./loadcell").my

    @property
    def mz(self) -> float:
        """
        Returns the moment in the z direction.

        Returns:
            float: The moment in the z direction.
        """
        return self.devmgr(Loadcell, "./loadcell").mz

    @property
    def load(self) -> float:
        """
        Returns the load put on the leg. This is the force imparted in the -Z as fraction of the users body weight.

        Returns:
            float: Load as fraction of users body weight
        """
        return ((-1) * self.fz) / (self.config["user"]["weight"] * 9.81)


if __name__ == "__main__":
    pass
