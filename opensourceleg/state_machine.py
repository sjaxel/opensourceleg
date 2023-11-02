#!/usr/bin/python3
# A simple and scalable Finite State Machine module

from typing import Any, Callable, ClassVar, Dict, List, Optional, Self

import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from logging import Logger
from time import monotonic

from transitions.core import _LOGGER as transitionsLogger
from transitions.extensions.nesting import HierarchicalMachine, NestedState

from opensourceleg.actpack import ActpackMode, Gains, IdleMode
from opensourceleg.config import DeviceConfig, OSLConfig, StatesConfig
from opensourceleg.device import DeviceManager, DevicePath, Interface
from opensourceleg.joints import Joint


class OSLState(NestedState, ABC):
    NAME: str
    SUBSTATES: ClassVar[list[type[Self]]] = []

    def __init__(
        self,
        devmgr: DeviceManager,
        config: OSLConfig,
        parent_name: str = None,
        **kwargs,
    ) -> None:
        # if parent_name is None:
        #     full_name = self.NAME
        # else:
        #     full_name = parent_name + "_" + self.NAME
        super().__init__(
            name=self.NAME,
            on_enter=[self.apply_device_states, self._oslstate_entry_cb],
            on_exit=[self._oslstate_exit_cb],
            **kwargs,
        )
        self._devmgr = devmgr

        self._entry_time: float = 0
        self._log: Logger = self._devmgr.getLogger("STATE:" + self.name)
        self._init_settings_storage(config)
        self.init_state_settings()
        self._log.info(f"Initialized")

    def _init_settings_storage(self, root_config: OSLConfig) -> None:
        self._config = StatesConfig({"device_states": {}})
        root_config["states"][self.name] = self._config

    def _oslstate_entry_cb(self) -> None:
        self._entry_time = monotonic()
        self._log.info(f"Entered state {self.name}")

    def _oslstate_exit_cb(self) -> None:
        self._log.info(f"Exited state {self.name}")

    def timeout(self, timeout: float) -> Callable[[], bool]:
        """Create a timeout for use in transitions

        Args:
            timeout (float): Timeout in seconds

        Returns:
            Callable[[], bool]: A function that returns True
                if [timeout] has passed since state entry

        """

        def _check_timeout() -> bool:
            return monotonic() - self._entry_time > timeout

        return _check_timeout

    @property
    def config(self) -> OSLConfig:
        return self._config

    @property
    def state_config(self) -> OSLConfig:
        return self.config

    @property
    def device_states(self) -> OSLConfig:
        return self._config["device_states"]

    def apply_device_states(self) -> None:
        try:
            for path, device_state in self.device_states.items():
                device = self._devmgr(Joint, path)
                self._log.debug(f"Applying {device_state} to {device}")
                device.apply_state(device_state)
        except AttributeError as e:
            self._log.error(f"Failed to apply device states to device at path {path}")
            raise e
        except Exception as e:
            self._log.error(f"Failed to apply device states to device at path {path}")
            raise e

    @abstractmethod
    def init_transitions(self) -> list[dict]:
        pass

    @abstractmethod
    def init_state_settings(self) -> None:
        pass


class ParentOSLState(OSLState):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)

        self.init_substates()

        for substate in self.SUBSTATES:
            self.add_substate(substate(args[0], args[1], parent_name=self.name))

    def init_transitions(self) -> list[dict]:
        """Initializes the transitions for all substates of a state.

        If overridden in a subclass, this method should call super().init_transitions()
        and add the transitions to the list returned by that method.

        Returns:
            list[dict]: A list of transitions for all substates

        """
        res: list[dict] = []
        for name, substate in self.states.items():
            res.extend(substate.init_transitions())
        return res

    @abstractmethod
    def init_substates(self) -> None:
        """Populate the substates list with the substates of this state

        This method should be called in the constructor of the subclass
        and add the substates to the SUBSTATES list.

        Example:
            self.SUBSTATES = [Substate1, Substate2]
            self.initial = Substate1 (optional)

        Returns:
            None
        """
        pass


class OSLMachine(HierarchicalMachine):
    state_cls = OSLState

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)
        transitionsLogger.setLevel("WARN")


if __name__ == "__main__":
    js1 = Joint.State(mode=IdleMode, angle=0.5)
    js1_dict = dict(js1)
    print(js1_dict)
    js2 = Joint.State(**js1_dict)
    print(js2)
