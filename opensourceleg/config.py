from typing import Dict, TypedDict


class DeviceConfig(TypedDict):
    """
    path: {attr: value}
    """

    pass


class StateConfig(TypedDict):
    device_states: DeviceConfig


class StatesConfig(TypedDict):
    pass


class UserConfig(TypedDict):
    height: float
    weight: float


class StreamConfig(TypedDict):
    attributes: list[dict[str, str]]


class OSLConfig(TypedDict):
    user: UserConfig
    states: StatesConfig
    stream: StreamConfig
