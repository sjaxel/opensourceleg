import pytest
import pytest_mock

from opensourceleg.actpack import Actpack, ActpackMode, Gains, ModeGains
from opensourceleg.device import DeviceDriver


@pytest.fixture
def mock_driver():
    class MockDriver(DeviceDriver):
        def __init__(self, name: str = "Driver", **kwargs) -> None:
            super().__init__(name=name, **kwargs)
            pass

        def _start(self):
            pass

        def _stop(self):
            pass

        def _update(self):
            pass

    return MockDriver


@pytest.fixture
def actpack_mode_factory():
    """
    A factory fixture that returns a factory function that dynamically creates
    a subclass of ActpackMode with the name specified by the actpack_mode_name
    """

    def _actpack_mode_factory(driver_type, actpack_mode_name: str, control_mode: int):
        def _mode_init(self, driver, **kwargs):
            ActpackMode.__init__(self, control_mode=control_mode, driver=driver)

        def _mode_enter(self):
            pass

        def _mode_exit(self):
            pass

        new_actpack_mode_class = type(
            actpack_mode_name,
            (ActpackMode,),
            {
                "DRIVER_TYPE": driver_type,
                "NAME": actpack_mode_name,
                "__init__": _mode_init,
                "_enter": _mode_enter,
                "_exit": _mode_exit,
            },
        )
        return new_actpack_mode_class

    return _actpack_mode_factory


@pytest.fixture
def actpack_patcher(monkeypatch):
    """Returns a mock actpack with a mock driver
    Uses monkeypatch to replace the driver_type call with the MockDriver
    class from this file.
    """

    def mock_init_driver(actpack, driver_type, driver_args):
        actpack._driver = driver_type(**driver_args)

    monkeypatch.setattr(Actpack, "_init_driver", mock_init_driver)


@pytest.fixture
def actpack_modes(mock_driver, actpack_mode_factory) -> list[ActpackMode]:
    """Returns a list of mocked actpack modes"""
    modes = [("idle", 0), ("impedence", 1), ("position", 2), ("velocity", 3)]

    mocked_actpack_modes = []

    for mode in modes:
        mode_name, control_mode = mode
        mocked_actpack_modes.append(
            actpack_mode_factory(
                mock_driver, actpack_mode_name=mode_name, control_mode=control_mode
            )
        )
    return mocked_actpack_modes


@pytest.mark.parametrize(
    "mode_names",
    [
        ["idle", "impedence", "position"],
        ["position"],
    ],
)
def test_actpack_init(
    actpack_patcher, mock_driver, actpack_mode_factory, mode_names: list[str]
):
    mocked_actpack_modes = []
    control_mode = 0
    for mode_name in mode_names:
        mocked_actpack_modes.append(
            actpack_mode_factory(
                mock_driver, actpack_mode_name=mode_name, control_mode=control_mode
            )
        )
        control_mode += 1

    actpack_kwargs = {
        "name": "MockActpack",
        "driver_type": mock_driver,
        "driver_args": {"name": "MockDriver"},
    }

    """Tests the initialization of an actpack object"""
    actpack_obj = Actpack(**actpack_kwargs)
    assert actpack_obj.name == actpack_kwargs["name"]
    assert actpack_obj._driver.name == actpack_kwargs["driver_args"]["name"]
    assert actpack_obj._mode == None
    assert list(actpack_obj._modes.keys()) == mode_names


def test_actpack_modes(actpack_patcher, mock_driver, actpack_modes):
    """Tests the initialization of actpack modes"""

    actpack_kwargs = {
        "name": "MockActpack",
        "driver_type": mock_driver,
        "driver_args": {"name": "MockDriver"},
    }

    actpack_obj = Actpack(**actpack_kwargs)

    assert list(actpack_obj._modes.keys()) == [mode.NAME for mode in actpack_modes]

    for mode in actpack_modes:
        actpack_obj.set_mode(mode.NAME)
        assert isinstance(actpack_obj._mode, mode)

    with pytest.raises(KeyError):
        actpack_obj.set_mode("invalid_mode")
