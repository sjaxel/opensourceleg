from typing import Any

from opensourceleg.com.router import Channel, Empty, Endpoint, OSLMsg, Router
from opensourceleg.config import OSLConfig, StreamConfig
from opensourceleg.device import DeviceManager
from opensourceleg.osl import OSL


class StreamServer:
    DEFAULT_STREAM_ATTRS = [
        {"path": "/leg", "attr": "state"},
    ]

    def __init__(self, devmgr: DeviceManager, router: Router) -> None:
        super().__init__()
        self._devmgr: DeviceManager = devmgr
        self._router: Router = router
        self._endpoint: Endpoint = Endpoint(router)
        self._config: StreamConfig
        self._init_config()
        self._endpoint.subscribe(Channel.STREAM)

    def _init_config(self) -> None:
        try:
            osl: OSL = self._devmgr.get("/leg")
            self._config = osl.config.setdefault("stream", StreamConfig())

            self.config.setdefault("attributes", self.DEFAULT_STREAM_ATTRS)
            self._validate_stream_attr(self.config["attributes"])
        except Exception as e:
            print(f"OSLConfig: {osl.config}")
            print(e)
            raise e

    @property
    def stream_attributes(self) -> list[dict[str, str]]:
        """Returns a list of attributes to stream

        Example:
            stream_attributes = [
                {"path": "/leg", "attr": "state"},
                {"path": "/leg/knee", "attr": "angle"},
                {...}
            ]

        Returns:
            list[dict[str, str]]: List of attributes to stream

        Raises:
            AttributeError: If attribute does not exist
        """
        return self.config["attributes"]

    def _validate_stream_attr(self, stream_attr: list[dict[str, str]]) -> None:
        try:
            for attr in stream_attr:
                device = self._devmgr.get(attr["path"])
                if not hasattr(device, attr["attr"]):
                    raise AttributeError(
                        f"Device {device} does not have attribute {attr['attr']}"
                    )
        except Exception as e:
            raise e

    def _get_data(self) -> list[Any]:
        res = []
        for attr in self.stream_attributes:
            device = self._devmgr.get(attr["path"])
            res += (getattr(device, attr["attr"]),)
        return res

    def _send(self) -> None:
        tick, time = self._devmgr.clock.current_tick()
        if self._router.has_active(Channel.STREAM):
            msg = OSLMsg(
                uid=tick, type="STREAM", data={"time": time, "attr": self._get_data()}
            )
            self._router.outbound(msg, Channel.STREAM)

    def process(self) -> None:
        try:
            while True:
                msg = self._endpoint.get(block=False)
                print(msg)
        except Empty:
            pass
        except Exception as e:
            raise e

        self._send()

    @property
    def config(self) -> StreamConfig:
        return self._config
