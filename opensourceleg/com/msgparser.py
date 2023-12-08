from typing import Self

import traceback
from queue import Empty, Queue

from opensourceleg.com.router import Channel, ComPacket, Endpoint, OSLMsg, Router
from opensourceleg.device import DeviceManager


class MsgParser:
    _endpoint: Endpoint

    def __init__(self, endpoint: Endpoint) -> None:
        super().__init__()
        self._endpoint: Endpoint = endpoint


class RPCMsgParser(MsgParser):
    def __init__(self, devmgr: DeviceManager, endpoint: Endpoint) -> None:
        super().__init__(endpoint)
        self._devmgr: DeviceManager = devmgr

    def process(self, block: bool = True, timeout: float | None = None) -> [ComPacket]:
        """Process all RPC messages in the queue and return unhandled messages

        Args:
            timeout (float, optional): Timeout for get. Defaults to None.

        Returns:
            [OSLMsg]: Unhandled messages

        Raises:
            Empty: Queue is empty

        """
        unhandles_msgs = []
        while True:
            try:
                pkt: ComPacket = self._endpoint.get(Channel.RPC, block=False)
                match pkt.msg.type:
                    case "GET":
                        self._process_get(pkt.msg)
                    case "CALL":
                        self._process_call(pkt.msg)
                    case "SET":
                        self._process_set(pkt.msg)
                    case _:
                        unhandles_msgs.append(pkt)
                        continue
                pkt.ack(block=False)
            except Empty:
                break
            except Exception as e:
                traceback.print_exc()
                pkt.nack(e, block=False)
        return unhandles_msgs

    def _process_get(self, msg: OSLMsg) -> None:
        """
        msg.uid = n
        msg.type = "GET"
        msg.data = [
            {path: path1, attr: attr1},
            {path: path2, attr: attr2},
            {...}
        ]
        """
        for pc in msg.data:
            device = self._devmgr.get(pc["path"])
            # self._log.info(f"GET {pc['path']}.{pc['attr']}")
            pc["res"] = getattr(device, pc["attr"])

    def _process_call(self, msg: OSLMsg) -> None:
        """
        msg = {
            "uid": n
            "type": "CALL"
            "data": [
                {path: path1, method: method1, args: [args1], kwargs: {kwargs1}},
                {path: path2, method: method2, args: [args2], kwargs: {kwargs2}},
                {...}
            ]
        }
        """
        for pc in msg.data:
            # self._log.info(f"CALL data: {pc}")
            device = self._devmgr.get(pc["path"])
            args = pc.get("args", [])
            kwargs = pc.get("kwargs", {})
            # self._log.info(f"CALL {pc['path']}.{pc['method']}({args}, {kwargs})")
            res = getattr(device, pc["method"])(*args, **kwargs)
            pc["res"] = res

    def _process_set(self, msg: OSLMsg) -> None:
        """
        msg = {
            "uid": n
            "type": "SET"
            "data": [
                {path: path1, attr: attr1, value: value1},
                {path: path2, attr: attr2, value: value2},
                {...}
            ]
        }
        """
        for pc in msg.data:
            device = self._devmgr.get(pc["path"])
            # self._log.info(f"SET {pc['path']}.{pc['attr']} = {pc['value']}")
            setattr(device, pc["attr"], pc["value"])


if __name__ == "__main__":
    pass
