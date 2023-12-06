from typing import Self

import traceback
from queue import Empty, Queue

from opensourceleg.com.protocol import OSLMsg
from opensourceleg.com.server import ComServer
from opensourceleg.device import DeviceManager


class MsgServer:
    def __init__(
        self,
        devmgr: DeviceManager,
        comsrv: ComServer,
        subscription: set[str],
        log_level=None,
    ) -> None:
        self._log = DeviceManager.getLogger("MsgServer")
        if log_level is not None:
            self._log.setLevel(log_level)
        self._devmgr = devmgr
        self._comsrv = comsrv
        self._subscription = subscription
        self._msg_queue: Queue[OSLMsg] = None

    def __enter__(self) -> Self:
        self._msg_queue = self._comsrv.subscribe(
            self._subscription, self.__class__.__name__
        )
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._comsrv.unsubscribe(self._msg_queue)

    def get(self, timeout: float = None) -> OSLMsg | None:
        try:
            return self._msg_queue.get(block=bool(timeout), timeout=timeout)
        except Empty:
            raise Empty
        except Exception as e:
            self._log.warning(f"Exception occured during message processing: {e}")
            traceback.print_exc()
            raise e

    def ack(self, msg: OSLMsg) -> None:
        msg.type = "ACK"
        self._comsrv.send(msg)

    def nack(self, msg: OSLMsg, error: Exception) -> None:
        msg.type = "NACK"
        msg.data = {"error": f"{error.__class__.__name__}: {error}"}
        self._comsrv.send(msg)


class RPCMsgServer(MsgServer):
    def __init__(
        self,
        devmgr: DeviceManager,
        comsrv: ComServer,
        subscription: set[str],
        log_level=None,
    ) -> None:
        super().__init__(devmgr, comsrv, subscription, log_level)

    def process(self, timeout: float = None) -> [OSLMsg]:
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
                msg = self.get(timeout=0)
                match msg.type:
                    case "GET":
                        self._process_get(msg)
                    case "CALL":
                        self._process_call(msg)
                    case "SET":
                        self._process_set(msg)
                    case _:
                        unhandles_msgs.append(msg)
                        self._log.warning(
                            f"Unhandled message type: {msg.type}. Return for manual process"
                        )
                        continue
                self.ack(msg)
            except Empty:
                break
            except Exception as e:
                self._log.warning(f"Exception occured during message processing: {e}")
                traceback.print_exc()
                self.nack(msg, e)
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
            self._log.info(f"GET {pc['path']}.{pc['attr']}")
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
            self._log.info(f"CALL data: {pc}")
            device = self._devmgr.get(pc["path"])
            args = pc.get("args", [])
            kwargs = pc.get("kwargs", {})
            self._log.info(f"CALL {pc['path']}.{pc['method']}({args}, {kwargs})")
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
            self._log.info(f"SET {pc['path']}.{pc['attr']} = {pc['value']}")
            setattr(device, pc["attr"], pc["value"])


if __name__ == "__main__":
    pass
