import traceback
from queue import Empty

from opensourceleg.com_protocol import OSLMsg
from opensourceleg.com_server import ComServer
from opensourceleg.device import DeviceManager


class MsgServer:
    def __init__(
        self, devmgr: DeviceManager, comsrv: ComServer, log_level=None
    ) -> None:
        self._log = DeviceManager.getLogger("MsgServer")
        if log_level is not None:
            self._log.setLevel(log_level)
        self._devmgr = devmgr
        self._comsrv = comsrv

    def __enter__(self):
        self._comsrv.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self._comsrv.stop()

    def process(self) -> [OSLMsg]:
        unhandles_msgs = []
        while True:
            try:
                msg = self._comsrv.recv()
                if msg.type == "GET":
                    self._process_get(msg)
                elif msg.type == "CALL":
                    self._process_call(msg)
                elif msg.type == "SET":
                    self._process_set(msg)
                else:
                    unhandles_msgs.append(msg)
                    self._log.warning(
                        f"Unhandled message type: {msg.type}. Return for manual process"
                    )
                    continue
                msg.type = "ACK"
                self._comsrv.send(msg)
            except Empty:
                break
            except Exception as e:
                msg.type = "NACK"
                msg.data = {"error": f"{e.__class__.__name__}: {e}"}
                self._log.warning(f"Exception occured during message processing: {e}")
                traceback.print_exc()
                self._comsrv.send(msg)
        return unhandles_msgs

    def _process_get(self, msg: OSLMsg) -> None:
        """
        msg.uid = n
        msg.type = "GET"
        msg.data = {
            "path1": {
                "attr1": res1,
                "attr2": res2
            },
            "path2": {
                "attr1": res1,
                "attr2": res2
            }
        }
        """
        for path in msg.data.keys():
            device = self._devmgr.get(path)
            for attr in msg.data[path]:
                self._log.info(f"GET {path}.{attr}")
                msg.data[path][attr] = getattr(device, attr)
                res = getattr(self._devmgr.get(path), attr)

    def _process_call(self, msg: OSLMsg) -> None:
        """
        msg = {
            "uid": n
            "type": "CALL"
            "data": {
                "path1": {
                    "method1": {
                        "args": [arg1, arg2],
                        "kwargs": {
                            "kwarg1": kwarg1,
                            "kwarg2": kwarg2
                        }
                    },
                    "res": res1,
                },
                "path2": {...}
            }
        }
        """
        for path in msg.data.keys():
            self._log.info(f"CALL data: {msg.data}")
            device = self._devmgr.get(path)
            for method in msg.data[path]:
                self._log.info(f"Method {method}")
                args = msg.data[path][method]["args"]
                kwargs = msg.data[path][method]["kwargs"]
                self._log.info(f"CALL {path}.{method}({args}, {kwargs})")
                res = getattr(device, method)(*args, **kwargs)
                msg.data[path][method]["res"] = res

    def _process_set(self, msg: OSLMsg) -> None:
        """
        msg = {
            "uid": n
            "type": "SET"
            "data": {
                "path1": {
                    "attr1": res1,
                    "attr2": res2
                },
                "path2": {
                    "attr1": res1,
                    "attr2": res2
                }
            }
        }
        """
        for path in msg.data.keys():
            device = self._devmgr.get(path)
            for attr in msg.data[path]:
                self._log.info(f"SET {path}.{attr} = {msg.data[path][attr]}")
                setattr(device, attr, msg.data[path][attr])


if __name__ == "__main__":
    pass
