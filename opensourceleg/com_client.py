from types import FunctionType
from typing import Any

import json
import signal
import socket
from pprint import pprint
from time import perf_counter, sleep, time

from opensourceleg.com_protocol import OSLMsg, SocketIOFrame
from opensourceleg.device import OSLDevice
from opensourceleg.joints import OSLv2Joint as Joint
from opensourceleg.osl import OSL

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 65431


class RemoteOSL:
    def __init__(self, host: str = DEFAULT_HOST, port: int = DEFAULT_PORT):
        self._host: str = host
        self._port: int = port
        self._socket: socket.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._connected: bool = False

    def __enter__(self) -> "RemoteOSL":
        print(f"[Client] Connecting to {self._host}:{self._port}")
        while True:
            try:
                self._socket.connect((self._host, self._port))
                self._connected = True
                print(f"[Client] Connected {self._host}:{self._port}")
                break
            except ConnectionRefusedError:
                print("[Client] Connection refused, retrying in 2s")
            sleep(2)
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._socket.close()
        self._connected = False
        if isinstance(exc_value, KeyboardInterrupt):
            # Handle IndexError here...
            print("[Client] User requested exit, closing socket")
            print(f"[Client] Disconnected from {self._host}:{self._port}")
            return True

    def _send(self, msg: OSLMsg):
        if not self._connected:
            raise ConnectionError("Client is not connected to server")
        payload = SocketIOFrame.encode(msg)
        self._socket.sendall(payload)

    def _recv(self) -> OSLMsg:
        if not self._connected:
            raise ConnectionError("Client is not connected to server")
        buffer = bytearray()
        buffer += self._socket.recv(2048)
        messages, buffer = SocketIOFrame.decode(buffer)
        if len(messages) != 1:
            raise ValueError(f"Expected 1 message, got {len(messages)}")
        if messages[0].type != "ACK":
            print(f"[Client] NACK: {messages[0].data['error']}")
            raise ValueError(f"Expected ACK message, got {messages[0].type}")
        print(f"[RemoteOSL] -> ACK")
        return messages[0]

    @property
    def connected(self) -> bool:
        return self._connected


class DeviceProxy:
    def __init__(self, proxyclass: type[OSLDevice], path: str, remote_osl: RemoteOSL):
        self._path = path
        self._remote_osl = remote_osl
        self._proxyclass = proxyclass

    def __getattr__(self, attr: str) -> Any:
        def call_function(*args, **kwargs):
            msg = OSLMsg(
                0, "CALL", {self._path: {attr: {"args": args, "kwargs": kwargs}}}
            )
            self._remote_osl._send(msg)
            return self._remote_osl._recv()

        try:
            if isinstance(self._proxyclass.__dict__[attr], FunctionType):
                return call_function
            elif isinstance(self._proxyclass.__dict__[attr], property):
                msg = OSLMsg(0, "GET", {self._path: {attr: None}})
                self._remote_osl._send(msg)
                res = self._remote_osl._recv().data[self._path][attr]
                return res
        except KeyError:
            print(f"Unknown attribute {attr}, trying GET")
            msg = OSLMsg(0, "GET", {self._path: {attr: None}})
            self._remote_osl._send(msg)
            res = self._remote_osl._recv().data[self._path][attr]
            return res

    def __setattr__(self, attr: str, value):
        if attr in ["_path", "_remote_osl", "_proxyclass"]:
            super().__setattr__(attr, value)
            return
        msg = OSLMsg(0, "SET", {self._path: {attr: value}})
        self._remote_osl._send(msg)
        return self._remote_osl._recv()

    def call(self, attr: str, *args, **kwargs):
        msg = OSLMsg(0, "CALL", {self._path: {attr: {"args": args, "kwargs": kwargs}}})
        self._remote_osl._send(msg)
        return self._remote_osl._recv()

    def __call__(self, *args: Any, **kwds: Any) -> Any:
        pass

    def __str__(self) -> str:
        return f"DeviceProxy({self._path})"


def signal_handler(signal, frame):
    raise KeyboardInterrupt("SIGINT received")


if __name__ == "__main__":
    print("[MAIN] Started")

    signal.signal(signal.SIGINT, signal_handler)

    with RemoteOSL() as osl:
        device: OSL = DeviceProxy(OSL, "/leg", osl)
        while True:
            try:
                cmd = input(f"${device._path}: ")
                (cmd, *args) = cmd.split(" ")
                if cmd == "exit":
                    exit(0)
                elif cmd == "set":
                    (method, value) = args
                    setattr(device, method, value)
                elif cmd == "get":
                    attr = args[0]
                    print(getattr(device, attr))
                elif cmd == "call":
                    (method, *args) = args
                    # TODO Parse and handle args/kwargs
                    leg_proxy.call(method)
                elif cmd == "device":
                    device = DeviceProxy(OSLDevice, args[0], osl)
                elif cmd == "get_config":
                    if len(args) == 0:
                        args = ["config.json"]
                    with open(args[0], "w") as f:
                        json.dump(device.config, f, indent=4)
                        print(f"[Client] Wrote config to {f.name}")
                elif cmd == "set_config":
                    if len(args) == 0:
                        args = ["config.json"]
                    with open(args[0]) as f:
                        device.config = json.load(f)
                elif cmd == "help":
                    print(
                        """
                    set - Set device state
                    get - Get device state
                    call - Call device method
                    list - List devices
                    help - Print this help
                    exit - Exit
                    """
                    )
                else:
                    print("Unknown command")
            except ValueError:
                continue
            except Exception as e:
                raise e
