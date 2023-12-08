from types import FunctionType
from typing import Any

import json
import signal
import socket
from pprint import pprint
from time import perf_counter, sleep, time

from opensourceleg.com.protocol import OSLMsg, SocketIOFrame

DEFAULT_HOST = "nb-rpi-100"
DEFAULT_PORT = 65431


class StreamClient:
    def __init__(self) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._addr = (None, None)

    def __enter__(self):
        self._sock.connect((DEFAULT_HOST, DEFAULT_PORT))
        print(f"[StreamClient] Connected {self}")
        msg = OSLMsg(0, "ROUT", {"channel": "STREAM"})
        self._send(msg)
        msg = self._recv()
        if msg.type != "ACK":
            raise ValueError(f"Expected ACK message, got {msg.type}")
        else:
            print(f"[StreamClient] -> {msg}")
        print(f"[StreamClient] [CONNECTED]")
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        print(f"[StreamClient] Disconnected {self}")
        self._sock.close()
        if isinstance(exc_value, KeyboardInterrupt):
            print("[StreamClient] User requested exit, closing socket")
            return True
        elif isinstance(exc_value, Exception):
            print(f"[StreamClient] Exception: {exc_value}")
        return True

    def _send(self, msg: OSLMsg):
        payload = SocketIOFrame.encode(msg)
        self._sock.sendall(payload)

    def _recv(self) -> OSLMsg:
        buffer = bytearray()
        buffer += self._sock.recv(2048)
        messages, buffer = SocketIOFrame.decode(buffer)
        if len(messages) != 1:
            raise ValueError(f"Expected 1 message, got {len(messages)}")
        return messages[0]

    def __str__(self) -> str:
        return f"StreamClient({self._addr[0]}:{self._addr[1]})"


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
        try:
            self.send_command("STOP")
        except Exception as e:
            print(str(e))
        finally:
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

    def send_command(self, command: str):
        self._send(OSLMsg(1, "CMD", command))
        self._recv()

    @property
    def connected(self) -> bool:
        return self._connected


class DeviceProxy:
    _remote_osl: RemoteOSL
    _path: str

    def __init__(self, path: str, remote_osl: RemoteOSL):
        self.__dict__["_path"] = path
        self.__dict__["_remote_osl"] = remote_osl

    def __getattribute__(self, attr: str) -> Any:
        return super().__getattribute__(attr)

    def __getattr__(self, attr: str) -> Any:
        msg = OSLMsg(0, "GET", [{"path": self._path, "attr": attr}])
        self._remote_osl._send(msg)
        return self._remote_osl._recv().data[0]["res"]

    def __setattr__(self, attr: str, value: Any):
        msg = OSLMsg(0, "SET", [{"path": self._path, "attr": attr, "value": value}])
        self._remote_osl._send(msg)
        return self._remote_osl._recv()

    def call(self, method: str, *args, **kwargs):
        msg = OSLMsg(
            0,
            "CALL",
            [{"path": self._path, "method": method, "args": args, "kwargs": kwargs}],
        )
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
        device = DeviceProxy("/leg", osl)

        while True:
            try:
                cmd = input(f"${device._path}: ")
                (cmd, *args) = cmd.split(" ")

                match cmd:
                    case "stop":
                        osl.send_command("STOP")
                    case "exit":
                        break
                    case "start":
                        osl.send_command("START")
                    case "set":
                        (method, value) = args
                        setattr(device, method, value)
                    case "get":
                        attr = args[0]
                        print(getattr(device, attr))
                    case "call":
                        (method, *args) = args
                        # TODO Parse and handle args/kwargs
                        device.call(method)
                    case "stream":
                        with StreamClient() as stream:
                            while True:
                                try:
                                    msg = stream._recv()
                                    print(f"[StreamClient] {msg}")
                                except TimeoutError:
                                    print("[StreamClient] Timeout")
                                    continue
                                except Exception as e:
                                    print(f"[StreamClient] {e}")
                                    break
                    case "set_device":
                        device = DeviceProxy(OSLDevice, args[0], osl)
                    case "get_config":
                        if len(args) == 0:
                            args = ["config.json"]
                        with open(args[0], "w") as f:
                            json.dump(device.config, f, indent=4)
                            print(f"[Client] Wrote config to {f.name}")
                    case "set_config":
                        if len(args) == 0:
                            args = ["config.json"]
                        with open(args[0]) as f:
                            device.config = json.load(f)
                    case "help":
                        print(
                            """
                            start - Start the device
                            set - Set an attribute
                            get - Get an attribute
                            call - Call a method
                            set_device - Set the device proxy
                            get_config - Get the device configuration
                            set_config - Set the device configuration
                            help - Print this help
                            exit - Exit
                            """
                        )
                    case _:
                        print("Unknown command")
                continue
            except TimeoutError:
                print("[Client] Timeout")
                continue
            except ValueError as e:
                if str(e) == "Expected ACK message, got NACK":
                    print("[Client] NACK")
                    continue
            except Exception as e:
                raise e
