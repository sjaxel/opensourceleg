import socket
from enum import Enum
from logging import getLogger
from queue import Empty, Queue
from threading import Event, Thread

from opensourceleg.com.protocol import OSLMsg, SocketIOFrame

QueueRegistry = list[tuple[set[str], Queue[OSLMsg], str]]


class ComServer(Thread):
    HOST: str = ""
    PORT: int = 65431

    def __init__(self):
        super().__init__()
        self._log = getLogger("ComServer")
        self.rx_registry: QueueRegistry = []
        self.tx_queue: Queue[OSLMsg] = Queue()
        self.sock_close_evt = Event()
        self._exit_evt = Event()

    def __enter__(self):
        self.start()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stop()

    def stop(self):
        self.sock_close_evt.set()
        self._exit_evt.set()
        self.join()

    def run(self):

        # Add your code for comserver here
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            self._log.info(f"Listen on {self.HOST}:{self.PORT}")

            s.settimeout(0.1)
            s.bind((self.HOST, self.PORT))
            s.listen()
            while True:
                try:
                    conn, addr = s.accept()
                    self._log.info(f"[SERVER] Connected by {addr}")

                    self.sock_close_evt.clear()

                    tx_thread = ComServerTx(conn, self.tx_queue, self.sock_close_evt)
                    rx_thread = ComServerRx(conn, self.rx_registry, self.sock_close_evt)
                    rx_thread.start()
                    tx_thread.start()

                    self.sock_close_evt.wait()
                    conn.close()
                    self._log.info("Waiting for TX thread to end")
                    tx_thread.join()
                except TimeoutError:
                    if self._exit_evt.is_set():
                        self._log.info("Recieved exit event")
                        break
                except Exception as e:
                    raise e

        self._log.info("Ended")

    def subscribe(self, match: set[str], name: str) -> Queue[OSLMsg]:
        self._log.info(f"Subscribing to {match} as {name}")
        queue = Queue()
        self.rx_registry.append((match, queue, name))
        return queue

    def unsubscribe(self, queue: Queue[OSLMsg]):
        for match, q, name in self.rx_registry:
            if q == queue:
                self.rx_registry.remove((match, q, name))
                self._log.info(f"Unsubscribed {name}")
                return

    def send(self, msg: OSLMsg, timeout: float = None):
        self.tx_queue.put(msg, timeout=timeout)


class ComServerRx(Thread):
    def __init__(
        self, conn: socket.socket, rx_registry: QueueRegistry, sock_close_evt: Event
    ):
        super().__init__(daemon=True)
        self.conn = conn
        self.rx_registry: QueueRegistry = rx_registry
        self.sock_close_evt = sock_close_evt

    def run(self):

        with self.conn:
            print("[ComServerRx] Started]")

            databuffer = bytearray()

            while True:
                try:
                    data = self.conn.recv(4096)
                    if not data:
                        self.sock_close_evt.set()
                        print(f"[ComServerRx] [Sock] Socket closed")
                        break
                    databuffer += data
                    messages, databuffer = SocketIOFrame.decode(databuffer)
                    for msg in messages:
                        print(f"[ComServerRx] [Sock] Recieved {msg}")
                        matched = False
                        for match, queue, name in self.rx_registry:
                            if msg.type in match:
                                print(f"[ComServerRx] passing to {name} queue")
                                queue.put(msg)
                                matched = True
                                break

                except OSError as e:
                    self.sock_close_evt.set()
                    print(f"[ComServerRx] [Sock] Socket closed with error: {e}")
                    break
                except Exception as e:
                    raise e

        print("[ComServerRx] Ended")


class ComServerTx(Thread):
    def __init__(self, conn: socket.socket, tx_queue: Queue, sock_close_evt: Event):
        super().__init__(daemon=True)
        self.conn = conn
        self.tx_queue = tx_queue
        self.sock_close_evt = sock_close_evt

    def run(self):
        with self.conn:
            print("[ComServerTx] Started]")
            while True:
                try:
                    msg: OSLMsg = self.tx_queue.get(timeout=0.1)
                    # print(f"[SERVER_TX] [TxQueue] Recieve {msg}")

                    encoded_frame = SocketIOFrame.encode(msg)
                    self.conn.sendall(encoded_frame)
                    # print(f"[SERVER_TX] [Sock] Send {encoded_frame}")
                except Empty:
                    if self.sock_close_evt.is_set():
                        print("[ComServerTx] [Sock] Recieved sock_close_evt")
                        break
                except OSError as e:
                    if e.errno == 9:
                        self.sock_close_evt.set()
                        print("[ComServerTx] [Sock] Client closed connection")
                        break
                except TypeError as e:
                    msg.type = "NACK"
                    msg.data = {"error": f"{e.__class__.__name__}: {e}"}
                    encoded_frame = SocketIOFrame.encode(msg)
                    self.conn.sendall(encoded_frame)
        print("[ComServerTx] Ended")


if __name__ == "__main__":
    print("[MAIN] Started")

    print("[MAIN] Ended")
