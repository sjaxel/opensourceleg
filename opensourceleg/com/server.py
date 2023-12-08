import socket
from logging import getLogger
from queue import Empty, Queue
from threading import Barrier, Event, Thread

from opensourceleg.com.protocol import OSLMsg, SocketIOFrame
from opensourceleg.com.router import Channel, ComConnection, ComPacket, Router

COM_SERVER_MAX_SIZE = 4096


class ComServer(Thread):
    HOST: str = ""
    PORT: int = 65431

    def __init__(self, router: Router):
        super().__init__()
        self._log = getLogger("ComServer")
        self._active_connections: list[ComConnection] = []
        self._exit_evt = Event()
        self._router = router

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if exc_type:
            self._log.error(f"Error: {exc_type} {exc_value}")
            raise exc_type(exc_value)
        self.stop()

    def stop(self):
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
                    sock, addr = s.accept()
                    rx_thread = ComServerRx(self._router, sock)
                    tx_thread = ComServerTx(self._router, sock)
                    ComConnection(tx_thread, rx_thread, addr)
                except TimeoutError:
                    if self._exit_evt.is_set():
                        self._log.info("Recieved exit event")
                        break
                except Exception as e:
                    self._log.error(f"Error: {e}")
                    break
            self._close_active_connections()
        self._log.info("Ended")

    def _close_active_connections(self):
        for conn in self._active_connections:
            conn.stop()


class ComServerRx(Thread):
    def __init__(self, router: Router, sock: socket.socket):
        super().__init__()
        self.rout: Router = router
        self.sock: socket.socket = sock
        self.conn: ComConnection = None

    def run(self):

        with self.sock:
            print("[ComServerRx] Started]")
            self.conn.sock_is_opened.wait()
            databuffer = bytearray()

            while True:
                try:
                    data = self.sock.recv(COM_SERVER_MAX_SIZE)
                    if not data:
                        print(f"[ComServerRx] [Sock] Socket closed")
                        break
                    databuffer += data
                    messages, databuffer = SocketIOFrame.decode(databuffer)
                    for msg in messages:
                        print(f"[ComServerRx] [Sock] Recieved {msg}")
                        self.rout.inbound(ComPacket(msg, self.conn))
                except TimeoutError:
                    if not self.conn.is_active:
                        print("[ComServerRx] [Sock] Inactive connection, stopping")
                        break
                except OSError as e:
                    print(f"[ComServerRx] [Sock] Socket closed with error: {e}")
                    break
                except Exception as e:
                    print(f"[ComServerRx] [Sock] Error: {e}")
                    break

            ## Signal tx thread to close, wait for it to close, then close socket
            self.conn.stop()
            self.conn._tx_thread.join(timeout=1)

        print("[ComServerRx] Ended")


class ComServerTx(Thread):
    def __init__(self, router: Router, sock: socket.socket):
        super().__init__()
        self.rout: Router = router
        self.sock: socket.socket = sock
        self.conn: ComConnection = None

    def run(self):
        print("[ComServerTx] Started]")

        ## Wait for the Rx thread to open the socket
        self.conn.sock_is_opened.wait()
        while True:
            try:
                msg: OSLMsg = self.conn._tx_queue.get(timeout=0.1)
                # print(f"[SERVER_TX] [TxQueue] Recieve {msg}")

                if not self.conn.is_active:
                    print("[ComServerTx] [Sock] Recieved sock_close_evt")
                    break
                encoded_frame = SocketIOFrame.encode(msg)
                self.sock.sendall(encoded_frame)
                # print(f"[SERVER_TX] [Sock] Send {encoded_frame}")
            except Empty:
                if not self.conn.is_active:
                    print("[ComServerTx] [Sock] Recieved sock_close_evt")
                    break
            except TypeError as e:
                msg.type = "NACK"
                msg.data = {"error": f"{e.__class__.__name__}: {e}"}
                encoded_frame = SocketIOFrame.encode(msg)
                self.sock.sendall(encoded_frame)
            except Exception as e:
                print(f"[ComServerTx] [Sock] Error: {e}")
                break

        self.conn.stop()
        print("[ComServerTx] Ended")


if __name__ == "__main__":
    print("[MAIN] Started")

    print("[MAIN] Ended")
