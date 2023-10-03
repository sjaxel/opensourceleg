import socket
from queue import Empty, Queue
from threading import Event, Thread

from opensourceleg.com_protocol import OSLMsg, SocketIOFrame


class ComServer(Thread):
    HOST: str = "127.0.0.1"
    PORT: int = 65431

    def __init__(self):
        super().__init__(daemon=True)
        self.rx_queue = Queue()
        self.tx_queue = Queue()
        self.sock_close_evt = Event()

    def run(self):

        print("[SERVER] Started]")
        # Add your code for comserver here
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            print(f"[SERVER] Binding to {self.HOST}:{self.PORT}")
            s.bind((self.HOST, self.PORT))
            s.listen()
            while True:
                conn, addr = s.accept()
                print(f"[SERVER] Connected by {addr}")

                self.sock_close_evt.clear()

                tx_thread = ComServerTx(conn, self.tx_queue, self.sock_close_evt)
                rx_thread = ComServerRx(conn, self.rx_queue, self.sock_close_evt)
                rx_thread.start()
                tx_thread.start()

                self.sock_close_evt.wait()
                conn.close()
                print("[SERVER] Waiting for TX thread to end")
                tx_thread.join()

        print("[SERVER] Ended")

    def send(self, msg: OSLMsg, timeout: float = None):
        self.tx_queue.put(msg, timeout=timeout)

    def recv(self, block=False, timeout: float = None) -> OSLMsg:
        return self.rx_queue.get(block=block, timeout=timeout)

    def stop(self):
        self.sock_close_evt.set()


class ComServerRx(Thread):
    def __init__(self, conn: socket.socket, rx_queue: Queue, sock_close_evt: Event):
        super().__init__(daemon=True)
        self.conn = conn
        self.rx_queue = rx_queue
        self.sock_close_evt = sock_close_evt

    def run(self):

        with self.conn:
            print("[SERVER_RX] Started]")

            databuffer = bytearray()

            while True:
                try:
                    data = self.conn.recv(1024)
                    if not data:
                        self.sock_close_evt.set()
                        print(f"[SERVER_RX] [Sock] Socket closed")
                        break
                    databuffer += data
                    messages, databuffer = SocketIOFrame.decode(databuffer)
                    for msg in messages:
                        # print(f"[SERVER_RX] [Sock] Decoded {msg}")
                        self.rx_queue.put(msg)
                except OSError as e:
                    self.sock_close_evt.set()
                    print(f"[SERVER_RX] [Sock] Socket closed with error: {e}")
                    break
                except Exception as e:
                    raise e

        print("[SERVER_RX] Ended")


class ComServerTx(Thread):
    def __init__(self, conn: socket.socket, tx_queue: Queue, sock_close_evt: Event):
        super().__init__(daemon=True)
        self.conn = conn
        self.tx_queue = tx_queue
        self.sock_close_evt = sock_close_evt

    def run(self):
        with self.conn:
            print("[SERVER_TX] Started]")
            while True:
                try:
                    msg: OSLMsg = self.tx_queue.get(timeout=0.1)
                    # print(f"[SERVER_TX] [TxQueue] Recieve {msg}")

                    encoded_frame = SocketIOFrame.encode(msg)
                    self.conn.sendall(encoded_frame)
                    # print(f"[SERVER_TX] [Sock] Send {encoded_frame}")
                except Empty:
                    if self.sock_close_evt.is_set():
                        print("[SERVER_TX] [Sock] Recieved sock_close_evt")
                        break
                except OSError as e:
                    if e.errno == 9:
                        self.sock_close_evt.set()
                        print("[SERVER_TX] [Sock] Client closed connection")
                        break
        print("[SERVER_TX] Ended")


def signal_handler(sig, frame):
    print(f"SIGINT received")
    exit(0)


if __name__ == "__main__":
    print("[MAIN] Started")

    print("[MAIN] Ended")
