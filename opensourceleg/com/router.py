from typing import ClassVar, Self

from enum import Enum, auto
from logging import getLogger
from queue import Empty, Full, Queue
from threading import Barrier, Event, Thread

from opensourceleg.com.protocol import OSLMsg


class Channel(Enum):
    NONE = auto()
    ROUTER = auto()
    RPC = auto()
    STREAM = auto()
    LOG = auto()


class ComPacket:
    def __init__(self, msg: OSLMsg, source: "ComConnection"):
        self.source = source
        self.msg = msg

    def ack(self, block: bool = True, timeout: float | None = None) -> None:
        """
        Acknowledge the message

        Args:
            block (bool, optional): If True, block until the message is sent. Defaults to True.
            timeout (float | None, optional): If block is True, the timeout in seconds. Defaults to None.

        Raises:
            ConnectionAbortedError: If the connection is not active
            Full: If the Tx queue is full (block=False or timeout expired)
        """
        self.msg.type = "ACK"
        self.source.send(self.msg, block, timeout)

    def nack(
        self, error: Exception, block: bool = True, timeout: float | None = None
    ) -> None:
        """
        Not acknowledge the message

        Args:
            block (bool, optional): If True, block until the message is sent. Defaults to True.
            timeout (float | None, optional): If block is True, the timeout in seconds. Defaults to None.

        Raises:
            ConnectionAbortedError: If the connection is not active
            Full: If the Tx queue is full (block=False or timeout expired)

        """
        self.msg.type = "NACK"
        self.msg.data = {"error": f"{error.__class__.__name__}: {error}"}
        self.source.send(self.msg, block, timeout)

    def __str__(self) -> str:
        return f"ComPacket({self.msg}, {self.source})"


class Router(Thread):

    ROUTES: dict[str, Channel] = {
        "ROUT": Channel.ROUTER,
        "LOG": Channel.LOG,
        "STREAM": Channel.STREAM,
        "CMD": Channel.RPC,
        "SET": Channel.RPC,
        "GET": Channel.RPC,
        "CALL": Channel.RPC,
    }

    def __init__(self, log_level: int | str = "INFO"):
        super().__init__()
        self._log = getLogger("ROUTER")
        self._log.setLevel(log_level)
        self._exit_evt: Event = Event()

        self._channels: dict[Channel, Queue[ComPacket] | None] = {}
        for ch in Channel:
            self._channels[ch] = None

        self.endpoint: Endpoint = Endpoint(self)
        self.endpoint.subscribe(Channel.ROUTER)

    def __enter__(self):
        self._log.debug("[ENTER]")
        if not self.is_alive():
            self._log.info("[START]")
            self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if exc_type is not None:
            self._log.error(f"Exception in router: {exc_type}: {exc_value}")
        else:
            self._log.info("Exiting router")
        self.stop()
        self.join(timeout=1)

    def stop(self) -> None:
        self._exit_evt.set()

    def run(self):
        while True:
            try:
                pkt: ComPacket = self.endpoint.get(block=True, timeout=0.1)
                self._log.info(f"Processing {pkt}")
                try:
                    self._process_rout(pkt)
                except Exception as e:
                    self._log.error(f"Error processing {pkt}: {e}")
                    pkt.nack(e, block=False)
            except Empty:
                self._log.debug("Router queue is empty")
            except Exception as e:
                self._log.error(f"Error processing router queue: {e}")
                break
            finally:
                if self._exit_evt.is_set():
                    break

        ## Flush router queue
        self.endpoint.unsubscribe_all()
        self.endpoint.flush()
        self._log.info("[STOP]")

    def inbound(self, pkt: ComPacket) -> None:
        try:
            ch: Channel = self.ROUTES[pkt.msg.type]
            self._channels[ch].put(pkt, block=False)
            return
        except Full:
            self._log.warning(f"Rx queue for {ch} is full")
            pkt.nack(ConnectionRefusedError(f"Rx queue for {ch} is full"), block=False)
        except KeyError:
            self._log.warning(f"Unknown message type {pkt.msg.type}")
            pkt.nack(ValueError(f"Unknown message type {pkt.msg.type}"), block=False)
        except AttributeError:
            self._log.warning(f"No active route for {pkt.msg.type}")
            pkt.nack(
                ConnectionRefusedError(f"No active route for {pkt.msg.type}"),
                block=False,
            )

    def has_active(self, ch: Channel) -> bool:
        for conn in ComConnection._active:
            if ch in conn.channel:
                return True
        return False

    def subscribe(self, queue: Queue, ch: Channel) -> None:
        if self._channels[ch] is not None and self._channels[ch] != queue:
            self._log.warning(f"Overwriting queue for {ch}")
        self._channels[ch] = queue
        self._log.info(f"Subscribed {queue} to {ch}")

    def unsubscribe(self, queue: Queue, ch: Channel | None = None) -> None:
        if ch is None:
            for ch in Channel:
                if self._channels[ch] == queue:
                    self._channels[ch] = None
        else:
            if self._channels[ch] == queue:
                self._channels[ch] = None

    def outbound(self, msg: OSLMsg, ch: Channel) -> None:
        for conn in ComConnection._active:
            if ch in conn.channel:
                try:
                    conn.send(msg, block=False)
                except Full:
                    self._log.warning(f"Tx queue for {conn} is full")
                else:
                    self._log.debug(f"Sent {msg} to {conn}")
                finally:
                    ## This means that we only to to send to one active connection
                    ## if one is found.
                    return
        ## No active connection found
        self._log.warning(f"No active connection for {ch}")
        raise ConnectionRefusedError(f"No active Channel for {ch}")

    def get(
        self, ch: Channel, block: bool = True, timeout: float | None = None
    ) -> ComPacket:
        return self._channels[ch].get(block, timeout)

    def _process_rout(self, pkt: ComPacket) -> None:
        """Process a ROUT message"""
        try:
            for key, value in pkt.msg.data.items():
                self._log.info(f"Processing {key}={value}")
                match key:
                    case "channel":
                        try:
                            pkt.source.channel.add(Channel[value])
                            self._log.info(
                                f"{pkt.source} with with registered channels {pkt.source.channel}"
                            )
                        except KeyError:
                            raise ValueError(f"Unknown channel {value}")

                    case _:
                        raise KeyError(f"Unknown ROUT key {key}")
        except Exception as e:
            self._log.error(f"Error processing ROUT: {e}")
            pkt.nack(e, block=False)
        else:
            pkt.ack(block=False)


class Endpoint:
    DEFAULT_QUEUE_SIZE: ClassVar[int] = 10

    def __init__(self, router: Router, queue_size: int = DEFAULT_QUEUE_SIZE) -> None:
        super().__init__()
        self._router = router
        self._pkt_queue: Queue[ComPacket] = Queue(maxsize=queue_size)

    def subscribe(self, *ch: Channel) -> None:
        for c in ch:
            self._router.subscribe(self._pkt_queue, c)

    def unsubscribe(self, *ch: Channel) -> None:
        for c in ch:
            self._router.unsubscribe(self._pkt_queue, c)

    def unsubscribe_all(self) -> None:
        self._router.unsubscribe(self._pkt_queue)

    def has_subsciption(self) -> bool:
        for queue in self._router._channels.values():
            if queue == self._pkt_queue:
                return True
        return False

    def get(self, block: bool = True, timeout: float | None = None) -> ComPacket:
        """Get a packet from the enpoint queue

        Args:
            block (bool, optional): If True, block until a packet is available. Defaults to True.
            timeout (float | None, optional): If block is True, the timeout in seconds. Defaults to None.

        Raises:
            Empty: If the queue is empty (block=False or timeout expired)
        """
        return self._pkt_queue.get(block, timeout)

    def flush(self) -> None:
        """Flush the queue"""
        while True:
            try:
                pkt = self._pkt_queue.get(block=False)
                pkt.nack(Exception("Endpoint closed"), block=False)
            except Exception:
                break

    def __del__(self) -> None:
        self.unsubscribe_all()
        self.flush()


class ComConnection:
    DEFAULT_TX_QUEUE_SIZE: ClassVar[int] = 10
    _active: ClassVar[set[Self]] = set[Self]()

    def __init__(
        self,
        tx_thread: Thread,
        rx_thread: Thread,
        addr: tuple[str, int],
        max_tx_queue: int = DEFAULT_TX_QUEUE_SIZE,
    ) -> None:
        self.addr = addr
        self._log = getLogger(str(self))
        self.channel: set[Channel] = set()
        self._close_evt = Event()
        self.sock_is_opened: Barrier = Barrier(2, action=self._register_active)
        self._tx_thread = tx_thread
        self._rx_thread = rx_thread
        setattr(self._tx_thread, "conn", self)
        setattr(self._rx_thread, "conn", self)
        self._tx_queue: Queue[OSLMsg] = Queue()

        self._tx_thread.start()
        self._rx_thread.start()
        self._log.info(f"[START]")

    def _register_active(self):
        self._log.info(f"[OPEN]")
        ComConnection._active.add(self)

    def _unregister_active(self):
        try:
            ComConnection._active.remove(self)
        except KeyError:
            pass
        else:
            self._log.info(f"[CLOSE]")

    def send(
        self, msg: OSLMsg, block: bool = True, timeout: float | None = None
    ) -> None:
        """Send a message to the connection

        Args:
            msg (OSLMsg): The message to send
            block (bool, optional): If True, block until the message is sent. Defaults to True.
            timeout (float | None, optional): If block is True, the timeout in seconds. Defaults to None.

        Raises:
            ConnectionAbortedError: If the connection is not active
            Full: If the Tx queue is full (block=False or timeout expired)
        """
        if not self.is_active:
            raise ConnectionAbortedError(f"Connection {self} is not active")
        self._tx_queue.put(msg, block, timeout)

    @property
    def is_active(self) -> bool:
        """Is connection active

        Returns:
            bool: True if the connection is active
        """
        return not self._close_evt.is_set()

    def stop(self) -> None:
        """Signal the connection to close

        This will signal the connection to close. The Rx thread will close the socket
        after the Tx thread has closed.
        """
        self._close_evt.set()
        self._unregister_active()

    def __del__(self) -> None:
        self.stop()

    def __str__(self) -> str:
        return f"ComConnection({self.addr[0]}:{self.addr[1]})"


if __name__ == "__main__":
    print("[ROUTER] Started")
