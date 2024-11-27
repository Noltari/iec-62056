"""IEC 62056 simulation."""

import argparse
import asyncio
import logging
from datetime import datetime, timedelta
from enum import Enum, EnumMeta, StrEnum

import serial

_LOGGER = logging.getLogger(__name__)


class DirectValueMeta(EnumMeta):
    "Metaclass that allows for directly getting an enum attribute"

    def __getattribute__(cls, name):
        value = super().__getattribute__(name)
        if isinstance(value, cls):
            value = value.value
        return value


class IEC62056Bytes(Enum, metaclass=DirectValueMeta):
    WAKE = b"\0"
    WAKE_SEQ = b"\0" * 65

    SOH = b"\x01"
    STX = b"\x02"
    ETX = b"\x03"
    EOT = b"\x04"
    ACK = b"\x06"
    EOL = b"\x0d\x0a"  # \r\n
    NAK = b"\x15"

    END = b"\x21"  # !
    START = b"\x2f"  # /
    REQ = b"\x3f"  # ?
    DLM = b"\x5c"  # \

    BINARY = b"2"

    CMD_P = b"P"  # Password command
    CMD_W = b"W"  # Write command
    CMD_R = b"E"  # Execute command
    CMD_B = b"B"  # Exit command


class IEC62056Mode(StrEnum):
    A = "A"
    B = "B"
    C = "C"
    D = "D"
    E = "E"


IEC_62056_BAUDRATES_B: dict[str, int] = {
    "0": 300,
    "A": 600,
    "B": 1200,
    "C": 2400,
    "D": 4800,
    "E": 9600,
    "F": 19200,
}
IEC_62056_BAUDRATES_B_REV = dict((v, k) for k, v in IEC_62056_BAUDRATES_B.items())
IEC_62056_BAUDRATES_C: dict[str, int] = {
    "0": 300,
    "1": 600,
    "2": 1200,
    "3": 2400,
    "4": 4800,
    "5": 9600,
    "6": 19200,
}
IEC_62056_BAUDRATES_C_REV = dict((v, k) for k, v in IEC_62056_BAUDRATES_C.items())

IEC_62056_DELAY_BAUD_CLIENT = timedelta(milliseconds=300)
IEC_62056_DELAY_BAUD_SERVER = timedelta(milliseconds=200)
IEC_62056_REACTION_200MS = timedelta(milliseconds=200)
IEC_62056_REACTION_20MS = timedelta(milliseconds=20)
IEC_62056_TIMEOUT = timedelta(seconds=1, milliseconds=500)

SERIAL_BAUDRATE = IEC_62056_BAUDRATES_C["0"]
SERIAL_BAUDRATE_MAX = IEC_62056_BAUDRATES_C["6"]
SERIAL_BUFFER_SIZE = 2048
SERIAL_BYTESIZE = serial.SEVENBITS
SERIAL_PARITY = serial.PARITY_EVEN
SERIAL_STOPBITS = serial.STOPBITS_ONE
SERIAL_TIMEOUT = None

SERVER_VENDOR = b"SIm"
SERVER_MODEL = b"Model-1234"
SERVER_SLEEP = timedelta(milliseconds=10)


class IEC62506:
    """IEC 62056 simulator."""

    def __init__(
        self,
        args: argparse.Namespace,
    ) -> None:
        """IEC 62056 simulator"""
        _LOGGER.info("Opening serial port: %s", args.serial)

        self.baudrate = int(args.baudrate)
        self.baudrate_max = int(args.max_baudrate)
        self.bytesize = int(args.bytesize)
        if args.device is None:
            self.device = None
        else:
            self.device = int(args.device)
        self.mode = IEC62056Mode(args.mode)
        self.parity = str(args.parity)
        self.stopbits = float(args.stopbits)
        if args.timeout is None:
            self.timeout = None
        else:
            self.timeout = float(args.timeout)
        self.wake_up = args.wake_up

        self.reaction = IEC_62056_REACTION_200MS.total_seconds()
        self.serial = serial.Serial(
            baudrate=self.baudrate,
            bytesize=self.bytesize,
            parity=self.parity,
            port=args.serial,
            stopbits=self.stopbits,
            timeout=self.timeout,
        )
        self.session_dt: datetime | None = None

        if not self.serial.is_open:
            _LOGGER.error("Error opening serial port: %s", args.serial)
            return

        self.serial.read_all()

    @classmethod
    def cls_parser_args(cls, parser: argparse.ArgumentParser) -> None:
        parser.add_argument(
            "-b",
            "--baudrate",
            help="serial baudrate",
            default=SERIAL_BAUDRATE,
        )
        parser.add_argument(
            "--bytesize",
            help="serial bytesize",
            default=SERIAL_BYTESIZE,
        )
        parser.add_argument(
            "--device",
            help="IEC 62056 device",
            default=None,
        )
        parser.add_argument(
            "--max-baudrate",
            help="serial max baudrate",
            default=SERIAL_BAUDRATE_MAX,
        )
        parser.add_argument(
            "-m",
            "--mode",
            help="IEC 62056 mode",
            default=IEC62056Mode.C,
        )
        parser.add_argument(
            "--parity",
            help="serial parity",
            default=SERIAL_PARITY,
        )
        parser.add_argument(
            "-s",
            "--serial",
            help="serial device",
        )
        parser.add_argument(
            "--stopbits",
            help="serial stopbits",
            default=SERIAL_STOPBITS,
        )
        parser.add_argument(
            "-t",
            "--timeout",
            help="serial timeout",
            default=SERIAL_TIMEOUT,
        )
        parser.add_argument(
            "-v",
            "--verbose",
            help="increase output verbosity",
            action="store_true",
        )
        parser.add_argument(
            "--wake-up",
            help="perform wake up sequence",
            action="store_true",
        )

    def baudrate_id_to_speed(self, baudrate_id: str, default: int = 300) -> int:
        if self.mode == IEC62056Mode.B:
            return IEC_62056_BAUDRATES_B.get(baudrate_id, default)
        return IEC_62056_BAUDRATES_C.get(baudrate_id, default)

    def baudrate_speed_to_id(self, baudrate: int, default: str = "0") -> str:
        if self.mode == IEC62056Mode.B:
            return IEC_62056_BAUDRATES_B_REV.get(baudrate, default)
        return IEC_62056_BAUDRATES_C_REV.get(baudrate, default)

    def get_baudrate_id(self) -> str:
        return self.baudrate_speed_to_id(self.baudrate)

    def get_max_baudrate_id(self) -> str:
        """Return max baudrate ID."""
        baudrate_def = self.get_baudrate_id()
        return self.baudrate_speed_to_id(self.baudrate_max, baudrate_def)

    def get_server_id(self) -> bytes:
        """Return Server ID."""
        baudrate_max = self.get_max_baudrate_id().encode(encoding="ascii")
        return (
            IEC62056Bytes.START
            + SERVER_VENDOR
            + baudrate_max
            + SERVER_MODEL
            + IEC62056Bytes.EOL
        )

    def is_session_expired(self) -> bool:
        """Check if session has expired."""
        if self.session_dt is None:
            return True

        return datetime.now() - self.session_dt >= IEC_62056_TIMEOUT

    async def serial_read(self, size: int = 1) -> None:
        """Read bytes on serial line."""
        # _LOGGER.info("serial_read: size=%s", size)
        return self.serial.read(size=size)

    async def serial_read_until(self, expected: bytes, size: int | None = None) -> None:
        """Read bytes on serial line until expected."""
        # _LOGGER.info("serial_read_until: expected=%s size=%s", expected, size)
        return self.serial.read_until(expected=expected, size=size)

    async def serial_write(
        self, msg: bytes, flush: bool = True, delay: bool = True
    ) -> None:
        """Write bytes on serial line."""
        # _LOGGER.info("serial_write: msg=%s flush=%s delay=%s", msg, flush, delay)

        self.serial.write(msg)
        if not msg.endswith(IEC62056Bytes.EOL):
            self.serial.write(IEC62056Bytes.EOL)

        if flush:
            self.serial.flush()

        if delay:
            await asyncio.sleep(self.reaction)

    async def client_baudrate(self, baudrate_id: str) -> None:
        """Client baudrate."""
        if self.mode == IEC62056Mode.A:
            return

        baudrate = self.baudrate_id_to_speed(baudrate_id, self.baudrate)
        _LOGGER.info("max_baudrate: server: %s -> %s", baudrate_id, baudrate)

        baudrate = min(baudrate, self.baudrate_max)
        baudrate_id = self.baudrate_speed_to_id(baudrate)
        _LOGGER.info("max_baudrate: client: %s -> %s", baudrate_id, baudrate)

        if self.mode != IEC62056Mode.B:
            protocol_bytes = b"0"
            baudrate_bytes = baudrate_id.encode(encoding="ascii")
            mode_bytes = b"0"
            msg = (
                IEC62056Bytes.ACK
                + protocol_bytes
                + baudrate_bytes
                + mode_bytes
                + IEC62056Bytes.EOL
            )
            await self.serial_write(msg, delay=False)

        if baudrate == self.serial.baudrate:
            return

        _LOGGER.info("client_baudrate: change=%s", baudrate)

        await asyncio.sleep(IEC_62056_DELAY_BAUD_CLIENT.total_seconds())
        self.serial.baudrate = baudrate

    async def client_data_readout(self) -> None:
        """Client data readout."""
        _LOGGER.info("client_data_readout")

        msg = await self.serial_read_until(
            expected=IEC62056Bytes.ETX,
            size=SERIAL_BUFFER_SIZE,
        )
        _LOGGER.info("client_data_readout: msg=%s", msg)
        bcc = await self.serial_read(1)
        _LOGGER.info("client_data_readout: bcc=%s", bcc)

        if not msg.startswith(IEC62056Bytes.STX):
            _LOGGER.error("client_data_readout: invalid data")
            return

        data = msg.lstrip(IEC62056Bytes.STX).rstrip(
            IEC62056Bytes.END + IEC62056Bytes.EOL + IEC62056Bytes.ETX
        )
        _LOGGER.info("client_data_readout: data=%s", data)

    def client_reaction(self, vendor: str) -> None:
        """Client reaction time."""
        reaction_20ms = vendor[2:3].islower()
        if reaction_20ms:
            self.reaction = IEC_62056_REACTION_20MS.total_seconds()
            _LOGGER.info("client_reaction: 20 ms")
        else:
            self.reaction = IEC_62056_REACTION_200MS.total_seconds()
            _LOGGER.info("client_reaction: 200 ms")

    async def client_sign_on(self) -> None:
        """Client login."""
        msg = IEC62056Bytes.START + IEC62056Bytes.REQ
        if self.device is not None:
            msg += str(self.device).encode(encoding="ascii")
        msg += IEC62056Bytes.END + IEC62056Bytes.EOL
        await self.serial_write(msg)

        msg = (
            (
                await self.serial_read_until(
                    expected=IEC62056Bytes.EOL,
                    size=SERIAL_BUFFER_SIZE,
                )
            )
            .lstrip(IEC62056Bytes.START)
            .rstrip(IEC62056Bytes.EOL)
        )
        _LOGGER.error("client_sign_on: msg=%s", msg)

        vendor = msg[:3].decode(encoding="ascii")
        baudrate = msg[3:4].decode(encoding="ascii")
        model_bytes = msg[4:]
        if model_bytes.startswith(IEC62056Bytes.DLM):
            binary = model_bytes.startswith(IEC62056Bytes.DLM + IEC62056Bytes.BINARY)
            model_bytes = model_bytes[2:]
        else:
            binary = False
        model = model_bytes.decode(encoding="ascii")
        _LOGGER.info(
            "vendor=%s baudrate=%s binary=%s model=%s", vendor, baudrate, binary, model
        )

        self.client_reaction(vendor)

        await self.client_baudrate(baudrate)
        await self.client_data_readout()

    async def client_wakeup(self) -> None:
        if not self.wake_up:
            return None

        await self.serial_write(IEC62056Bytes.WAKE_SEQ, delay=False)
        await asyncio.sleep(IEC_62056_TIMEOUT.total_seconds())

    async def client(self) -> None:
        """Client."""
        self.serial.timeout = IEC_62056_TIMEOUT.total_seconds()
        self.reaction = IEC_62056_REACTION_200MS.total_seconds()

        await self.client_wakeup()
        await self.client_sign_on()

        self.serial.close()

    async def server_change_baudrate(self, baudrate: int) -> bool:
        """Server change baudrate."""
        _LOGGER.info("server_change_baudrate: baudrate=%s", baudrate)

        if baudrate != self.serial.baudrate:
            await asyncio.sleep(IEC_62056_DELAY_BAUD_SERVER.total_seconds())
            self.serial.baudrate = baudrate
            await asyncio.sleep(IEC_62056_DELAY_BAUD_SERVER.total_seconds())

    async def server_data_readout(self) -> None:
        """Server data readout."""
        data = "1-0:0.2.0*255(V1.0)"

        msg = (
            IEC62056Bytes.STX
            + data.encode(encoding="ascii")
            + IEC62056Bytes.END
            + IEC62056Bytes.EOL
            + IEC62056Bytes.ETX
        )
        bcc = b"\x00"
        _LOGGER.info("server_data_readout: data=%s", msg)
        await self.serial_write(msg + bcc)

        await asyncio.sleep(IEC_62056_TIMEOUT.total_seconds())

        _LOGGER.info("server_data_readout: session closed")
        self.session_dt = None
        await self.server_change_baudrate(self.baudrate)

    async def server_sign_on(self, msg: bytes) -> None:
        """Server sign on."""
        _LOGGER.info("server_sign_on: session started")

        self.session_dt = datetime.now()

        msg = msg.lstrip(IEC62056Bytes.REQ).rstrip(IEC62056Bytes.END)
        if len(msg) > 0:
            self.device = int(msg)
            _LOGGER.info("server_sign_on: address=%s", self.device)

        delay = self.mode != IEC62056Mode.A
        await self.serial_write(self.get_server_id(), delay=delay)

        if self.mode == IEC62056Mode.B:
            await self.server_change_baudrate(self.baudrate_max)

        if self.mode in [IEC62056Mode.A, IEC62056Mode.B]:
            await self.server_data_readout()

    async def server_ack(self, msg: bytes) -> None:
        """Server ACK."""
        if self.is_session_expired():
            return False

        protocol = msg[1:2].decode("ascii")
        baudrate_id = msg[2:3].decode("ascii")
        baudrate = self.baudrate_id_to_speed(baudrate_id, self.baudrate)
        mode = msg[3:4].decode("ascii")
        _LOGGER.info(
            "server_ack: protocol=%s baudrate={%s -> %s} mode=%s",
            protocol,
            baudrate_id,
            baudrate,
            mode,
        )

        await self.server_change_baudrate(baudrate)
        await self.server_data_readout()

    async def server_message(self, msg: bytes) -> None:
        """Server message."""
        if msg.startswith(IEC62056Bytes.START):
            msg = msg.lstrip(IEC62056Bytes.START)
            if msg.startswith(IEC62056Bytes.REQ):
                await self.server_sign_on(msg)
            else:
                _LOGGER.warning("server_message: unknown start: %s", msg)
        elif msg.startswith(IEC62056Bytes.ACK):
            await self.server_ack(msg)
        elif msg.startswith(IEC62056Bytes.WAKE):
            _LOGGER.info("server_message: wake-up sequence")
        else:
            _LOGGER.warning("server_message: unknown: %s", msg)

    async def server_loop(self) -> None:
        """Server loop."""
        if not self.serial.in_waiting:
            await asyncio.sleep(SERVER_SLEEP.total_seconds())
            return

        msg = (
            await self.serial_read_until(
                expected=IEC62056Bytes.EOL,
                size=SERIAL_BUFFER_SIZE,
            )
        ).rstrip(IEC62056Bytes.EOL)
        await self.server_message(msg)

    async def server(self) -> None:
        """Server."""
        self.reaction = IEC_62056_REACTION_20MS.total_seconds()

        while True:
            await self.server_loop()
