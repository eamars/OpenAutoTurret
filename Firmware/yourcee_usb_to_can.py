import logging

import can
from can.interfaces.serial.serial_can import SerialBus
from typing import Any, List, Optional, Tuple
from can import (
    BusABC,
    CanInitializationError,
    CanInterfaceNotImplementedError,
    CanOperationError,
    CanProtocol,
    CanTimeoutError,
    Message,
)
import struct

logger = logging.getLogger("can.serial+at")

try:
    import serial
except ImportError:
    logger.warning(
        "You won't be able to use the serial can backend without "
        "the serial module installed!"
    )
    serial = None




class YourCeeSerialBus(SerialBus):
    """
    References:
        https://wit-motion.yuque.com/wumwnr/docs/nrngkq
    """
    def __init__(
            self,
            channel: str,
            baudrate: int = 921600,
            timeout: float = 0.1,
            rtscts: bool = False,
            bitrate: int = 1000000,
            *args,
            **kwargs
    ):
        super().__init__(channel, baudrate, timeout, rtscts, *args, **kwargs)

        self._ser.reset_input_buffer()
        # Set baud
        self.write_at_cmd("AT+CG\r\n")
        self.read_at_response()

        self.write_at_cmd(f"AT+CAN_BAUD={bitrate}\r\n")
        self.read_at_response()

        # Verify
        self.write_at_cmd(f"AT+CAN_BAUD=?\r\n")
        self.read_at_response(expected=f"+CAN_BAUD:{bitrate}\r\n")

        # Enable AT+AT mode (command mode)
        self.write_at_cmd(f"AT+AT\r\n")
        self.read_at_response()

    def write_at_cmd(self, string):
        logger.debug(f"Write {string}")
        self._ser.write(string.encode())

    def read_at_response(self, expected="OK\r\n"):
        response = self._ser.readline().decode()
        logger.debug(f"Read {response}")
        if expected is not None:
            self.assert_response(response, expected)
        return response

    def assert_response(self, response, expected):
        if response != expected:
            raise RuntimeError(f"Expected {expected}, Actual: {response}")

    def send(self, msg: Message, timeout: Optional[float] = None) -> None:
        # Higher 29 bits are for arbitration id
        can_id = msg.arbitration_id << 3
        can_id |= 1 << 2  # Extended frame
        try:
            can_id_byte = struct.pack(">I", can_id)
        except struct.error:
            raise ValueError("Arbitration ID is out of range")

        # Assemble message
        byte_msg = bytearray()
        byte_msg.append(0x41)           # "AT"
        byte_msg.append(0x54)
        byte_msg += can_id_byte      # Arbitration ID
        byte_msg.append(len(msg.data))  # Length of data
        byte_msg += msg.data            # Payload
        byte_msg.append(0x0D)           # "\r\n" (CRLF)
        byte_msg.append(0x0A)

        # Write to serial device
        try:
            self._ser.write(byte_msg)
        except serial.PortNotOpenError as error:
            raise CanOperationError("writing to closed port") from error
        except serial.SerialTimeoutException as error:
            raise CanTimeoutError() from error

        # self._ser.flush()

    def _recv_internal(
        self, timeout: Optional[float]
    ) -> Tuple[Optional[Message], bool]:
        # set the new read timeout
        self._ser.timeout = timeout

        while True:
            # Read prefix. If reads invalid data then return empty data
            prefix = self._ser.read(2)
            if len(prefix) != 2 or prefix[0] != ord('A') or prefix[1] != ord('T'):
                break

            # Read can_id
            can_id_byte = self._ser.read(4)
            can_id = struct.unpack(">I", can_id_byte)[0]
            arbitration_id = can_id >> 3

            if arbitration_id >= 0x20000000:
                raise ValueError(
                    "received arbitration id may not exceed 2^29 (0x20000000)"
                )

            # Read DLC
            dlc = ord(self._ser.read(1))
            if dlc > 8:
                raise ValueError("received DLC may not exceed 8 bytes")

            # Read data
            payload = self._ser.read(dlc)

            # Read postfix
            postfix = self._ser.read(2)
            if len(postfix) != 2 or postfix[0] != ord('\r') or postfix[1] != ord('\n'):
                raise CanOperationError(
                    f"invalid postfix while reading message: {postfix}"
                )

            # Rebuild the message
            msg = Message(
                arbitration_id=arbitration_id,
                dlc=dlc,
                data=payload
            )

            return msg, False

        return None, False


if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    bus = YourCeeSerialBus("COM4")
    try:
        msg = can.Message(arbitration_id=0 << 24 | 0xfd << 8 | 100, dlc=1, data=[0])
        bus.send(msg)
        print(bus.recv(1))

    finally:
        bus.shutdown()
