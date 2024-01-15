import logging
import can
import time
import struct
from dataclasses import dataclass
import enum
import math



def bitmask(n, shift):
    return ((1 << n + 1) - 1) << shift


def bitfield(in_value, n, shift):
    mask = bitmask(n, shift)

    out_value = (in_value & mask) >> shift

    return out_value


class ModeStatusEnum(enum.Enum):
    RESET_MODE = 0
    CALI_MODE = 1
    MOTOR_MODE = 2


class RunModeEnum(enum.Enum):
    CONTROL_MODE = 0
    POSITION_MODE = 1
    SPEED_MODE = 2
    CURRENT_MODE = 3


class RegisterMode(enum.Enum):
    READ_WRITE = 0
    READ_ONLY = 1

@dataclass
class MessageType0:
    source_motor_can_id: int
    unique_id: str


@dataclass
class MessageType2:
    source_motor_can_id: int

    # Flags
    hall_error: bool
    encoder_error: bool
    over_temperature_error: bool
    over_current_error: bool
    under_voltage_error: bool
    mode_status: ModeStatusEnum

    # Status
    current_angle: float
    current_angular_velocity: float
    current_torque: float
    current_temperature: float


@dataclass
class MessageType17:
    address: int
    data: object  # any


def byte_to_float(v):
    return struct.unpack("<f", v)[0]

def byte_to_uint8(v):
    return struct.unpack("<B", v)[0]

def byte_to_unit16(v):
    return struct.unpack("<H", v)[0]


def _scale_to_range(v, v_min, v_max, data_size):
    full_datasize = (1 << data_size) - 1
    full_range = v_max - v_min

    scaled_v = v / full_datasize * full_range + v_min

    return scaled_v


def _scale_to_value(v, v_min, v_max, data_size):
    full_datasize = (1 << data_size) - 1
    full_range = v_max - v_min

    scaled_v = (v - v_min) / full_range * full_datasize

    return int(round(scaled_v))


def float_to_byte(float_value):
    encoded_byte = struct.pack("<f", float_value)
    return encoded_byte


def int_to_byte4(int_value):
    encoded_byte = struct.pack("<L", int_value)
    return encoded_byte


def int_to_byte2(int_value):
    encoded_byte = struct.pack("<H", int_value)
    return encoded_byte


class CyberGearMotorController:
    DEFAULT_HOST_CAN_ID = 0
    DEFAULT_MOTOR_CAN_ID = 0x7F

    REGISTER_DATA_MODEL = {
        0x7005: {"name": "run_mode", "decoder": lambda v: RunModeEnum(byte_to_uint8(v)), "size": 1, "encoder": lambda v: int_to_byte4(v.value)},
        0x7006: {"name": "iq_ref", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x700A: {"name": "spd_ref", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x700B: {"name": "imit_torque", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x7010: {"name": "cur_kp", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x7011: {"name": "cur_ki", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x7014: {"name": "cur_flit_gain", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x7016: {"name": "loc_ref", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x7017: {"name": "limit_spd", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x7018: {"name": "limit_cur", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x7019: {"name": "mechPos", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x701A: {"name": "iqf", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x701B: {"name": "mechVel", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x701C: {"name": "VBUS", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x701D: {"name": "rotation", "decoder": byte_to_unit16, "size": 2, "encoder": int_to_byte4},
        0x701E: {"name": "loc_kp", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x701F: {"name": "spd_kp", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
        0x7020: {"name": "spd_ki", "decoder": byte_to_float, "size": 4, "encoder": float_to_byte},
    }

    PARAM_DATA_MODEL = {
        0x300C: {"name": "VBUS", "decoder": byte_to_float, "size": 4},
        0x3014: {"name": "rotation", "decoder": byte_to_float, "size": 4},
        0x3015: {"name": "modPos", "decoder": byte_to_float, "size": 4},
        0x3016: {"name": "mechPos", "decoder": byte_to_float, "size": 4},
        0x3017: {"name": "mechVel", "decoder": byte_to_float, "size": 4},
    }

    def __init__(self, bus, motor_can_id=DEFAULT_MOTOR_CAN_ID, host_can_id=DEFAULT_HOST_CAN_ID):
        self.log = logging.getLogger(self.__class__.__name__)
        self.bus = bus
        self.host_can_id = host_can_id
        self._motor_can_id = None
        self.block_wait_time_s = 0.01
        self.ask_retry_cnt = 5

        # self.bus_filters = {"can_id": motor_can_id, "can_mask": bitmask(7, 8), "extended": True}
        self.motor_can_id = motor_can_id

        # Axis attributes
        self._min_position = None
        self._max_position = None
        self._current_position_set_point = None

        # Housekeeping
        self._register_name_value_dict = {v["name"]: {"address": key, **v} for key, v in self.REGISTER_DATA_MODEL.items()}
        self._parameter_name_value_dict = {v["name"]: {"address": key, **v} for key, v in self.PARAM_DATA_MODEL.items()}

    @property
    def motor_can_id(self):
        return self._motor_can_id

    @motor_can_id.setter
    def motor_can_id(self, new_id):
        self._motor_can_id = new_id

        # Update filter
        # self.bus_filters["can_id"] = (new_id & 0xff) << 8
        # self.bus.set_filters([self.bus_filters])

    @property
    def min_position(self):
        if self._min_position is None:
            raise RuntimeError("You need to call `zero_axis` first before querying the min position")
        return self._min_position

    @property
    def max_position(self):
        if self._max_position is None:
            raise RuntimeError("You need to call `zero_axis` first before querying the min position")
        return self._max_position

    def flush_rx_buffer(self, timeout_s=10):
        timeout_time = time.time() + timeout_s
        while time.time() < timeout_time:
            msg = self.bus.recv(self.block_wait_time_s)
            self.log.debug(f'Message discarded: {msg}')
            if msg is None:
                break
        else:
            raise TimeoutError(f'Unable to clear RX buffer in {timeout_s}s')

    def write(self, mode, data2, data1):
        if hasattr(data1, 'to_bytes'):
            data1 = data1.to_bytes(8, 'little')

        self.log.debug(f'Write mode={mode}, data2={data2}, data1={data1}')

        arbitration_id = mode << 24 | data2 << 8 | self.motor_can_id
        msg = can.Message(arbitration_id=arbitration_id, is_extended_id=True, data=data1, dlc=8)

        self.log.debug(f'Message outbound: {msg}')

        self.bus.send(msg)

        return None

    def ask(self, mode, data2, data1, cb_before_recv=None):
        self.log.debug(f'Ask mode={mode}, data2={data2}, data1={data1}, cb_before_recv={cb_before_recv}')
        self.flush_rx_buffer()

        retry = self.ask_retry_cnt
        while retry > 0:
            retry -= 1

            # Send command first, then wait for response
            self.write(mode, data2, data1)

            # Run the callback function
            if cb_before_recv:
                cb_before_recv()

            try:
                msg = self.bus.recv(self.block_wait_time_s)
                if msg is None:
                    raise ValueError("No data received")
            except Exception as e:
                self.log.debug(f"Unable to read from the motor due to {e}")
                continue

            # Do simple parse of the message
            try:
                mode = bitfield(msg.arbitration_id, 4, 24)
            except AttributeError as e:
                self.log.error(f"Unable to parse msg {msg} due to {e}")
                continue

            try:
                decoded_msg = None
                if mode == 0:
                    decoded_msg = self.parse_message_type_0(msg)
                elif mode == 2:
                    decoded_msg = self.parse_message_type_2(msg)
                    self.update_motor_status(decoded_msg)
                elif mode == 8:
                    decoded_msg = self.parse_message_type_8(msg)
                elif mode == 17:
                    decoded_msg = self.parse_message_type_17(msg)
                else:
                    self.log.warning('Unable to decode message')
                    decoded_msg = msg

                self.log.debug(f'Decoded Msg: {decoded_msg}')
            except Exception as e:
                self.log.warning(f'Unable to decode message {msg} due to {e}')
                continue

            break
        else:
            raise RuntimeError("Unable to read response data for command")

        return decoded_msg

    def update_motor_status(self, decoded_msg):
        pass

    def parse_message_type_0(self, msg):
        decoded_msg = MessageType0(
            source_motor_can_id=bitfield(msg.arbitration_id, 8, 0),
            unique_id=f'{int.from_bytes(msg.data, "big"):x}'
        )

        return decoded_msg

    def parse_message_type_2(self, msg):
        arbitration_id = msg.arbitration_id
        data1 = msg.data

        angle = byte_to_unit16(data1[0:2])
        angular_velocity = byte_to_unit16(data1[2:4])
        torque = byte_to_unit16(data1[4:6])
        temperature = byte_to_unit16(data1[6:8])

        decoded_msg = MessageType2(
            mode_status=ModeStatusEnum(bitfield(arbitration_id, 2, 22)),
            hall_error=bool(arbitration_id & (1 << 20)),
            encoder_error=bool(arbitration_id & (1 << 19)),
            over_temperature_error=bool(arbitration_id & (1 << 18)),
            over_current_error=bool(arbitration_id & (1 << 17)),
            under_voltage_error=bool(arbitration_id & (1 << 16)),
            source_motor_can_id=bitfield(arbitration_id, 8, 8),

            current_angle=_scale_to_range(angle, -12.57, 12.57, 16),
            current_angular_velocity=_scale_to_range(angular_velocity, -30, 30, 16),
            current_torque=_scale_to_range(torque, -12, 12, 16),
            current_temperature=temperature * 10,
        )

        return decoded_msg

    def parse_message_type_17(self, msg):
        data1 = msg.data
        self.log.debug(data1)
        address = int.from_bytes(data1[0:2], "little")
        decoder = self.REGISTER_DATA_MODEL[address]["decoder"]
        size = self.REGISTER_DATA_MODEL[address]["size"]

        data = decoder(data1[4:4+size])

        self.log.debug(f"Message 17 Address={address}, datatype={decoder}, size={size}, data={data}")

        decoded_msg = MessageType17(
            address=address,
            data=data
        )

        return decoded_msg

    def parse_message_type_8(self, msg):
        data1 = msg.data
        self.log.debug(data1)
        address = int.from_bytes(data1[0:2], "little")
        decoder = self.PARAM_DATA_MODEL[address]["decoder"]
        size = self.PARAM_DATA_MODEL[address]["size"]

        data = decoder(data1[4:4+size])

        self.log.debug(f"Message 8 Address={address}, datatype={decoder}, size={size}, data={data}")

        decoded_msg = MessageType17(
            address=address,
            data=data
        )

        return decoded_msg

    def query_id(self):
        decoded_msg = self.ask(mode=0, data2=self.host_can_id, data1=0)
        return decoded_msg.unique_id

    def enable(self):
        self.write(mode=3, data2=self.host_can_id, data1=0)

    def disable(self):
        self.write(mode=4, data2=self.host_can_id, data1=0)

    def force_zero(self):
        self.write(mode=6, data2=self.host_can_id, data1=[1])

    def set_motor_can_id(self, new_can_id):
        def update_can_id():
            self.motor_can_id = new_can_id

        data2 = (new_can_id & 0xff) << 8 | self.host_can_id
        decoded_msg = self.ask(mode=7, data2=data2, data1=0,
                               cb_before_recv=update_can_id)

    def read_register(self, address):
        data1 = 0xffff & address
        decoded_msg = self.ask(mode=17, data2=self.host_can_id, data1=data1)

        return decoded_msg.data

    def read_register_by_name(self, name):
        register_datamodel = self._register_name_value_dict[name]
        return self.read_register(register_datamodel["address"])

    def write_register_by_name(self, name, value, require_response=False):
        register_datamodel = self._register_name_value_dict[name]

        encoded_value = register_datamodel["encoder"](value)
        address = register_datamodel["address"]

        data1 = struct.pack("<HH", address, 0x0) + encoded_value

        if require_response:
            return self.ask(mode=18, data2=self.host_can_id, data1=data1)
        else:
            return self.write(mode=18, data2=self.host_can_id, data1=data1)

    def read_parameter(self, address):
        data1 = 0xffff & address
        decoded_msg = self.ask(mode=19, data2=self.host_can_id, data1=data1)

        return decoded_msg.data

    def read_parameter_by_name(self, name):
        parameter_datamodel = self._parameter_name_value_dict[name]
        return self.read_parameter(parameter_datamodel["address"])

    def set_run_mode(self, run_mode: RunModeEnum):
        """
        Set CyberGear run mode

        Args:
            run_mode (RunModeEnum): A list of RunMode enum
        """
        return self.write_register_by_name("run_mode", run_mode)

    def set_control_mode_param(self, torque: float, angle: float, angular_velocity: float, kp: float, kd: float):
        torque_value = _scale_to_value(torque, -12, 12, 16)
        angle_value = _scale_to_value(angle, -12.57, 12.57, 16)
        angular_velocity_value = _scale_to_value(angular_velocity, -30, 30, 16)
        kp_value = _scale_to_value(kp, 0, 500, 16)
        kd_value = _scale_to_value(kd, 0, 5.0, 16)

        self.log.debug(f"torque_value={torque_value}, angle_value={angle_value}, "
                       f"angular_velocity_value={angular_velocity_value}, "
                       f"kp_value={kp_value}, kd_value={kd_value}")
        data1 = struct.pack("<HHHH", angle_value, angular_velocity_value, kp_value, kd_value)

        data2 = (torque_value & 0xff) << 8 | self.host_can_id

        decoded_msg = self.ask(mode=1,data2=data2, data1=data1)

        return decoded_msg

    def set_iq_ref(self, current: float):
        """
        Set the current mode operating current

        Args:
            current (float): Current between -23 and 23A
        """
        return self.write_register_by_name("iq_ref", current)

    def set_limit_cur(self, current: float):
        """
        Set speed mode current limit

        Args:
            current (float): Current between 0 and 23A
        """
        return self.write_register_by_name("limit_cur", current)

    def set_spd_ref(self, speed: float, require_response=False):
        """
        Set speed mode target speed
        Args:
            speed (float): Speed between -30 and 30rad/s
        """
        return self.write_register_by_name("spd_ref", speed, require_response=require_response)

    def set_limit_spd(self, speed: float):
        """
        Set position mode speed limit

        Args:
            speed (float): Speed between 0 to 30rad/s
        """
        return self.write_register_by_name("limit_spd", speed)

    def set_loc_ref(self, angle: float):
        """
        Set position mode angle

        Args:
            angle (float): Angle between -2Pi to 2Pi
        """
        return self.write_register_by_name("loc_ref", angle)

    def zero_axis(self, zero_speed=0.8, zero_current=2, zero_timeout_s=30):
        # Reset position variables
        self._min_position = None
        self._max_position = None
        self._current_position_set_point = None

        # Declare constants
        NUM_SAMPLE = 3
        STABLE_THRESHOLD = 1e-2
        MIN_SAMPLES = 5

        # Enable speed mode
        self.set_run_mode(RunModeEnum.SPEED_MODE)

        # Set speed to 0
        self.set_spd_ref(0)

        # Set current limit
        self.set_limit_cur(zero_current)

        # Enable the motor
        self.enable()

        clockwise_pos = 0
        counter_clockwise_pos = 0

        try:
            self.log.info("clockwise move")
            # Move clockwise until stop
            timeout_time = time.time() + zero_timeout_s
            positions = []
            self.set_spd_ref(zero_speed)

            while time.time() < timeout_time:
                mech_pos = self.read_register_by_name("mechPos")
                current = self.read_register_by_name("iqf")
                self.log.debug(f"MechPos={mech_pos}, Current={current}")

                positions.append(mech_pos)
                if len(positions) > MIN_SAMPLES:
                    samples = positions[-NUM_SAMPLE:]
                    if (max(samples) - min(samples)) < STABLE_THRESHOLD and current > 0.5 * zero_current:
                        break
                time.sleep(0.5)
            else:
                raise RuntimeError("Unable to zero with current speed")

            # Record the clockwise position max
            self._min_position = positions[-1]

            # Set zero position
            self.force_zero()

            self.log.info("counter-clockwise move")
            # Move clockwise until stop
            timeout_time = time.time() + zero_timeout_s
            positions = []
            self.set_spd_ref(-zero_speed)

            while time.time() < timeout_time:
                mech_pos = self.read_register_by_name("mechPos")
                current = self.read_register_by_name("iqf")
                self.log.debug(f"MechPos={mech_pos}, Current={current}")

                positions.append(mech_pos)
                if len(positions) > MIN_SAMPLES:
                    samples = positions[-NUM_SAMPLE:]
                    if (max(samples) - min(samples)) < STABLE_THRESHOLD and current < 0.5 * -zero_current:
                        break

                time.sleep(0.5)
            else:
                raise RuntimeError("Unable to zero with current speed")

            self._max_position = positions[-1]

        except RuntimeError:
            self.disable()
            raise

        self.set_spd_ref(0)

        # # Switch to position mode and hold
        # # Calculate total travel
        # travel = self.get_travel()
        # self.log.info(f"min_position={self.min_position}, max_position={self.max_position}")
        # self.set_position_mode()
        # self.set_position(abs(travel) / 2)

    def set_position_mode(self, speed_limit=4, current_limit=8):
        self.set_limit_cur(current_limit)
        self.set_limit_spd(speed_limit)
        self.set_run_mode(RunModeEnum.POSITION_MODE)
        self.enable()

    def set_position(self, position_set_point, guard=True, block_wait=False):
        # Check against range
        abs_min = abs(self.min_position)
        abs_max = abs(self.max_position)

        if guard:
            if position_set_point < abs_min:
                angle_radian = abs_min
            elif position_set_point > abs_max:
                angle_radian = abs_max

        else:
            if position_set_point < abs_min:
                raise ValueError(f"Position {position_set_point} is smaller than minimum {abs_min}")
            elif position_set_point > abs_max:
                raise ValueError(f"Position {position_set_point} is smaller than minimum {abs_max}")

        self.set_loc_ref(self.min_position - position_set_point)

        self._current_position_set_point = position_set_point

        if block_wait:
            while True:
                current_position = self.get_position()
                print(current_position, position_set_point)
                if abs(current_position - position_set_point) < math.radians(5):
                    break
                time.sleep(0.5)

    def set_position_deg(self, position_set_point, guard=True, block_wait=False):
        position_rad = math.radians(position_set_point)
        self.set_position(position_rad, guard, block_wait)

    def get_position_set_point(self):
        return self._current_position_set_point

    def get_position(self):
        mech_pos = self.read_register_by_name("mechPos")
        current_position = self.min_position - mech_pos

        return current_position

    def get_travel(self):
        travel = abs(self.min_position - self.max_position)

        return travel


if __name__ == '__main__':
    pass
