import math
import logging
from cybergear_motor_controller import CyberGearMotorController


class Turret:
    def __init__(self,
                 yaw_motor: CyberGearMotorController,
                 pitch_motor: CyberGearMotorController,
                 config: dict):
        self.log = logging.getLogger(self.__class__.__name__)

        self.yaw_motor = yaw_motor
        self.pitch_motor = pitch_motor

        self.config = config.copy()

    def zero_pitch_axis(self):
        # Zero pitch motor first
        self.pitch_motor.zero_axis(
            zero_speed=self.config.get("pitch_motor_zero_speed", 0.5),
            zero_current=self.config.get("pitch_motor_zero_current_limit", 3),
            zero_timeout_s=self.config.get("pitch_motor_zero_timeout_s", 30)
        )

        # Verify the travel distance
        # With the current design, the pitch motor should be able to move 80 degrees. Use 75deg for robustness
        min_travel_deg = self.config.get("pitch_motor_min_travel_deg", 75)
        measured_travel = self.pitch_motor.get_travel()
        measured_travel_deg = math.degrees(measured_travel)

        self.log.info(f"Measured travel {measured_travel_deg}")

        if measured_travel_deg < min_travel_deg:
            raise RuntimeError(
                f"Measured travel {measured_travel_deg} deg is less than designed minimum travel {min_travel_deg} deg")

        # Set to position mode and move to mid position
        self.pitch_motor.set_position_mode(
            speed_limit=self.config.get("pitch_motor_speed_limit", 4),
            current_limit=self.config.get("pitch_motor_current_limit", 8)
        )
        self.pitch_motor.set_position(measured_travel / 2)

    def zero_yaw_axis(self):
        self.yaw_motor.zero_axis(
            zero_speed=self.config.get("yaw_motor_zero_speed", 0.8),
            zero_current=self.config.get("yaw_motor_zero_current_limit", 1),
            zero_timeout_s = self.config.get("yaw_motor_zero_timeout_s", 30)
        )

        min_travel_deg = self.config.get("pitch_motor_min_travel_deg", 340)
        measured_travel = self.yaw_motor.get_travel()
        measured_travel_deg = math.degrees(measured_travel)

        self.log.info(f"Measured travel {measured_travel_deg}")

        if measured_travel_deg < min_travel_deg:
            raise RuntimeError(
                f"Measured travel {measured_travel_deg} deg is less than designed minimum travel {min_travel_deg} deg")

        # Set to position mode and move to mid position
        self.yaw_motor.set_position_mode(
            speed_limit=self.config.get("yaw_motor_speed_limit", 4),
            current_limit=self.config.get("yaw_motor_current_limit", 8)
        )
        self.yaw_motor.set_position(measured_travel / 2)

    def zero_all(self):
        self.zero_pitch_axis()
        self.zero_yaw_axis()

    def emergency_stop(self):
        try:
            self.pitch_motor.disable()
        except:
            pass

        try:
            self.yaw_motor.disable()
        except:
            pass

    def disable(self):
        self.log.info("Moving to safe position")
        self.yaw_motor.set_position_mode(0.8, 2)
        self.yaw_motor.set_position(self.yaw_motor.get_travel() / 2, block_wait=True)

        self.pitch_motor.set_position_mode(0.8, 2)
        self.pitch_motor.set_position_deg(10, block_wait=True)

        self.pitch_motor.disable()
        self.yaw_motor.disable()


if __name__ == '__main__':
    import logging
    import can
    from yourcee_usb_to_can import YourCeeSerialBus

    logging.basicConfig(level=logging.INFO)

    try:
        bus = can.Bus(channel="can0", interface="socketcan")
    except OSError as e:
        bus = YourCeeSerialBus(channel="COM4")

    # Initialize motors
    pitch_motor = CyberGearMotorController(bus=bus, motor_can_id=100)
    yaw_motor = CyberGearMotorController(bus=bus, motor_can_id=101)

    # Initialize turret
    turret = Turret(yaw_motor, pitch_motor, dict())
    turret.zero_all()

    #

    # turret.disable()
    # bus.shutdown()
