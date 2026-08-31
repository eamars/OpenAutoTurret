from cybergear_motor_controller import CyberGearMotorController


if __name__ == '__main__':
    from yourcee_usb_to_can import YourCeeSerialBus
    bus = YourCeeSerialBus(channel="COM4")

    pitch_motor = CyberGearMotorController(bus=bus, motor_can_id=100)
    yaw_motor = CyberGearMotorController(bus=bus, motor_can_id=101)

    pitch_motor.disable()
    yaw_motor.disable()

    bus.shutdown()
