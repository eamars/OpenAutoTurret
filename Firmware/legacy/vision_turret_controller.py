import time


class VisionTurretController:
    def __init__(self, turret, center_coordinate, yaw_config, pitch_config, queue):
        self.turret = turret
        self.center_coordinate = center_coordinate
        self.yaw_config = yaw_config.copy()
        self.pitch_config = pitch_config.copy()

        self.queue = queue

        # Private variables
        self._last_sample_time = None

        # Initialize some variables
        self.yaw_config["intergal"] = 0
        self.pitch_config["intergal"] = 0
        self.yaw_config["last_error"] = 0
        self.pitch_config["last_error"] = 0

    def update_coordinate(self, new_coordinate, *put_args, **put_kwargs):
        self.queue.put(new_coordinate, *put_args, **put_kwargs)

    def control(self):
        new_coordinate = self.queue.get(block=True)

        # Feedback
        sample_time = time.time()
        delta_x = self.center_coordinate[0] - new_coordinate[0]
        delta_y = self.center_coordinate[1] - new_coordinate[1]

        # Get current position
        current_yaw = self.turret.yaw_motor.get_position()
        current_pitch = self.turret.pitch_motor.get_position()

        # Control
        if self._last_sample_time is None:
            self._last_sample_time = sample_time - 1
        dt = sample_time - self._last_sample_time

        feed_forward_yaw = self.apply_feed_forward_control(current_yaw, self.yaw_config)
        feed_forward_pitch = self.apply_feed_forward_control(current_pitch, self.yaw_config)

        delta_yaw = self.apply_feed_back_kp_control(delta_x, self.yaw_config) + \
                    self.apply_feed_back_ki_control(delta_x, dt, self.yaw_config) + \
                    self.apply_feed_back_kd_control(delta_x, dt, self.yaw_config)

        delta_pitch = self.apply_feed_back_kp_control(delta_y, self.yaw_config) + \
                      self.apply_feed_back_ki_control(delta_y, dt, self.pitch_config) + \
                      self.apply_feed_back_kd_control(delta_x, dt, self.pitch_config)

        new_yaw = current_yaw + delta_yaw + feed_forward_yaw
        new_pitch = current_pitch + delta_pitch + feed_forward_pitch

        self.turret.yaw_motor.set_position(new_yaw)
        self.turret.pitch_motor.set_position(new_pitch)

    def apply_feed_forward_control(self, set_point, config):
        kf = config.get("kf")
        control = kf * set_point
        return control

    def apply_feed_back_kp_control(self, error, config):
        kp = config.get("kp")
        control = kp * error
        return control

    def apply_feed_back_ki_control(self, error, dt, config):
        ki = config.get("ki")
        config["intergal"] += error

        control = ki * config["intergal"]
        return control

    def apply_feed_back_kd_control(self, error, dt, config):
        kd = config.get("kd")
        derivative = (error - config["last_error"]) / dt

        control = kd * derivative
        return control

