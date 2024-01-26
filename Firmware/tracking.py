from ultralytics import YOLO
import cv2
import threading
import queue
import math
import time
import logging

from cybergear_motor_controller import CyberGearMotorController
from turret import Turret
from vision_turret_controller import VisionTurretController
import can


def run_tracker_in_thread(filename, model, file_index, vision_controller: VisionTurretController):
    """
    Runs a video file or webcam stream concurrently with the YOLOv8 model using threading.

    This function captures video frames from a given file or camera source and utilizes the YOLOv8 model for object
    tracking. The function runs in its own thread for concurrent processing.

    Args:
        filename (str): The path to the video file or the identifier for the webcam/external camera source.
        model (obj): The YOLOv8 model object.
        file_index (int): An index to uniquely identify the file being processed, used for display purposes.

    Note:
        Press 'q' to quit the video display window.
    """
    video = cv2.VideoCapture(filename)  # Read the video file

    while True:
        ret, frame = video.read()  # Read the video frames

        # Exit the loop if no more frames in either video
        if not ret:
            break

        # Track objects in frames if available
        results = model.track(frame, persist=True, verbose=False)

        person_list = []
        for box in results[0].boxes:
            object_type = box.cls[0].item()
            object_propability = box.conf[0].item()

            if object_type != 0:
                continue

            if object_propability < 0.5:
                continue
            object_coordinate = box.xyxy[0].tolist()

            object_center_coordinate = ((object_coordinate[0] + object_coordinate[2]) / 2,
                                        (object_coordinate[1] + object_coordinate[3]) / 2.8)
            person_list.append((object_propability, object_center_coordinate))

        # Plot the rendered image from Yolo
        res_plotted = results[0].plot()

        # Select the highest possible detection
        if person_list:
            most_confident_person_coord = max(person_list, key=lambda pair: pair[0])[1]

            try:
                vision_controller.update_coordinate(most_confident_person_coord, block=False)
            except queue.Full:
                pass

            # Draw the center dot
            cv2.circle(res_plotted, (int(most_confident_person_coord[0]), int(most_confident_person_coord[1])), 5,
                       (0, 0, 255), -1)
            cv2.circle(res_plotted, (int(most_confident_person_coord[0]), int(most_confident_person_coord[1])), 50,
                       (0, 0, 255), 2)

        cv2.imshow(f"Tracking_Stream_{file_index}", res_plotted)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        # time.sleep(0.3)

    # Release video sources
    video.release()


def run_motor_controller(vision_controller: VisionTurretController):

    while True:
        vision_controller.control()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    from yourcee_usb_to_can import YourCeeSerialBus
    bus = YourCeeSerialBus(channel="COM4")

    # Load the models
    model1 = YOLO('yolov8n.pt')
    model2 = YOLO('yolov8n-seg.pt')
    video_stream = 0

    # Initialize the motor
    # bus = can.Bus(channel="can0", interface="socketcan")
    pitch_motor = CyberGearMotorController(bus=bus, motor_can_id=100)
    yaw_motor = CyberGearMotorController(bus=bus, motor_can_id=101)
    turret = Turret(yaw_motor, pitch_motor, dict())
    turret.zero_all()

    # Initialize the data exchange
    data_queue = queue.Queue(1)

    # Initial controller
    yaw_config = dict(
        kf=0.01,
        kp=0.0005,
        ki=0.0001,
        kd=0.0001,
    )

    pitch_config = dict(
        kf=0.01,
        kp=0.0005,
        ki=0.0001,
        kd=0.0001,
    )
    vision_controller = VisionTurretController(turret, (320, 240), yaw_config, pitch_config, data_queue)

    tracker_thread = threading.Thread(target=run_tracker_in_thread, args=(video_stream, model2, 1, vision_controller), daemon=True)
    motor_thread = threading.Thread(target=run_motor_controller, args=(vision_controller,), daemon=True)

    tracker_thread.start()
    motor_thread.start()

    tracker_thread.join()
    motor_thread.join()
    cv2.destroyAllWindows()
