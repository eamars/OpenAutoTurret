from ultralytics import YOLO
import cv2
import threading
import queue
import math
import time
import logging

import cybergear_motor_controller
from cybergear_motor_controller import CyberGearMotorController
import can


def run_tracker_in_thread(filename, model, file_index, data_queue: queue.Queue):
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

            if object_propability < 0.7:
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
                data_queue.put(most_confident_person_coord, block=True)
            except queue.Full:
                pass

            # Draw the center dot
            cv2.circle(res_plotted, (int(most_confident_person_coord[0]), int(most_confident_person_coord[1])), 5,
                       (255, 0, 0), -1)
            cv2.circle(res_plotted, (int(most_confident_person_coord[0]), int(most_confident_person_coord[1])), 60,
                       (255, 0, 0), 2)

        # cv2.imshow(f"Tracking_Stream_{file_index}", res_plotted)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

        # time.sleep(0.3)

    # Release video sources
    video.release()


def run_motor_controller(pitch_motor: cybergear_motor_controller.CyberGearMotorController,
                         yaw_motor: cybergear_motor_controller.CyberGearMotorController,
                         data_queue: queue.Queue,
                         resolution,
                         diagonal_fov=55):
    center_coord = (resolution[0] / 2, resolution[1] / 2)
    diagonal_resolution = math.sqrt(resolution[0] ** 2 + resolution[1] ** 2)
    hfov = 2 * math.atan(math.tan(math.radians(diagonal_fov/2)) * (resolution[0] / diagonal_resolution))
    vfov = 2 * math.atan(math.tan(math.radians(diagonal_fov/2)) * (resolution[1] / diagonal_resolution))

    print(f"hfov={math.degrees(hfov)}, vfov={math.degrees(vfov)}")

    while True:
        person_coord = data_queue.get(True)
        print(f"Person coord={person_coord}")

        delta_x = center_coord[0] - person_coord[0]
        delta_y = center_coord[1] - person_coord[1]

        delta_yaw = math.atan2(delta_x, center_coord[0] / (2 * math.tan(hfov/2))) / 2
        delta_pitch = math.atan2(delta_y, center_coord[1] / (2 * math.tan(vfov/2))) / 2

        current_yaw = yaw_motor.get_position()
        current_pitch = pitch_motor.get_position()
        new_yaw = current_yaw + delta_yaw
        new_pitch = current_pitch + delta_pitch
        print(f"current_yaw={current_yaw}, current_pitch={current_pitch}")
        print(f"delta_yaw={delta_yaw}, delta_pitch={delta_pitch}")
        print(f"new_yaw={new_yaw}, new_pitch={new_pitch}")

        yaw_motor.set_position(new_yaw)
        pitch_motor.set_position(new_pitch)


if __name__ == "__main__":
    # logging.basicConfig(level=logging.DEBUG)

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

    pitch_motor.zero_axis()
    yaw_motor.zero_axis()

    # Initialize the data exchange
    data_queue = queue.Queue(1)

    tracker_thread = threading.Thread(target=run_tracker_in_thread, args=(video_stream, model2, 1, data_queue), daemon=True)
    motor_thread = threading.Thread(target=run_motor_controller, args=(pitch_motor, yaw_motor, data_queue, (640, 480), 55), daemon=True)

    tracker_thread.start()
    motor_thread.start()

    tracker_thread.join()
    motor_thread.join()
    cv2.destroyAllWindows()
