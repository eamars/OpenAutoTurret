from ultralytics import YOLO
import cv2
import threading
import queue
import math

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
            if object_type != 0:
                continue
            object_coordinate = box.xyxy[0].tolist()
            object_propability = box.conf[0].item()

            object_center_coordinate = ((object_coordinate[0] + object_coordinate[2]) / 2, (object_coordinate[1] + object_coordinate[3]))
            person_list.append((object_propability, object_center_coordinate))

        # Select the highest possible detection
        if person_list:
            most_confident_person_coord = max(person_list, key=lambda pair: pair[0])[1]
            data_pack = (most_confident_person_coord, frame.shape[0:2])
            data_queue.put(data_pack, False)

        # res_plotted = results[0].plot()
        # cv2.imshow(f"Tracking_Stream_{file_index}", res_plotted)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    # Release video sources
    video.release()


def run_motor_controller(pitch_motor: cybergear_motor_controller.CyberGearMotorController,
                         yaw_motor: cybergear_motor_controller.CyberGearMotorController,
                         data_queue: queue.Queue):
    fov = math.radians(60)

    while True:
        person_coord, frame_size = data_queue.get(True)
        print(person_coord)

        center_x = frame_size[0]/2
        center_y = frame_size[1]/2

        delta_x = person_coord[0] - center_y
        delta_y = person_coord[1] - center_x

        delta_yaw = math.atan2(delta_x, frame_size[1] / (2 * math.tan(fov/2)))
        delta_pitch = math.atan2(delta_y, frame_size[0] / (2 * math.tan(fov/2)))

        print(f"delta_yaw={delta_yaw}, delta_pitch={delta_pitch}")

        yaw_motor.move_to_position(yaw_motor.current_position + delta_yaw)
        pitch_motor.move_to_position(pitch_motor.current_position + delta_pitch)


if __name__ == "__main__":
    # Load the models
    model1 = YOLO('yolov8n.pt')
    model2 = YOLO('yolov8n-seg.pt')
    video_stream = 0

    # Initialize the motor
    bus = can.Bus(channel="can0", interface="socketcan")
    pitch_motor = CyberGearMotorController(bus=bus, motor_can_id=100)
    yaw_motor = CyberGearMotorController(bus=bus, motor_can_id=101)

    pitch_motor.zero_axis()
    yaw_motor.zero_axis()

    # Initialize the data exchange
    data_queue = queue.Queue()

    tracker_thread = threading.Thread(target=run_tracker_in_thread, args=(video_stream, model2, 1, data_queue), daemon=True)
    motor_thread = threading.Thread(target=run_motor_controller, args=(pitch_motor, yaw_motor, data_queue), daemon=True)

    tracker_thread.start()
    motor_thread.start()

    tracker_thread.join()
    motor_thread.join()
    cv2.destroyAllWindows()
