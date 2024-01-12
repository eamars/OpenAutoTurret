from ultralytics import YOLO
import cv2

def run_tracker_in_thread(filename, model, file_index):
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

            object_center_coordinate = ((object_coordinate[0] + object_coordinate[2]) / 2, (object_coordinate[1] + object_coordinate[3]) / 2)
            person_list.append((object_propability, object_center_coordinate))

        # Plot the rendered image from Yolo
        res_plotted = results[0].plot()

        # Select the highest possible detection
        if person_list:
            most_confident_person_coord = max(person_list, key=lambda pair: pair[0])[1]

            # Draw the center dot
            cv2.circle(res_plotted, (int(most_confident_person_coord[0]), int(most_confident_person_coord[1])), 5, (255, 0, 0), -1)


        cv2.imshow(f"Tracking_Stream_{file_index}", res_plotted)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    # Release video sources
    video.release()


if __name__ == "__main__":
    model2 = YOLO('yolov8n-seg.pt')
    run_tracker_in_thread(0, model2, 1)