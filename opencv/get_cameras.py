import cv2

def list_available_cameras():
    index = 0
    available_cameras = []

    while True:
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            break
        else:
            available_cameras.append(index)
            cap.release()
        index += 1

    return available_cameras

print("Available cameras:", list_available_cameras())