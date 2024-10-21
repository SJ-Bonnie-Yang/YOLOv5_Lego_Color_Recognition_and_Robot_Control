"""
Opens the camera to capture images for detection by a trained model.

Press 's' to save a snapshot as 1000.jpg (this is an example file name), which will then be displayed.

Usage:
1. Ensure a camera is connected.
2. Install OpenCV: `pip install opencv-python`
3. Run the script: `python your_script_name.py`
4. Press 's' to capture an image.

Note: Check the camera index if the feed is not accessible.
"""

import os
import cv2
import time

# Set environment variable for OpenCV
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"

def capture_image(cap):
    print("Press 's' to capture image")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Failed to read frame.")
            break
        cv2.imshow("Capture", frame)
        if cv2.waitKey(1) & 0xFF == ord('s'):
            cv2.imwrite('1000.jpg', frame)
            break

def main():
    start = time.time()

    with cv2.VideoCapture(1, cv2.CAP_DSHOW) as cap:
        if not cap.isOpened():
            print("Error: Could not open video device.")
            return
        
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        end = time.time()
        print("Initialization time: %f seconds" % (end - start))

        capture_image(cap)

    img = cv2.imread("1000.jpg")
    cv2.imshow("Show", img)
    key = cv2.waitKey(0)
    if key == ord("q"):
        print("exit")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
