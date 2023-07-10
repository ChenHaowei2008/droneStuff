from simple_pid import PID
import tellopy
import time
import cv2
import av
import numpy as np
from arucoStuff import detect_aruco_marker

#idk what any of these numbers mean but they work
pidX = PID(0.3, 0.1, 0.1, setpoint=150, output_limits=(-30, 30))
pidY = PID(0.3, 0.1, 0.1, setpoint=150, output_limits=(-30, 30))

def handler(event, sender, data, **args):
    drone = sender
    if(event is drone.EVENT_FLIGHT_DATA):
        print(data)
    
drone = tellopy.Tello()
drone.subscribe(drone.EVENT_FLIGHT_DATA, handler)

drone.connect()
drone.wait_for_connection(10)
drone.send_packet_data('downvision 1')

retry = 3
container = None
while container is None and 0 < retry:
    retry -= 1
    try:
        container = av.open(drone.get_video_stream())
    except av.AVError as ave:
        print(ave)
        print('retry...')

while True:
    for frame in container.decode():
        image = np.array(frame.to_image())
        result = detect_aruco_marker(image, 0)

        if(result is not None):
            moveX = pidX(result[0])
            moveY = pidY(result[1])

            print(f"Move X: {moveX}")
            print(f"Move Y: {moveY}")

        # cv2.imshow('hi', image)
        # cv2.waitKey(1)
        print(result)
 