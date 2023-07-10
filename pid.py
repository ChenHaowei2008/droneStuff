from simple_pid import PID
from djitellopy import Tello
import time
tello = Tello()
#idk what any of these numbers mean
pid = PID(0.3, 0.1, 0.1, setpoint=150, output_limits=(-30, 30))
tello.connect()
tello.takeoff()

#This look basically makes the drone hover around 1.5m high.
#This is just a proof of concept
while True:
    time.sleep(0.05)
    height = tello.get_distance_tof()
    target = pid(height)
    print(f"pid: {target}")
    print(f"height: {height}")
    if(target < 20 and target > -20):
        # Hover
        continue

    if(target < 0):
        tello.move_down(int(abs(target)))
    else:
        tello.move_up(int(target))