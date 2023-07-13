from multi_robomaster import multi_robot
import time
import socket
import select
import threading
import queue
import h264decoder
import numpy as np
import cv2
from PIL import Image as PImage
from config import ip_sn_dict
from config import robot_sn_list
from arucoStuff import detect_aruco_marker

decoded_video_data_queue = {}
video_socket = None
video_process_threads = {}
video_server = []
decoder = {}
video_packet_data = {}
display_video_process_thread = None
video_receive_process_thread = None

def takeoff(robot_group):
    robot_group.takeoff()
    time.sleep(3)

def group_streamon(robot_group):
    robot_group.send_command("streamon") # turn on video stream
    print('trying')        
    time.sleep(3)
    print("stream on")

def display_video():
    global decoded_video_data_queue
    print('Start display')
    while True:
        img = None
        servers = decoded_video_data_queue.copy().keys()
        for a in servers:         
            dframe = decoded_video_data_queue[a].get()         
            if dframe is None:
                continue
            timage = PImage.fromarray(dframe)
            timg = cv2.cvtColor(np.array(timage), cv2.COLOR_RGB2BGR) # Tello color channel is in RGB while OpenCV process photo in channel BGR. So have to convert.
            if img is None:
                img = timg
            else:
                img1 = img.copy()
                img = np.concatenate((img1,timg), axis=1) 
        if img is not None and len(img) > 0:
            if img.shape[1] > 1024:
                cv2.resize(img, (int(img.shape[1] * 10 / 100), int(img.shape[0] * 10 /100)))
            cv2.imshow('video', img) # do not remove this line. Without this line that is no video.
            cv2.waitKey(1)

def process_sock():
    global video_data_queue,video_server,display_video_process_thread, video_process_threads
    print ('Start video logger')
    h264decoder.disable_logging()
    video_sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    video_sock.bind(('0.0.0.0', 11111))
    video_sock.setblocking(False)
    r_socks = []
    w_socks = []
    a_socks = []
    w_socks.append(video_sock)
    r_socks.append(video_sock)
    a_socks.append(video_sock)
    print("start video listening socket")

    while True:
        try:
            readable, writable, exceptional = select.select(r_socks,w_socks,a_socks)
        except Exception:
            readable = []
            writable = []
            exceptional = []
        for r in readable:
            if r is video_sock:
                vdata, address = video_sock.recvfrom(4096)
                if address[0] not in video_server or len(video_server) == 0:               
                    video_server.append(address[0])
                    print("New video client %s" % address[0])
                    decoded_video_data_queue[address[0]] = queue.Queue(32)
                    video_packet_data[address[0]] = vdata
                    decoder[address[0]] = h264decoder.H264Decoder()
                else:
                    video_packet_data[address[0]] += vdata
                video_process(vdata,address[0])
                       
def video_process(vdata,address):
    global video_packet_data, decoder, decoded_video_data_queue
    a = address
    if len(vdata) != 1460 and len(vdata) > 0:
        for frame in h264_decode(video_packet_data[a],address):
            try:
                decoded_video_data_queue[a].put(frame,timeout=2)
            except queue.Full:
                print("Drone %s - video queue full" % a)
                decoded_video_data_queue[a].get()
                decoded_video_data_queue[a].put(frame)
                continue
        video_packet_data[a] = b''
 
def h264_decode(packet_data, address):
    global decoder
    res_frame_list = []
    frames = decoder[address].decode(packet_data)
    for framedata in frames:
        (frame, w, h, ls) = framedata
        if frame is not None:
            frame = np.frombuffer(frame, dtype=np.ubyte, count=len(frame))
            frame = (frame.reshape((h, ls//3, 3)))
            frame = frame[:, :w, :]
            res_frame_list.append(frame)
    return res_frame_list

def moveToAruco(drone_obj, marker_id, ip, extraMovement = None):
    global decoded_video_data_queue
    if(extraMovement is not None):
        drone_obj.send_command(extraMovement)
    while True:
        dframe = decoded_video_data_queue[ip].get()         
        if dframe is None:
            continue
        timage = PImage.fromarray(dframe)
        img = cv2.cvtColor(np.array(timage), cv2.COLOR_RGB2BGR) # Tello color channel is in RGB while OpenCV process photo in channel BGR. So have to convert.
        results = detect_aruco_marker(image, marker_id)

        if(results is not None):
            moved = False
            moveX = results[0]
            moveY = results[0]
            if(moveX < 0):
                moved = True
                drone_obj.send_command(f"back {-1 * moveX}")
            else:
                moved = True
                drone_obj.send_command(f"forward {moveX}")
        
            if(moveY < 0):
                moved = True
                drone_obj.send_command(f"left {-1 * moveY}")
            else:
                moved = True
                drone_obj.send_command(f"right {moveY}")
            
            if(moved == False):
                return      
        return          

def flipDrones(robot_group):
    robot_group.send_command("flip")
    time.sleep(1)
    robot_group.send_command("flip")
    
def allUp(robot_group):
    robot_group.send_command(f"go 0 0 120 40")

def allDown(robot_group):
    robot_group.send_command(f"go 0 0 100 40")

def allDown2(robot_group):
    robot_group.send_command(f"go 0 0 100 40")

if __name__ == '__main__':

    print("starting swarm")
    
    multi_drone = multi_robot.MultiDrone()
    multi_drone.initialize(len(robot_sn_list), ip_sn_dict)
    print(robot_sn_list)
    print(ip_sn_dict)
    multi_drone.number_id_by_sn(*([i, robot_sn_list[i]] for i in range(len(robot_sn_list))))
    
    all_drone_group = multi_drone.build_group([i for i in range(len(robot_sn_list))])

    print('Swarm found.')

    multi_drone.run([all_drone_group, group_streamon])

    video_receive_process_thread = threading.Thread(target=process_sock)
    video_receive_process_thread.daemon = True
    video_receive_process_thread.start()
    time.sleep(7)

    print("all launched.")

    vidThread = threading.Thread(target=display_video)
    vidThread.start()

    # multi_drone.run([all_drone_group,takeoff])

    
    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 8, ip_sn_dict[robot_sn_list[i]][0], "go 0 0 100 40")))
        if(temp == 2):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 9, ip_sn_dict[robot_sn_list[i]][0], "go 0 0 100 40")))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 10, ip_sn_dict[robot_sn_list[i]][0], "go 0 0 100 40")))
        if(temp == 4): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 11, ip_sn_dict[robot_sn_list[i]][0], "go 0 0 100 40")))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 14, ip_sn_dict[robot_sn_list[i]][0], "go 0 0 100 40")))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 15, ip_sn_dict[robot_sn_list[i]][0], "go 0 0 100 40")))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 16, ip_sn_dict[robot_sn_list[i]][0], "go 0 0 100 40")))
        if(temp == 8): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 17, ip_sn_dict[robot_sn_list[i]][0], "go 0 0 100 40")))
    for thread in allthreads:
        thread.start()

    time.sleep(5)
    print('running 1')    
    multi_drone.run([all_drone_group, allUp])
    
    time.sleep(1)
    print('running 1')

    multi_drone.run([all_drone_group, allDown])

    time.sleep(1)

    print('running 1')
    multi_drone.run([all_drone_group, allDown2])

    time.sleep(1)

    print('running 1')
    multi_drone.run([all_drone_group, allDown])

    time.sleep(1)

    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 7, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 2): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 8, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 11, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 4):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 12, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 13, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 14, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 17, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 8):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 18, ip_sn_dict[robot_sn_list[i]][0])))
    for thread in allthreads:
        thread.start()

    time.sleep(5)

    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 7, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 2): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 20, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 23, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 4):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 12, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 13, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 25, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 28, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 8):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 18, ip_sn_dict[robot_sn_list[i]][0])))
    for thread in allthreads:
        thread.start()

    time.sleep(5)

    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 8, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 2): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 20, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 23, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 4):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 11, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 14, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 25, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 28, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 8):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 17, ip_sn_dict[robot_sn_list[i]][0])))
    for thread in allthreads:
        thread.start()
    
    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 9, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 2): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 21, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 22, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 4):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 10, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 15, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 26, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 27, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 8):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 16, ip_sn_dict[robot_sn_list[i]][0])))
    for thread in allthreads:
        thread.start()
        
    time.sleep(5)

    print('running 1')
    multi_drone.run([all_drone_group, allUp])

    time.sleep(5)

    print('running 1')
    multi_drone.run([all_drone_group, flipDrones])

    time.sleep(1)

    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 8, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 2): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 20, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 23, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 4):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 11, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 14, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 25, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 28, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 8):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 17, ip_sn_dict[robot_sn_list[i]][0])))
    for thread in allthreads:
        thread.start()
    
    time.sleep(5)

    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 7, ip_sn_dict[robot_sn_list[i]][0], "flip ")))
        if(temp == 2): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 8, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 11, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 4):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 12, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 13, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 14, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 17, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 8):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 18, ip_sn_dict[robot_sn_list[i]][0])))
    for thread in allthreads:
        thread.start()

    time.sleep(5)

    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 7, ip_sn_dict[robot_sn_list[i]][0], "down 20")))
        if(temp == 2): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 8, ip_sn_dict[robot_sn_list[i]][0], "down 20")))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 11, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 4):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 12, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 13, ip_sn_dict[robot_sn_list[i]][0], "down 20")))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 14, ip_sn_dict[robot_sn_list[i]][0], "down 20")))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 17, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 8):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 18, ip_sn_dict[robot_sn_list[i]][0])))
    for thread in allthreads:
        thread.start()

    time.sleep(5)

    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 7, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 2): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 8, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 11, ip_sn_dict[robot_sn_list[i]][0], "down 20")))
        if(temp == 4):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 12, ip_sn_dict[robot_sn_list[i]][0], "down 20")))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 13, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 14, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 17, ip_sn_dict[robot_sn_list[i]][0], "down 20")))
        if(temp == 8):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 18, ip_sn_dict[robot_sn_list[i]][0], "down 20")))
    for thread in allthreads:
        thread.start()

    time.sleep(5)

    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 7, ip_sn_dict[robot_sn_list[i]][0], "up 20")))
        if(temp == 2): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 8, ip_sn_dict[robot_sn_list[i]][0], "up 20")))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 11, ip_sn_dict[robot_sn_list[i]][0], "up 20")))
        if(temp == 4):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 12, ip_sn_dict[robot_sn_list[i]][0], "up 20")))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 13, ip_sn_dict[robot_sn_list[i]][0], "up 20")))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 14, ip_sn_dict[robot_sn_list[i]][0], "up 20")))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 17, ip_sn_dict[robot_sn_list[i]][0], "up 20")))
        if(temp == 8):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 18, ip_sn_dict[robot_sn_list[i]][0], "up 20")))
    for thread in allthreads:
        thread.start()

    time.sleep(5)

    print('running 1')
    allthreads = []
    for i, sn in enumerate(robot_sn_list):
        temp = i + 1
        if(temp == 1):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 7, ip_sn_dict[robot_sn_list[i]][0], "down 100")))
        if(temp == 2): 
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 8, ip_sn_dict[robot_sn_list[i]][0], "down 100")))
        if(temp == 3):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 11, ip_sn_dict[robot_sn_list[i]][0], "down 100")))
        if(temp == 4):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 12, ip_sn_dict[robot_sn_list[i]][0], )))
        if(temp == 5):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 13, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 6):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 14, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 7):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 17, ip_sn_dict[robot_sn_list[i]][0])))
        if(temp == 8):
            allthreads.append(threading.Thread(target = moveToAruco, args = (all_drone_group.get_robot(i), 18, ip_sn_dict[robot_sn_list[i]][0])))
    for thread in allthreads:
        thread.start()

    time.sleep(2000)