from djitellopy import Tello
from subprocess import check_output
import re

ips = check_output(["nmap", "-sn", "192.168.50.1/24"]).strip().split(b'\n')
ips = [re.findall(b"((?:\d+\.){3}\d+)", i)[0] for i in ips if b"Nmap scan report for RMTT" in i]
# ten million iq play

robot_sn_list = [
    "0TQZK8CCNT2CBY", # drone 1
    "0TQZK88CNT28AZ", # drone 2
    "0TQZK8CCNT2CDF", # drone 3
    "0TQZK8BCNT2BPP", # drone 3 part 2
    "0TQZK4BCNT1YQ6", # drone 4
    "0TQZK8CCNT2CAV", # drone 7
    "0TQZK8CCNT2CA2", # drone 8
    "0TQZK5WCNT1ZEB", # drone 8 part 2
]

ip_sn_dict = {}

def construct_sn_ip(tello, ip):
     global ip_sn_dict
     ip_sn_dict.update({tello.query_serial_number(): [ip, 8889]})
     tello.send_command_without_return("downvision 1")
     
for ip in ips:
     tello = Tello(ip.decode())
     construct_sn_ip(tello, ip)
