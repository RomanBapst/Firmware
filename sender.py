import socket
import pymavlink.dialects.v10.v10 as mavlink
import pymavlink.mavutil as mav
from datetime import datetime
import time

UDP_IP = "127.0.0.1"
UDP_PORT = 14550
MESSAGE = "Hello World"

sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

mav = mavlink.MAVLink(":")
m = mavlink.MAVLink_hil_sensor_message(0,0,0,0,0,0,0,0,0,0,0,0,0,0,0)

m.pack(mav)
sock.sendto(m.get_msgbuf(),(UDP_IP,UDP_PORT))
sock.setblocking(0)

while True:
    dt = datetime.now().microsecond
    m = mavlink.MAVLink_hil_sensor_message(dt,0,0,9.81,0,0,0,0.2,0,0,0,0,0,0,0)
    m.pack(mav)
    sock.sendto(m.get_msgbuf(),(UDP_IP,UDP_PORT))
    time.sleep(0.05)
    try:
        print sock.recvfrom(5000)
    except:
        print "fail"
        
