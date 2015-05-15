import socket
import pymavlink.dialects.v10.common as mavlink
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
#sock.setblocking(0)
sock.settimeout(0.01)

while True:
    dt = datetime.now().microsecond
    m = mavlink.MAVLink_hil_sensor_message(dt,0,0,9.81,0,0,0,0.2,0,0,0,0,0,0,0)
    m.pack(mav)
    sock.sendto(m.get_msgbuf(),(UDP_IP,UDP_PORT))
    try:
        mess_raw = 0
        mess_raw = sock.recvfrom(5000)
        message = 0
        message = mav.decode(mess_raw[0])
        print message
    except:
        print "fail"
        
