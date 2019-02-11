#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import serial
import time
import threading
import Queue

import ctypes

"""
typedef struct{
  uint8_t  marker[2];           //  2 bytes: 0xFF, 0xAB
  uint8_t  size;                //  1 bytes
  uint8_t  reserved;            //  1 bytes
  TWIST    twist;               //  8 bytes
  uint8_t  sum;                 //  1 bytes
  uint8_t  reserved2[3];        //  3 bytes
}PACKET_TWIST_COMMAND;
"""
class PacketTwistCommand(ctypes.Structure):
    _fields_ = (
        ('marker'     , ctypes.c_uint8 * 2),
        ('size'       , ctypes.c_uint8),
        ('reserved'   , ctypes.c_uint8),
        ('tw_linear'  , ctypes.c_float),
        ('tw_angular' , ctypes.c_float),
        ('sum'        , ctypes.c_uint8),
        ('reserved2'  , ctypes.c_uint8 * 3),
    )

class SerCom:
    def __init__(self, tty, baud='115200'):
        self.ser = serial.Serial(tty, baud, timeout=1)
        self.queue = Queue.Queue()

        self.event = threading.Event()
        self.thread_r = threading.Thread(target=self.recv_)
        self.thread_r.start()

    def recv_(self):
        while not self.event.is_set():
            c = self.ser.read(1)
            if len(c) < 1:
                continue
            #print('{:02x}'.format(ord(c)))
            #self.queue.put(line)

    def send(self, data):
        self.ser.write(data)

    def stop(self):
        self.event.set()
        self.thread_r.join()

class TopicStreamer:
    def __init__(self):
        rospy.init_node('topic_streamer', anonymous=True)
        self.pub_speed = rospy.Publisher('odom/twist', geometry_msgs.msg.TwistStamped, queue_size=10)
        self.pub_test = rospy.Publisher('command/twist', geometry_msgs.msg.TwistStamped, queue_size=10)
        rospy.Subscriber("command/twist", geometry_msgs.msg.TwistStamped, self.callback)
        
        self.ser = SerCom('/dev/ttyS0', '115200')

    def callback(self, msg):
        rospy.loginfo('{}, {}, {}'.format(msg.header.stamp, msg.twist.linear.x, msg.twist.angular.z))
        packet = PacketTwistCommand()
        packet.marker = (ctypes.c_uint8 * 2)(ctypes.c_uint8(ord(b'\xff')), ctypes.c_uint8(ord(b'\xab')))
        packet.size   = ctypes.sizeof(PacketTwistCommand)
        packet.tw_linear = ctypes.c_float(msg.twist.linear.x)
        packet.tw_angular = ctypes.c_float(msg.twist.angular.z)
        self.ser.send(packet)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            twist = geometry_msgs.msg.TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x = 0.05
            twist.twist.angular.z = 0.0
            #self.pub_test.publish(twist)
            r.sleep()
        self.ser.stop()

if __name__ == '__main__':
    streamer = TopicStreamer()
    streamer.run()
