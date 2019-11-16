#!/usr/bin/env python
import rospy
import geometry_msgs.msg
import serial
import time
import threading
import Queue
import ctypes

from packet import (PacketTwistCommand, PacketSpeed, PacketSensor, PacketTwist)

class SerCom:
    def __init__(self, tty, baud='115200'):
        self.ser = serial.Serial(tty, baud, timeout=1)
        self.queue = Queue.Queue()

        self.event = threading.Event()
        self.thread_r = threading.Thread(target=self.recv_)
        self.thread_r.start()

    def recv_(self):
        while not self.event.is_set():
            delimiter = self.ser.read(1)
            if len(delimiter) < 1 or delimiter != b'\xff':
                continue
            header = self.ser.read(2)
            if len(header) < 2:
                continue
            type_ = header[0]
            size = ord(header[1])
            data = self.ser.read(size - 3)
            if len(data) < size - 3:
                continue
            data = delimiter + header + data
            if type_ == b'\xac':
                speed = PacketSpeed.from_buffer_copy(data)
                self.queue.put(speed)
            elif type_ == b'\xad':
                sensor = PacketSensor.from_buffer_copy(data)
                self.queue.put(sensor)
            elif type_ == b'\xae':
                twist = PacketTwist.from_buffer_copy(data)
                self.queue.put(twist)
            # hexstr = ['{:02x}'.format(ord(c)) for c in data]

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

    def publish_speed(self, struct):
        print('{}, {}'.format(struct.speed[0], struct.speed[1]))

    def publish_sensor(self, struct):
        print('{}, {}, {}, {}, {}, {}'.format( \
                struct.value[0], \
                struct.value[1], \
                struct.value[2], \
                struct.value[3], \
                struct.value[4], \
                struct.value[5]))

    def publish_twist(self, struct):
        print('{}, {}'.format(struct.tw_linear, struct.tw_angular))
        twist = geometry_msgs.msg.TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = 'odom'
        twist.twist.linear.x = struct.tw_linear
        twist.twist.angular.z = struct.tw_angular
        self.pub_speed.publish(twist)

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            twist = geometry_msgs.msg.TwistStamped()
            twist.header.stamp = rospy.Time.now()
            twist.twist.linear.x = 0.05
            twist.twist.angular.z = 0.0
            #self.pub_test.publish(twist)
            if not self.ser.queue.empty():
                struct = self.ser.queue.get()
                if type(struct) is PacketSpeed:
                    self.publish_speed(struct)
                elif type(struct) is PacketSensor:
                    self.publish_sensor(struct)
                elif type(struct) is PacketTwist:
                    self.publish_twist(struct)
            r.sleep()
        self.ser.stop()

if __name__ == '__main__':
    streamer = TopicStreamer()
    streamer.run()
