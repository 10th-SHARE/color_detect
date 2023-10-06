import sys
import struct

import rclpy
from rclpy.node import Node

import sensor_msgs.msg
import can_msgs.msg

from cv_bridge import CvBridge
import cv2 as cv

import colordetect

def serialize_int(h):
    return bytearray(struct.pack('<h', h))

def construct_color_can_frame(color, color_id):
    if color == 'other':
        data = bytearray(b'\x00\x00\x00\x00\x00\x00\x00\x00')
    elif color == 'red':
        data = bytearray(b'\x01\x00\x00\x00\x00\x00\x00\x00')
    elif color == 'blue':
        data = bytearray(b'\x02\x00\x00\x00\x00\x00\x00\x00')
    elif color == 'purple':
        data = bytearray(b'\x04\x00\x00\x00\x00\x00\x00\x00')
    elif color == 'black':
        data = bytearray(b'\x08\x00\x00\x00\x00\x00\x00\x00')
    elif color == 'error':
        data = bytearray(b'\xFF\x00\x00\x00\x00\x00\x00\x00')
    else:
        raise Exception("bad color")

    return can_msgs.msg.Frame(id=color_id, dlc=8, data=data)



class Detect(Node):
    def __init__(self, cvb):
        super().__init__('detect')
        
        self.cvb = cvb

        self.image_subscriber = self.create_subscription(
                sensor_msgs.msg.Image, '/image_raw', self.image_received, 10)

        self.can_sender_publisher = self.create_publisher(
                can_msgs.msg.Frame, 'to_can_bus', 10)

    def image_received(self, msg):
        img = self.cvb.imgmsg_to_cv2(msg, 'bgr8')

        color, frame, hsv = colordetect.colorthresh(img)

        #cv.imshow("img", img)
        #cv.imwrite("img.png", img)
        #cv.imshow("hsv", hsv)
        #cv.imshow("mask", mask)
        #cv.waitKey(0)

        print(color)

        frame = construct_color_can_frame(color, color_id=0x104)

        self.can_sender_publisher.publish(frame)


def main(args=None):
    rclpy.init(args=args)

    cvb = CvBridge()

    detect = Detect(cvb)
    
    rclpy.spin(detect)

    rclpy.shutdown()

if __name__ == '__main__':
    main()