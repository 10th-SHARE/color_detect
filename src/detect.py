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

class ColorSerializer:
    CAN_MSG_ID_COLOR=0x110

    def construct_can_frame(self, color):
    
        color_frame = can_msgs.msg.Frame(id=self.CAN_MSG_ID_COLOR, dlc=2, data=
                serialize_int(
                    color
                )
                + serialize_int(
                    0
                )
                + serialize_int(
                    0
                )
                + serialize_int(
                    0
                )
            )
    
        return color_frame


class Detect(Node):
    def __init__(self, cser, cvb):
        super().__init__('detect')

        self.cser = cser        
        self.cvb = cvb

        self.image_subscriber = self.create_subscription(
                sensor_msgs.msg.Image, '/camera/image_raw', self.image_received, 10)

        self.can_sender_publisher = self.create_publisher(
                can_msgs.msg.Frame, 'to_can_bus', 10)

    def image_received(self, msg):
        img = self.cvb.imgmsg_to_cv2(msg, 'bgr8')

        res, frame, hsv, mask = colordetect.colorthresh(img)

        #cv.imshow("img", img)
        #cv.imwrite("img.png", img)
        #cv.imshow("hsv", hsv)
        #cv.imshow("mask", mask)
        #cv.waitKey(0)

        print(res)

        if res == 'red':
            color = 1
        elif res == 'blue':
            color = 2
        elif res == 'purple':
            color = 3
        elif res == 'search':
            color = 4
        else:
            print("bad direction {}".format(res), file=sys.stderr)

        frame = self.cser.construct_can_frame(color)

        self.can_sender_publisher.publish(frame)


def main(args=None):
    rclpy.init(args=args)

    cvb = CvBridge()

    detect = Detect(ColorSerializer(), cvb)
    
    rclpy.spin(detect)

    rclpy.shutdown()

if __name__ == '__main__':
    main()