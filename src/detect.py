from functools import partial

import rclpy
from rclpy.node import Node

import sensor_msgs.msg
import can_msgs.msg

from cv_bridge import CvBridge
import cv2 as cv

import colordistinguish

def serialize_int(h):
    return bytearray(struct.pack('<h', h))

def detect_line_and_publish(pub, cvb, msg):
    img = cvb.imgmsg_to_cv2(msg, 'bgr8')

    res, frame, hsv, mask = colordistinguish.colorthresh(img)
    
    #cv.imshow("img", img)
    #cv.imwrite("img.png", img)
    #cv.imshow("hsv", hsv)
    #cv.imshow("mask", mask)
    #cv.waitKey(0)

    #print(res)

    print(res)

    if res == 'turn left':
        color = 1
    elif res == 'turn right':
        color = 2
    elif res == 'go straight':
        color = 3
    elif res == 'search':
        color = 4
    else:
        print("bad direction {}".format(res), file=sys.stderr)

    CAN_MSG_ID_COLOR=0x110

    msg = can_msgs.msg.Frame(id=CAN_MSG_ID_COLOR, dlc=2, data=
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

    pub.publish(msg)


image_msg = None
def image_received(msg):
    global image_msg
    image_msg = msg

def main(args=None):
    rclpy.init(args=args)

    node = Node('detect')

    color_pub = node.create_publisher(
            can_msgs.msg.Frame,
            '/to_can_bus',
            10)
    
    image_sub = node.create_subscription(
            sensor_msgs.msg.Image,
            '/camera/image_raw',
            image_received,
            10)
    
    cvb = CvBridge()

    while rclpy.ok():
        if image_msg is not None:
            detect_line_and_publish(color_pub, cvb, image_msg)
        rclpy.spin_once(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()