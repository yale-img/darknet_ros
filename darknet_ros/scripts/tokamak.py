#!/usr/bin/env python3
"""
Node to synchronise incoming Kinect RGB camera streams to relay to Darknet

Created 08/13/2023 by Sasha Lew
"""
from message_filters import ApproximateTimeSynchronizer, Subscriber
import rospy

from sensor_msgs.msg import Image

class Tokamak:
    """Feedforward synchronised master and sub Kinect RGB images."""
    def __init__(self):
        self.image_fusion_pub = rospy.Publisher("/kinect/combined/image_raw", Image, queue_size=15)
        master_image = Subscriber("/kinect/master/rgb/image_raw", Image)
        sub_image = Subscriber("/kinect/sub/rgb/image_raw", Image)
        ts = ApproximateTimeSynchronizer([master_image, sub_image], queue_size=15, slop=0.1)
        ts.registerCallback(self.image_callback)

    def image_callback(self, master_im, sub_im):
        self.image_fusion_pub.publish(master_im)
        # NOTE: sleeping between publishing allows Darknet threading to fetch both images
        rospy.sleep(0.03)
        self.image_fusion_pub.publish(sub_im)

def main():
    try:
        rospy.init_node("tokamak")
        Tokamak()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()
