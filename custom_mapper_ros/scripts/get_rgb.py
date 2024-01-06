import open3d as o3d
import rospy
import rosbag 
import numpy as np 
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

if __name__ == "__main__":
    bag = rosbag.Bag("/home/hnm942/workspace/src/custom_mapper_ros/custom_mapper_ros/data/rgbd_dataset_freiburg1_360.bag")
    camera_msg = None
    bridge = CvBridge()


    for topic, msg, t in bag.read_messages(topics = ["/camera/rgb/image_color"]):
        img_msg = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        plt.imsave("/home/hnm942/workspace/src/custom_mapper_ros/custom_mapper_ros/data/rgbd_dataset_freiburg1_360/color_images/image_color_{}.png".format(t), img_msg)
    print("a")
    bag.close()

    