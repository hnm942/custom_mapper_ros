import open3d as o3d
import rospy
import rosbag 
import numpy as np 
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

if __name__ == "__main__":
    bag = rosbag.Bag("/home/hnm942/workspace/dataset/rgbd_dataset_freiburg1_360.bag")
    camera_msg = None
    for topic, msg, t in bag.read_messages(topics = ["/camera/depth/camera_info"]):
        camera_msg = msg
        break
    bridge = CvBridge()

    for topic, msg, t in bag.read_messages(topics = ["/camera/depth/image"]):
        img_msg = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        break
    bag.close()
    fx = camera_msg.K[0]
    fy = camera_msg.K[4]
    cx = camera_msg.K[2]
    cy = camera_msg.K[5]
    height = camera_msg.height
    width = camera_msg.width
    pcl = []
    print(img_msg)
    # for i in range(width):
    #     for j in range(height):
    #         depth = img_msg.data[i * width + j]
    #         pcl.append(depth)
    # pcl = np.array(pcl)
    # print(np.max(pcl))
    
        # for i in range(width):
            # for j in range(height):
                # dep
