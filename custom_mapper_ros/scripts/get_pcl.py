import open3d as o3d
import rospy
import rosbag 
import numpy as np 
import matplotlib.pyplot as plt
from cv_bridge import CvBridge

if __name__ == "__main__":
    bag = rosbag.Bag("/home/hnm942/workspace/src/custom_mapper_ros/custom_mapper_ros/data/rgbd_dataset_freiburg1_360.bag")
    camera_msg = None
    for topic, msg, t in bag.read_messages(topics = ["/camera/depth/camera_info"]):
        camera_msg = msg
        break
    bridge = CvBridge()
    fx = camera_msg.K[0]
    fy = camera_msg.K[4]
    cx = camera_msg.K[2]
    cy = camera_msg.K[5]
    height = camera_msg.height
    width = camera_msg.width

    for topic, msg, t in bag.read_messages(topics = ["/camera/depth/image"]):
        img_msg = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        pcl = []
        for i in range(height):
            for j in range(width):
                depth = img_msg[i, j]
                if np.isfinite(depth):
                    y = (i - cy) / fy * depth
                    x = (j - cx) / fx * depth
                    z = depth
                    pcl.append(np.array([x, y, z]))
        pcl = np.array(pcl)
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcl)
        # o3d.visualization.draw_geometries([pcd])
        o3d.io.write_point_cloud("/home/hnm942/workspace/src/custom_mapper_ros/custom_mapper_ros/data/rgbd_dataset_freiburg1_360/point_cloud_{}.ply".format(t.secs), pcd)
    bag.close()

    