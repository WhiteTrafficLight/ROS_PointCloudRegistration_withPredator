#!/usr/bin/env python3.8
import os
import rospy
import open3d as o3d
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import ros_numpy
import numpy as np

def convert_o3d_to_ros(pcd):
    points = np.asarray(pcd.points, dtype=np.float32)
    
    points[:,1] += -1.0
    points[:,2] += 0.5
    dtype = np.dtype([('x', np.float32), ('y', np.float32), ('z', np.float32)])
    structured_points = np.zeros(points.shape[0], dtype=dtype)
    structured_points['x'] = points[:, 0]
    structured_points['y'] = points[:, 1]
    structured_points['z'] = points[:, 2]
    # Convert structured NumPy array to ROS PointCloud2 message
    ros_pcd = ros_numpy.msgify(PointCloud2, structured_points)
    return ros_pcd

def load_and_publish_point_cloud(path_to_ply, publisher):
    if not os.path.isfile(path_to_ply):
        rospy.logerr(f"File {path_to_ply} does not exist.")
        return
    
    try:
        pcd = o3d.io.read_point_cloud(path_to_ply)  # Load the point cloud
    except Exception as e:
        rospy.logerr(f"Failed to read point cloud: {e}")
        return

    if not pcd.has_points():
        rospy.logwarn("Loaded point cloud has no points.")
        return
    #pcd = o3d.io.read_point_cloud(path_to_ply)  # Load the point cloud
    cleaned_pcd, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    voxel_down_pcd = cleaned_pcd.voxel_down_sample(voxel_size=0.01)
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'  # Frame relative to which the point cloud is defined

    # Convert Open3D PCD to ROS PointCloud2
    ros_pcd = convert_o3d_to_ros(voxel_down_pcd)
    ros_pcd.header = header
    publisher.publish(ros_pcd)

if __name__ == "__main__":
    rospy.init_node('static_point_cloud_publisher')
    pcl_pub = rospy.Publisher('/input_point_cloud', PointCloud2, queue_size=1)
    path_to_ply = '/home/ubuntu/catkin_ws/src/point_cloud_publisher/scripts/ply_files/Studio.ply'
    
    rospy.sleep(2)
    
    # Publish the point cloud once
    load_and_publish_point_cloud(path_to_ply, pcl_pub)

    rospy.loginfo("Point cloud published, now waiting for octomap to be updated...")

    # Keep the node running until the octomap is updated
    rospy.spin()



