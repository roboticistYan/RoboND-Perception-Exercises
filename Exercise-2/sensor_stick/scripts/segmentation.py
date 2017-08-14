#!/usr/bin/env python
from pcl_helper import *

def pcl_callback(pcl_msg):
    
    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE,LEAF_SIZE,LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name( filter_axis )
    axis_min = 0.75
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    # TODO: RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)

    # TODO: Extract inliers and outliers
    inliers, coefficients = seg.segment()
    extracted_objects = cloud_filtered.extract(inliers, negative=True)
    extracted_table = cloud_filtered.extract(inliers, negative=False)

    # Uncomment if you want to see the filtered point cloud
    #filename = 'extracted_inliers.pcd'
    #pcl.save(extracted_inliers, filename)

    # TODO: Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ( extracted_objects )
    tree = white_cloud.make_kdtree()
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(200)
    ec.set_MaxClusterSize(2000)
    ec.set_SearchMethod(tree)
    cluster_indices = ec.Extract()

    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately
    cluster_color = get_color_list(len(cluster_indices))
    color_cluster_point_list = []
    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # TODO: Convert PCL data to ROS messages
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_table_cloud = pcl_to_ros(extracted_table)
    # TODO: Publish ROS messages
    pcl_objects_pub.publish(ros_cluster_cloud)
    pcl_table_pub.publish(ros_table_cloud)

    return


if __name__ == '__main__':
    # TODO: ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # TODO: Create Subscriber
    sub = rospy.Subscriber('/sensor_stick/point_cloud',pc2.PointCloud2,pcl_callback,queue_size=1)

    # TODO: Create Publishers
    pcl_table_pub = rospy.Publisher('/sensor_stick/pcl_table',PointCloud2,queue_size=1)
    pcl_objects_pub = rospy.Publisher('/sensor_stick/pcl_objects',PointCloud2,queue_size=1)

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
