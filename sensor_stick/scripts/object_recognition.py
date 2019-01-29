#!/usr/bin/env python

import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder

import pickle

from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker

from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # TODO: Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)
    
    # TODO: Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # TODO: PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    passthrough.set_filter_field_name('z')
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
    inliers, coefficients = seg.segment()
    

    # TODO: Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # TODO: Euclidean Clustering
    # Convert the xyzrgb cloud to only xyz since k-d tree only needs xyz
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    # k-d tree decreases the computation of searching for neighboring points
    #   PCL's euclidian clustering algo only supports k-d trees
    tree = white_cloud.make_kdtree()
    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    # Set tolerances for distance threshold 
    # as well as minimum and maximum cluster size (in points)
    # NOTE: These are poor choices of clustering parameters
    # Your task is to experiment and find values that work for segmenting objects.
    ec.set_ClusterTolerance(0.05)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(3000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()
    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))
    
    # Store the detected objects and labels in these lists
    detected_objects_labels = []
    detected_objects = []
    color_cluster_point_list = []

    # Iterate through each detected object cluster for object recognition
    for index, pts_list in enumerate(cluster_indices):
        
        # Store the object's cloud in this list
        object_cluster = []

        # Create an individual cluster just for the object being processed
        for i, pts in enumerate(pts_list):
            # Retrieve cloud values for the x, y, z, rgb object
            object_cluster.append([cloud_objects[pts][0],
                                   cloud_objects[pts][1],
                                   cloud_objects[pts][2],
                                   cloud_objects[pts][3]])
            
            # Retrieve cloud values for the x, y, z object, assigning a
            #   preidentified color to all cloud values
            color_cluster_point_list.append([white_cloud[pts][0],
                                             white_cloud[pts][1],
                                             white_cloud[pts][2],
                                             rgb_to_float(cluster_color[index])])


        # Convert list of point cloud features (x,y,z,rgb) into a point cloud
        pcl_cluster = pcl.PointCloud_PointXYZRGB()
        pcl_cluster.from_list(object_cluster)

        # Convert the cluster from pcl to ROS using helper function
        ros_cloud = pcl_to_ros(pcl_cluster)
        #pcl_objects_pub.publish(ros_cloud)

        # Extract histogram features (similar to capture_features.py)
        histogram_bins = 64
        chists = compute_color_histograms(ros_cloud,
                                          using_hsv=True)
        normals = get_normals(ros_cloud)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction, retrieve the label for the result and add it
        #   to detected_objects_labels list
        prediction = clf.predict(scaler.transform(feature.reshape(1, -1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label, label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_cloud
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))
    
    # Create new cloud containing all clusters, each with a unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Convert PCL data to ROS messages
    ros_cloud_object_cluster = pcl_to_ros(cluster_cloud)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cloud_table = pcl_to_ros(cloud_table)

    # Publish ROS messages of the point clouds and detected objects
    pcl_objects_cloud_pub.publish(ros_cloud_object_cluster) # solid color objects
    pcl_objects_pub.publish(ros_cloud_objects)      # original color objects
    pcl_table_pub.publish(ros_cloud_table)          # table cloud
    detected_objects_pub.publish(detected_objects)  # detected object labels


    # TODO: Create Cluster-Mask Point Cloud to visualize each cluster separately

    # TODO: Convert PCL data to ROS messages

    # TODO: Publish ROS messages

# Exercise-3 TODOs: 

    # Classify the clusters! (loop through each detected cluster one at a time)

        # Grab the points for the cluster

        # Compute the associated feature vector

        # Make the prediction

        # Publish a label into RViz

        # Add the detected object to the list of detected objects.

    # Publish the list of detected objects

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/sensor_stick/point_cloud', pc2.PointCloud2,
                               pcl_callback, queue_size=1)

    # TODO: Create Publishers
    object_markers_pub = rospy.Publisher('/object_markers', Marker,
                                         queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects',
                                           DetectedObjectsArray,
                                           queue_size=1)
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    pcl_objects_cloud_pub = rospy.Publisher('/pcl_objects_cloud', PointCloud2,
                                            queue_size=1)
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)

    # TODO: Load Model From disk
    model = pickle.load(open('model_world1.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
