#!/usr/bin/env python

# Import modules
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

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml


# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

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
    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    '''
    # TODO: Initialize variables

    # TODO: Get/Read parameters

    # TODO: Parse parameters into individual variables

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list

        # TODO: Get the PointCloud for a given object and obtain it's centroid

        # TODO: Create 'place_pose' for the object

        # TODO: Assign the arm to be used for pick_place

        # TODO: Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(TEST_SCENE_NUM, OBJECT_NAME, WHICH_ARM, PICK_POSE, PLACE_POSE)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    # TODO: Output your request parameters into output yaml file
    '''
    # Initialize output list that'll store multiple object yaml dictionaries
    output_list = []

    # Load the parameters from the YAML files located in /pr2_robot/config/
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')

    # TODO: Get this PR2 rotation working
    ## Rotate PR2 in place to capture side tables for the collision map. The
    ##   world_joint_controller controls the revolute joing world_joint between
    ##   the robot's base_footprint and world coordinate frames
    #pub_j1 = rospy.Publisher('/pr2/world_joint_controller/command',
    #                         Float64, queue_size=10)
    #pub_j1.publish(np.pi/2)     # Rotate the PR2 counter-clockwise 90 deg
    #pub_j1.publish(-np.pi)      # Rotate the PR2 clockwise 180 deg
    #pub_j1.publish(np.pi/2)     # Rotate the PR2 back to neutral position
    test_num = 2
    # Iterate through all objects that should be moved
    for object_params in object_list_param:
        object_name = object_params['name']
        object_group = object_params['group']

        # Check if the object to be moved was found in the perception analysis,
        #   populating the pick_pose message if it was
        for object_i, object_val in enumerate(object_list):
            if object_name != object_val.label:
                # Skip this object b/c it doesn't match the object to be moved
                continue

            # Assign the scene number
            ros_scene_num = Int32()
            # TODO: Figure out what parameter holds the scene data
            #ros_scene_num.data = rospy.get_param('/world_name')
            #test_num = 2
            ros_scene_num.data = test_num

            # Assign the object name
            ros_object_name = String()
            ros_object_name.data = object_name

            # Assign the arm that'll be used to pickup the object
            ros_arm_to_use = String()
            if object_group == 'green':
                # The green bin is on the robot's right
                ros_arm_to_use.data = 'right'
            else:
                # The red bin is on the robot's left
                ros_arm_to_use.data = 'left'
            
            # Get the PointCloud for the object and obtain it's centroid
            #   (the average position of all points in the object cloud).
            #   Convert the cloud to an array, then calculate the average
            #   of the array.
            points_array = ros_to_pcl(object_val.cloud).to_array()
            centroid_numpy = np.mean(points_array, axis=0)[:3]
            # Convert the numpy float64 to native python floats
            centroid = [np.asscalar(x) for x in centroid_numpy]

            # Assign the object pick pose
            ros_pick_pose = Pose()
            ros_pick_pose.position.x = centroid[0]
            ros_pick_pose.position.y = centroid[1]
            ros_pick_pose.position.z = centroid[2]

            # Find the correct dropbox's position
            box_pos = [0, 0, 0]     # Set a default box position
            for box_params in dropbox_list_param:
                if box_params['group'] == object_group:
                    box_pos = box_params['position']
                    break
            # TODO: Add a random offset to the dropbox's position

            # Assign the dropbox pose
            ros_place_pos = Pose()
            ros_place_pos.position.x = box_pos[0]
            ros_place_pos.position.y = box_pos[1]
            ros_place_pos.position.z = box_pos[2]

            # Add the object's yaml dict to the output_list
            obj_yaml_dict = make_yaml_dict(ros_scene_num, ros_arm_to_use,
                                           ros_object_name, ros_pick_pose,
                                           ros_place_pos)
            output_list.append(obj_yaml_dict)
            print('processed %s' % ros_object_name.data)

            ## Wait for 'pick_place_routine' service to come up
            #rospy.wait_for_service('pick_place_routine')
            ## Uses /srv/PickPlace.srv file
            #pick_place_routine = rospy.ServiceProxy('pick_place_routine',
            #                                        PickPlace)
            #try:
            #    # Insert the message variables to be sent as a service request
            #    resp = pick_place_routine(ros_scene_num, ros_object_name,
            #                              ros_arm_to_use, ros_pick_pose,
            #                              ros_place_pos)

            #    print("Response: ", resp.success)
            #except rospy.ServiceException, e:
            #    print("Service call failed: %s" % e)

            # Remove the object from object_list to indicate it was picked up
            del object_list[object_i]
            
            # Stop looking through the other identified objects
            break

    # Output your request parameters into an output yaml file
    send_to_yaml('output_%i.yaml' % test_num, output_list)

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # TODO: Create Subscribers
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2,
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
