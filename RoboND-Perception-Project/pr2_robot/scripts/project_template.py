#!/usr/bin/env python

import pickle
import yaml

import numpy as np
import rospy
import sklearn
from sklearn.preprocessing import LabelEncoder
import tf


from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker

from pr2_robot.srv import *
from rospy_message_converter import message_converter
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

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
    with open(yaml_filename, 'a+') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

# Exercise-2 TODOs:

    # Convert ROS msg to PCL data (XYZRGB)
    cloud = ros_to_pcl(pcl_msg)
    '''
    # Statistical outlier filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    # Set the number of neighboring points to analyze for any given point
    outlier_filter.set_mean_k(20)
    outlier_filter.set_std_dev_mul_thresh(0.1)
    cloud_filtered = outlier_filter.filter()
    '''
    # Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # PassThrough Filter to crop table sides
    passthrough_y = cloud_filtered.make_passthrough_filter()
    passthrough_y.set_filter_field_name('y')
    y_axis_min = -0.4
    y_axis_max = 0.4
    passthrough_y.set_filter_limits(y_axis_min, y_axis_max)
    cloud_filtered = passthrough_y.filter()


    # PassThrough Filter to crop along z: height
    passthrough_z = cloud_filtered.make_passthrough_filter()
    passthrough_z.set_filter_field_name('z')
    z_axis_min = 0.6
    z_axis_max = 0.9
    passthrough_z.set_filter_limits(z_axis_min, z_axis_max)
    cloud_filtered = passthrough_z.filter()

    #publish to make sure
    #cloud_filtered_ros=pcl_to_ros(cloud_filtered)
    #pcl_filter_pub.publish(cloud_filtered_ros)

    # Segmenting with RANSAC a Plane model :table
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers: table and objects
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)
    pcl.save(cloud_table, 'pipeline_4_extracted_inliers.pcd')
    pcl.save(cloud_objects, 'pipeline_5_extracted_outliers.pcd')

    # Euclidean Clustering
    # Convert the xyzrgb cloud to only xyz, KDtree is only spatially dependent
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    #   PCL's euclidian clustering algorithims only takes Kdtrees
    tree = white_cloud.make_kdtree()
    # Euclidean clustering used cluster point clouds. 
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(100)
    ec.set_MaxClusterSize(2500)
    ec.set_SearchMethod(tree)       # Search the k-d tree for clusters
    # Extract indices for each found cluster. This is a list of indices for
    #   each cluster (list of lists)
    cluster_indices = ec.Extract() # a [number of clusters][clouds] 

    # Assign a color corresponding segmented objectsin the scene
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
            # Retrieve cloud values for the spacial and rgb object
            object_cluster.append([cloud_objects[pts][0],
                                   cloud_objects[pts][1],
                                   cloud_objects[pts][2],
                                   cloud_objects[pts][3]])
            
            # Retrieve cloud values for the spacial object, and coloring them
            color_cluster_point_list.append([white_cloud[pts][0],
                                             white_cloud[pts][1],
                                             white_cloud[pts][2],
                                             rgb_to_float(cluster_color[index])])


        # Convert list of point cloud features (x,y,z,rgb) into a point cloud
        pcl_cluster = cloud_objects.extract(pts_list)

        # Convert the cluster from pcl to ROS using helper function
        ros_cloud = pcl_to_ros(pcl_cluster)


        # Extract histogram features
        chists = compute_color_histograms(ros_cloud,nbins=128,using_hsv=True)
        normals = get_normals(ros_cloud)
        nhists = compute_normal_histograms(normals, nbins=128)
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
    pcl_objects_pub.publish(ros_cloud_objects)  
    pcl_table_pub.publish(ros_cloud_table)       
    detected_objects_pub.publish(detected_objects)

    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(detected_objects):
    # : Initialize variables
    test_num=3
    
    yaml_obj_list = []
    ros_scene_num = Int32()
    
    ros_object_name = String()
    ros_pick_pose = Pose()
    ros_scene_num.data = test_num
    
    # : Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_list_param = rospy.get_param('/dropbox')
    # TODO: Rotate PR2 in place to capture side tables for the collision map

    labels = []
    centroids = [] #list of tuples x, y, z
    # : Loop through the yaml_pick list
    for object_params in object_list_param:
	object_name = object_params['name']
        object_group = object_params['group']
        ros_object_name.data = object_name
        box_pos = [0, 0, 0]
        centroid = [0, 0, 0]
        
        #identify associated item in the scene  
        for object in detected_objects:
	    if object_name == object.label:
	        #obtain centroid from object's cloud	
		    points_array = ros_to_pcl(object.cloud).to_array()
		    centroid_numpy = np.mean(points_array, axis=0)[:3]
		    centroid = [np.asscalar(x) for x in centroid_numpy]
		    #print(centroid)
		    #print(type(centroid[1]))
	    	
		    #object pick pose
		    ros_pick_pose.position.x = centroid[0]
		    ros_pick_pose.position.y = centroid[1]
		    ros_pick_pose.position.z = centroid[2]
	    	
		    # 'place_pose' for the object
		    if dropbox_list_param[0]['group'] == object_group:
		        box_pos = dropbox_list_param[1]['position']
		    else:
		        box_pos=box_pos = dropbox_list_param[1]['position']
	    	
		    # Assign the dropbox (placing) pose
		    ros_place_pos = Pose()
		    ros_place_pos.position.x = box_pos[0]
		    ros_place_pos.position.y = box_pos[1]
		    ros_place_pos.position.z = box_pos[2]
	    	
		    #Assign the arm to be used for pick_place
		    ros_arm_to_use = String()
		    if object_group == 'green':
		        #green bin: right arm
		        arm_name = 'right'
		    else:
		        #red :left arm
		        arm_name = 'left'
		        
		    ros_arm_to_use.data=arm_name
		    #: Create a list of dictionaries
		    yaml_obj=make_yaml_dict(ros_scene_num, ros_arm_to_use,
					    ros_object_name, ros_pick_pose,
					    ros_place_pos)
		    yaml_obj_list.append(yaml_obj)
		    #item found, break from loop
		    break 
		        

	# Wait for 'pick_place_routine' service to come up
	rospy.wait_for_service('pick_place_routine')
	try:
	    pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
        except rospy.ServiceException, e:
            print("Service call failed: %s" % e)
            del detected_objects.object
    
    # : Output your request parameters into output yaml file
    send_to_yaml('output_%i.yaml' % test_num, yaml_obj_list)


if __name__ == '__main__':
    
    # ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # Create Subscriber to receive the published data
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2,
                               pcl_callback, queue_size=1)

    # Create Publishers
    object_markers_pub = rospy.Publisher('/object_markers', Marker,
                                         queue_size=1)
    detected_objects_pub = rospy.Publisher('/detected_objects',
                                           DetectedObjectsArray,
                                           queue_size=1)
    # Isolated object point cloud 
    pcl_objects_pub = rospy.Publisher('/pcl_objects', PointCloud2, queue_size=1)
    # Isolated object point cloud clusters
    pcl_objects_cloud_pub = rospy.Publisher('/pcl_objects_cloud', PointCloud2,
                                            queue_size=1)
    # Table point cloud 
    pcl_table_pub = rospy.Publisher('/pcl_table', PointCloud2, queue_size=1)

    # Load model from disk
    model = pickle.load(open('model_world1_final_50.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
