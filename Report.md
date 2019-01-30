# RoboND 3D Perception Project


[//]: # (Image References)

[img_world1_camera]: perception-images/world1_camera.png
[img_world1_points]: perception-images/world1_points.png
[img_world1_pcl_table]: perception-images/world1_pcl_table.png
[img_world1_pcl_objects]: perception-images/world1_pcl_objects.png
[img_world1_pcl_objects_cloud]: perception-images/world1_pcl_objects_cloud.png

[img_world2_camera_glue]: perception-images/world2_camera_glue.png
[img_world2_points_glue]: perception-images/world2_points_glue.png
[img_world2_pcl_table_glue]: perception-images/world2_pcl_table_glue.png
[img_world2_pcl_objects]: perception-images/world2_pcl_objects.png
[img_world2_pcl_objects_cloud_glue]: perception-images/world2_pcl_objects_cloud_glue.png

[img_world2_camera_no_glue]: perception-images/world2_camera_no_glue.png
[img_world2_points_no_glue]: perception-images/world2_points_no_glue.png
[img_world2_pcl_table_no_glue]: perception-images/world2_pcl_table_no_glue.png
[img_world2_pcl_objects_no_glue]: perception-images/world2_pcl_objects_no_glue.png
[img_world2_pcl_objects_cloud_no_glue]: perception-images/world2_pcl_objects_cloud_no_glue.png

[img_world3_camera]: perception-images/world3_camera.png
[img_world3_points]: perception-images/world3_points.png
[img_world3_pcl_table]: perception-images/world3_pcl_table.png
[img_world3_pcl_objects]: perception-images/world3_pcl_objects.png
[img_world3_pcl_objects_cloud]: perception-images/wordl3_pcl_pbjects_cloud.png

[cmwn]: perception-images/confusion_matrix_with_normalization.png
[cmwon]: perception-images/confusion_matrix_without_normalization.png




## Introduction

The objective of this project is to implement a perception pipeline that identifies target objects from the respecive 
enrironments (called worlds). The data for the identification is coming from the PR2 robot's RGB-D camera. 

The techniques (filtering, segmentation, clustering and object detection) learned in lessons will be applied for implementing 
the perception pipeline. 

![IMG1][img_world1_camera]

##

The steps followed to implement the project are as follows.

  - [Perception pipeline](#perception_pipeline)
    - [Access the data from cameras](#access_data)
    - [Point cloud creation](#point_cloud_creation)
    - [Filtering](#filtering)
    - [Table segmentation](#table_segmentation)
    - [Clustering](#clustering)
    - [Object detection](#object_detection)
      -[Capture Features](#capture_features)
      -[Train the model](#train_model)
    - [Output to .yaml files](#yaml_files)

##

## Access the data from the camera <a id='access_data'></a>

The data is accessed from the cameras mounted on the PR2 robot. To do so, we need to create a ROS node 'recognition' and 
subscribed to the /pr2/world/points topic. The following is the code.

    # ROS node initialization
    rospy.init_node('recognition', anonymous=True)

    # Create Subscriber to receive the published data
    pcl_sub = rospy.Subscriber('/pr2/world/points', pc2.PointCloud2, pcl_callback, queue_size=1)
    
The following is a sample of the image coming from camera in each of the worlds. 

![IMG2][img_world1_camera]


## Perception pipeline <a id='perception_pipeline'></a>

### Point cloud creation <a id='point_cloud_creation'></a>

The data coming from the cameras have a lot of information and that slows the processing. In order for the application to 
perform better we need to downsample the images. One technique to do so is to create point clouds. Depending on the 
ganularity of LEAF, we can really reduce the information we can use. So we need to select an optimal leaf size to create the 
point clouds. Based on the experience in the lessons a leaf size of 0.01 seems to reduce the size significantly.

The following is the code to create a point cloud and down sample the images coming from the camera.

    # Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.01
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

### Filtering <a id='filtering'></a>

To improve the quality of the point cloud data, we apply some filters. I used passthrough filers. The code to do so is;

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
    
After creating and filtering the following are the how the objects looks like.

![IMG5][img_world1_pcl_objects]


### Table Segmentation <a id='table_segmentation'></a>

I used RASNAC plane segmentation to separate out each of the objects from one another. The following is the code doing it. 

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
    
After segmentation the table and the objects are separated. The following are the separated images for table. 

![IMG8][img_world1_pcl_table]


### Clustering <a id='clustering'></a>

Finally used Euclidean Clustering to distinguish the objects from one another. The following is the code used.

    # Euclidean Clustering
    # Convert the xyzrgb cloud to only xyz, KDtree is only spatially dependent
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    #   PCL's euclidian clustering algorithims only takes Kdtrees
    tree = white_cloud.make_kdtree()
    # Euclidean clustering used cluster point clouds. 
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.02)
    ec.set_MinClusterSize(40)
    ec.set_MaxClusterSize(900)
    ec.set_SearchMethod(tree)       # Search the k-d tree for clusters
    # Extract indices for each found cluster. This is a list of indices for
    #   each cluster (list of lists)
    cluster_indices = ec.Extract() # a [number of clusters][clouds] 
    
The images segmented and clustered are shown below. These correspond to the outliers in the segmentation step.

![IMG11][img_world1_pcl_objects_cloud]


### Object Detection <a id='object_detection'></a>

Object detection involves two steps. 

- Capture the features
- Training the model

#### Capture the features <a id='capture_features'></a>

During the excercises, we directly ran the following commands to generate the features.

First, launch the training simulator with:
    $ roslaunch sensor_stick training.launch
Then to capture features:
    $ rosrun sensor_stick pr2_capture_features_world1.py

The code that extracts the features come from the capture_features.py where we will use compute_color_histograms and compute_normal_histograms functions

    # Extract histogram features
    chists = compute_color_histograms(sample_cloud, using_hsv=True)
    normals = get_normals(sample_cloud)
    nhists = compute_normal_histograms(normals)
    feature = np.concatenate((chists, nhists))
    labeled_features.append([feature, model_name])

But during the actual project, we needed a master list of all the objects. So, by looking into pick_list yaml files under /pr2_robot/config/ i made the master list. Then launched the above commands. Also, i tried with various(random) positions. When the number is low, the accuracy sufferred and when the number is high, the accuracy improved. I trained with 50 parameters finally to achieve 86.88 accuracy score. I tried with 20/30/50 and as the number went up the accuracy improved.

#### Train the model <a id='train_model'></a>

Training the model is done using the Support Vector Machines(SVM) model carrying on the work from the lessons. I played around with all the hyper parameters in SVC. But eventually reverted back to the simple linear kernel classifier. I didn't see much improvement when i tried either poly or rbf kernels in this particular case. 

For training i used the script pr2_train_svm_world1.py from sensorstick/scripts.

As mentioned in the lessons, I increased the number of poses for objects to 50 currently (though i tried with different numbers and other hyper parameters), set using_hsv to True. Even the kernel i tried rbf and poly but linear was better in the test i did. 

The following is the output from training the network.

    robond@udacity:~/catkin_ws$ rosrun sensor_stick pr2_train_svm_world1.py
    Features in Training Set: 450
    Invalid Features in Training set: 0
    SVC(C=1.0, cache_size=200, class_weight=None, coef0=0.0,
      decision_function_shape='ovr', degree=3, gamma='auto_deprecated',
      kernel='linear', max_iter=-1, probability=False, random_state=None,
      shrinking=True, tol=0.001, verbose=False)
    Scores: [ 0.81111111  0.9         0.86666667  0.87777778  0.88888889]
    Accuracy: 0.87 (+/- 0.06)
    accuracy score: 0.868888888889
    robond@udacity:~/catkin_ws$ 


The following are the confusion matrices with and without normalization

![IMG17][cmwn]


![IMG18][cmwon]





### Output to yaml files <a id='output_to_yaml'></a>

The last step is to generate the yaml files, which are used by the robot to choose the target to pick and place, should we choose to implement the challenge. But i have not chosen to do the challenge at this point in time but to generate the yaml files as it is the requirement for the project

In order to generate the yaml files, we do the following.

  - Read the paramters
  - Look through the yaml pick list and check if the detected object is in the pick list. 
  - If so, calculate the centroid of each object.
  
Then we create a yaml_dict using the function make_yaml_dict and then write to a file using send_to_yaml function.

    def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict
    
    def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'a+') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)


### Testing the Project <a id='test_project'></a>

To test with the project, first run:

$ roslaunch pr2_robot pick_place_project.launch
and then,

$ rosrun pr2_robot project_template.py

## Observations and Results <a id='observations_results'></a>

The recognition for all three tasks is done successfully.

The following is the percentage result.

    World1: 3/3 (100%)
    world2: 4/5(80%)
    world3: 6/8(75%)

The output_(1/2/3).yamml files are present in the main repository



As mentioned earlier, for capturing the features and training I have used  pr2_train_svm_world1.py and pr2_capture_features_world1.py from sensorstick/scripts from the repo. 

All the images displayed above at the beginning are captured during testing so the tags are diaplayed as well.

What did work at times is that the model found it diffcult to recognize some objects always missing on world2 and world3. This definitely needs improvement and ill work on them. 

Here are the predictions

![IMG21][img_world1_camera]
![IMG22][img_world2_camera]
![IMG23][img_world3_camera]

# Future work

There are many ways this project can be improved. Some of the obvious ways i can take this forward are:
  - Take up the challenge part of the project
  - Experiment with other models or algorithms such as DBSCAN
  - Debug and finetune the model to make sure it always performs 100% and als research why glue is not always identified as glue.

