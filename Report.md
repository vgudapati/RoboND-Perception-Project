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
[img_world3_pcl_objects_cloud]: perception-images/world3_pcl_objects_cloud.png



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
![IMG3][img_world2_camera_glue]
![IMG4][img_world3_camera]

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

### Filtering <a id='point_cloud_creation'></a>

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
![IMG6][img_world2_pcl_objects]
![IMG7][img_world3_pcl_objects]

### Table Segmentation <a id='table_segmentation'></a>


