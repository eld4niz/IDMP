# IDMP - Interactive Distance Field Mapping and Planning to Enable Human-Robot Collaboration

This is the code for IDMP, implementing a gaussian process based distance and gradient field algorithm.

 * [Project Page](https://uts-ri.github.io/IDMP/)
 * [Paper](https://arxiv.org/pdf/2403.09988v3)
 * [Video](https://www.youtube.com/watch?v=NpbDjCqXyrs)
 
This codebase is implemented using ROS in both C++ and Python.

## Description

We present an interactive distance field mapping and planning (IDMP) framework that handles dynamic objects and collision avoidance through an efficient representation. Given depth sensor data, our framework builds a continuous field that allows to query the distance and gradient to the closest obstacle at any required position. The key aspect of this work is an efficient Gaussian Process field that performs incremental updates and implicitly handles dynamic objects with a simple and elegant formulation based on a temporary latent model. 

## Dependencies

- Python3
- C++14
- ROS Noetic
- Eigen
- OpenMP (for multithreading. optional but strongly recommended)
- sensor-filters (`sudo apt install ros-noetic-sensor-filters`)
- point-cloud2-filters (`sudo apt install ros-noetic-point-cloud2-filters`)
- robot-body-filter (optional for removing the robot from the pointcloud.) ()`sudo apt install ros-noetic-point-cloud2-filters`)
- Camera specific ros driver
    - [Azure kinect](https://github.com/microsoft/Azure_Kinect_ROS_Driver)
    - [Intel Realsense](https://github.com/IntelRealSense/realsense-ros)
    - [ZED 2i](https://github.com/stereolabs/zed-ros-wrapper)
- For Python tools:
    - numpy
    - OpenCV 4.9.0
    - Open3D (for evaluation only)

## Installation

IDMP is provided as catkin package

- clone the package into the src folder of your workspace
- run a `catkin build`
- resource your workspace

## Running IDMP
### Launch Files

We provide a few launch files to get started:

- realsense.launch: Launches the realsense driver, publishes the camera pose as TF and runs IDMP
- zed.launch: Launches the zed driver, publishes the camera pose as TF and runs IDMP
- kinect_UR5e.launch: Launches the Azure Kinect driver, an inoffical UR5e Ros driver, a filter node that filters the robot points and IDMP. Note that we use our own custom Universal Robots Ros Driver and not the official one. 

You can run these with `roslaunch idmp_ros {name}.launch`.

The camera pose has to be published as a TF transform. This is already done in the launch files. To change the pose, adjust the corresponding parameters in the static transform publisher args

### Running IDMP standalone

If you do not want to use a launch file, you can also run the IDMP node standalone

`rosrun idmp_ros idmp`

Keep in mind that it needs the parameters of the config file in the parameter server at launch. You can lode these with `rosparam load {file}.yaml`.

These config also contains the topics of the input pointcloud, the camera intrinsics and the reference frame. Make sure to adjust these to your setup.

### Running Robot40 with CHOMP Planner using IDMP Distances and Gradients

First you have to run Gazebo model of robot40 with following command:

`roslaunch robot40_gazebo robot40rightUR5_simulation.launch withGazeboGUI:=true world:=office`

Then to change camera position to look down, you can run this command:

`rostopic pub /controller_ur/order std_msgs/String "data: 'moveLookingDown'"`

**Please note that you should run all these commands in docker environment of robot40 with ros noetic**

After running these commands, we're done with setup of robot40, now we should move to the setup of IDMP.

To run IDMP with robot40, you can run following command:
`roslaunch idmp_ros robot40-realsense.launch`

This command will run IDMP with robot40 and realsense camera.

Then we should run CHOMP planner to plan the path for robot40. To run CHOMP planner, you can run following command:

`python3 src/tools/ChompIDMPlanner.py`

### Running Benchmark Datasets

The Datasets can be found here: 
- [Cow and Lady](https://projects.asl.ethz.ch/datasets/doku.php?id=iros2017)
- [Dynamic Ball](https://drive.google.com/file/d/1pLu1bw5k_bogKoMEMdHErIggIJFj3JQt/view)

Running the dataset with IDMP:
- Launch the corresponding launch file (run_cow_and_lady.launch or run_dynamic_ball.launch)-
- Play the rosbag
- To visualize the distance field and gradients, run the query tool `rosrun idmp_ros queryTool.py`

### Using multiple cameras

Idmp supports the use of multiple cameras to capture a bigger scene. For this, each camera has to publish its pose as TF and its intrinsic parameters as CameraInfo message. The pointclouds of the cameras need to be merged into one pointcloud (preferably in the base frame), before being fed into IDMP.

- Merge pointcloud of all cameras. (For example using [something like this](https://github.com/aseligmann/pointcloud_concatenate))
- set the merged pointcloud topic as input for IDMP
- for each camera, add the cameraInfo topic and the TF frame name to the list in idmp_caminfo_topic.
- idmp_caminfo_topic for two cameras might look like this: 
```
idmp_caminfo_topic: 
    - ["/camera_1/depth/camera_info", "camera_1_depth_optical_frame"]
    - ["/camera_2/depth/camera_info", "camera_2_depth_optical_frame"]
```

### Tools

We provide a set of tools to interact with IDMP for testing and demonstration purposes. You can run these with `rosrun idmp_ros {tool}.py

- calibPose: This can be used to calibrate the camera pose using an aruco marker. Adjust the topics and marker parameters for your setting and run the script. Make sure the determined pose is aligned properly and copy the output into your launchfile to replace the transform.
- queryTool: This creates an interactive marker and queries a distance field slice. It is then published and can be visualized in RVIZ. You can adjust the slice properties and make it move by setting the corresponding variables in the file.
- cowAndLadyInfoPub: This publishes the pose topic found in the cow and lady dataset as TF and the cameraInfo for the kinect
- evalTool: This is similar to the queryTool but it can load a groundtruth pointcloud and calculate real time metrics like RMSE
- speedMove: This is a basic implementation of obstacle avoidance with repulsive vectors by querying the distance field. This needs a robot driver to run and sends velocity commands to control the robot.
- The eval folder contains some tools used to evaluate our results for the publication


### Parameters

All IDMP parameters can be found as yaml files in the config folder. These need to be loaded into the parameter server before launching IDMP.

|Parameter|Description|Recommended Value|
|---|---|---|
|idmp_rleng|Cluster overlap factor for GP training|1.8|
|idmp_tree_hl_min|Octree half-length min|0.025|
|idmp_tree_hl_max|Octree half-length max|3.2|
|idmp_tree_hl_clust|Octree half-length of cluster|0.05|
|idmp_tree_hl_init|Initial octree half-length|1.6|
|idmp_map_scale|Kernel map scale|1024|
|idmp_filt_outl|Enables outlier filtering for input|True|
|idmp_pub_pcl|Publish the pointcloud of the octree for visualization|True|
|idmp_world_frame|Reference frame (needs to be published as TF)|"base_link"|
|idmp_pcl_topic|Topic of input pointcloud|"/points_filt"|
|idmp_caminfo_topic|List of cameraInfo and tf pose pairs for every sensor|" - ["/depth/camera_info", "depth_camera_link"]"|
|idmp_dynamic|Enables dynamic object mode|True|
|idmp_fusion|Enables local fusion|False|
|idmp_dyn_tresh|Dynamic movement treshold|0.08|
|idmp_fus_min|Fusion minimum treshold|0.0005|
|idmp_fus_max|Fusion maximum treshold|0.01|





















