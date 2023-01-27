close all
clear 
clc 

# import utilites
source "utilities/geometry_helpers_3d.m"
source "utilities/loader.m"
source "utilities/LS_helpers.m"

#load matrix data 
disp('************************** Loading the camera data ******************************');

[K, T_cam, z_near, z_far, widht, height] = load_camera_parameters();

#load trajectory data #
disp('************************** Loading the trajectory data **************************');
data_trajectory = load("data/trajectory.dat");

# PLOT the groundtruth trajectory 
TrueTrajectory=compute_odometry_trajectory(data_trajectory(:,2:4));
disp('Groundtruth trajectory');
#hold on;
#plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);
#pause(1);

# PLOT the odometry trajectory
OdomTrajectory=compute_odometry_trajectory(data_trajectory(:,5:7));
disp('Odometry trajectory');
#hold on;
#plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);
#pause(1);
disp("***************** Loading measurement and creating data structure ***************");
# Load measurement and create data structure 
[pos, odometry_pose, dict_pos_land] = load_measurements();

disp("************************** Performing triangulation *****************************");
################## TRIANGULATION ##################

Xl_initial_guess = triangulation(pos, odometry_pose, dict_pos_land, K, T_cam); # initial guess for landmark position


