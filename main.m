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
disp('************************** Loading the trajectory data ******************************');
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

# Load measurement and create data structure 
[odometry_pose, dict_pos_land] = load_measurements();


################## TRIANGULATION ##################
dict_point_initial_guess = containers.Map('KeyType','double','ValueType','any'); # store a dictionary in which the keys are the id of the landmark and the values are its coordinates in the world
X_guess = ones(1,10000)*-1;


A = -10 + rand(3,1)*20
