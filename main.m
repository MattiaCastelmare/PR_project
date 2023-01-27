close all
clear 
clc 

# import utilites
source "utilities/geometry_helpers_3d.m"
source "utilities/loader.m"
source "utilities/LS_helpers.m"


#load landmark data
A = load("data/world.dat");
landmark_id = A(:,1) + 1;
for i = 1:size(landmark_id)
    land_gt(:,i) = [A(i,2); A(i,3); A(i,4)];
end

#load matrix data 
disp('************************** Loading the camera data ******************************');

[K, T_cam, z_near, z_far, widht, height] = load_camera_parameters();

#load trajectory data #
disp('************************** Loading the trajectory data **************************');
data_trajectory = load("data/trajectory.dat");

# PLOT the groundtruth trajectory 
TrueTrajectory=compute_odometry_trajectory(data_trajectory(:,2:4));
disp('*************************** Groundtruth trajectory ******************************');
#hold on;
#plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);
#title({'Red - Groundtruth'; 'Green - Odometry'});
#pause(1);

# PLOT the odometry trajectory
OdomTrajectory=compute_odometry_trajectory(data_trajectory(:,5:7));
disp('*************************** Odometry trajectory *********************************');
#hold on; 
#plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);

#pause(5);



disp("***************** Loading measurement and creating data structure ***************");
# Load measurement and create data structure 
[pos, odometry_pose, dict_pos_land] = load_measurements();





disp("************************** Performing triangulation *****************************");
################## TRIANGULATION ##################

Xl_initial_guess = triangulation(pos, odometry_pose, dict_pos_land, K, T_cam); # initial guess for landmarks position

hold on;
S = 40;
scatter3(land_gt(1,:),land_gt(2,:), land_gt(3,:), S,[1,0,0])
scatter3(Xl_initial_guess(1,:),Xl_initial_guess(2,:), Xl_initial_guess(3,:), S,[0,1,0])
view(40,40)
title({"Red - Groundtruh"; "Green - Inital guess"})
pause(10)

