close all
clear 
clc 

# import utilites
source "utilities/helpers.m"
source "utilities/loader.m"
source "utilities/pose_landmark.m"
source "utilities/pose_pose.m"
source "utilities/Total_least_square.m"

#load landmark data
A = load("data/world.dat");
landmark_id = A(:,1) + 1;
for i = 1:size(landmark_id)
    land_gt(:,i) = [A(i,2); A(i,3); A(i,4)];
endfor

#load matrix data 
disp('************************** Loading the camera data ******************************');

[K, T_cam, z_near, z_far, img_width, img_height] = load_camera_parameters();

#load trajectory data #
disp('************************** Loading the trajectory data **************************');
data_trajectory = load("data/trajectory.dat");

#h = figure(1);

# PLOT the groundtruth trajectory 
TrueTrajectory=compute_odometry_trajectory(data_trajectory(:,5:7));

pose_gt_plot = data_trajectory(:,5:7);
pose_gt = pose_gt_plot';
pose_gt = [pose_gt(1,:); pose_gt(2,:); zeros(1,size(pose_gt)(2)); zeros(1,size(pose_gt)(2)); zeros(1,size(pose_gt)(2)); pose_gt(3,:)];

% disp('*************************** Groundtruth trajectory ******************************');
% hold on;
% plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);
% title({'Red - Groundtruth'; 'Green - Odometry'});
% pause(1);


% # PLOT the odometry trajectory
% OdomTrajectory=compute_odometry_trajectory(data_trajectory(:,2:4));
% disp('*************************** Odometry trajectory *********************************');
% hold on; 
% plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);
% pause(5);
% hold off;


disp("***************** Loading measurement and creating data structure ***************");
# Load measurement and create data structure 
[pos, odometry_pose, dict_pos_land] = load_measurements();


# Create a tensor in which the third dimension is the number of measure while the matrix is the relative position of the pos i^th and j^th

robot_measurement = odometry_measure(odometry_pose);


disp("************************** Performing triangulation *****************************");
% ################## TRIANGULATION ##################

Xl_initial_guess = triangulation(K, 
                                T_cam, 
                                pos, 
                                odometry_pose, 
                                dict_pos_land); # initial guess for landmarks position

g = figure(1);
hold on;

scatter3(land_gt(1,:),land_gt(2,:), land_gt(3,:), 40,[1,0,0])
hold on;
scatter3(Xl_initial_guess(1,:),Xl_initial_guess(2,:), Xl_initial_guess(3,:), 40,[0,1,0])
view(40,40)
title({"Red - Groundtruh"; "Green - Inital guess"})
pause(5)
hold off;

################ Mean error after triangulation ############

% Xl_initial_guess1 = Xl_initial_guess(1:end-1,:);
% error = 0;
% max_error = 0;
% id = 0;
% for i=1: size(Xl_initial_guess1)(2)
%     error = error + abs(Xl_initial_guess1(:,i) - land_gt(:,i));
%     if abs(Xl_initial_guess1(:,i) - land_gt(:,i)) > max_error
%         max_error = abs(Xl_initial_guess1(:,i) - land_gt(:,i));
%         id = i;
%     endif
% endfor
% mean_error = error/size(land_gt)(2);
% max_error;

#############################################################
########## Useful variables ################################# 
global pos_dim = 3;
global landmark_dim = 3;
global Rz0;
global damping = 100;
global threshold_proj = 5000;
global threshold_pose = 0.01;
global num_iterations = 15;
global num_poses = size(odometry_pose)(2);
global num_landmarks = size(Xl_initial_guess)(2);
#############################################################

Xr = odometry_pose;
Xl = Xl_initial_guess;

#Xr = pose_gt; ## GROUNDTRUTH!!!!!!
#Xl = [land_gt; ones(1,size(land_gt)(2))]; # # GROUNDTRUTH!!!!!!

disp("************************ Performing Total Least Square **************************");

[Xl, Xr] = DoTLS(Xr,
                Xl,
                robot_measurement, 
                pos_dim, 
                num_poses, 
                num_landmarks, 
                landmark_dim, 
                K, 
                T_cam, 
                dict_pos_land, 
                z_near, 
                z_far, 
                img_width, 
                img_height, 
                threshold_pose,
                threshold_proj,
                damping,
                num_iterations);

Xr = [Xr(1,:); Xr(2,:); Xr(6,:)]'; # I am interested only in x y and theta (the others are 0)
Xl = Xl(1:end-1,:); # from homogenous to x y and z
OdomTrajectory=compute_odometry_trajectory(Xr);
Groundtruh_traj=compute_odometry_trajectory(pose_gt_plot);
hold on; 
l = figure(2);
plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 4);
hold on;
plot(Groundtruh_traj(:,1),Groundtruh_traj(:,2), 'r-', 'linewidth', 2);
title({'Red - Groundtruth'; 'Green - Least square solution'});
pause(5);
hold off;




m = figure(3);
hold on;
scatter3(Xl(1,:),Xl(2,:), Xl(3,:), 60,[0,1,0]);
hold on;
scatter3(land_gt(1,:),land_gt(2,:), land_gt(3,:), 40,[1,0,0]);
view(40,40);
title({'Red - Groundtruth'; 'Green - Least square solution'});

pause(100);
hold off;
