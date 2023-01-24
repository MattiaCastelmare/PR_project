close all
clear 
clc 

# import 2d geometry utilities
source "utilities/geometry_helpers_2d.m"

# load matrix data
disp('************************** Loading the camera data ******************************');
data_camera = dlmread("data/camera.dat");
K = data_camera(2:4,1:3);
T_cam = data_camera(6:9, 1:4);
z_near = data_camera(10,2);
z_far = data_camera(11,2);
widht = data_camera(12,2);
height = data_camera(13,2);

# load trajectory data
disp('************************** Loading the trajectory data ******************************');
data_trajectory = load("data/trajectory.dat");

#h = figure(1);
more off;

# plot the groundtruth trajectory
TrueTrajectory=compute_odometry_trajectory(data_trajectory(:,2:4));
disp('Groundtruth trajectory');
hold on;
plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);
pause(1);

# plot the odometry trajectory
OdomTrajectory=compute_odometry_trajectory(data_trajectory(:,5:7));
disp('Odometry trajectory');
hold on;
plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);
pause(5);


# load measurement file
disp('************************** Loading the measurement data ******************************');

measurements = [];
landmark_id = [];
image_x = [];
image_y = [];

for i = 0 : 199
    if i < 10
    num = num2str(i);
    filename = strcat("data/meas-0000",num,".dat");
    elseif i > 9 && i < 100
    num = num2str(i);
    filename = strcat("data/meas-000",num,".dat");
    else
    num = num2str(i);
    filename = strcat("data/meas-00",num,".dat");    
    end

    [a, b, c, d, f] = textread(filename);
    
    seq(i+1,1) = b(1) + 1;

    groundtruth_pose((i+1)*3-2:(i+1)*3,1) = [d(1), f(1), a(2)];

    odometry_pose((i+1)*3-2:(i+1)*3,1) = [c(2), d(2), f(2)];

    measurements = vertcat(measurements,b(3:end-1));

    landmark_id = vertcat(landmark_id,c(3:end-1));

    image_x = vertcat(image_x,d(3:end-1));
    image_y = vertcat(image_y,f(3:end-1));
end

