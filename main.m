close all
clear 
clc 

######### import 2d geometry utilities
source "utilities/geometry_helpers_3d.m"

######### load matrix data
disp('************************** Loading the camera data ******************************');
data_camera = dlmread("data/camera.dat");

K = data_camera(2:4,1:3);
T_cam = data_camera(6:9, 1:4);
z_near = data_camera(10,2);
z_far = data_camera(11,2);
widht = data_camera(12,2);
height = data_camera(13,2);

######### load trajectory data
disp('************************** Loading the trajectory data ******************************');
data_trajectory = load("data/trajectory.dat");

################## DISPLAY the GROUNDTRUTH TRAJECTORY and the ODOMETRY ONE ##############
#h = figure(1);
#more off;

######### plot the groundtruth trajectory
TrueTrajectory=compute_odometry_trajectory(data_trajectory(:,2:4));
disp('Groundtruth trajectory');
#hold on;
#plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);
#pause(1);

######### plot the odometry trajectory
OdomTrajectory=compute_odometry_trajectory(data_trajectory(:,5:7));
disp('Odometry trajectory');
#hold on;
#plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);
#pause(1);

#################### Create data structure ################

dict = containers.Map('KeyType','double','ValueType','any'); # dict in which KEYS = number of robot poses VALUES = landmark dictionary

X_initial = ones(3, 1000);
hom = ones(1,1000);
P_initialguess = [X_initial; hom];
threshold = 1e-6;

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
    
    pos(i+1,1) = b(1) + 1; # column array in wich I store the number of all the poses

    groundtruth_pose(:,i+1) = [d(1), f(1), a(2)]; # matrix in which each column is the gt pose and the column is the number of pose

    odometry_pose(:,i+1) = [c(2), d(2), f(2)]; # matrix in which each column is the odometry pose and the column is the number of pose

    measurement_number = b(3:end-1); 
    landmark_id = c(3:end-1); # list of landmark id
    image_x = d(3:end-1); # list of image coordinates
    image_y = f(3:end-1);

    pair = [image_x, image_y];
    values = {};
    for j = 1:rows(pair)
        values(j) = pair(j,:)';
    end
    land_dict = containers.Map(landmark_id, values);  # dict in which KEYS = landmark id VALUES = x and y in the images
    
    dict(pos(i+1)) = land_dict;
    

end

# Second way to build the dictionary but it take longer time 
#for j = 1:size(landmark_id)
#    land_dict(landmark_id(j)) = [image_x(j), image_y(j)]';  # dict in which KEYS = number of robot poses VALUES = landmark dictionary
#end
#dict(pos(i+1)) = land_dict;