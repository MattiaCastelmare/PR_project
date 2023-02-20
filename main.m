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
    Xl_true(:,i) = [A(i,2); A(i,3); A(i,4)];
endfor

#load matrix data 
disp('************************** CAMERA DATA ******************************************');

[K, T_cam, z_near, z_far, img_width, img_height] = load_camera_parameters();

#load trajectory data #
disp('************************** TRAJECTORY DATA **************************************');
data_trajectory = load("data/trajectory.dat");

Xr_true_plot = data_trajectory(:,5:7);
Xr_t = Xr_true_plot';
Xr_true = [Xr_t(1,:); Xr_t(2,:); zeros(1,size(Xr_t)(2)); zeros(1,size(Xr_t)(2)); zeros(1,size(Xr_t)(2)); Xr_t(3,:)];


disp("************************** ROBOT MEASUREMENT ************************************");

[pos, odometry_pose, dict_pos_land] = load_measurements();

# Create a tensor in which the third dimension is the number of measure while the matrix is the relative position of the pos i^th and j^th
robot_measurement = odometry_measure(odometry_pose); #disp("Xr true in robot measurement");

disp("************************** TRIANGULATION ****************************************");


Xl_initial_guess = triangulation(K, 
                                T_cam, 
                                pos, 
                                odometry_pose, 
                                dict_pos_land);  #disp("Xr true in triangulation");

#Xl_initial_guess = randn(4,1000); disp("Wrong landmark initial guess");



################ RMSE AFTER TRIANGULATION #######################

Xl_initial_guess1 = Xl_initial_guess(1:end-1,:);

error = 0;
max_error = 0;
id = 0;
for i=1: size(Xl_initial_guess1)(2)
    if Xl_initial_guess1(:,i) == zeros(3,1) # unseen landmark put it to 0 as default
        continue
    endif
    ex = Xl_initial_guess1(1,i) - Xl_true(1,i);
    ey = Xl_initial_guess1(2,i) - Xl_true(2,i);
    ez = Xl_initial_guess1(3,i) - Xl_true(3,i);

    error+= sqrt(ex**2+ey**2+ez**2);
    if sqrt(ex**2+ey**2+ez**2) > max_error
        max_error = sqrt(ex**2+ey**2+ez**2);
        id = i;
    endif
endfor

MSE_initial = error/size(Xl_true)(2);
A = ["RMSE initial guess ", num2str(MSE_initial)];
B = ["Max RMSE ", num2str(max_error)];
disp(A); disp(B);

#############################################################

########## USEFUL VARIABLES ################################# 

global pos_dim = 3;
global landmark_dim = 3;
global damping = 1; # CHANGE
global threshold_proj = 20000; # CHANGE
global threshold_pose = 1; # CHANGE
global num_iterations = 15;
global num_poses = size(odometry_pose)(2);
global num_landmarks = size(Xl_initial_guess)(2);

#############################################################

Xr = odometry_pose;
Xl = Xl_initial_guess;


#Xr = Xr_true; disp("GT pose initial guess");
#Xl = [Xl_true; ones(1,size(Xl_true)(2))]; disp("GT landmark initial guess");

disp("************************** TOTAL LEAST SQUARE ***********************************");

[XL, XR, chi_stats_l, num_inliers_l, chi_stats_r, num_inliers_r] = DoTLS(
                Xr,
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

######################## PLOT ##################################
Xr_sol = [XR(1,:); XR(2,:); XR(6,:)]'; # 2D PLANAR MOTION only in x y and theta (the others are 0)
Xl_sol = XL(1:end-1,:); # coordinates in format (x y z)

TrueTrajectory=compute_odometry_trajectory(data_trajectory(:,5:7));
OdomTrajectory=compute_odometry_trajectory(data_trajectory(:,2:4));
LSTrajectory=compute_odometry_trajectory(Xr_sol);

figure(1);
hold on;
grid; 

subplot(2,2,1);
title("Odometry poses and Groundtruth");
plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 4);
hold on;
plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'b-', 'linewidth', 2);
legend("Groundtruth", "Odometry poses");grid;

subplot(2,2,2);
title("Poses after optimization and Groundtruth");
plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 4);
hold on;
plot(LSTrajectory(:,1),LSTrajectory(:,2), 'b-', 'linewidth', 2);
legend("Groundtruth", "LS solution for poses");grid;


subplot(2,2,3);
title("Poses after optimization and Groundtruth");
plot3(Xl_true(1,:),Xl_true(2,:),Xl_true(3,:),'ro',"linewidth",3);
hold on;
plot3(Xl_initial_guess(1,:),Xl_initial_guess(2,:),Xl_initial_guess(3,:),'b*',"linewidth",2);
legend("Groundtruth", "Landmarks initial guess");grid;

subplot(2,2,4);
title("Landmark after optimization and Groundtruth");
plot3(Xl_true(1,:),Xl_true(2,:),Xl_true(3,:),'ro',"linewidth",3);
hold on;
plot3(Xl_sol(1,:),Xl_sol(2,:), Xl_sol(3,:),'b*','linewidth',2);
legend("Groundtruth", "LS solution landmarks");grid;


figure(2);
hold on;
grid;
title("chi evolution");

subplot(3,2,1);
plot(chi_stats_r, 'r-', "linewidth", 2);
legend("Chi Poses"); grid; xlabel("iterations");
subplot(3,2,2);
plot(num_inliers_r, 'b-', "linewidth", 2);
legend("# Pose inliers"); grid; xlabel("iterations");

subplot(3,2,3);
plot(chi_stats_l, 'r-', "linewidth", 2);
legend("Chi Landmark"); grid; xlabel("iterations");
subplot(3,2,4);
plot(num_inliers_l, 'b-', "linewidth", 2);
legend("Landmark inliers"); grid; xlabel("iterations");

pause();

disp("********************** EVALUATION LEAST SQUARE **************************");

###############################################################################
disp("************************* LANDMARKS *************************************");
################## LANDMARK EVALUATION AFTER LEAST SQUARE ####################
error = 0;
max_error = 0;
id = 0;
for i=1: size(Xl_sol)(2)
    if Xl_sol(:,i) == zeros(3,1) # unseen landmark put it to 0 as default
        continue
    endif
    ex = Xl_sol(1,i) - Xl_true(1,i);
    ey = Xl_sol(2,i) - Xl_true(2,i);
    ez = Xl_sol(3,i) - Xl_true(3,i);

    error+= sqrt(ex**2+ey**2+ez**2);
    if sqrt(ex**2+ey**2+ez**2) > max_error
        max_error = sqrt(ex**2+ey**2+ez**2);
        id = i;
    endif
endfor

MSE_initial = error/size(Xl_true)(2);
A = ["RMSE of landmarks after least square is ", num2str(MSE_initial)];
B = ["The maximum value of landmarks RMSE is ", num2str(max_error)];
disp(A); disp(B);

###############################################################################
disp("************************ POSES ****************************************");
################ POSE EVALUATION AFTER LEAST SQURE ############################
Xr_sol = Xr_sol';
Xr_true = Xr_sol';
rotation_error = 0;
translation_error = 0;
for pose=1:(size(Xr_true,2) - 1)
    T_0 = v2t2d(Xr_sol(:,pose));
    T_1 = v2t2d(Xr_sol(:,pose+1));
    GT_0 = v2t2d(Xr_true(:,pose));
    GT_1 = v2t2d(Xr_true(:,pose+1));

    rel_T = inv(T_0)*T_1;
    rel_GT = inv(GT_0)*GT_1;
    error_T = inv(rel_T)*rel_GT;

    rotation_error+= abs(atan2(error_T(2,1), error_T(1,1)));
    translation_error+=sqrt(error_T(1:2,3)(1)**2+error_T(1:2,3)(2)**2);
endfor

RMSE_translation = translation_error/(size(Xr_true,2));
ME_rotation = rotation_error/(size(Xr_true,2) - 1);
R = ["RMSE of translation poses is ", num2str(RMSE_translation)];
K = ["Mean error fo rotation part is ", num2str(ME_rotation)];
disp(R); disp(K);

###############################################################################