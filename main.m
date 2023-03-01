close all
clear 
clc 

#import utilites
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

disp("************************** LOADING MEASUREMENT **********************************");

[pos, odometry_pose, dict_pos_land] = load_measurements();

# Create a tensor in which the third dimension is the number of measure while the matrix is the relative position of the pos i^th and j^th

robot_measurement = odometry_measure(odometry_pose); #disp("Xr true in robot measurement"); # odometry_pose or Xr_true

#################################################################################

disp("************************** EVALUATION INITIAL POSES *****************************");

##################################################################################

Xr_odom = data_trajectory(:,2:4)';
Xr_gt = Xr_t;
rotation_error = 0;
translation_error = 0;
for pose=1:(size(Xr_gt,2) - 1)
    T_0 = v2t2d(Xr_odom(:,pose));
    T_1 = v2t2d(Xr_odom(:,pose+1));
    GT_0 = v2t2d(Xr_gt(:,pose));
    GT_1 = v2t2d(Xr_gt(:,pose+1));

    rel_T = inv(T_0)*T_1;
    rel_GT = inv(GT_0)*GT_1;
    error_T = inv(rel_T)*rel_GT;

    rotation_error+= abs(atan2(error_T(2,1), error_T(1,1)));
    translation_error+=sqrt(error_T(1:2,3)(1)**2+error_T(1:2,3)(2)**2);
endfor

RMSE_translation = translation_error/(size(Xr_gt,2)-1);
ME_rotation = rotation_error/(size(Xr_gt,2) - 1);
P = ["RMSE of translation part of the odometry pose is ", num2str(RMSE_translation)];
O = ["MEAN error of rotation part of the odometry pose is ", num2str(ME_rotation)];
disp(P); disp(O);

#################################################################################
disp("************************** TRIANGULATION ****************************************");

[Xl_initial_guess, num_lan_discard, num_lan_good]= triangulation(K, 
                                T_cam, 
                                pos, 
                                odometry_pose, # odometry_pose or Xr_true
                                dict_pos_land);  # disp("Xr true in triangulation");
R = ["The number of landmarks discarded is: ", num2str(num_lan_discard)];
C = ["The number of landmarks seen at least four times is: ", num2str(num_lan_good)];
disp(R);
disp(C);
Xl_initial_guess = [Xl_initial_guess(1,:); Xl_initial_guess(2,:); Xl_initial_guess(3,:); ones(1, size(Xl_initial_guess,2))];

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

O = ["RMSE initial guess ", num2str(MSE_initial)];
D = ["Max RMSE ", num2str(max_error)];
disp(O); disp(D);

#################################################################################

########## USEFUL VARIABLES ################################# 

global pos_dim = 3;
global landmark_dim = 3;
global damping = 1; 
global threshold_proj = 10000; 
global threshold_pose = 10; 
global num_iterations = 30;
global num_poses = size(odometry_pose)(2);
global num_landmarks = size(Xl_initial_guess)(2);

#################################################################################

Xr = odometry_pose;
Xl = Xl_initial_guess;

disp("************************** TOTAL LEAST SQUARE ***********************************");

[XL, XR, chi_stats_l, num_inliers_l, chi_stats_r, num_inliers_r, H] = DoTLS(
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

######################## PLOT #################################################

disp("************************** PLOT **************************************************");
Xr_sol = [XR(1,:); XR(2,:); XR(6,:)]'; # 2D PLANAR MOTION only in x y and theta (the others are 0)
Xr_sol_plot = [XR(1,:); XR(2,:); XR(6,:)];
Xl_sol = XL(1:end-1,:); # coordinates in format (x y z)

lan_to_remove = [0;0;0];
% Identify the columns to remove using logical indexing
cols_to_remove = all(Xl_sol == lan_to_remove, 1);
% Remove the identified columns from each matrix
Xl_sol(:, cols_to_remove) = [];
Xl_true(:, cols_to_remove) = [];

figure(1);
hold on;
grid; 
subplot(2,2,1);
title("Odometry poses and Groundtruth");
plot(Xr_t(1,:),Xr_t(2,:), 'ro', 'linewidth', 3);
hold on;
plot(Xr_odom(1,:),Xr_odom(2,:), 'b-', 'linewidth', 4);
l1 = legend("Groundtruth", "Odometry poses");
set(l1, 'location','northoutside', 'fontsize', 14); grid;

subplot(2,2,2);
title("Poses after optimization and Groundtruth");
plot(Xr_t(1,:),Xr_t(2,:), 'ro', 'linewidth', 3);
hold on;
plot(Xr_sol_plot(1,:),Xr_sol_plot(2,:), 'b-', 'linewidth', 4);
l2 = legend("Groundtruth","LS solution poses");
set(l2, 'location','northoutside', 'fontsize', 14); grid;


subplot(2,2,3);
title("Landmarks initial guess");
plot(Xl_true(1,:),Xl_true(2,:),'ro',"linewidth",2);
hold on;
plot(Xl(1,:),Xl(2,:),'b*',"linewidth",2);
l3 = legend("Groundtruth", "Landmarks initial guess");
set(l3, 'location','northoutside', 'fontsize', 14); grid;

subplot(2,2,4);
title("Landmark after optimization and Groundtruth");
plot(Xl_true(1,:),Xl_true(2,:),'ro',"linewidth",2);
hold on;
plot(Xl_sol(1,:),Xl_sol(2,:),'b*','linewidth',2);
l4 = legend("Groundtruth", "LS solution landmarks");
set(l4, 'location','northoutside', 'fontsize', 14); grid;


figure(2);
hold on;
grid;
title("chi evolution");

subplot(3,2,1);
plot(chi_stats_r, 'r-', "linewidth", 3);
l5 = legend("Chi Poses"); grid; xlabel("iterations");
set(l5, 'location','northoutside', 'fontsize', 14);
subplot(3,2,2);
plot(num_inliers_r, 'b-', "linewidth", 3);
l6 = legend("# Pose inliers"); grid; xlabel("iterations");
set(l6, 'location','northoutside', 'fontsize', 14);

subplot(3,2,3);
plot(chi_stats_l, 'r-', "linewidth", 3);
l7 = legend("Chi Landmark"); grid; xlabel("iterations");
set(l7, 'location','northoutside', 'fontsize', 14);
subplot(3,2,4);
plot(num_inliers_l, 'b-', "linewidth", 3);
l8 = legend("# Landmark inliers"); grid; xlabel("iterations");
set(l8, 'location','northoutside', 'fontsize', 14);


############################# 3D Plotting #######################################

figure(3);
title("3D plotting of the map");
scatter3(Xl_true(1,:),Xl_true(2,:),Xl_true(3,:), 'r');
hold on;
scatter3(Xl_sol(1,:),Xl_sol(2,:),Xl_sol(3,:), 'b');
l9 = legend("Groundtruth", "LS solution landmarks");
set(l9, 'location','northoutside', 'fontsize', 14);
pause()


#################################################################################

disp("********************** EVALUATION LEAST SQUARE **************************");

#################################################################################

disp("************************* LANDMARKS *************************************");

################## LANDMARK EVALUATION AFTER LEAST SQUARE #######################

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
D = ["RMSE of landmarks after least square is ", num2str(MSE_initial)];
I = ["The maximum value of landmarks RMSE is ", num2str(max_error)];
disp(D); disp(I);

###############################################################################

disp("************************ POSES ****************************************");

################ POSE EVALUATION AFTER LEAST SQURE ############################

Xr_sol = Xr_sol';
Xr_true = Xr_t;
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

RMSE_translation = translation_error/(size(Xr_true,2)-1);
ME_rotation = rotation_error/(size(Xr_true,2) - 1);
O = ["RMSE of translation part of the LS solution is ", num2str(RMSE_translation)];
O2 = ["Mean error of rotation part of the LS solution is ", num2str(ME_rotation)];
disp(O); disp(O2);

#################################################################################
# The following code was taken from https://gitlab.com/grisetti/probabilistic_robotics_2018_19  
disp("************************ H Matrix ****************************************");

figure(4);
title("H matrix");
H_ =  H./H;                      # NaN and 1 element
H_(isnan(H_))=0;                 # Nan to Zero
H_ = abs(ones(size(H_)) - H_);   # switch zero and one
H_ = flipud(H_);                 # switch rows
colormap(gray(64));
hold on;
image([0.5, size(H_,2)-0.5], [0.5, size(H_,1)-0.5], H_*64);
pause();
hold off;

#################################################################################