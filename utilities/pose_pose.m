source "utilities/helpers.m"

# Error and jacobian of projection
# Input: Xr: robot poses
#        robot_measurement: relative position of the i+1^th robot pose w.r.t. i^th robot pose
#        i: pose index
#
# Output: pose_error: 6x1 flatten pose error
#         Jj: 6x3 Jacobian w.r.t. the perturbation of the i^th robot pose (xi, yi and thetai)
#         Ji: 6x3 Jacobian w.r.t. the perturbation of the i+1^th robot pose (xj, yj, thetaj)

function [pose_error, Jj, Ji] = pose_ErrorandJacobian(Xr, robot_measurement,i)
    # Flatten both prediction and measurement in order to have easier Jacobian and to not have box minus operation

    global Rz0

    Ri = (v2t(Xr(:,i)))(1:2,1:2);
    Rj = (v2t(Xr(:,i+1)))(1:2,1:2);
    
    ti = (v2t(Xr(:,i)))(1:2,4);
    tj = (v2t(Xr(:,i+1)))(1:2,4);

    g_alphaz = [Ri'*Rz0(1:2,1:2)*Rj, Ri'*Rz0(1:2,1:2)*tj];
    
    g_tx = [zeros(2,2), Ri'*[1;0]];
    g_ty = [zeros(2,2), Ri'*[0;1]];

    Jj(:,1) = reshape(g_tx,6,1);
    Jj(:,2) = reshape(g_ty,6,1);
    Jj(:,3) = reshape(g_alphaz,6,1);
    Ji = -Jj;

    prediction(1:2,1:2) = Ri'*Rj;
    prediction(1:2,3) = Ri'*(tj-ti);
    Z(1:2,1:2) = robot_measurement(:,:,i)(1:2,1:2);
    Z(1:2,3) = robot_measurement(:,:,i)(1:2,4);
    pose_error = reshape((prediction - Z),6,1);
endfunction

function [H, b, chi_stat, num_inliers, num_outliers] = Pose_H_b(Xr, 
                                                  robot_measurement, 
                                                  pos_dim, 
                                                  num_poses,
                                                  num_landmarks, 
                                                  landmark_dim, 
                                                  threshold_pose)

    system_size = pos_dim*num_poses + landmark_dim*num_landmarks;
    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);
    chi_stat = 0;
    num_inliers = 0;
    num_outliers = 0;

    for pose_index = 1:(size(Xr)(2) - 1)

        [pose_error, Jj, Ji] = pose_ErrorandJacobian(Xr, robot_measurement,pose_index);
        omega = eye(6)*1e3; 

        inlier = 1; # boolean for the inlier
        chi = pose_error'*omega*pose_error;
        if chi > threshold_pose
            omega*= sqrt(threshold_pose/chi);
            chi = threshold_pose;
            inlier = 0;
            num_outliers ++;
        else
            num_inliers ++;
        end
        if ~inlier
          continue
        endif
        chi_stat += chi;

           
        
        Hii = Ji'*omega*Ji;
        Hij = Ji'*omega*Jj;
        Hjj = Jj'*omega*Jj;
        Hji = Jj'*omega*Ji;
        bi = Ji'*omega*pose_error;
        bj = Jj'*omega*pose_error;
        
      

        pose_i_matrix_index = poseMatrixIndex(pose_index, num_poses, num_landmarks);
        pose_j_matrix_index = poseMatrixIndex(pose_index + 1, num_poses, num_landmarks);
 
        H(pose_i_matrix_index:pose_i_matrix_index+pos_dim-1,
          pose_i_matrix_index:pose_i_matrix_index+pos_dim-1)+=Hii;

        H(pose_i_matrix_index:pose_i_matrix_index+pos_dim-1,
          pose_j_matrix_index:pose_j_matrix_index+pos_dim-1)+=Hij;

        H(pose_j_matrix_index:pose_j_matrix_index+pos_dim-1,
          pose_j_matrix_index:pose_j_matrix_index+pos_dim-1)+=Hjj;

        H(pose_j_matrix_index:pose_j_matrix_index+pos_dim-1,
          pose_i_matrix_index:pose_i_matrix_index+pos_dim-1)+=Hji;

        b(pose_i_matrix_index:pose_i_matrix_index+pos_dim-1)+=bi;

        b(pose_j_matrix_index:pose_j_matrix_index+pos_dim-1)+=bj;
    endfor
endfunction