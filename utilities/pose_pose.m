function [pose_error, Jj, Ji] = pose_ErrorandJacobian(Xr, robot_measurement,i)
    # Flatten both prediction and measurement in order to have easier Jacobian and to not have box minus operation
    prediction = inv(v2t(Xr(:,i)))*v2t(Xr(:,i+1));
    Z = robot_measurement(:,:,i);
    pose_error = flattenIsometryByColumns(prediction - Z);
    Rz0=[0 -1 0;
	     1  0 0;
	     0  0 0];

    Ri = (v2t(Xr(:,i)))(1:3,1:3);
    Rj = (v2t(Xr(:,i+1)))(1:3,1:3);
    ti = (v2t(Xr(:,i)))(1:3,4);
    tj = (v2t(Xr(:,i+1)))(1:3,4);

    zero = zeros(3,3);
    g_tx(1:3, 1:3) = zero;
    g_tx(1:3, 4) = Ri'*[1;0;0];

    g_ty(1:3, 1:3) = zero;
    g_ty(1:3, 4) = Ri'*[0;1;0];


    g_alphaz(1:3, 1:3) = Ri'*Rz0*Rj;
    g_alphaz(1:3, 4) = Ri'*Rz0*tj;

    Jj(1:12,1) = flattenIsometryByColumns(g_tx);
    Jj(1:12,2) = flattenIsometryByColumns(g_ty);
    Jj(1:12,3) = flattenIsometryByColumns(g_alphaz);
    Ji = -Jj;

     
endfunction

function [H, b, chi_stat, num_inliers] = Pose_H_b(Xr, 
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

    for pose_index = 1:(size(Xr)(2) - 1)
        [pose_error, Jj, Ji] = pose_ErrorandJacobian(Xr, robot_measurement,pose_index);
        omega = eye(12);
        inlier = 1;
        
        chi = pose_error'*pose_error;
        if chi > threshold_pose
            omega*= sqrt(threshold_pose/chi);
            chi = threshold_pose;
            inlier = 0;
        else
            num_inliers +=1;
        end
        chi_stat += chi;

        if ~inlier
            continue
        endif
            
        
        Hii = Ji'*omega*Ji;
        Hij = Ji'*omega*Jj;
        Hjj = Jj'*omega*Jj;

        bi = Ji'*omega*pose_error;
        bj = Jj'*omega*pose_error;


        pose_i_matrix_index = poseMatrixIndex(pose_index, num_poses, num_landmarks);
        pose_j_matrix_index = poseMatrixIndex(pose_index + 1, num_poses, num_landmarks);


        H(pose_i_matrix_index:pose_i_matrix_index+pos_dim-1,
            pose_i_matrix_index:pose_i_matrix_index+pos_dim-1)+=Hii;
        H(pose_i_matrix_index:pose_i_matrix_index+pos_dim-1,
            pose_j_matrix_index:pose_j_matrix_index+landmark_dim-1)+=Hij;
        H(pose_j_matrix_index:pose_j_matrix_index+landmark_dim-1,
            pose_j_matrix_index:pose_j_matrix_index+landmark_dim-1)+=Hjj;
        H(pose_j_matrix_index:pose_j_matrix_index+landmark_dim-1,
            pose_i_matrix_index:pose_i_matrix_index+pos_dim-1)+=Hij';
        b(pose_i_matrix_index:pose_i_matrix_index+pos_dim-1)+=bi;
        b(pose_j_matrix_index:pose_j_matrix_index+landmark_dim-1)+=bj;
    endfor
endfunction