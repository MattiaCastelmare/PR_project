function [Xl, Xr] = DoTLS(XR,
                          XL, 
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
                          num_iterations) 
    for i=1:num_iterations

        % [H_pose, b_pose, chi_stat_pose, num_inliers_pose] = Pose_H_b(XR, 
        %                                       robot_measurement, 
        %                                       pos_dim, 
        %                                       num_poses, 
        %                                       num_landmarks, 
        %                                       landmark_dim,
        %                                       threshold_pose);

        [H_proj, b_proj, chi_stat_proj, num_inliers_proj] = Proj_H_b(K, 
                                                       T_cam, 
                                                       XR, 
                                                       XL, 
                                                       dict_pos_land, 
                                                       num_poses, 
                                                       num_landmarks, 
                                                       threshold_proj, 
                                                       pos_dim, 
                                                       landmark_dim, 
                                                       z_near, 
                                                       z_far, 
                                                       img_width, 
                                                       img_height);
        A = ["*********************** Iteration number ", num2str(i), " ***************************" ];
        B = ["Projection error ", num2str(chi_stat_proj)];
        #C = ["Pose error ", num2str(chi_stat_pose)];
        D = ["Number of projection inliers ", num2str(num_inliers_proj)];
        #E = ["Number of pose inliers ", num2str(num_inliers_pose)];
        disp(A); disp(B); #disp(C); disp(D); disp(E);
        H = H_proj;
        b = b_proj; 
        H+=eye(size(H))*damping; 
        dx = -H\b;

        [Xr, Xl] = boxPlus(XR, XL, num_poses, num_landmarks, dx);
        XR = Xr;
        XL = Xl;


    
    endfor
endfunction