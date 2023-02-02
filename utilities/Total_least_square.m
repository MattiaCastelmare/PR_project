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
                          num_iterations) 
    for i=0:num_iterations
        [H_pose, b_pose, chi_stat, num_inliers] = Pose_H_b(XR, 
                                              robot_measurement, 
                                              pos_dim, 
                                              num_poses, 
                                              num_landmarks, 
                                              landmark_dim,
                                              threshold_pose);

        [H_proj, b_proj, chi_stat, num_inliers] = Proj_H_b(K, 
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
        H = H_pose + H_proj;
        b = b_pose + b_proj; 
        dx = -H\b;

        [Xr, Xl] = boxPlus(XR, XL, num_poses, num_landmarks, dx);
        XR = Xr;
        XL = Xl;

    
    endfor
endfunction