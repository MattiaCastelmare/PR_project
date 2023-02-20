function [XL, XR] = DoTLS(XR,
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
    system_size = pos_dim*num_poses + landmark_dim*num_landmarks;
    chi_stats_l=zeros(1,num_iterations);
    num_inliers_l=zeros(1,num_iterations);
    chi_stats_p=zeros(1,num_iterations);
    num_inliers_p=zeros(1,num_iterations);
    chi_stats_r=zeros(1,num_iterations);
    num_inliers_r=zeros(1,num_iterations);

    for i=1:num_iterations

        [H_pose, b_pose, chi_stats_r, num_inliers_r, num_outliers_pose] = Pose_H_b(
                                              XR, 
                                              robot_measurement, 
                                              pos_dim, 
                                              num_poses, 
                                              num_landmarks, 
                                              landmark_dim,
                                              threshold_pose);

        [H_proj, b_proj, chi_stats_l, num_inliers_l, num_outliers_proj] = Proj_H_b(
                                                       K, 
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
        B = ["Projection error ", num2str(chi_stats_l)];
        C = ["Pose error ", num2str(chi_stats_r)];
        D = ["Number of projection inliers ", num2str(num_inliers_l)];
        G = ["Number of projection outliers ", num2str(num_outliers_proj)];
        E = ["Number of pose inliers ", num2str(num_inliers_r)];
        F = ["Number of pose outliers ", num2str(num_outliers_pose)];
        
        disp(A); disp(B); disp(C); disp(D); disp(G); disp(E); disp(F);
        
        H = H_proj+H_pose;
        b = b_proj+b_pose; 
        H+=eye(size(H))*damping; 
        dx = zeros(system_size,1);
        
        dx(pos_dim+1:end)=-(H(pos_dim+1:end,pos_dim+1:end)\b(pos_dim+1:end,1));

        [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx);

    
    endfor
endfunction