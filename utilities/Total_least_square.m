function [XL, XR, chi_stats_l, num_inliers_l, chi_stats_r, num_inliers_r, H] = DoTLS(
                                                                                    XR,
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
    chi_stats_r=zeros(1,num_iterations);
    num_inliers_r=zeros(1,num_iterations);

    for i=1:num_iterations
        H=zeros(system_size, system_size);
        b=zeros(system_size,1);
        [H_pose, b_pose, chi_r, inliers_r, num_outliers_pose] = Pose_H_b(
                                              XR, 
                                              robot_measurement, 
                                              pos_dim, 
                                              num_poses, 
                                              num_landmarks, 
                                              landmark_dim,
                                              threshold_pose);

        [H_proj, b_proj, chi_l, inliers_l, num_outliers_proj] = Proj_H_b(
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
        B = ["Chi landmark ", num2str(chi_l)];
        C = ["Chi pose ", num2str(chi_r)];
        D = ["Number of projection inliers ", num2str(inliers_l)];
        G = ["Number of projection outliers ", num2str(num_outliers_proj)];
        E = ["Number of pose inliers ", num2str(inliers_r)];
        F = ["Number of pose outliers ", num2str(num_outliers_pose)];
        
        disp(A); disp(B); disp(C); disp(D); disp(G); disp(E); disp(F);
        chi_stats_l(1,i)=chi_l;
        num_inliers_l(1,i)=inliers_l;
        chi_stats_r(1,i)=chi_r;
        num_inliers_r(1,i)=inliers_r;

        H_proj+=eye(size(H_proj))*1; 

        H = H_proj + H_pose;
        b = b_proj + b_pose; 
        
        H+=eye(size(H))*damping; 
        dx = zeros(system_size,1);
        dx(pos_dim+1:end)=-(H(pos_dim+1:end,pos_dim+1:end)\b(pos_dim+1:end,1)); # block the first pose 
        error = sum(abs(dx));
        A = ["The perturbation is: ", num2str(error)];
        disp(A);
        [XR, XL] = boxPlus(XR, XL, num_poses, num_landmarks, dx);
    
    endfor
endfunction