source "utilities/helpers.m" 

function [Xl_initial_guess, lan_discard, lan_good] = triangulation(
                                          K, 
                                          T_cam, 
                                          pos, 
                                          odometry_pose, 
                                          dict_pos_land)

    dict_matrix = containers.Map('KeyType','double','ValueType','any'); # dict in which KEYS = id of landmarks VALUES = matrix A for the optimization problem

    # The idea is to have a dictionary in which for each landmark I have the matrix A for the optimization problem AX=0, after constructing this dictionary

    cam = K*eye(3,4)*inv(T_cam); # first part of the projection matrix without the camera pose

    # create the dictionary 
    for i = 1:size(pos) # loop for all the robot poses
        Proj_mat = cam*inv(v2t(odometry_pose(:,i))); # projection matrix

        for k = keys(dict_pos_land(i)) # for all the keys (landmark id) of the dictionary of the robot poses
        
            measurement = dict_pos_land(i)(k{1}); # projection of landmark in that robot position
            if measurement(1) > 50 && measurement(2) > 50
                
                eq1 = measurement(2)*Proj_mat(3,:) - Proj_mat(2,:); # first equation given by x=PX

                eq2 = Proj_mat(1,:) - measurement(1)*Proj_mat(3,:); # second equation given by x=PX
                
                if isKey(dict_matrix, k{1}) # if the landmark has been seen put the new equations below the old ones

                    dict_matrix(k{1}) = [dict_matrix(k{1});eq1; eq2];    
                else
                    dict_matrix(k{1}) = [eq1; eq2]; # if the landmark has NOT been seet initalize the dictionary key
                endif
            endif

        endfor
    endfor
    lan_discard = 0;
    lan_good = 0;
    for lan = keys(dict_matrix)
        
        [U, D, V] = svd(dict_matrix(lan{1}));
        if size(D,2) == 4
            if size(dict_matrix(lan{1}),1) > 7
                Xl_initial_guess(:,lan{1}) = [V(1,4)/V(4,4); V(2,4)/V(4,4); V(3,4)/V(4,4); 1]; # the first column of the matrix V is the solution of the optimization problem AX=0
                                                                                            # I have to divide for the element in position (4,4) to have the landmarks in homogenous coordinate
                lan_good+=1;
            else 
                Xl_initial_guess(:,lan{1}) = [0;0;0;1];
                lan_discard+=1;
            endif 
        endif                                                                          
    endfor
endfunction

# Error and jacobian of projection
# Input: K: camera matrix
#        T_cam: transformation matrix of the camera
#        Xr: robot pose in world reference frame (4x4 homogenous matrix)
#        Xl: landmark pose in world reference frame (4x1 vector)
#        Z: measurement of the projection of the landmark in the image plane
#        z_near, z_far, img_width, img_height camera parameters
#
# Output: proj_error: 2x1 projection error in the image (prediction - measurement)
#         Jr: 2x3 Jacobian w.r.t. the perturbation of the robot pose (x, y and theta)
#         Jl: 2x3 Jacobian w.r.t. the perturbation of the landmark (xl, yl and zl)
#         valid_point: boolean that indicates if the predicted point respects the bounds

function [proj_error, Jr, Jl, valid_point] = proj_ErrorandJacobian(
                                                                   K, 
                                                                   T_cam,  
                                                                   Xr, 
                                                                   Xl, 
                                                                   Z, 
                                                                   z_near, 
                                                                   z_far, 
                                                                   img_width, 
                                                                   img_height)

    valid_point = 1;
    proj_error = [0;0];
    Jr = zeros(2,3);
    Jl = zeros(2,3);

    Picp = eye(3,4)*inv(T_cam)*inv(v2t(Xr))*Xl; # icp point
    Pcam = K*Picp; # point in camera frame

    prediction = [Pcam(1)/Pcam(3); Pcam(2)/Pcam(3)]; # point in image plane
    proj_error = prediction - Z;
    Jproj = [1./Pcam(3), 0, - Pcam(1)/(Pcam(3)**2);
            0,     1./Pcam(3), -Pcam(2)/(Pcam(3)**2)]; # Jacobian of the projection

    w2c = v2t(Xr)*T_cam; # transformation from world to camera
    iR = w2c(1:3,1:3)'; # transpose of rotation matrix because in the prediction I have to invert these matrices
    it = -iR*w2c(1:3,4); # inverted translation
    pw = iR*Xl(1:end-1) + it; # point in world reference frame
    Jwr(1:3,1:3) = -iR; # translation part of the robot Jacobian
    Jwr(:, 4:6) = iR*skew(Xl(1:end-1));  # rotation part of the robot Jacobian
    Jwr = Jwr(1:3, [1,2,6]); # since it is a PLANAR motion I consider only the first, the second and the last column of the Jacobian

    Jwl = iR; # landmark Jacobian

    Jr = Jproj*K*Jwr;
    Jl = Jproj*K*Jwl;
    
    
    if (pw(3)> z_far || 
       pw(3) < z_near || 
       prediction(1) < 0 || 
       prediction(1) > img_width || 
       prediction(2) < 0 || 
       prediction(2) > img_height)

       valid_point = 0;
        
    endif
    if Xl == [0;0;0;1];
        valid_point = 0;
    endif
     
endfunction

# Function that determines H and b matrices
# Input: K: camera matrix
#        T_cam: transformation matrix of the camera
#        XR: matrix of robot pose in world reference frame (rows = vector of robot pose, columns = number of pose)
#        Xl: matrix of landmark in world reference frame (rows = landmark coordinates, columns = id of landmark)
#        dict_pos_land: dictionary in which KEYS = num of poses e VALUES = dict in which KEYS = land_id VALUES = projection in the image
#
# Output: H: matrix for the Gaussian algorithm
#         b: vector for the Gaussian algorithm

function [H, b, chi_stat, num_inliers, num_outliers] = Proj_H_b(
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
                                                                img_height)

    system_size = pos_dim*num_poses + landmark_dim*num_landmarks;
    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);
    chi_stat = 0;
    num_inliers = 0;
    num_outliers = 0;

    for pose_index=1: size(XR)(2)
        Xr = XR(:,pose_index);


        for landmark_index = keys(dict_pos_land(pose_index))


            Xl = XL(:,landmark_index{1}); # in the matrix Xl i choose the columns corresponding to the k^th landmark
            Z = dict_pos_land(pose_index)(landmark_index{1})'; # projection of the k^th landmark in the i^th robot pose
            
            [proj_error, Jr, Jl, point_valid] = proj_ErrorandJacobian(K, 
                                                                      T_cam, 
                                                                      Xr, 
                                                                      Xl, 
                                                                      Z, 
                                                                      z_near, 
                                                                      z_far, 
                                                                      img_width, 
                                                                      img_height);

            if point_valid
                chi = proj_error'*proj_error;
                if chi > threshold_proj
                    proj_error*= sqrt(threshold_proj/chi);
                    chi = threshold_proj;
                    
                else
                    num_inliers +=1;
                endif
                chi_stat += chi;
            
                pose_matrix_index = poseMatrixIndex(pose_index, num_poses, num_landmarks);
                landmark_matrix_index = landmarkMatrixIndex(landmark_index{1}, num_poses, num_landmarks);

                omega = eye(2)*1e-2; # CHANGE 
                Hrr = Jr'*omega*Jr;
                Hrl = Jr'*omega*Jl;
                Hlr = Jl'*omega*Jr;

                Hll=Jl'*omega*Jl;
                br=Jr'*omega*proj_error;
                bl=Jl'*omega*proj_error;
  

                H(pose_matrix_index:pose_matrix_index+pos_dim-1,
                  pose_matrix_index:pose_matrix_index+pos_dim-1)+=Hrr;

                H(pose_matrix_index:pose_matrix_index+pos_dim-1,
                  landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Hrl;

                H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
                  landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Hll;

                H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
                  pose_matrix_index:pose_matrix_index+pos_dim-1)+=Hlr;

                b(pose_matrix_index:pose_matrix_index+pos_dim-1)+=br;

                b(landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=bl;
            else
                num_outliers +=1;
            endif
        endfor
    endfor        
endfunction