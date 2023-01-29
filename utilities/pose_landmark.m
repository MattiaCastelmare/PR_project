source "utilities/helpers.m" 

function Xl_initial_guess = triangulation(K, T_cam, pos, odometry_pose, dict_pos_land)

    dict_matrix = containers.Map('KeyType','double','ValueType','any'); # dict in which KEYS = id of landmarks VALUES = matrix A for the optimization problem

    # The idea is to have a dictionary in which for each landmark I have the matrix A for the optimization problem AX=0, after constructing this dictionary
    # I can simply apply the SVD of A for each landmark id UDV'= SVD(A). The solution of the problem will be the last column of the matrix V.


    cam = K*eye(3,4)*inv(T_cam); # first part of the projection matrix without the camera pose

    # create the dictionary 
    for i = 1:size(pos) # loop for all the robot poses
        for k = keys(dict_pos_land(i)) # for all the keys of the dictionary
            Proj_mat = cam*inv(v2t(odometry_pose(:,i))); # projection matrix
            measurement = dict_pos_land(i)(k{1}); # projection of landmark in that robot position
            eq1 = measurement(2)*Proj_mat(3,:) - Proj_mat(2,:); # first equation given by x=PX
            eq2 = Proj_mat(1,:) - measurement(1)*Proj_mat(3,:); # second equation given by x=PX
            
            if isKey(dict_matrix, k{1}) # if the landmark has been seen put the new equations below the old ones
                dict_matrix(k{1}) = [dict_matrix(k{1});eq1; eq2];    
            else
                dict_matrix(k{1}) = [eq1; eq2]; # if the landmark has NOT been seet initalize the dictionary key
            end

        end
    end

    disp("********************* SVD and initial guess **********************");
   
    for lan = keys(dict_matrix)
     
        [U, D, V] = svd(dict_matrix(lan{1}));
        Xl_initial_guess(:,lan{1}) = [V(1,4)/V(4,4); V(2,4)/V(4,4); V(3,4)/V(4,4); 1]; # the first column of the matrix V is the solution of the optimization problem AX=0
                                                                                    # I have to divide for the element in position (4,4) to have the landmarks in homogenous coordinate
    end
endfunction



function [error, Jr, Jl, valid_point] = proj_ErrorandJacobian(K, T_cam, Xr, Xl, Z, z_near, z_far, img_width, img_height)

    valid_point = 1;
    Picp = eye(3,4)*inv(T_cam)*inv(v2t(Xr))*Xl; # icp point
    Pcam = K*Picp; # point in camera frame

    prediction = [Pcam(1)/Pcam(3); Pcam(2)/Pcam(3)]; # point in image plane
    error = prediction - Z;

    Jproj = [1/Pcam(3), 0, - Pcam(1)/Pcam(3)*2;
            0,     1/Pcam(3), -Pcam(2)/Pcam(3)*2]; # Jacobian of the projection

    w2c = v2t(Xr)*T_cam; # transformation from world to camera
    iR = w2c(1:3,1:3)'; # transpose of rotation matrix because in the prediction I have to invert these matrices
    it = -iR*w2c(1:3,3); # inverted translation
    pw = iR*Xl(1:end-1) + it; # poin in world reference frame
    Jwr(1:3,1:3) = -iR; # translation part of the robot Jacobian
    Jwr(1:3, 4:6) = iR*skew(Xl(1:end-1)); # rotation part of the robot Jacobian
    Jwl = iR; # landmark Jacobian

    Jr = Jproj*K*Jwr
    Jl = Jproj*K*Jwl;

    if Pcam(3)> z_far | Pcam(3) < z_near | prediction(1) < 0 | prediction(1) > img_width | prediction(2) < 0 | prediction(2) > img_height
        valid_point = 0
    endif
     
endfunction

function [H, b, chi_stat, num_inliers] = Proj_H_b(K, T_cam, XR, XL, dict_pos_land, num_poses, num_landmarks, threshold, pos_dim, landmark_dim, z_near, z_far, img_width, img_height)
    # Xr = odometry pose MATRIX rows = vector of the pose columns = the number of the pose 
    # Xl = initial guess MATRIX rows = coords of landmark columns = landmark id

    # dict_pos_land dictionary in which keys = num of poses e values = dict in which keys = land_id keys = projections
    
    system_size = pos_dim*num_poses + landmark_dim*num_landmarks;
    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);
    chi_stat = 0;
    num_inliers = 0;

    for pose_index=1: size(XR)(2)
        Xr = XR(:,pose_index);
        for landmark_index = keys(dict_pos_land(pose_index))
            Xl = XL(:,landmark_index{1}); # in the matrix Xl i choose the columns corresponding to the k^th landmark
            Z = dict_pos_land(pose_index)(landmark_index{1}); # projection of the k^th landmark in the i^th robot pose

            [proj_error, Jr, Jl, point_valid] = proj_ErrorandJacobian(K, T_cam, Xr, Xl, Z, z_near, z_far, img_width, img_height);

            if point_valid
                chi = proj_error'*proj_error;
                if chi > threshold
                    proj_error*= sqrt(threshold/chi);
                    chi = threshold
                else
                    num_inliers +=1;
                end
                chi_stat += chi;
            
            end
            pose_matrix_index = poseMatrixIndex(pose_index, num_poses, num_landmarks);
            landmark_matrix_index = landmarkMatrixIndex(landmark_index{1}, num_poses, num_landmarks);
            

            Hrr = Jr'*Jr;
            Hrl = Jr'*Jl;
            Hll=Jl'*Jl;
            br=Jr'*proj_error;
            bl=Jl'*proj_error;

            H(pose_matrix_index:pose_matrix_index+pos_dim-1,
                pose_matrix_index:pose_matrix_index+pos_dim-1)+=Hrr;
                H(pose_matrix_index:pose_matrix_index+pos_dim-1,
                landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Hrl;
                H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
                landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=Hll;
                H(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,
                pose_matrix_index:pose_matrix_index+pos_dim-1)+=Hrl';
                b(pose_matrix_index:pose_matrix_index+pos_dim-1)+=br;
                b(landmark_matrix_index:landmark_matrix_index+landmark_dim-1)+=bl;
        endfor
    endfor        
endfunction