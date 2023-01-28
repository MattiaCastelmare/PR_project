function Xl_initial_guess = triangulation(pos, odometry_pose, dict_pos_land, K, T_cam)
    
    dict_matrix = containers.Map('KeyType','double','ValueType','any'); # dict in which KEYS = id of landmarks VALUES = matrix A for the optimization problem

    # The idea is to have a dictionary in which for each landmark I have the matrix A for the optimization problem AX=0, after constructing this dictionary
    # I can simply apply the SVD of A for each landmark id UDV'= SVD(A). The solution of the problem will be the last column of the matrix V.


    cam = K*eye(3,4)*inv(T_cam); # first part of the projection matrix without the camera pose

    # create the dictionary 
    for i = 1:size(pos) # loop for all the robot poses
        for k = keys(dict_pos_land(i)) # for all the keys of the dictionary
            Proj_mat = cam*inv(v2t(odometry_pose(:,i))); # projection matrix
            measurement = dict_pos_land(i)(k{1}); # measurements of that robot position
            eq1 = measurement(2)*Proj_mat(3,:) - Proj_mat(2,:); # first equation given by x=PX
            eq2 = Proj_mat(1,:) - measurement(1)*Proj_mat(3,:); # second equation given by x=PX
            
            if isKey(dict_matrix, k{1}) # if the landmark has been seen put the new equations below the old ones
                dict_matrix(k{1}) = [dict_matrix(k{1});eq1; eq2];
                
            else
                dict_matrix(k{1}) = [eq1; eq2]; # if the landmark has NOT been seet initalize the dictionary key
            end

        end
    end

    disp("********************* SVD and determining landmark positions **********************");

    for lan = keys(dict_matrix)
     
        [U, D, V] = svd(dict_matrix(lan{1}));
        Xl_initial_guess(:,lan{1}) = [V(1,4)/V(4,4); V(2,4)/V(4,4); V(3,4)/V(4,4)]; # the first column of the matrix V is the solution of the optimization problem AX=0
                                                                                    # I have to divide for the element in position (4,4) to have the landmarks in homogenous coordinate
    end

    
end


#function [robot_pose, landmarks_location] = DoTLS(pos, K, T_cam, odometry_pose, Xl_initial_guess, )
#end