function Xl_initial_guess = triangulation(pos, odometry_pose, dict_pos_land, K, T_cam)
    
    dict_matrix = containers.Map('KeyType','double','ValueType','any'); # dict in which KEYS = id of landmarks VALUES = matrix A for the optimization problem

    cam = K*eye(3,4)*inv(T_cam); # first part of the projection matrix without the camera pose
    Xl_initial_guess = {};
    # create the dictionary 
    for i = 1:size(pos)
        for k = keys(dict_pos_land(i))
            Proj_mat = cam*inv(v2t(odometry_pose(:,i)));
            measurement = dict_pos_land(i)(k{1});
            eq1 = measurement(2)*Proj_mat(3,:) - Proj_mat(2,:);
            eq2 = Proj_mat(1,:) - measurement(1)*Proj_mat(3,:);
            
            if isKey(dict_matrix, k{1})
                dict_matrix(k{1}) = [dict_matrix(k{1});eq1; eq2];
                
            else
                dict_matrix(k{1}) = [eq1; eq2];
            end

        end
    end
    disp("********************* SVD and determining landmark positions **********************");

    for lan = keys(dict_matrix)
        [U, D, Vt] = svd(dict_matrix(lan{1}));
        Xl_initial_guess(lan{1}) = [Vt(4,1)/Vt(4,4); Vt(4,2)/Vt(4,4); Vt(4,3)/Vt(4,4)];
    end

end