function dict_point_initial_guess = triangulation(odometry_pose, dict_pos_land, K, T_cam)
    dict_point_initial_guess = containers.Map('KeyType','double','ValueType','any'); # store a dictionary in which the keys are the id of the landmark and the values are its coordinates in the world
    X_guess = ones(1,10000)*-1















end