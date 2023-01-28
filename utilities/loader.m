function [K, T_cam, z_near, z_far, widht, height] = load_camera_parameters()

    data_camera = dlmread("data/camera.dat");

    K = data_camera(2:4,1:3);
    T_cam = data_camera(6:9, 1:4);
    z_near = data_camera(10,2);
    z_far = data_camera(11,2);
    widht = data_camera(12,2);
    height = data_camera(13,2);
end

function [pos, odometry_pose, dict_pos_land] = load_measurements()

    dict_pos_land = containers.Map('KeyType','double','ValueType','any'); # dict in which KEYS = number of robot poses VALUES = landmark dictionary

    for i = 0 : 199

        if i < 10
        num = num2str(i);
        filename = strcat("data/meas-0000",num,".dat");
        elseif i > 9 & i < 100
        num = num2str(i);
        filename = strcat("data/meas-000",num,".dat");
        else
        num = num2str(i);
        filename = strcat("data/meas-00",num,".dat");    
        end

        [a, b, c, d, f] = textread(filename);
        
        pos(i+1,1) = b(1) + 1; # column array in wich I store the number of all the poses

        groundtruth_pose(:,i+1) = [d(1), f(1), 0, 0, 0, a(2)]; # matrix in which each column is the gt pose and the column number is the number of pose (saved in 6D because Tcam is 4x4 matrix)

        odometry_pose(:,i+1) = [c(2), d(2), 0, 0, 0, f(2)]; # same as gt (saved in 6D because Tcam is 4x4 matrix)

        measurement_number = b(3:end-1); 
        
        landmark_id = c(3:end) + 1; # list of landmark id
        image_x = d(3:end); # list of image coordinates
        image_y = f(3:end);

        landmark_id = landmark_id(~isnan(landmark_id));
        image_x = image_x(~isnan(image_x));
        image_y = image_y(~isnan(image_y));

        pair = [image_x, image_y];
        values = num2cell(pair,2);
        dict_land_id = containers.Map(landmark_id, values);  # dict in which KEYS = landmark id VALUES = [x;y] of landmark in the corresponding images


        dict_pos_land(pos(i+1)) = dict_land_id; # dict in which KEYS = number of robot poses VALUES = landmark dictionary
    end

end