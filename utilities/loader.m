function [K, T_cam, z_near, z_far, widht, height] = load_camera_parameters()

    data_camera = dlmread("dataset/camera.dat");

    K = data_camera(2:4,1:3);
    T_cam = data_camera(6:9, 1:4);
    z_near = data_camera(10,2);
    z_far = data_camera(11,2);
    widht = data_camera(12,2);
    height = data_camera(13,2);
endfunction

function [pos, odometry_pose, dict_pos_land] = load_measurements()

    dict_pos_land = containers.Map('KeyType','double','ValueType','any'); # dict in which KEYS = i^th robot pose VALUES = landmark dictionary

    for i = 0 : 335

        if i < 10
        num = num2str(i);
        filename = strcat("dataset/meas-0000",num,".dat");
        elseif i > 9 & i < 100
        num = num2str(i);
        filename = strcat("dataset/meas-000",num,".dat");
        else
        num = num2str(i);
        filename = strcat("dataset/meas-00",num,".dat");    
        end

        #[a, b, c, d, f] = textread(filename);
        fid = fopen(filename);
        tline = fgetl(fid);
        dict_land_id = containers.Map('KeyType','double','ValueType','any');
        while ischar(tline)
            seq_match = regexp(tline, '^seq:\s+(\d+)', 'tokens');
            if ~isempty(seq_match)
                seq = str2double(seq_match{1}{1});
                pos(i+1,1) = seq+1; # column array in wich I store the number of all the poses
            end
            
            gt_pose_match = regexp(tline, '^gt_pose:\s+([\d\s.-]+)', 'tokens');
            if ~isempty(gt_pose_match)
                gt_pose = str2num(gt_pose_match{1}{1});
                groundtruth_pose(:,i+1) = [gt_pose(1), gt_pose(2), 0, 0, 0, gt_pose(3)]; # matrix in which each column is the gt pose and the column number is the number of pose (saved in 3D because Tcam is 4x4 matrix)
            end
            
            odom_pose_match = regexp(tline, '^odom_pose:\s+([\d\s.-]+)', 'tokens');
            if ~isempty(odom_pose_match)
                odom_pose = str2num(odom_pose_match{1}{1});
                odometry_pose(:,i+1) = [odom_pose(1), odom_pose(2), 0, 0, 0, odom_pose(3)];
            end
            
            point_match = regexp(tline, '^point\s+\d+\s+(\d+)\s+([\d.-]+)\s+([\d.-]+)', 'tokens');
            if ~isempty(point_match)
                point = str2num(point_match{1}{1});
                x = str2double(point_match{1}{2}); # image coordinates
                y = str2double(point_match{1}{3});
                landmark_id = point + 1; # list of landmark id
                pair = [x, y];
                dict_land_id(landmark_id) = pair;  # dict in which KEYS = landmark id VALUES = [x;y] of landmark in the corresponding images
                
            end
            dict_pos_land(pos(i+1,1)) = dict_land_id;
            
            tline = fgetl(fid);
        end
    endfor

endfunction