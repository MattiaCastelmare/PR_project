1;

#from 6d vector to homogeneous matrix
function T=v2t(v)
    T=eye(4);
    T(1:3,1:3)=angles2R(v(4:6));
    T(1:3,4)=v(1:3);
endfunction;


function S=skew(v)
  S=[0,    -v(3), v(2);
     v(3),  0,    -v(1);
     -v(2), v(1), 0];
endfunction


function v=flattenIsometryByColumns(T)
v=zeros(12,1);
v(1:9)=reshape(T(1:3,1:3),9,1);
v(10:12)=T(1:3,4);
endfunction


#derivative of rotation matrix w.r.t rotation around z, in 0
global  Rz0=[0 -1 0;
	     1  0 0;
	     0  0 0];

#homogeneous division 
function p_img = hom(p)
  p_img=p(1:2)/p(3)
endfunction;


function v=t2v(T)
  v = zeros(6,1);
  v(1:3) = T(1:3,4);
  v(4:6) = r2a(T(1:3,1:3));
endfunction


% computes the pose 2d pose vector v from an homogeneous transform A
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function v=t2v2d(A)
	v(1:2, 1)=A(1:2,3);
	v(3,1)=atan2(A(2,1),A(1,1));
endfunction;


% computes the homogeneous transform matrix A of the pose vector v
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function A=v2t2d(v)
  	c=cos(v(3));
  	s=sin(v(3));
	A=[c, -s, v(1) ;
	s,  c, v(2) ;
	0   0  1  ];
endfunction;



function [XR, XL]=boxPlus(Xr, XL, num_poses, num_landmarks, dx)
  global pos_dim;
  global landmark_dim;
  for(pose_index=1:num_poses)
    pose_matrix_index=poseMatrixIndex(pose_index,
                                      num_poses,
                                      num_landmarks);
    dxr=dx(pose_matrix_index:pose_matrix_index+pos_dim-1);
    new = [Xr(:,pose_index)(1);Xr(:,pose_index)(2);Xr(:,pose_index)(6)];
    vect = t2v2d(v2t2d(dxr)*v2t2d(new));
    XR(:,pose_index)=[vect(1); vect(2);0;0;0; vect(3)];
    
  endfor;
  for(landmark_index=1:num_landmarks)
    landmark_matrix_index=landmarkMatrixIndex(landmark_index,
                                                  num_poses,
                                                  num_landmarks);
    dxl=dx(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,:);
    XL(1:end-1,landmark_index)+=1.2*dxl;
  endfor;
endfunction;


function pose_matrix_index = poseMatrixIndex(pose_index, num_poses, num_landmarks)
    global pos_dim;
    global landmark_dim;

    pose_matrix_index = 1+(pose_index-1)*pos_dim;
    if (pose_index > num_poses)
      pose_matrix_index = -1;
      return;
    endif
    
endfunction

function land_matrix_index = landmarkMatrixIndex(land_index, num_poses, num_landmarks)
    global pos_dim;
    global landmark_dim;
    land_matrix_index = 1+(num_poses)*pos_dim + (land_index - 1)*landmark_dim;
    if (land_index > num_landmarks)
      land_matrix_index = -1;
      return;
    endif   
    
endfunction
# Robot measurements function
# Input: 
#       - odometry poses
# Output: 
#        - tensor in which each element is the relative position measured 
#          (i^th third dimension is the relative position of the i+1^th pose w.r.t the i^st one and so on)
function robot_measurement = odometry_measure(odometry_pose);
    for columns = 1:(size(odometry_pose)(2) - 1)
        Xi = v2t(odometry_pose(:,columns));
        Xj = v2t(odometry_pose(:,columns+1));
        robot_measurement(:,:,columns) = inv(Xi)*Xj; 
    endfor
endfunction
