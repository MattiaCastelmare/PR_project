1;

#rotation matrix around x axis
function R=Rx(rot_x)
 c=cos(rot_x);
 s=sin(rot_x);
 R= [1  0  0;
     0  c  -s;
     0  s  c];
endfunction

#rotation matrix around y axis
function R=Ry(rot_y)
 c=cos(rot_y);
 s=sin(rot_y);
 R= [c  0  s;
     0  1  0;
     -s  0 c];
endfunction

#rotation matrix around z axis
function R=Rz(rot_z)
 c=cos(rot_z);
 s=sin(rot_z);
 R= [ c  -s  0;
      s  c  0;
      0  0  1];
endfunction

#derivative of rotation matrix around z
function R=Rx_prime(rot_x)
 dc=-sin(rot_x); #derivative of cos(rot(x)
 ds=cos(rot_x);  #derivative of sin(rot(x)
 R= [0  0  0;
     0  dc  -ds;
     0  ds  dc];
endfunction

#derivative of rotation matrix around y
function R=Ry_prime(rot_y)
 dc=-sin(rot_y); #derivative of cos(rot(x)
 ds=cos(rot_y);  #derivative of sin(rot(x)
 R= [dc  0 ds;
     0  0  0;
     -ds  0 dc];
endfunction

#derivative of rotation matrix around z
function R=Rz_prime(rot_z)
 dc=-sin(rot_z); #derivative of cos(rot(x)
 ds=cos(rot_z);  #derivative of sin(rot(x)
 R= [ dc  -ds  0;
      ds  dc  0;
      0  0  0];
endfunction

function R=angles2R(a)
  R=Rx(a(1))*Ry(a(2))*Rz(a(3));
endfunction;

#from 6d vector to homogeneous matrix
function T=v2t(v)
    T=eye(4);
    T(1:3,1:3)=angles2R(v(4:6));
    T(1:3,4)=v(1:3);
endfunction;

#from 7d vector to similiarity matrix Sim(3)
function S=v2s(v)
	S=eye(4);
	S=v2t(v(1:6));
	S(4,4) = exp(v(7));
endfunction

function S=skew(v)
  S=[0,    -v(3), v(2);
     v(3),  0,    -v(1);
     -v(2), v(1), 0];
endfunction

function v=flattenIsometry(T)
v=zeros(12,1);
v(1:9)=reshape(T(1:3,1:3)',9,1);
v(10:12)=T(1:3,4);
endfunction

function T=unflattenIsometry(v)
  T=eye(4);
  T(1:3,1:3)=reshape(v(1:9),3,3)';
  T(1:3,4)=v(10:12);
endfunction

function v=flattenIsometryByColumns(T)
v=zeros(12,1);
v(1:9)=reshape(T(1:3,1:3),9,1);
v(10:12)=T(1:3,4);
endfunction

function T=unflattenIsometryByColumns(v)
  T=eye(4);
  T(1:3,1:3)=reshape(v(1:9),3,3);
  T(1:3,4)=v(10:12);
endfunction

function M=flatTransformationMatrix(v)
  T=unflattenIsometry(v);
  R=T(1:3,1:3);
  t=T(1:3,4);
  M=eye(12);
  M(1:3,1:3)=R';
  M(4:6,4:6)=R';
  M(7:9,7:9)=R';
  M(10,1:3)=t';
  M(11,4:6)=t';
  M(12,7:9)=t';
endfunction;

#derivative of rotation matrix w.r.t rotation around x, in 0
global  Rx0=[0 0 0;
	     0 0 -1;
	     0 1 0];

#derivative of rotation matrix w.r.t rotation around y, in 0
global  Ry0=[0 0 1;
	     0 0 0;
	     -1 0 0];

#derivative of rotation matrix w.r.t rotation around z, in 0
global  Rz0=[0 -1 0;
	     1  0 0;
	     0  0 0];

#homogeneous division 
function p_img = hom(p)
  p_img=p(1:2)/p(3)
endfunction;

#jacobian of homogeneous division
function J = J_hom(p)
  x=p(1);
  y=p(2);
  iw=1./p(3);
  iw2=iw**2;
  J = [ iw, 0,  -x*iw2;
        0,  iw, -y*iw2];
endfunction;

%derivative of icp, euler, left mult, inverse transform and manifold
function J = J_icp(p)
  J=zeros(3,6);
  J(1:3,1:3)=eye(3);
  J(1:3,4:6)=-skew(p);
endfunction

#jacobian of the norm function
function J = J_norm(p)
  n=norm(p);
  J = (1/n) * p';
endfunction;

#p/norm(p)
function p2=normalize(p)
  p2=p/norm(p);
endfunction;

# transform 2 vector
function v=r2a(R)
  v = [0;0;0];
  s = norm(diag(R));
  if(s < 1e-6)
    v(1) = atan2(R(3,2),R(3,3));
    v(2) = atan2(-R(3,1),s);
    v(3) = atan2(R(2,1),R(1,1));
  else
    v(1) = atan2(-R(2,3),R(2,2));
    v(2) = atan2(-R(3,1), s);
  endif
endfunction

# TODO check t2v
function v=t2v(T)
  v = zeros(6,1);
  v(1:3) = T(1:3,4);
  v(4:6) = r2a(T(1:3,1:3));
endfunction

#from 3D similiarity 3D to 7d vector
function v=s2v(S)
  v = zeros(7,1);
	v(1:6) = t2v(S); # not working as expected
	v(7) = log(S(4,4));
endfunction

# jacobian of the normalize function (it's square)
function J = J_normalize(p)
  n=norm(p);
  in=1./n;
  in3=in**3;
  J = eye(size(p,1))*in - in3*p*p';
endfunction;

# cartesian2polar:
# p: coordinates in cartesian space
# p_polar: coordinates in polar space (range, azimuth, elevation)
function p_polar = c2p(p)
  n2_xy=p(1)^2+p(2)^2;  # x^2+y^2
  n2 = n2_xy+p(3)^2;    # x^2+y^2+z^2
  n = sqrt(n2);         # norm(p)
  n_xy = sqrt(n2_xy);   # norm(p(1:2)
  p_polar =  [n;
              atan2(p(2),p(1));
              atan2(p(3), n_xy)
             ];
endfunction;

function J = J_c2p(p)
  n2_xy=p(1)^2+p(2)^2;
  n2 = n2_xy+p(3)^2;
  n = sqrt(n2);
  n_xy = sqrt(n2_xy);
  n32_xy = n_xy**3;
  
  J=zeros(3,3);
  J(1,:)=(1/n)*p';
  J(2,1:2)= 1./n2_xy * [-p(2) p(1)];
  J(3,:) = (n2_xy/n2) * [
                           -p(3)*p(1)/n32_xy;
                           -p(3)*p(2)/n32_xy;
                           1./n_xy];
endfunction;


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



#computes the trajectory of the robot by chaining up
#the incremental movements of the odometry vector
#U:	a Nx3 matrix, each row contains the odoemtry ux, uy utheta
#T:	a Nx3 matrix, each row contains the robot position (starting from 0,0,0)

function T=compute_odometry_trajectory(U)
	T=zeros(size(U,1),3);
	current_T=v2t2d(zeros(1,3));
	for i=1:size(U,1),
		u=U(i,1:3)';
		# this is P in the slides (thank bart)

		# HINT : current_T is the absoulute pose obtained by concatenating 
		# all the relative transformation until the current timestamp i
		current_T *= v2t2d(u);
		T(i,1:3)=t2v2d(current_T)';
	endfor

endfunction

function [XR, XL]=boxPlus(Xr, XL, num_poses, num_landmarks, dx)
  global pos_dim;
  global landmark_dim;
  for(pose_index=1:num_poses)
    pose_matrix_index=poseMatrixIndex(pose_index,
                                      num_poses,
                                      num_landmarks);

    dxr=dx(pose_matrix_index:pose_matrix_index+pos_dim-1);
    vect = t2v2d(v2t2d(dxr)*v2t2d(Xr(:,pose_index)));
    XR(:,pose_index)=[vect(1); vect(2);0;0;0; vect(3)];
    
  endfor;
  for(landmark_index=1:num_landmarks)
    landmark_matrix_index=landmarkMatrixIndex(landmark_index,
                                                  num_poses,
                                                  num_landmarks);
    dxl=dx(landmark_matrix_index:landmark_matrix_index+landmark_dim-1,:);
    XL(1:end-1,landmark_index)+=dxl;
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
# Input: odometry poses
# Output: tensor in which each element is the relative position measured 
# (1^st column relative position of the 2^nd pose w.r.t the 1^st one and so on)
function robot_measurement = odometry_measure(odometry_pose);
    for columns = 1:(size(odometry_pose)(2) - 1)
        Xi = v2t(odometry_pose(:,columns));
        Xj = v2t(odometry_pose(:,columns+1));
        robot_measurement(:,:,columns) = inv(Xi)*Xj; 
    endfor
endfunction
