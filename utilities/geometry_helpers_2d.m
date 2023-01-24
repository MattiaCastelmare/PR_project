% computes the pose 2d pose vector v from an homogeneous transform A
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function v=t2v(A)
	v(1:2, 1)=A(1:2,3);
	v(3,1)=atan2(A(2,1),A(1,1));
end

% computes the homogeneous transform matrix A of the pose vector v
% A:[ R t ] 3x3 homogeneous transformation matrix, r translation vector
% v: [x,y,theta]  2D pose vector
function A=v2t(v)
  	c=cos(v(3));
  	s=sin(v(3));
	A=[c, -s, v(1) ;
	s,  c, v(2) ;
	0   0  1  ];
end


% normalizes and angle between -pi and pi
% th: input angle
% o: output angle
function o = normalizeAngle(th)
	o = atan2(sin(th),cos(th));
end

#computes the derivative of atan2(p.y/p.x);
function J = J_atan2(p)
  n2=p'*p;
  J= 1./n2 * [-p(2) p(1)];
endfunction

#computes the trajectory of the robot by chaining up
#the incremental movements of the odometry vector
#U:	a Nx3 matrix, each row contains the odoemtry ux, uy utheta
#T:	a Nx3 matrix, each row contains the robot position (starting from 0,0,0)

function T=compute_odometry_trajectory(U)
	T=zeros(size(U,1),3);
	current_T=v2t(zeros(1,3));
	for i=1:size(U,1),
		u=U(i,1:3)';
		# this is P in the slides (thank bart)

		# HINT : current_T is the absoulute pose obtained by concatenating 
		# all the relative transformation until the current timestamp i
		current_T *= v2t(u);
		T(i,1:3)=t2v(current_T)';
	end
end