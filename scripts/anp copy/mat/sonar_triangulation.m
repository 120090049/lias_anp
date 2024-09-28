% T1=[R1 t1;0 1] is the first sonar pose w.r.t. the world frame, T2=[R2 t2;0 1] is the second sonar pose
% mea1=[d1,theta1] is the measurement of the first sonar, mea2=[d2,theta2] is the measurement of the second sonar
% x is the estimated coordinates of the 3D point in the first sonar frame 
% the unit of d1 and d2 is meter, the unit of theta1 and theta2 is rad

function x=sonar_triangulation(T1,T2,d1,theta1,d2,theta2)
% Ts=[cal_rotation(0,25,0) [1 0 0]';0 0 0 1];
T=inv(T2)*(T1);
R=T(1:3,1:3);
t=T(1:3,4);
r1=R(1,:);
r2=R(2,:);
A1=[tan(theta1) -1 0; tan(theta2)*r1-r2;t'*R];
b1=[0; t(2)-tan(theta2)*t(1); (d2^2-d1^2-norm(t)^2)/2];

x=inv(A1)*b1;

A2=[A1;x'];
b2=[b1;d1^2];

x=inv(A2'*A2)*A2'*b2;