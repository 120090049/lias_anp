% T1=[R1 t1;0 1] is the first sonar pose w.r.t. the world frame, T2=[R2 t2;0 1] is the second sonar pose
% mea1=[d1,theta1] is the measurement of the first sonar, mea2=[d2,theta2] is the measurement of the second sonar
% x is the estimated coordinates of the 3D point in the first sonar frame 
% the unit of d1 and d2 is meter, the unit of theta1 and theta2 is rad

function x=sonar_triangulation4(R,t,d1,theta1,d2,theta2)
r1=R(1,:);
r2=R(2,:);
A1=[-1 tand(theta1) 0 0; tand(theta2)*r2-r1 0;t'*R 0;0 0 0 1];
b1=[0; t(1)-tand(theta2)*t(2); (d2^2-d1^2-norm(t)^2)/2;d1^2];

D=diag([1 1 1 0]);
g=[zeros(3,1);-0.5];
ATA=A1'*A1;
ATb=A1'*b1;
cvx_begin SDP quiet;
variables t(1) v(1);
minimize(t);
[ATA+v*D ATb-v*g;(ATb-v*g)' t]>=0;
cvx_end;

y=inv(ATA+v*D)*(ATb-v*g);
x=y(1:3);

% A2=[A1;x'];
% b2=[b1;d1^2];
% 
% x=inv(A2'*A2)*A2'*b2;