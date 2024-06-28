function [tz] = cal_tz(R_SW,P_W,t_S,P_SI,tz_ini)

[~,n]=size(P_W);   % n is the number of points
iter_num=10;       % Gauss-Newton iteration number
J=zeros(2*n,1);
Residual=zeros(2*n,1);
tz=tz_ini;
e3=[0 0 1]';

for k=1:iter_num
    t=[t_S;tz];
    for i=1:n
        P_S=R_SW*P_W(:,i)+t;
        J(2*i-1:2*i)=((R_SW*P_W(:,i)+t)'*e3)/(norm(R_SW*P_W(:,i)+t)*norm(P_S(1:2)))*(R_SW(1:2,:)*P_W(:,i)+t_S);
        Residual(2*i-1:2*i)=norm(R_SW*P_W(:,i)+t)/norm(P_S(1:2))*(R_SW(1:2,:)*P_W(:,i)+t_S)-P_SI(:,i);
    end
    tz=tz-(J'*Residual)/(J'*J);
end