% known position error variance and range error variance, obtain initial estimate from GTRS (consider the constraint \|x\|^2) 

function [pos,pos_GN] = CTOA_GTRS_COPY(p,r,sig_p,sig_r)  % p is n脳m, r is m脳1

[n,m]=size(p);
A=zeros(m,n+1);
b=zeros(m,1);
for i=1:m
    A(i,:)=[-2*p(:,i)' 1];
    b(i)=r(i)^2-norm(p(:,i))^2+3*sig_p;
end
bar_p=sum(p,2)/m;


ATA_be=A'*A/m-4*sig_p*diag([1 1 1 0]);
ATb_be=A'*b/m-4*sig_p*[bar_p;0]+2*sig_r*[bar_p;-0.5];
D=blkdiag(eye(n),0);
g=[zeros(n,1);-0.5];

eig_lambda=eig(D,ATA_be);
lambda=-1/max(eig_lambda);

%找二分法的初始左端点
lambda_l=lambda+2;
y_l=(ATA_be+lambda_l*D)\(ATb_be-lambda_l*g);
while (y_l'*D*y_l+2*g'*y_l)<0
    lambda_l=(lambda+lambda_l)/2;
    y_l=(ATA_be+lambda_l*D)\(ATb_be-lambda_l*g);
end


%找二分法的初始右端点
lambda_u=lambda+2;
y_u=(ATA_be+lambda_u*D)\(ATb_be-lambda_u*g);
while (y_u'*D*y_u+2*g'*y_u)>0
    lambda_u=lambda_u+(lambda_u-lambda)*2;
    y_u=(ATA_be+lambda_u*D)\(ATb_be-lambda_u*g);
end

%二分法找最优lambda
while (lambda_u-lambda_l)>0.00001
    lambda_temp=(lambda_u+lambda_l)/2;
    y_temp=(ATA_be+lambda_temp*D)\(ATb_be-lambda_temp*g);
    if (y_temp'*D*y_temp+2*g'*y_temp)>0
        lambda_l=lambda_temp;
    else
        lambda_u=lambda_temp;
    end
end



y=inv(ATA_be+lambda_u*D)*(ATb_be-lambda_u*g);
pos=y(1:n);

%% Gauss-Newton iteration
% J=zeros(n*m+m,n*m+n);
% Re=zeros(n*m+m,1);
% inv_sig_p=1/sqrt(sig_p);
% inv_sig_r=1/sqrt(sig_r);
% for i=1:m
%     Re(n*m+i)=(norm(p(:,i)-pos)-r(i))*inv_sig_r;
% end
% 
% J(1:n*m,1:n*m)=eye(n*m)*inv_sig_p;
% for i=1:m
%     J(n*m+i,n*(i-1)+1:n*i)=((p(:,i)-pos)'/norm(p(:,i)-pos))*inv_sig_r;
%     J(n*m+i,n*m+1:n*m+n)=((pos-p(:,i))'/norm(p(:,i)-pos))*inv_sig_r;
% end
% 
% y=[p(:);pos]-inv(J'*J)*J'*Re;
% pos_GN=y(n*m+1:n*m+n);


J=zeros(m,n);
Re=zeros(m,1);
for i=1:m
    Re(i)=norm(p(:,i)-pos)-r(i);
end

for i=1:m
    J(i,:)=(pos-p(:,i))'/norm(p(:,i)-pos);
end
pos_GN=pos-inv(J'*J/m)*(J'*Re/m);