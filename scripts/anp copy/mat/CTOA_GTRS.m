% known position error variance and range error variance, obtain initial estimate from GTRS (consider the constraint \|x\|^2) 

function pos = CTOA_GTRS(A,b)  % p is n脳m, r is m脳1

ATA_be=A'*A;
ATb_be=A'*b;
D=blkdiag(eye(3),0);
g=[zeros(3,1);-0.5];

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
while (lambda_u-lambda_l)>10^(-15)
    lambda_temp=(lambda_u+lambda_l)/2;
    y_temp=(ATA_be+lambda_temp*D)\(ATb_be-lambda_temp*g);
    if (y_temp'*D*y_temp+2*g'*y_temp)>0
        lambda_l=lambda_temp;
    else
        lambda_u=lambda_temp;
    end
end


y=inv(ATA_be+lambda_u*D)*(ATb_be-lambda_u*g);
pos=y(1:3);
y(4)-norm(pos)^2

