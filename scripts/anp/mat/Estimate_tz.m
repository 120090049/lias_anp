% TOA localization algorithm
% Input: a (m×n) is the coordinates (a 1×n vector) of m sensors
%        r (m×1) is the range measurements
% Output: pos_est (n×1) is the Bias-Eli solution
%         pos_est_GN (n×1) is the refined solution by a one-step Gauss-Newton iteration
%         var_est is the estimated variance of measurement noises

% Copyright <2022>  <Guangyang Zeng, Biqiang Mu, Jiming Chen, Zhiguo Shi, Junfeng Wu>
% zengguangyang@cuhk.edu.cn, https://guangyangzeng.github.io/ 
% paper info. Global and Asymptotically Efficient Localization From Range Measurements, IEEE Transactions on Signal Processing, 70: 5041-5057, 2022.



function [t_z]=Estimate_tz(R_SW, P_W,P_S,t_S_Noise_He,P_SI) 

[m, n] = size(P_W);

t_z = 100;
ans = []

for i=1:2
    
    while(1)
    t = [t_S_Noise_He ; t_z];
    J = (R_SW*P_W(: ,i) + t)'*[0; 0; 1]/(norm(R_SW*P_W(:,i) + t)*norm(P_S(1:2,i)))*R_SW(1:2,:)*P_W(:,i);
    Res = norm(R_SW*P_W(:,i) + t)/norm(P_S(1:2,i))*R_SW(1:2,:)*P_W(:,i) + t_S_Noise_He - P_SI(:,i);
    temp = t_z - inv(J'*J)*J'*Res;
    if abs(temp - t) < exp(-5)
        t_z = temp;
        break;
    end
    t_z = temp;
    end
    
    ans = [ans;t_z]

end

t_z = norm(ans);



