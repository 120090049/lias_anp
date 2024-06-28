


% 也可以用 sin(theta) 进行判断
% sin_theta_vatify_1 = P_S_Estimate_my_1(2,1)/sqrt(P_S_Estimate_my_1(1,1)^2+P_S_Estimate_my_1(2,1)^2);
% sin_theta_vatify_2 = P_S_Estimate_my_2(2,1)/sqrt(P_S_Estimate_my_2(1,1)^2+P_S_Estimate_my_2(2,1)^2);
% 
% sin_theta_true = P_SI_Noise(2,1)/sqrt(P_SI_Noise(1,1)^2+P_SI_Noise(2,1)^2);

% 如果 估计的 cos(theta)_1 和 真值 cos(theta) 同号

% % 有两个R可以选择
% R_SW_Noise_my_1 = [r1_Noise_my';r2_Noise_my';r3_Noise_my'];
% R_SW_Noise_my_2 = [-r1_Noise_my';-r2_Noise_my';r3_Noise_my']; 
% 
% % % 验证选择哪一个R比较好
% [k_Noise_my_1, theta_Noise_my_1] = rot2aa(R_SW'*R_SW_Noise_my_1);
% [k_Noise_my_2, theta_Noise_my_2] = rot2aa(R_SW'*R_SW_Noise_my_2);
% 
% 
% % 根据 R_SW 估计声呐坐标系中的坐标 P_S
% P_S_Estimate_my_1 = R_SW_Noise_my_1 * (P_W - t_W_Noise_my);
% P_S_Estimate_my_2 = R_SW_Noise_my_2 * (P_W - t_W_Noise_my);
% 
% % 计算 估计的 cos(theta) 
% cos_theta_vatify_my_1 = P_S_Estimate_my_1(1,1)/sqrt(P_S_Estimate_my_1(1,1)^2+P_S_Estimate_my_1(2,1)^2);
% cos_theta_vatify_my_2 = P_S_Estimate_my_2(1,1)/sqrt(P_S_Estimate_my_2(1,1)^2+P_S_Estimate_my_2(2,1)^2);
% 
% % 计算 真值 cos(theta)
% cos_theta_true = P_SI_Noise(1,1)/sqrt(P_SI_Noise(1,1)^2+P_SI_Noise(2,1)^2);
% 
% %也可以用 sin(theta) 进行判断
% % sin_theta_vatify_1 = P_S_Estimate_my_1(2,1)/sqrt(P_S_Estimate_my_1(1,1)^2+P_S_Estimate_my_1(2,1)^2);
% % sin_theta_vatify_2 = P_S_Estimate_my_2(2,1)/sqrt(P_S_Estimate_my_2(1,1)^2+P_S_Estimate_my_2(2,1)^2);
% % 
% % sin_theta_true = P_SI_Noise(2,1)/sqrt(P_SI_Noise(1,1)^2+P_SI_Noise(2,1)^2);
% 
% 




