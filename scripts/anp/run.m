% MATLAB script to test compute_t_R function

% Initialize parameters
clc
clear
point_num = 50;
P_W = randn(3,point_num)*5 + randn(3,point_num)*5;
% % 
% P_W = [4.01398693, 4.14420807, 4.32659229, 3.57898031, 4.62524004, ...
%        2.53439912, 2.37290791, 2.92203137, 4.74740917, 4.4339805, ...
%        4.65539456, 2.69388094, 5.05178386, 2.2541195, 2.8000855, ...
%        4.553101, 4.0377846, 3.54886843, 3.15894055, 3.86783644, ...
%        3.225728, 4.81897145, 2.97160881, 2.66377559, 4.14797142, ...
%        5.04479198, 4.91589477, 3.80825367, 2.33517812; 
% 
%        3.32507861, 2.98633423, 0.36589002, 1.76739533, 3.40097916, ...
%        1.94232965, 1.42191891, -0.59761111, -0.97792378, 2.89282256, ...
%        -1.10248539, -0.27824162, -0.71949274, 0.4283563, 2.29046146, ...
%        1.71617653, 0.45081748, -0.02527929, -0.57252434, -0.10329507, ...
%        2.70341004, -0.19826834, -0.45016959, 0.11943769, 1.48413903, ...
%        -0.38693323, 0.03323147, -0.87652375, 2.02522025; 
% 
%        0.00793917, 0.11863106, 0.20218476, 0.45847291, 0.4032435, ...
%        0.06235353, 0.34978184, -0.11087785, -0.38486538, 0.49727416, ...
%        -0.50759619, -0.37549141, -0.51327746, 0.37311051, 0.11160917, ...
%        0.14894067, 0.4736446, -0.13407187, 0.36255587, 0.45950438, ...
%        0.50665149, -0.10995821, 0.20389852, 0.42015753, -0.28143116, ...
%        -0.17127721, -0.17202155, -0.28689548, -0.04151965];

R_SW = [0.98007119, 0.19820539, 0.0132322; 
        -0.19686029, 0.96018782, 0.19820539; 
        0.02657998, -0.19686029, 0.98007119];

t_S = [-1.99112736; 0.13173626; -0.49556368];

P_S_est = R_SW*(P_W - t_S);

d = vecnorm(P_S_est);
tan_theta = P_S_est(2,:)./P_S_est(1,:);
temp = vecnorm(P_S_est(1:2,:));
cos_theta = P_S_est(1,:)./vecnorm(P_S_est(1:2,:));
sin_theta = P_S_est(2,:)./vecnorm(P_S_est(1:2,:));
theta_est = atan(tan_theta);
P_SI(1,:) = d.*cos_theta;
P_SI(2,:) = d.*sin_theta;

% [ R_sw_my,t_s_my] = compute_t_R(P_W, P_SI,0,0);
% disp('t_s_my:');
% disp(t_s_my);
% disp('R_sw_my:');
% disp(R_sw_my);

[ R_sw_nonapp,t_s_nonapp] = Wang_nonapp_algorithm(P_W, P_SI, R_SW);
disp('t_s_nonapp:');
disp(t_s_nonapp);
disp('R_sw_nonapp:');
disp(R_sw_nonapp);
% 
% t_s_true = -R_SW*t_S;
% [ R_sw_app,t_s_app] = Wang_app_algorithm( P_W, P_SI, t_s_true ,0);
% disp('t_s_app:');
% disp(t_s_app);
% disp('R_sw_app:');
% disp(R_sw_app);

% % Noise data
% tan_theta_noise = tan_theta + randn() * 0.0005;
% theta_noise = atan(tan_theta_noise);
% d_noise = d + randn() * 0.0002;
% P_SI_Noise(1, :) = d_noise .* cos(theta_noise); % Matlab索引从1开始
% P_SI_Noise(2, :) = d_noise .* sin(theta_noise);
% % Call the compute_t_R function
% [ R_sw_cal,t_s_cal] = compute_t_R(P_W, P_SI_Noise,0,0);
% 
% % Display results
% disp('t_s_cal:');
% disp(t_s_cal);
% 
% disp('R_sw_cal:');
% disp(R_sw_cal);