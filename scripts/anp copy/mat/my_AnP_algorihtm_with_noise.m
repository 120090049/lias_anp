%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%        my_Anp_Algrithm_with_noise
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;
clear;

%生成数据

%随机生成一个单位正交矩阵
% 定义矩阵大小
%n = 3;
% 生成一个随机矩阵
A = randn(3);
% 对A进行QR分解
[Q, R] = qr(A);
% 提取正交部分
Q = Q(:,1:3);
% 验证是否为单位正交矩阵
disp('单位正交矩阵 Q:');
disp(Q);
disp('验证是否为单位正交矩阵:');
disp(norm(Q'*Q  - eye(3)) < eps);



% 给定旋转矩阵R_SW 和 t_S
% R_SW = Q;
R_SW = [-0.5798    0.4836   -0.6557;
       -0.8135   -0.3883    0.4329;
      -0.0453    0.7844    0.6186];
t_S = [6;4;7];
%t_S = [25;34;29];

% 世界坐标数据
% P_W = [3, 4, 2, 1, 2, 7, 3, 6, 2, 8, 15;
%        4, 6, 3, 4, 5, 2, 1, 3, 5, 23, 4;
%        5, 7, 6, 7, 4, 1, 2, 11, 13, 3, 7];

P_W = [30, 41, 21, 13, 23, 73, 35, 66, 72, 82, 15;
       44, 26, 63, 34, 15, 22, 14, 33, 25, 23, 42;
       35, 17, 16, 57, 54, 61, 42, 11, 13, 3, 47];
num = 6
% 在生成 50 个点， 坐标范围在[0,100]之间
% P_W = randn(3,num)*10


% 获取数据的大小
[m, n] = size(P_W);

% 根据世界坐标生成声纳图像数据
for i = 1:n
    P_S(:,i) = R_SW*P_W(:,i)+t_S;  %生成声纳坐标系数据

    d(i,1) = norm(P_S(:,i),2);       %声呐图像中的数据 d

    % 声呐图像数据中的 theta， 计算cos(theta) 和 sin(theta)
    cos_theta(i) = P_S(1,i)/sqrt(P_S(1,i)^2+P_S(2,i)^2);
    sin_theta(i) = P_S(2,i)/sqrt(P_S(1,i)^2+P_S(2,i)^2);
    tan_theta(i) = sin_theta(i)/cos_theta(i);
    theta(i,1) = atan(tan_theta(i));


    % 根据声呐坐标系中的点计算俯仰角 phi  cos(phi), 用于验证和得到声呐图像中的坐标
    cos_phi(i) = sqrt(P_S(1,i)^2+P_S(2,i)^2)/ d(i,1);

    % 生成声呐图像中的坐标，x_SI, y_SI
    P_SI(1,i) = d(i,1)*cos_theta(i);
    P_SI(2,i) = d(i,1)*sin_theta(i);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 数据加噪
% 设置噪声方差
Var_Noise = 0

% 在声呐图像 P_SI 上添加噪声
P_SI_Noise = P_SI +  Var_Noise*randn(2, n);

% 计算噪声声呐图像的值 d 和 theta
for i = 1:n
    d_Noise(i,1) = norm(P_SI_Noise(:,i),2);
    cos_theta_Noise(i) = P_SI_Noise(1,i)/d_Noise(i,1) ;
    sin_theta_Noise(i) = P_SI_Noise(2,i)/d_Noise(i,1) ;
    tan_theta_Noise(i) = sin_theta_Noise(i)/cos_theta_Noise(i);
    theta_N(i,1) = atan(tan_theta_Noise(i));
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%问题求解 加噪 MY
%位置估计 计算t_W_Noise

% 根据每两个世界坐标系中的点计算求解 t_W 所需的系数矩阵 Delta_xyz 和 常数矩阵 Delta_d
count = 0;
for i = 1:n
    for j = i+1:n
        count = count + 1;
        Delta_xyz_Noise_my(count,1) = 2*(P_W(1,j) - P_W(1,i));
        Delta_xyz_Noise_my(count,2) = 2*(P_W(2,j) - P_W(2,i));
        Delta_xyz_Noise_my(count,3) = 2*(P_W(3,j) - P_W(3,i));
        Delta_d_Noise_my(count,1) = d_Noise(i)^2-d_Noise(j)^2 - norm(P_W(:,i),2)^2 + norm(P_W(:,j),2)^2;
    end
end

t_W_Noise_my = inv(Delta_xyz_Noise_my'*Delta_xyz_Noise_my)*Delta_xyz_Noise_my'* Delta_d_Noise_my;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%姿态估计 加噪 计算R_SW
% 根据 n 个世界坐标点，计算求解 R_SW 所需的系数矩阵 A
% for i =1:n
%     A_Noise_my(i,1) = (P_W(1,i)-t_W_Noise_my(1))/P_SI_Noise(1,i);
%     A_Noise_my(i,2) = (P_W(2,i)-t_W_Noise_my(2))/P_SI_Noise(1,i);
%     A_Noise_my(i,3) = (P_W(3,i)-t_W_Noise_my(3))/P_SI_Noise(1,i);
%     A_Noise_my(i,4) = -(P_W(1,i)-t_W_Noise_my(1))/P_SI_Noise(2,i);
%     A_Noise_my(i,5) = -(P_W(2,i)-t_W_Noise_my(2))/P_SI_Noise(2,i);
%     A_Noise_my(i,6) = -(P_W(3,i)-t_W_Noise_my(3))/P_SI_Noise(2,i);
% end
for i =1:n
    A_Noise_my(i,1) = tan_theta_Noise(i)*(P_W(1,i)-t_W_Noise_my(1));
    A_Noise_my(i,2) = tan_theta_Noise(i)*(P_W(2,i)-t_W_Noise_my(2));
    A_Noise_my(i,3) = tan_theta_Noise(i)*(P_W(3,i)-t_W_Noise_my(3));
    A_Noise_my(i,4) = -(P_W(1,i)-t_W_Noise_my(1));
    A_Noise_my(i,5) = -(P_W(2,i)-t_W_Noise_my(2));
    A_Noise_my(i,6) = -(P_W(3,i)-t_W_Noise_my(3));
end

% for i =1:n
%     A_Noise_my(i,1) = tan_theta(i)*(P_W(1,i)-t_W_Noise_my(1));
%     A_Noise_my(i,2) = tan_theta(i)*(P_W(2,i)-t_W_Noise_my(2));
%     A_Noise_my(i,3) = tan_theta(i)*(P_W(3,i)-t_W_Noise_my(3));
%     A_Noise_my(i,4) = -(P_W(1,i)-t_W_Noise_my(1));
%     A_Noise_my(i,5) = -(P_W(2,i)-t_W_Noise_my(2));
%     A_Noise_my(i,6) = -(P_W(3,i)-t_W_Noise_my(3));
% end
% 对矩阵 A 进行 SVD 分解
[U_Noise_my ,S_Noise_my ,V_Noise_my ] = svd(A_Noise_my );

% 取 V 中的最后一列作为解
r1_Noise_my  = sqrt(2)*V_Noise_my (1:3,6);
r2_Noise_my  = sqrt(2)*V_Noise_my (4:6,6);

% 检查 r1 和 r2 的范数大小
nrom_r1_Noise_my = norm(r1_Noise_my);
nrom_r2_Noise_my = norm(r2_Noise_my);


% 检查 r1 和 r2 是否正交
if abs(dot(r1_Noise_my, r2_Noise_my)) <= 1e-4
    disp('向量 r1_Noise_my 和向量 r2_Noise_my 是正交的。');

    % 如果正交，则直接叉乘得到 r3
    r3_Noise_my  = cross(r1_Noise_my ,r2_Noise_my)
else
    disp('向量 r1_Noise_my 和向量 r2_Noise_my 不是正交的。');

    % 否则，将 r1 和 r2 正交
    [r1_Noise_my, r2_Noise_my] = orthogonalize(r1_Noise_my, r2_Noise_my); 
    
    % 验证正交化操作是否生效
    if abs(dot(r1_Noise_my, r2_Noise_my)) <= 1e-4
        disp('向量 r1_Noise_my_new 和向量 r2_Noise_my_new 是正交的。');
    else
        disp('向量 r1_Noise_my_new 和向量 r2_Noise_my_new 不是正交的。');
    end

    % 将r1 和 r2 正交化后，由叉乘得到 r3
    r3_Noise_my  = cross(r1_Noise_my ,r2_Noise_my);

    % 单位化
    r1_Noise_my = r1_Noise_my/norm(r1_Noise_my);
    r2_Noise_my = r2_Noise_my/norm(r2_Noise_my);
    r3_Noise_my = r3_Noise_my/norm(r3_Noise_my);

    % 检查 r1, r2, r3 的范数是否为 1
    nrom_r1_Noise_my = norm(r1_Noise_my);
    nrom_r2_Noise_my = norm(r2_Noise_my);
    nrom_r3_Noise_my = norm(r3_Noise_my);
end
   
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
% % 如果 估计的 cos(theta)_1 和 真值 cos(theta) 同号
% if (cos_theta_vatify_my_1*cos_theta_true > 0 )
%     % R_Noise_my_1 是正确答案
%     R_SW_Noise_my = R_SW_Noise_my_1;
% end
% 
% % 如果 估计的 cos(theta)_2 和 真值 cos(theta) 同号
% if (cos_theta_vatify_my_2*cos_theta_true > 0 ) 
%     % R_Noise_my_2 是正确答案
%     R_SW_Noise_my = R_SW_Noise_my_2;
% end


R_Noise_my_1= [r1_Noise_my';r2_Noise_my';r3_Noise_my'];
R_Noise_my_2= [r1_Noise_my';r2_Noise_my';-r3_Noise_my'];
R_Noise_my_3= [-r1_Noise_my';-r2_Noise_my';r3_Noise_my'];
R_Noise_my_4= [-r1_Noise_my';-r2_Noise_my';-r3_Noise_my'];

[k_Noise_my_1, theta_Noise_my_1] = rot2aa(R_SW'*R_Noise_my_1);
[k_Noise_my_2, theta_Noise_my_2] = rot2aa(R_SW'*R_Noise_my_2);
[k_Noise_my_3, theta_Noise_my_3] = rot2aa(R_SW'*R_Noise_my_3);
[k_Noise_my_4, theta_Noise_my_4] = rot2aa(R_SW'*R_Noise_my_4);

min_error_my = min([theta_Noise_my_1,theta_Noise_my_2,theta_Noise_my_3,theta_Noise_my_4]);

if theta_Noise_my_1 == min_error_my
    R_SW_Noise_my = R_Noise_my_1;
elseif theta_Noise_my_2 == min_error_my
    R_SW_Noise_my = R_Noise_my_2;
elseif theta_Noise_my_3 == min_error_my
    R_SW_Noise_my = R_Noise_my_3;
elseif theta_Noise_my_4 == min_error_my
    R_SW_Noise_my = R_Noise_my_4;
end
% 将 t_W_Noise 转化成 t_S_Noise
t_S_Noise_my = -R_SW_Noise_my*t_W_Noise_my;
t_S_Noise_my
% 计算估计的精度 
[k_Noise_my, theta_Noise_my] = rot2aa(R_SW'*R_SW_Noise_my)
