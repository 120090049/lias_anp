%  function [R_first,R_est_noise_new, t_first,t_est_noise_Gau] = final_algorithm(p_w, d_noise, Var_noise_d, Var_noise_theta, tan_theta_noise,cos_theta_noise)
function [t_est_noise_Gau, R_est_noise_new] = compute_t_R_p(p_w, p_si, Var_noise_d, Var_noise_theta)
    % 先单独对t进行高斯牛顿迭代，再由整体函数对R和t进行高斯牛顿迭代
 
    tan_theta_noise = p_si(2,:)./p_si(1,:);
    theta_noise = atan(tan_theta_noise);
    cos_theta_noise = cos(theta_noise);
    d_noise = p_si(1,:)./cos_theta_noise;
    % 计算 t 的初始估计值
    num_points = size(p_w, 2);
    
    % 构造矩阵 A
    A_noise = [-2 * p_w', ones(num_points, 1)];
    
    % 构造向量 b
    b_noise = (d_noise.^2 - vecnorm(p_w, 2, 1).^2 - Var_noise_d)';
    
    % 求解 x = [t^T ||t||]^T
    x_noise = (A_noise' * A_noise) \ (A_noise' * b_noise);
    t_est_noise = x_noise(1:3);
    
    % 高斯牛顿迭代
%     residuals = d_noise.^2 - vecnorm(p_w - t_est_noise, 2, 1).^2 - Var_noise_d;
%     J = 2 * (p_w - t_est_noise)';
%     
%     delta_t = (J' * J) \ (J' * residuals');
%     t_est_noise_Gau = t_est_noise - delta_t;

    residuals = vecnorm(p_w - t_est_noise, 2, 1)-d_noise;
    J =  ( t_est_noise - p_w )'./vecnorm(p_w - t_est_noise, 2, 1)';
    delta_t = (J' * J) \ (J' * residuals');
    t_est_noise_Gau = t_est_noise - delta_t;

%     t_est_noise_Gau = t_est_noise;

    % 估计 R
    % 构造矩阵 B
    B_noise = [tan_theta_noise' .* (p_w - t_est_noise_Gau)', -(p_w - t_est_noise_Gau)'];
    
    temp_B_noise = B_noise' * B_noise / num_points;
    
    % 修正矩阵
    temp_matrix = zeros(3, 3);
    for i = 1:num_points
        temp_matrix = temp_matrix + (p_w(:, i) - t_est_noise_Gau) * (p_w(:, i) - t_est_noise_Gau)';
    end
    temp_matrix = temp_matrix * Var_noise_theta / num_points;
    
    modify_matrix = zeros(6, 6);
    modify_matrix(1:3, 1:3) = temp_matrix;
    
    temp_B_modify = temp_B_noise - modify_matrix;
    
    % 求矩阵 temp_B 的特征值和特征向量
    [C_modify, D_modify] = eig(temp_B_modify);
    
    % 解有两种
    
    R_est_noise_1 = [sqrt(2) * C_modify(1:3, 1)'; sqrt(2) * C_modify(4:6, 1)'];
    R_est_noise_2 = [-sqrt(2) * C_modify(1:3, 1)'; -sqrt(2) * C_modify(4:6, 1)'];
    
    % 判断最终解
    p_s_est_noise_1 = R_est_noise_1 * (p_w(:, 1) - t_est_noise_Gau);
    
    if p_s_est_noise_1(1) * cos_theta_noise(1) > 0
        R_est_noise = R_est_noise_1;
    else
        R_est_noise = R_est_noise_2;
    end


    R_est_noise(3, :) = cross(R_est_noise(1, :), R_est_noise(2, :));
%     
    R_est_noise_new = R_est_noise;

     if ~isRotationMatrix(R_est_noise_new)
        R_est_noise_new = ToRotationMatrix(R_est_noise_new);
     else
        R_est_noise_new = R_est_noise_new;
    end

    

%     % 判断是否为旋转矩阵
%     if ~isRotationMatrix(R_est_noise)
%         R_est_noise_new = ToRotationMatrix(R_est_noise);
%     else
%         R_est_noise_new = R_est_noise;
%     end


    %             ------------------------------
    %         高斯-牛顿迭代优化 R
    %         ------------------------------

    e_1 = [1 0 0]';
    e_2 = [0 1 0]';
% 
%     for i = 1:num_points
%         %残差
%         Residuals_R(i,1) = tan_theta_noise(1,i)-(e_2'*R_est_noise_new*(p_w(:,i)-t_est_noise_Gau))/(e_1'*R_est_noise_new*(p_w(:,i)-t_est_noise_Gau));
%     
%         J_phi = [0,0,0;
%                  0,0,1;
%                  0,-1,0;
%                  0,0,-1;
%                  0,0,0;
%                  1,0,0;
%                  0,1,0;
%                  -1,0,0;
%                  0,0,0];
%         ukronR = kron((p_w(:,i)-t_est_noise_Gau)',R_est_noise_new);
%         g = e_2'*kron((p_w(:,i)-t_est_noise_Gau)',eye(3))*vec(R_est_noise_new);
%         h = e_1'*kron((p_w(:,i)-t_est_noise_Gau)',eye(3))*vec(R_est_noise_new);
% 
%         %雅可比矩阵
%         J_R(i,1:3) = -(h*e_2'-g*e_1')*ukronR*J_phi/h^2;
%         J_R(i,4:6) = (h*e_2'*R_est_noise_new-g*e_1'*R_est_noise_new)/h^2;
%     
%     end
% 
%     temp_result = [0;0;0;t_est_noise_Gau] - inv(J_R'*J_R)*J_R'*Residuals_R;
% 
%     t_est_noise_Gau = temp_result(4:6);
%     s_new = temp_result(1:3);
%     s_matrix = [0,-s_new(3),s_new(2);
%                 s_new(3),0,-s_new(1);
%                 -s_new(2),s_new(1),0];



    for i = 1:num_points
        %残差
        Residuals_R(2*i-1,1) = d_noise(1,i)-norm(p_w(:,i)-t_est_noise_Gau);
        Residuals_R(2*i,1) = tan_theta_noise(1,i)-(e_2'*R_est_noise_new*(p_w(:,i)-t_est_noise_Gau))/(e_1'*R_est_noise_new*(p_w(:,i)-t_est_noise_Gau));
    
        J_phi = [0,0,0;
                 0,0,1;
                 0,-1,0;
                 0,0,-1;
                 0,0,0;
                 1,0,0;
                 0,1,0;
                 -1,0,0;
                 0,0,0];
        ukronR = kron((p_w(:,i)-t_est_noise_Gau)',R_est_noise_new);
        g = e_2'*kron((p_w(:,i)-t_est_noise_Gau)',eye(3))*vec(R_est_noise_new);
        h = e_1'*kron((p_w(:,i)-t_est_noise_Gau)',eye(3))*vec(R_est_noise_new);

        %雅可比矩阵
        J_R(2*i-1,1:3) = [0,0,0];
        J_R(2*i-1,4:6) = (p_w(:,i)-t_est_noise_Gau)'/norm(p_w(:,i)-t_est_noise_Gau);
        J_R(2*i,1:3) = -(h*e_2'-g*e_1')*ukronR*J_phi/h^2;
        J_R(2*i,4:6) = (h*e_2'*R_est_noise_new-g*e_1'*R_est_noise_new)/h^2;
    
    end
t_first=t_est_noise_Gau;
    temp_result = [0;0;0;t_est_noise_Gau] - inv(J_R'*J_R)*J_R'*Residuals_R;

    t_est_noise_Gau = temp_result(4:6);
    s_new = temp_result(1:3);
    s_matrix = [0,-s_new(3),s_new(2);
                s_new(3),0,-s_new(1);
                -s_new(2),s_new(1),0];

   
R_first=R_est_noise_new;
    R_est_noise_new = R_est_noise_new*expm(s_matrix);
end