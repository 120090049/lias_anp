function [R_Noise_He_app, t_S_Noise_He_app] = Wang_app_algorithm_2(p_si_noise, p_w,t_s_true,Var_noise_d)
    % 将第一个点作为原点
   t_S_Noise_He_app = t_s_true(1:2,:) + (Var_noise_d)*randn(2, 1);
%      t_S_Noise_He_app = t_s_true(1:2,:);
%    t_S_Noise_He_app = p_si_noise(:,1);


    A_app = [];
    b_app = [];
    for i = 2:length(p_w)
        A_app = [A_app ; [p_w(:,i)', 0, 0, 0; 0, 0, 0, p_w(:,i)']];
        b_app = [b_app ; p_si_noise(:,i) - t_S_Noise_He_app];
    end

    r_app = inv(A_app'*A_app) * A_app' * b_app;

    r_app(1:3,:) = r_app(1:3,:)/norm(r_app(1:3,:));
    r_app(4:6,:) = r_app(4:6,:)/norm(r_app(4:6,:));

    % 计算矩阵的秩
    rank_A = rank(A_app);
    min_dim_A = min(size(A_app));

    % 判断是否满秩
    if rank_A ~= min_dim_A
        disp('矩阵A_app不是满秩的');

        [U_Noise_He_app, S_Noise_He_app, V_Noise_He_app] = svd(A_app);
        v_1_1 = V_Noise_He_app(1:3, 5);
        v_1_2 = V_Noise_He_app(4:6, 5);
        v_2_1 = V_Noise_He_app(1:3, 6);
        v_2_2 = V_Noise_He_app(4:6, 6);

        % 构造矩阵F
        F = zeros(3, 3);
        F(1,1) = v_1_1' * v_1_1;
        F(1,2) = v_2_1' * v_2_1;
        F(1,3) = 2 * v_1_1' * v_2_1;
        F(2,1) = v_1_2' * v_1_2;
        F(2,2) = v_2_2' * v_2_2;
        F(2,3) = 2 * v_1_2' * v_2_2;
        F(3,1) = v_1_1' * v_1_2;
        F(3,2) = v_2_1' * v_2_2;
        F(3,3) = v_1_1' * v_2_2 + v_1_2' * v_2_1;

        c = [1; 1; 0];

        % 定义目标函数
        objective = @(x) norm(F * [x(1)^2; x(2)^2; x(1) * x(2)] - c, 2)^2;

        % 定义初始猜测值
        x0 = [0.5; 0.5];

        % 使用 fminunc 进行非线性优化求解
        options_a = optimoptions('fminunc', 'Algorithm', 'quasi-newton', 'Display', 'off');
        [x_opt, fval] = fminunc(objective, x0, options_a);
        alpha_1 = x_opt(1);
        alpha_2 = x_opt(2);

        r_app = r_app + alpha_1 * V_Noise_He_app(:, 5) + alpha_2 * V_Noise_He_app(:, 6);
    end

    R_Noise_He_app = zeros(3, 3);
    R_Noise_He_app(1, 1:3) = r_app(1:3);  
    R_Noise_He_app(2, 1:3) = r_app(4:6); 
    R_Noise_He_app(3, 1:3) = cross(r_app(1:3), r_app(4:6)); 

    % 判断是否为旋转矩阵并进行矫正
    if ~isRotationMatrix(R_Noise_He_app)
        R_Noise_He_app = ToRotationMatrix(R_Noise_He_app);
    end

%     % 估计 tz
%     tz_init = 0.0;  % 初始值
%     phi_max = 7 / 180 * pi;   % 定义 phi_max (根据你的需求)
% 
%     tx = t_S_Noise_He_app(1,1);
%     ty = t_S_Noise_He_app(2,1);
% 
%     x3d = P_W;
%     x2d = P_SI_Noise;
% 
%     R_c2n = R_Noise_He_app;
% 
%     % 非线性约束
%     nonlcon = @(tz) deal([-phi_max - con_1(tz, R_c2n, tx, ty, x3d); con_1(tz, R_c2n, tx, ty, x3d) - phi_max], []);
% 
%     % 优化求解
%     options_tz = optimoptions('fmincon', 'Algorithm', 'sqp','Display', 'off');
%     t_S_Noise_He_app(3,:) = fmincon(@(tz) func_z_norm(tz, R_c2n, tx, ty, x3d, x2d), tz_init, [], [], [], [], [], [], nonlcon, options_tz);
    
    t_z =  cal_tz(R_Noise_He_app,p_w,t_S_Noise_He_app,p_si_noise,0);
    t_S_Noise_He_app = [t_S_Noise_He_app; t_z];

    t_S_Noise_He_app = -R_Noise_He_app'*t_S_Noise_He_app;
end