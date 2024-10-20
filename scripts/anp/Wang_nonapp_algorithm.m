function [R_Noise_He_new, t_S_Noise_He] = Wang_nonapp_algorithm(P_W,P_SI_Noise,R_true)
    n = length(P_W);

    % 计算 W_Noise_He 和 H_Noise_He
    W_Noise_He = zeros(n, 2);
    H_Noise_He = zeros(n, 6);
    
    for i = 1:n
        W_Noise_He(i, 1) = -P_SI_Noise(2, i);
        W_Noise_He(i, 2) = P_SI_Noise(1, i);
        H_Noise_He(i, 1:3) = -P_SI_Noise(2, i) * P_W(:, i)';
        H_Noise_He(i, 4:6) = P_SI_Noise(1, i) * P_W(:, i)';
    end

    % 计算矩阵 M_Noise_He
    M_Noise_He = W_Noise_He * inv(W_Noise_He' * W_Noise_He) * W_Noise_He' * H_Noise_He - H_Noise_He;

    % 对 M_Noise_He 进行 SVD 分解
    [U_Noise_He, S_Noise_He, V_Noise_He] = svd(M_Noise_He);

    % 计算 r_1 和 r_2
    r_1 = sqrt(2) * V_Noise_He(:, 6);
    r_2 = -sqrt(2) * V_Noise_He(:, 6);


    
    r = r_1; 
    t_S_Noise_He = inv(W_Noise_He' * W_Noise_He) * W_Noise_He' * H_Noise_He * r_2;
    R_Noise_He = zeros(3, 3);
    R_Noise_He(1, 1:3) = r(1:3)';  
    R_Noise_He(2, 1:3) = r(4:6)'; 
    R_Noise_He(3, 1:3) = cross(r(1:3), r(4:6)); 
    R_Noise_He_new = R_Noise_He';
    
  
    % 判断是否小于阈值
    if norm(R_Noise_He_new - R_true, 'fro') > 0.3
        r = r_2; 
        t_S_Noise_He = inv(W_Noise_He' * W_Noise_He) * W_Noise_He' * H_Noise_He * r_1;
        R_Noise_He = zeros(3, 3);
        R_Noise_He(1, 1:3) = r(1:3)';  
        R_Noise_He(2, 1:3) = r(4:6)'; 
        R_Noise_He(3, 1:3) = cross(r(1:3), r(4:6)); 
        R_Noise_He_new = R_Noise_He';
    end
    
    t_z =  cal_tz(R_Noise_He,P_W,t_S_Noise_He,P_SI_Noise,0);
    t_S_Noise_He = [t_S_Noise_He; t_z];

    t_S_Noise_He = -R_Noise_He_new*t_S_Noise_He;
    R_Noise_He_new = R_Noise_He_new';

end


