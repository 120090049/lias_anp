function [t_s, R_sw] = compute_t_R_test(P_SI, P_W)
    num = size(P_SI, 2);
    d_Noise = zeros(1, num);
    cos_theta_Noise = zeros(1, num);
    sin_theta_Noise = zeros(1, num);
    tan_theta_Noise = zeros(1, num);
    theta_N = zeros(1, num);

    for i = 1:num
        d_Noise(i) = norm(P_SI(:, i));
        cos_theta_Noise(i) = P_SI(1, i) / d_Noise(i);
        sin_theta_Noise(i) = P_SI(2, i) / d_Noise(i);
        tan_theta_Noise(i) = sin_theta_Noise(i) / cos_theta_Noise(i);
        theta_N(i) = atan(tan_theta_Noise(i));
    end

    count = 0;
    Delta_xyz_Noise_my = [];
    Delta_d_Noise_my = [];

    for i = 1:num
        for j = i+1:num
            count = count + 1;
            Delta_xyz_Noise_my = [Delta_xyz_Noise_my; 2 * (P_W(:, j) - P_W(:, i))'];
            Delta_d_Noise_my = [Delta_d_Noise_my; d_Noise(i)^2 - d_Noise(j)^2 - norm(P_W(:, i))^2 + norm(P_W(:, j))^2];
        end
    end

    t_W_Noise_my = inv(Delta_xyz_Noise_my' * Delta_xyz_Noise_my) * Delta_xyz_Noise_my' * Delta_d_Noise_my;

    A_Noise_my = zeros(num, 6);

    for i = 1:num
        A_Noise_my(i, 1) = tan_theta_Noise(i) * (P_W(1, i) - t_W_Noise_my(1));
        A_Noise_my(i, 2) = tan_theta_Noise(i) * (P_W(2, i) - t_W_Noise_my(2));
        A_Noise_my(i, 3) = tan_theta_Noise(i) * (P_W(3, i) - t_W_Noise_my(3));
        A_Noise_my(i, 4) = -(P_W(1, i) - t_W_Noise_my(1));
        A_Noise_my(i, 5) = -(P_W(2, i) - t_W_Noise_my(2));
        A_Noise_my(i, 6) = -(P_W(3, i) - t_W_Noise_my(3));
    end

    [U_Noise_my, S_Noise_my, V_Noise_my] = svd(A_Noise_my);
    r1_Noise_my = sqrt(2) * V_Noise_my(1:3, 6);
    r2_Noise_my = sqrt(2) * V_Noise_my(4:6, 6);

    if abs(dot(r1_Noise_my, r2_Noise_my)) <= 1e-4
        r3_Noise_my = cross(r1_Noise_my, r2_Noise_my);
    else
        [r1_Noise_my, r2_Noise_my] = orthogonalize(r1_Noise_my, r2_Noise_my);
        r3_Noise_my = cross(r1_Noise_my, r2_Noise_my);
        r1_Noise_my = r1_Noise_my / norm(r1_Noise_my);
        r2_Noise_my = r2_Noise_my / norm(r2_Noise_my);
        r3_Noise_my = r3_Noise_my / norm(r3_Noise_my);
    end

    R_Noise_my_1 = [r1_Noise_my'; r2_Noise_my'; r3_Noise_my'];
    R_Noise_my_2 = [r1_Noise_my'; r2_Noise_my'; -r3_Noise_my'];
    R_Noise_my_3 = [-r1_Noise_my'; -r2_Noise_my'; r3_Noise_my'];
    R_Noise_my_4 = [-r1_Noise_my'; -r2_Noise_my'; -r3_Noise_my'];

    P_S_Estimate_my_1 = R_Noise_my_1 * (P_W - t_W_Noise_my);
    P_S_Estimate_my_2 = R_Noise_my_2 * (P_W - t_W_Noise_my);
    P_S_Estimate_my_3 = R_Noise_my_3 * (P_W - t_W_Noise_my);
    P_S_Estimate_my_4 = R_Noise_my_4 * (P_W - t_W_Noise_my);

    cos_theta_vatify_1 = P_S_Estimate_my_1(1, 1) / sqrt(P_S_Estimate_my_1(1, 1)^2 + P_S_Estimate_my_1(2, 1)^2);
    cos_theta_vatify_2 = P_S_Estimate_my_2(1, 1) / sqrt(P_S_Estimate_my_2(1, 1)^2 + P_S_Estimate_my_2(2, 1)^2);
    cos_theta_vatify_3 = P_S_Estimate_my_3(1, 1) / sqrt(P_S_Estimate_my_3(1, 1)^2 + P_S_Estimate_my_3(2, 1)^2);
    cos_theta_vatify_4 = P_S_Estimate_my_4(1, 1) / sqrt(P_S_Estimate_my_4(1, 1)^2 + P_S_Estimate_my_4(2, 1)^2);

    cos_theta_true = P_SI(1, 1) / sqrt(P_SI(1, 1)^2 + P_SI(2, 1)^2);

    if cos_theta_vatify_1 * cos_theta_true > 0
        R_sw = R_Noise_my_1;
    elseif cos_theta_vatify_2 * cos_theta_true > 0
        R_sw = R_Noise_my_2;
    elseif cos_theta_vatify_3 * cos_theta_true > 0
        R_sw = R_Noise_my_3;
    elseif cos_theta_vatify_4 * cos_theta_true > 0
        R_sw = R_Noise_my_4;
    else
        error("No valid R_sw found")
    end

    t_s = -R_sw * t_W_Noise_my;
end

function [r1_Noise_new, r2_Noise_new] = orthogonalize(r1_Noise, r2_Noise)
    angle_Noise_rad = acos(dot(r1_Noise, r2_Noise) / (norm(r1_Noise) * norm(r2_Noise)));
    angle_tran = (pi / 2 - angle_Noise_rad) / 2;
    k = cross(r1_Noise, r2_Noise);
    k = k / norm(k);
    r1_Noise_new = r1_Noise * cos(-angle_tran) + cross(k, r1_Noise) * sin(-angle_tran) + k * dot(k, r1_Noise) * (1 - cos(-angle_tran));
    r2_Noise_new = r2_Noise * cos(angle_tran) + cross(k, r2_Noise) * sin(angle_tran) + k * dot(k, r2_Noise) * (1 - cos(angle_tran));
end
