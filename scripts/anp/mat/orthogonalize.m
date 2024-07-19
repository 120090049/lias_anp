function [r1_Noise_new, r2_Noise_new] = orthogonalize(r1_Noise,r2_Noise)

%计算两个向量之间的夹角
angle_Noise_rad = acos(dot(r1_Noise,r2_Noise)/(norm(r1_Noise) * norm(r2_Noise)));
%定义各自要旋转的角度
angle_tran = (pi/2 - angle_Noise_rad)/2;
%定义旋转轴
k = cross(r1_Noise,r2_Noise);
% 归一化旋转轴
k = k / norm(k);
% 计算旋转后的向量
r1_Noise_new = r1_Noise * cos(-angle_tran) + cross(k, r1_Noise) * sin(-angle_tran) + k * dot(k, r1_Noise) * (1 - cos(-angle_tran));
r2_Noise_new = r2_Noise * cos(angle_tran) + cross(k, r2_Noise) * sin(angle_tran) + k * dot(k, r2_Noise) * (1 - cos(angle_tran));

end