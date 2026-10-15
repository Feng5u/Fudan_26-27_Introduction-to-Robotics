function [alpha, beta, gamma, alpha_, beta_, gamma_] = zyx_euler_inverse(R)
% 给定旋转矩阵 R，反解 alpha，beta 和 gamma，角度限制在 (-180, 180]
% 非奇异情况下，给出两组解 [alpha, beta, gamma] 以及 [alpha_, beta_, gamma_]
% 奇异情况下，给出一组代表解 [alpha, beta, gamma[
% 奇异情况下，beta_ 作为奇异信号，值为 -180, alpha_ 表示 beta_ 符号，gamma 给出自由参数关系

eps = 1e-9;
if norm(R'*R - eye(3), 'fro') > eps
    error('不是正交矩阵');
end
if abs(det(R) - 1) > eps
    error('行列式不为 1');
end

beta = rad2deg(asin(-R(3,1)));

if (abs(abs(R(3,1)) - 1) < eps)
    beta_ = -180;
    if (abs(R(3, 1) + 1) < eps)
        gamma = 0;
        alpha = rad2deg(atan2(-R(1,2), R(2,2)));
        alpha_ = 1;
        gamma_ = alpha - gamma;
    else
        gamma = 0;
        alpha = rad2deg(atan2(-R(1,2), R(2,2)));
        alpha_ = -1;
        gamma_ = alpha + gamma;
    end
else
    alpha = rad2deg(atan2(R(2,1), R(1,1)));
    gamma = rad2deg(atan2(R(3,2), R(3,3)));
    beta_ = 180 - beta;
    if beta_ > 180
        beta_ = beta_ - 360;
    end
    alpha_ = mod(alpha + 180 + 180, 360) - 180;
    gamma_ = mod(gamma + 180 + 180, 360) - 180;
end

end