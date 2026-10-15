function R = zyx_euler(alpha, beta, gamma)
% 给定三个角度值，计算旋转矩阵

a = deg2rad(alpha);
b = deg2rad(beta);
c = deg2rad(gamma);

R_x = [1, 0, 0;
    0, cos(c), -sin(c);
    0, sin(c), cos(c)];
R_y = [cos(b), 0, sin(b);
    0, 1, 0;
    -sin(b), 0, cos(b)];
R_z = [cos(a), -sin(a), 0;
    sin(a), cos(a), 0;
    0, 0, 1];

R = R_z * R_y * R_x;

end