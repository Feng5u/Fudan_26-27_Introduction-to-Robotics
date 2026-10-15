clear;clc;
format short e;

cases = [20, 30, 10;
    25, 90, -45;
    -35, -90, 15];

%% (a) 正解
disp('(a)');
for i = 1:3
    R = zyx_euler(cases(i,1), cases(i,2), cases(i,3));
    disp([norm(R'*R-eye(3),'fro'), abs(det(R)-1), norm(inv(R)-R','fro')]);
end

%% (b) 反解
disp('(b)');
for i = 1:3
    R = zyx_euler(cases(i,1), cases(i,2), cases(i,3));
    [a,b,g,a_,b_,g_] = zyx_euler_inverse(R);
    disp([a,b,g,a_,b_,g_]);
end

%% (c) 闭环
disp('(c)');
for i = 1:3
    R1 = zyx_euler(cases(i,1), cases(i,2), cases(i,3));
    [a,b,g,~,~,~] = zyx_euler_inverse(R1);
    R2 = zyx_euler(a,b,g);
    disp(norm(R1-R2,'fro'));
end

%% (d) 坐标变换
disp('(d)');
A_R_B = zyx_euler(0,30,0);
A_p = A_R_B * [1;0;2];
disp(A_p);

%% (e) 非法矩阵
disp('(e)');
try
    zyx_euler_inverse(diag([1,1,-1]));
catch ME
    disp(ME.message);
end

try
    zyx_euler_inverse([1, 1/5, 0; 0, 1, 0; 0, 0, 1]);
catch ME
    disp(ME.message);
end