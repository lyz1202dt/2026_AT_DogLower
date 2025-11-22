%注意：声明竖直向下为x正方向

%声明狗腿关键几何参数
syms l0 l1 l2 l3 d2
%声明狗腿关节角度
syms q1 q2 q3
%声明每个连杆的m*g的（重力）
syms G1 G2 G3
%声明每个连杆的质心
syms x1 y1 z1 x2 y2 z2 x3 y3 z3

A=[1, 0, 0,  0;
   0, 1, 0,  0;
   0, 0, 1, l0;
   0, 0, 0,  1];

B=[cos(q1), -sin(q1), 0, 0;
   sin(q1),  cos(q1), 0, 0;
         0,        0, 1, 0;
         0,        0, 0, 1];

C=[1, 0, 0,  0;
   0, 1, 0,  0;
   0, 0, 1, l1;
   0, 0, 0,  1];

D=[0 , -1 , 0 , 0;
   0 , 0 , -1 , 0;
   1 , 0 , 0 , 0 ;
   0 , 0 , 0 , 1 ];

E=[cos(q2), -sin(q2), 0, 0;
   sin(q2),  cos(q2), 0, 0;
         0,        0, 1, 0;
         0,        0, 0, 1];

F=[1, 0, 0, l2;
   0, 1, 0,  0;
   0, 0, 1, d2;
   0, 0, 0,  1];

G=[cos(q3), -sin(q3), 0, 0;
   sin(q3),  cos(q3), 0, 0;
      0,        0, 1, 0;
      0,        0, 0, 1];

H=[1, 0, 0, l3;
  0, 1, 0,  0;
  0, 0, 1,  0;
  0, 0, 0,  1];

I=[0;
    0;
    0;
    1];

K=A*B*C*D*E*F*G*H*I;

fprintf("末端位置:\n");
P=[K(1);K(2);K(3)];
%输出末端位置向量
disp(P);

fprintf("雅可比矩阵:\n");
J=jacobian(P,[q1,q2,q3]);
%输出雅可比速度映射矩阵
disp(J);

gv1=[x1;y1;z1;1];
gv2=[x2;y2;z2;1];
gv3=[x3;y3;z3;1];

M1=G1*A*B*gv1;
M2=G2*A*B*C*D*E*gv2;
M3=G3*A*B*C*D*E*F*G*gv3;

V=-(M1(1)+M2(1)+M3(1));
%输出系统总势能表达式
fprintf("系统总势能:\n");
disp(V);


T=[diff(V,q1);diff(V,q2);diff(V,q3)];
T_expr=expand(T);
%输出电机力矩表达式
fprintf("关节力矩:\n");

disp(T_expr);

syms p1 p2 p3 p4 p5 p6 p7 p8 p9 p10

pi=[p1,p2,p3,p4,p5,p6,p7,p8,p9,p10];
opi=[G1*x1,G1*y1,G2*x2,G2*y2,G2*z2,G3*x3,G3*y3,G3*z3,G3*d2,G3*l2];


%符号代换
T_sub=subs(T_expr,opi,pi);

fprintf("代换后的表达式:\n");
disp(T_sub)

%构建线性回归矩阵（初步，会有线性相关项）
Y=jacobian(T_sub,pi);

groups={};
used = false(1,length(pi));
for i=1:length(pi)
    if ~used(i)
        g = i;
        for j=i+1:length(pi)
            if isequal(simplify(Y(:,i) - Y(:,j)), sym(zeros(size(Y,1),1)))
                g(end+1) = j;
                used(j) = true;
            end
        end
        groups{end+1} = g;
    end
end

% 定义新的最小基参数符号
pi_min = sym('pi', [length(groups),1]);

% 替换等价参数
T_min = T_sub;
for k = 1:length(groups)
    for idx = groups{k}
        T_min = subs(T_min, pi(idx), pi_min(k));
    end
end

Y_min = jacobian(T_min, pi_min);

fprintf("回归矩阵:\n");
disp(Y_min);


fprintf("参数关系：\n");
for k = 1:length(groups)
    fprintf('pi_min(%d) = ', k);
    for idx = groups{k}
        fprintf('%s ', char(pi(idx)));
    end
    fprintf('\n');
end

fprintf("数学运算关系:\n")
pi_min_expr = sym(zeros(length(groups),1));
for k = 1:length(groups)
    pi_min_expr(k) = sum(pi(groups{k}));
end
disp(pi_min_expr)



[filename, pathname] = uigetfile('*.csv', '请选择数据文件(第一列力矩；第二列弧度)');
if isequal(filename,0) || isequal(pathname,0)
    disp('用户取消选择文件');
    return;
end
fullpath = fullfile(pathname, filename);
%读取csv中的数据（第一列为力矩，第二列为关节弧度）
M = readmatrix(fullpath);
torque=M(:,1);
rad=M(:,2);

num_data = size(M,1);
Y_value = [];
for k = 1:num_data
    % 获取当前测量的角度
    q_vals = num2cell(q_data(k,:));
    
    % 代入角度计算数值回归矩阵
    Yk = double(subs(Y_min, [q1,q2,q3], q_vals));
    
    % 堆叠回归矩阵（3行对应3个关节）
    Y_value = [Y_value; Yk];
end

pi_value = Y_value \ torque;

fprintf("辨识结果(pi_min):\n");
disp(pi_value);
