function    [U,idx,latError ] = ...
            mpc_control(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, NC, NP) ;
        
%% MPC预设参数
NX = 3;         % 状态量的个数
NU = 2;         % 控制量的个数
row = 10;       % 松弛因子
Q = 100*eye(NP*NX);      % (Np*Nx) × (Np*Nx)
R = 1*eye(NC*NU);        % (Nc*Nu) × (Nc*Nu)

% 控制量约束条件
umin = [0.02; -3.14/2];
umax = [1; 3.14/2];


%% 原运动学误差状态空间方程的相关矩阵
% 计算参考控制量
idx = calc_target_index(x, y, ref_x, ref_y); 
x_r = ref_x(idx);
y_r = ref_y(idx);
fai_r = ref_fai(idx);
v_r = ref_v(idx);
w_r = ref_w(idx);

%% 求位置、航向角的误差;根据百度Apolo，计算横向误差
x_error  = x - x_r;
y_error = y - y_r;
latError = y_error*cos(fai_r) - x_error*sin(fai_r);

%% 实际状态量与参考状态量
X_real = [x; y; fai];
X_r = [x_r; y_r; fai_r];

X_piao = X_real - X_r; 

%% a,b两个矩阵
A_piao = [1    0   -v_r*sin(fai_r)*T;
     0    1   v_r*cos(fai_r)*T;
     0    0   1];
 
B_piao = [cos(fai_r)*T     0;
     sin(fai_r)*T     0;
     0                T];

%% 状态空间方程的相关矩阵

% PHI矩阵
PHI_cell = cell(NP,1);
for i = 1:NP
    PHI_cell{i,1}=A_piao^i;  % Nx × (Nx+Nu)
end
PHI = cell2mat(PHI_cell);   % (Nx * Np) × (Nx + Nu)

% THETA矩阵
THETA_cell = cell(NP,NC);
for i = 1:NP
    for j = 1:NC
        if j <= i
            THETA_cell{i,j} = A_piao^(i-j)*B_piao;    % Nx × Nu
        else
            THETA_cell{i,j} = zeros(NX,NU);
        end
    end
end
THETA = cell2mat(THETA_cell);                 % (Nx * Np) × (Nu * Nc)


%% 二次型目标函数的相关矩阵

% H矩阵
H_cell{1,1} = THETA'*Q*THETA + R;  % (Nu * Nc) × (Nu * Nc)

H = cell2mat(H_cell);            % (Nu * Nc + 1) × (Nu * Nc + 1)

% E矩阵
E = PHI * X_piao;                    % (Nx * Np) × 1

% g矩阵
g_cell = cell(1,1);
g_cell{1,1} = E'*Q*THETA;          % (Nu * Nc ) × 1，行数为了和H的列数匹配，新添加一列0
g = cell2mat(g_cell);              % (Nu * Nc + 1 ) × 1

%% 约束条件的相关矩阵

% 控制量与控制量变化量的约束
Umin = kron(ones(NC,1),umin);
Umax = kron(ones(NC,1),umax);
% delta_Umin = kron(ones(Nc,1),delta_umin);
% delta_Umax = kron(ones(Nc,1),delta_umax);

% △U的上下界约束
lb = Umin;
ub = Umax;

%% 开始求解过程

options = optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
delta_U = quadprog(H,g,[],[],[],[],lb,ub,[],options);   %(Nu * Nc +1) × 1

%% 计算输出

% 只选取求解的delta_U的第一组控制量。注意：这里是v_tilde的变化量和Delta_tilde的变化量
delta_v_tilde = delta_U(1);
delta_w_tilde = delta_U(2);

% 更新这一时刻的控制量。注意，这里的“控制量”是v_tilde和Delta_tilde，而不是真正的v和Delta
U(1) = v_r + delta_v_tilde; 
U(2) = w_r + delta_w_tilde; 


end