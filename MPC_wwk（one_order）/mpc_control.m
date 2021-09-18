function    [U,idx,latError ] = ...
            mpc_control(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, NC, NP) ;
        
%% MPCԤ�����
NX = 3;         % ״̬���ĸ���
NU = 2;         % �������ĸ���
row = 10;       % �ɳ�����
Q = 100*eye(NP*NX);      % (Np*Nx) �� (Np*Nx)
R = 1*eye(NC*NU);        % (Nc*Nu) �� (Nc*Nu)

% ������Լ������
umin = [0.02; -3.14/2];
umax = [1; 3.14/2];


%% ԭ�˶�ѧ���״̬�ռ䷽�̵���ؾ���
% ����ο�������
idx = calc_target_index(x, y, ref_x, ref_y); 
x_r = ref_x(idx);
y_r = ref_y(idx);
fai_r = ref_fai(idx);
v_r = ref_v(idx);
w_r = ref_w(idx);

%% ��λ�á�����ǵ����;���ݰٶ�Apolo������������
x_error  = x - x_r;
y_error = y - y_r;
latError = y_error*cos(fai_r) - x_error*sin(fai_r);

%% ʵ��״̬����ο�״̬��
X_real = [x; y; fai];
X_r = [x_r; y_r; fai_r];

X_piao = X_real - X_r; 

%% a,b��������
A_piao = [1    0   -v_r*sin(fai_r)*T;
     0    1   v_r*cos(fai_r)*T;
     0    0   1];
 
B_piao = [cos(fai_r)*T     0;
     sin(fai_r)*T     0;
     0                T];

%% ״̬�ռ䷽�̵���ؾ���

% PHI����
PHI_cell = cell(NP,1);
for i = 1:NP
    PHI_cell{i,1}=A_piao^i;  % Nx �� (Nx+Nu)
end
PHI = cell2mat(PHI_cell);   % (Nx * Np) �� (Nx + Nu)

% THETA����
THETA_cell = cell(NP,NC);
for i = 1:NP
    for j = 1:NC
        if j <= i
            THETA_cell{i,j} = A_piao^(i-j)*B_piao;    % Nx �� Nu
        else
            THETA_cell{i,j} = zeros(NX,NU);
        end
    end
end
THETA = cell2mat(THETA_cell);                 % (Nx * Np) �� (Nu * Nc)


%% ������Ŀ�꺯������ؾ���

% H����
H_cell{1,1} = THETA'*Q*THETA + R;  % (Nu * Nc) �� (Nu * Nc)

H = cell2mat(H_cell);            % (Nu * Nc + 1) �� (Nu * Nc + 1)

% E����
E = PHI * X_piao;                    % (Nx * Np) �� 1

% g����
g_cell = cell(1,1);
g_cell{1,1} = E'*Q*THETA;          % (Nu * Nc ) �� 1������Ϊ�˺�H������ƥ�䣬�����һ��0
g = cell2mat(g_cell);              % (Nu * Nc + 1 ) �� 1

%% Լ����������ؾ���

% ��������������仯����Լ��
Umin = kron(ones(NC,1),umin);
Umax = kron(ones(NC,1),umax);
% delta_Umin = kron(ones(Nc,1),delta_umin);
% delta_Umax = kron(ones(Nc,1),delta_umax);

% ��U�����½�Լ��
lb = Umin;
ub = Umax;

%% ��ʼ������

options = optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
delta_U = quadprog(H,g,[],[],[],[],lb,ub,[],options);   %(Nu * Nc +1) �� 1

%% �������

% ֻѡȡ����delta_U�ĵ�һ���������ע�⣺������v_tilde�ı仯����Delta_tilde�ı仯��
delta_v_tilde = delta_U(1);
delta_w_tilde = delta_U(2);

% ������һʱ�̵Ŀ�������ע�⣬����ġ�����������v_tilde��Delta_tilde��������������v��Delta
U(1) = v_r + delta_v_tilde; 
U(2) = w_r + delta_w_tilde; 


end