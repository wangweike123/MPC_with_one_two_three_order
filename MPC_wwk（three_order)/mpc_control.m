function    [U,idx,latError,du_k_1 ] = ...
            mpc_control(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, NC, NP,du_k_1) ;
        
%% MPCԤ�����
NX = 7;         % ״̬���ĸ���
NU = 2;         % �������ĸ���
row = 10;       % �ɳ�����
Q = 100*eye(NP*NX);      % (Np*Nx) �� (Np*Nx)
R = 1*eye(NC*NU);        % (Nc*Nu) �� (Nc*Nu)

% ������Լ������
umin = [0.02; -3.14/2];
umax = [1; 3.14/2];
delta_umin = [-3; -3.14/10];
delta_umax = [3; 3.14/10];
jmin = [-1;-3.14/30];
jmax = [ 1;3.14/30];

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
X_real = [x; y; fai;v;w;v-v_r;w-w_r];
X_r = [x_r; y_r; fai_r;v_r;w_r;du_k_1(1);du_k_1(2)];
X_piao = X_real - X_r; 
du_k_1 = [X_piao(4);X_piao(5)];
%% a,b��������
A_piao = [1    0   -v_r*sin(fai_r)*T cos(fai_r)*T     0  cos(fai_r)*T     0;
          0    1   v_r*cos(fai_r)*T  sin(fai_r)*T     0  sin(fai_r)*T     0; 
          0    0   1                 0                T  0                T;
          0    0   0                 1                0  1                0;
          0    0   0                 0                1  0                1;
          0    0   0                 0                0  1                0;
          0    0   0                 0                0  0                1];
 
B_piao = [cos(fai_r)*T     0;
          sin(fai_r)*T     0;
          0                T;
          1                0;
          0                1;
          1                0;
          0                1];

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
H_cell = cell(2,2);
H_cell{1,1} = THETA'*Q*THETA + R;  % (Nu * Nc) �� (Nu * Nc)
H_cell{1,2} = zeros(NU*NC,1);
H_cell{2,1} = zeros(1,NU*NC);
H_cell{2,2} = row;
H = cell2mat(H_cell);            % (Nu * Nc + 1) �� (Nu * Nc + 1)

% E����
E = PHI * X_piao;                    % (Nx * Np) �� 1

% g����
g_cell = cell(1,1);
g_cell{1,1} = E'*Q*THETA;          % (Nu * Nc ) �� 1������Ϊ�˺�H������ƥ�䣬�����һ��0
g_cell{1,2} = 0;
g = cell2mat(g_cell);              % (Nu * Nc + 1 ) �� 1

%% Լ����������ؾ���

% ��������������仯����Լ��
Umin = kron(ones(NC,1),umin);
Umax = kron(ones(NC,1),umax);
delta_Umin = kron(ones(NC,1),delta_umin);
delta_Umax = kron(ones(NC,1),delta_umax);
Jmin = kron(ones(NC,1),jmin);
Jmax = kron(ones(NC,1),jmax);

% A_I����
A_t = zeros(NC,NC);     % �����Ƿ���
for i = 1:NC
    A_t(i,1:i) = 1;
end
A_I = kron(A_t,eye(NU));       % (Nu * Nc) �� (Nu * Nc)

% Ut����
dUt = kron(ones(NC,1),[X_piao(6)+X_piao(4)+v_r; X_piao(7)+X_piao(5)+w_r]);       % (Nu * Nc) �� 1
% ����quadprog��������ʽԼ��Ax <= b�ľ���A
A_cons_cell = {A_I, zeros(NU*NC,1);;       % ����Ϊ�˺�H������ƥ�䣬�����һ��0��(Nu * Nc) �� (Nu * Nc), (Nu * Nc) ��1
    -A_I, zeros(NU*NC,1);}; 
A_cons = cell2mat(A_cons_cell);           % (Nu * Nc * 2) �� (Nu * Nc +1)

% ����quadprog��������ʽԼ��Ax <= b������b
b_cons_cell = {Umax-dUt;
    -Umin+dUt};
b_cons = cell2mat(b_cons_cell);

% ��U�����½�Լ��
lb = Jmin;
ub = Jmax;

%% ��ʼ������

options = optimoptions('quadprog','Display','iter','MaxIterations',100,'TolFun',1e-16);
delta_U = quadprog(H,g,[],[],[],[],lb,ub,[],options)   %(Nu * Nc +1) �� 1

%% �ж��Ƿ��н�
if size(delta_U)==0
    disp('�޽⣬��������ȷԼ������!!!')
end

%% �������

% du_piao(k)
ddelta_v_tilde = delta_U(1);
ddelta_w_tilde = delta_U(2);

delta_v_tilde = ddelta_v_tilde+X_piao(6);
delta_w_tilde = ddelta_w_tilde+X_piao(7);

% u_piao(k)
U_piao(1) = X_piao(4) + delta_v_tilde; 
U_piao(2) = X_piao(5) + delta_w_tilde; 

% U
U(1) = U_piao(1) + v_r;
U(2) = U_piao(2) + w_r;

end