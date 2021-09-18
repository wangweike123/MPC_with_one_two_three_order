function     [x, y, fai, v, w] = updateState(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, U,idx)

%% ͨ����ɢŷ�����̸���
% 
% X_real = [x; y; fai];
% X_r = [ref_x(idx); ref_y(idx); ref_fai(idx)]
% 
% X_piao = X_real - X_r
% 
% A_piao = [1    0   -ref_v(idx)*sin(ref_fai(idx))*T;
%      0    1   ref_v(idx)*cos(ref_fai(idx))*T;
%      0    0   1];
%  
% B_piao = [cos(ref_fai(idx))*T     0;
%      sin(ref_fai(idx))*T     0;
%      0                T];
% 
% U_piao = (U-[v,w])'
% 
% X = A_piao * X_piao + B_piao * U_piao + X_r;
% 
% x = X(1), y = X(2), fai = X(3);

% %% �򵥵�ֱ�Ӹ���

fai = fai + U(2) * T; 
x = x + U(1) * cos(fai) * T;
y = y + U(1) * sin(fai) * T;
v = U(1);
w = U(2);

end