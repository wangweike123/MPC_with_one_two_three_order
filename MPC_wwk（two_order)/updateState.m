function     [x, y, fai, u_last,v,w] = updateState(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, U,idx, u_last)

%% 通过离散欧拉方程更新
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

% %% 简单的直接更新
a_v = 0.5;
a_w = 0.5;
dt = 0.5;

if U(1) - u_last(1) > a_v*dt
    U(1) = u_last(1) + a_v * dt;
end
if  U(1) - u_last(1) < - a_v*dt
    U(1) = u_last(1) - a_v * dt;
end
if U(2) - u_last(2) > a_w*dt
    U(2) = u_last(2) + a_w * dt;
end
if  U(2) - u_last(2) < - a_w*dt
    U(2) = u_last(2) - a_w * dt;
end


fai = fai + U(2) * T; 
if fai>2*3.1415
    fai = fai - 2*3.1415;
end
if fai<-2*3.1415
    fai = fai + 2*3.1415;
end

x = x + U(1) * cos(fai) * T;
y = y + U(1) * sin(fai) * T;

u_last(1) = U(1);
u_last(2) = U(2);
v = U(1);
w = U(2);
end