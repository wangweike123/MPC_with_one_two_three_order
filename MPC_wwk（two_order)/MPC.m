% 使用了单车模型进行MPC轨迹跟踪，状态为(x,y,fai),输入为(v,w)
% 考虑到了二阶，(dv,dw)
% 作者：WWK
% 日期：2021.09.13

clc
clear
close all

%% 设置路径，计算各参考点信息

[ref_x,ref_y,ref_fai,ref_v,ref_w] = Bezier_curve()
plot(ref_x, ref_y,'LineWidth',2.5)
hold on

%% 生成机器人初始位置

x   = ref_x(1)+0.1; 
y   = ref_y(1)+0.1; 
fai = ref_fai(1)+0.02; 
v   = 0.1;
w   = 0.5;
idx =0;

%% 设置参数值
T  = 0.5;  %欧拉离散
NC = 5;     %控制时域
NP = 20;    %预测时域
idx =0;

%% 记录
pos_actual = [x,y]
latError_MPC = [];
Error_V = [];
Error_W = [];
u_last = [0,0];

%% 循迹

while idx < length(ref_x)-1
    
    % 调用MPC控制器
    [U,idx,latError ] = mpc_control(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, NC, NP) ;
    
    % 误差太大，退出程序
%     if abs(latError) > 3
%         disp('误差过大，退出程序!\n')
%         break
%     end
    
    % 更新状态量
    [x, y, fai, u_last,v,w] = updateState(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, U,idx,u_last); 
    
    % 保存每一步的实际量
    pos_actual(end+1,:) = [x,y];
    latError_MPC(end+1,:) = [idx,latError];
    Error_V(end+1) = U(1);
    Error_W(end+1) = U(2);
    % 画跟踪轨迹图
    plot(x,y,'r*')
    hold on
    pause(0.01);
end
figure(2)
plot(latError_MPC(:,2),'LineWidth',2)
figure(3)
plot(Error_V(2:end)-Error_V(1:end-1),'LineWidth',2)
figure(4)
plot(Error_W(2:end)-Error_W(1:end-1),'LineWidth',2)
%% 保存
path_MPC = pos_actual;
save path_MPC.mat path_MPC
save latError_MPC.mat latError_MPC