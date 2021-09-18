% 使用了单车模型进行MPC轨迹跟踪，状态为(x,y,fai),输入为(v,w)
% 简单的一阶，只考虑了(v,w)
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

x   = ref_x(1); 
y   = ref_y(1); 
fai = ref_fai(1)+0.02; 
v   = 0.5;
w   = 0.2;
idx =0;

%% 设置参数值
T  = 0.5;  %欧拉离散
NC = 5;     %控制时域
NP = 20;    %预测时域
max_steer = 3.14/3; %最大摆角
idx =0;

%% 记录
pos_actual = [x,y]
latError_MPC = [];

%% 循迹

while idx < length(ref_x)-1
    
    % 调用MPC控制器
    [U,idx,latError ] = mpc_control(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, NC, NP) ;
    
    % 误差太大，退出程序
    if abs(latError) > 3
        disp('误差过大，退出程序!\n')
        break
    end
    
    % 更新状态量
    [x, y, fai] = updateState(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, U,idx); 
    
    % 保存每一步的实际量
    pos_actual(end+1,:) = [x,y];
    latError_MPC(end+1,:) = [idx,latError];
    
    % 画跟踪轨迹图
    subplot(2,1,1)
    plot(x,y,'r*')
    hold on
    pause(0.01);
end
title('位置')
subplot(2,1,2)
plot(latError_MPC(:,2),'LineWidth',2)
title('误差')
%% 保存
path_MPC = pos_actual;
save path_MPC.mat path_MPC
save latError_MPC.mat latError_MPC