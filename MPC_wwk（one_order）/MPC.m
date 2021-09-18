% ʹ���˵���ģ�ͽ���MPC�켣���٣�״̬Ϊ(x,y,fai),����Ϊ(v,w)
% �򵥵�һ�ף�ֻ������(v,w)
% ���ߣ�WWK
% ���ڣ�2021.09.13

clc
clear
close all

%% ����·����������ο�����Ϣ

[ref_x,ref_y,ref_fai,ref_v,ref_w] = Bezier_curve()
plot(ref_x, ref_y,'LineWidth',2.5)
hold on

%% ���ɻ����˳�ʼλ��

x   = ref_x(1); 
y   = ref_y(1); 
fai = ref_fai(1)+0.02; 
v   = 0.5;
w   = 0.2;
idx =0;

%% ���ò���ֵ
T  = 0.5;  %ŷ����ɢ
NC = 5;     %����ʱ��
NP = 20;    %Ԥ��ʱ��
max_steer = 3.14/3; %���ڽ�
idx =0;

%% ��¼
pos_actual = [x,y]
latError_MPC = [];

%% ѭ��

while idx < length(ref_x)-1
    
    % ����MPC������
    [U,idx,latError ] = mpc_control(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, NC, NP) ;
    
    % ���̫���˳�����
    if abs(latError) > 3
        disp('�������˳�����!\n')
        break
    end
    
    % ����״̬��
    [x, y, fai] = updateState(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, U,idx); 
    
    % ����ÿһ����ʵ����
    pos_actual(end+1,:) = [x,y];
    latError_MPC(end+1,:) = [idx,latError];
    
    % �����ٹ켣ͼ
    subplot(2,1,1)
    plot(x,y,'r*')
    hold on
    pause(0.01);
end
title('λ��')
subplot(2,1,2)
plot(latError_MPC(:,2),'LineWidth',2)
title('���')
%% ����
path_MPC = pos_actual;
save path_MPC.mat path_MPC
save latError_MPC.mat latError_MPC