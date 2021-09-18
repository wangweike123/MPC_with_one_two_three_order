% ʹ���˵���ģ�ͽ���MPC�켣���٣�״̬Ϊ(x,y,fai),����Ϊ(v,w)
% ���ǵ��˶��ף�(dv,dw)
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

x   = ref_x(1)+0.1; 
y   = ref_y(1)+0.1; 
fai = ref_fai(1)+0.02; 
v   = 0.1;
w   = 0.5;
idx =0;

%% ���ò���ֵ
T  = 0.5;  %ŷ����ɢ
NC = 5;     %����ʱ��
NP = 20;    %Ԥ��ʱ��
idx =0;

%% ��¼
pos_actual = [x,y]
latError_MPC = [];
Error_V = [];
Error_W = [];
u_last = [0,0];

%% ѭ��

while idx < length(ref_x)-1
    
    % ����MPC������
    [U,idx,latError ] = mpc_control(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, NC, NP) ;
    
    % ���̫���˳�����
%     if abs(latError) > 3
%         disp('�������˳�����!\n')
%         break
%     end
    
    % ����״̬��
    [x, y, fai, u_last,v,w] = updateState(x, y, fai, v, w, ref_x, ref_y, ref_fai, ref_v, ref_w, T, U,idx,u_last); 
    
    % ����ÿһ����ʵ����
    pos_actual(end+1,:) = [x,y];
    latError_MPC(end+1,:) = [idx,latError];
    Error_V(end+1) = U(1);
    Error_W(end+1) = U(2);
    % �����ٹ켣ͼ
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
%% ����
path_MPC = pos_actual;
save path_MPC.mat path_MPC
save latError_MPC.mat latError_MPC