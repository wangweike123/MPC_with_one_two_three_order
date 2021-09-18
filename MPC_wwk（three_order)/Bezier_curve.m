function [path_x,path_y,path_fai,path_v,path_w] = Bezier_curve()


%% 控制点
p0=[20,-10];
p1=[230,200];
p2=[300,300];
p3=[350,400];
p12=[420,450];
p13=[450,505];
p14=[600,405];
p15=[800,800];
p6=p3;
p7=3*(p3-p2)/5+p6;
p8=3*(p3-2*p2+p1)/10+2*p7-p6;
p11=p12;
p10=p11-3*(p13-p12)/5;
p9=3*[p14-2*p13+p12]/10-p11++2*p10;


%% 曲线 x y
t = 0:0.01:1;
p11_x_2 = (1-t)*p6(1) + t.*p7(1);
p12_x_2 = (1-t)*p7(1) + t*p8(1);
p13_x_2 = (1-t)*p8(1) + t*p9(1);
p14_x_2 = (1-t)*p9(1) + t*p10(1);
p15_x_2 = (1-t)*p10(1) + t*p11(1);

p21_x_2 = (1-t).*p11_x_2 + t.*p12_x_2;
p22_x_2 = (1-t).*p12_x_2 + t.*p13_x_2;
p23_x_2 = (1-t).*p13_x_2 + t.*p14_x_2;
p24_x_2 = (1-t).*p14_x_2 + t.*p15_x_2;

p31_x_2 = (1-t).*p21_x_2 + t.*p22_x_2;
p32_x_2 = (1-t).*p22_x_2 + t.*p23_x_2;
p33_x_2 = (1-t).*p23_x_2 + t.*p24_x_2;

p41_x_2 = (1-t).*p31_x_2 + t.*p32_x_2;
p42_x_2 = (1-t).*p32_x_2 + t.*p33_x_2;

p51_x_2 = (1-t).*p41_x_2 + t.*p42_x_2;

p11_y_2 = (1-t)*p6(2) + t.*p7(2);
p12_y_2 = (1-t)*p7(2) + t*p8(2);
p13_y_2 = (1-t)*p8(2) + t*p9(2);
p14_y_2 = (1-t)*p9(2) + t*p10(2);
p15_y_2 = (1-t)*p10(2) + t*p11(2);
 
p21_y_2 = (1-t).*p11_y_2 + t.*p12_y_2;
p22_y_2 = (1-t).*p12_y_2 + t.*p13_y_2;
p23_y_2 = (1-t).*p13_y_2 + t.*p14_y_2;
p24_y_2 = (1-t).*p14_y_2 + t.*p15_y_2;
 
p31_y_2 = (1-t).*p21_y_2 + t.*p22_y_2;
p32_y_2 = (1-t).*p22_y_2 + t.*p23_y_2;
p33_y_2 = (1-t).*p23_y_2 + t.*p24_y_2;
 
p41_y_2 = (1-t).*p31_y_2 + t.*p32_y_2;
p42_y_2 = (1-t).*p32_y_2 + t.*p33_y_2;
 
p51_y_2 = (1-t).*p41_y_2 + t.*p42_y_2;

path_x=p51_x_2;
path_y=p51_y_2;


%% 计算 航向角fai
diff_x = diff(path_x) ;
diff_x(end+1) = diff_x(end);
diff_y = diff(path_y);
diff_y(end+1) = diff_y(end);
path_v = (diff_x.^2+diff_y.^2).^(0.5);   %参考 path_v
derivative1 = gradient(path_y) ./ abs(diff_x);              % 一阶导数
derivative2 = del2(path_y) ./ abs(diff_x);                  % 二阶导数
path_fai = atan2(diff_y , diff_x);   % 参考航向角 fai_r
refK = abs(derivative2) ./ (1+derivative1.^2).^(3/2);  % 计算曲率
path_w = path_v .* refK ; % 参考角速度w


end