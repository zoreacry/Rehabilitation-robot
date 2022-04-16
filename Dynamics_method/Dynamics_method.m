clc
clear;


position=zeros(6,10);
Velocity=zeros(6,10);          %w位移速度与加速度位移数组
accelrate=zeros(6,10);
X_out_M=zeros(8,10);

    p=1;
%--------------------------------------------外力------------
F_ex=0;
F_ey=0;
F_ez=0;
M_ex=0;
M_ey=0;
M_ez=0;

%----------------------------------------初始位置的姿态------------------------------------

x_0 = 0;                                       %动平台 相对 静平台 的初始位置的坐标
y_0 = 0;                                       %可以改动，意为：更改初始时刻动平台相对于静平台的坐标
z_0 = 0;

%----------------------------------------动平台的位姿---------------------------

Roll = 0;                                      %相对静平台的姿态
Pitch = 0;                                     %可以改动，意为：更改初始时刻动平台相对于静平台的姿态
Yaw = 0;

%---------------------------------------其他设定------------------------------------

Roll_start = 0;                               %设定末端点的姿态
Pitch_start = 0;                              %可以改动，意为：更改为末端点的姿态
Yaw_start = 0;

x_1 = 400;                                       %设定末端点的位置
y_1 =300;                                       %可以改动，意为：更改为末端点的位置
z_1 = 400;

target = sqrt((x_1 - x_0) ^ 2 + (y_1 - y_0) ^ 2 + (z_1 - z_0) ^ 2);           %计算所走位移
progress = 0;                                                                 %设定当前位移所走距离
currentvel = 0;                                                               %设定当前速度
time = 0;                                                                     %设定初始时刻

%=======================================以下都可以进行相应改动==========================%

cycle_time = 0.01;                                            %设定步长时间

maxaccel = 150;                                                 %设定最大加速度

feed_override = 1.2;                                          %设定倍率（进行瞬调）

maxvel = 10000;                                                %设定机床规定最大速度

reqvel = 150;                                                  %设定此用户给定的速度



%----------------------------------------------------------------------------------------------


%---------------------------------平台基本尺寸（可以进行改动）---------------------------------

L1_point_x = - 1000;                                              %第一接触点位置(mm)
L1_point_y = - 1000;
L1_point_z = - 1000;

L2_point_x = - 1000;                                              %第二接触点位置
L2_point_y = 1000;
L2_point_z = - 1000;

L3_point_x = 1000;                                               %第三接触点位置
L3_point_y = 1000;
L3_point_z = - 1000;

L4_point_x = 1000;                                              %第四接触点位置
L4_point_y = - 1000;
L4_point_z = - 1000;

L5_point_x = - 1000;                                             %第五接触点位置
L5_point_y = - 1000;
L5_point_z = 1000;

L6_point_x = - 1000;                                             %第六接触点位置
L6_point_y = 1000;
L6_point_z = 1000;

L7_point_x = 1000;                                             %第七接触点位置
L7_point_y = 1000;
L7_point_z = 1000;

L8_point_x = 1000;                                             %第八接触点位置
L8_point_y = - 1000;
L8_point_z = 1000;

%--------------------------------------------------进行每次画图-------------------------------
while progress < target  %progress:当前位姿
    [progress, currentvel,newaccel] = tcRunCycle(progress, reqvel, currentvel,cycle_time,target,maxaccel, feed_override, maxvel);         %调用速度规划函数
    time = time + cycle_time;                                                       %进行时间迭代
    
    x_now = progress / target * (x_1 - x_0) + x_0;                                 %计算当前位置
    y_now = progress / target * (y_1 - y_0) + y_0;
    z_now = progress / target * (z_1 - z_0) + z_0;
    
    Roll_now_degree = (progress / target) * (Roll_start - Roll) + Roll;                    %计算当前角度
    Pitch_now_degree = (progress / target) * (Pitch_start - Pitch) + Pitch;
    Yaw_now_degree = (progress / target) * (Yaw_start - Yaw) + Yaw;
    
    P = [x_now;y_now;z_now];                                                       %动平台相对于静平台的实时位置
    
    TransM = [cosd(Pitch_now_degree) * cosd(Yaw_now_degree), sind(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree) - cosd(Roll_now_degree) * sind(Yaw_now_degree), sind(Roll_now_degree) * sind(Yaw_now_degree) + cosd(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree);cosd(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * cosd(Yaw_now_degree) + sind(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree) - sind(Roll_now_degree) * cosd(Yaw_now_degree); - sind(Pitch_now_degree), sind(Roll_now_degree) * cosd(Pitch_now_degree), cosd(Roll_now_degree) * cosd(Pitch_now_degree)];  %XYZ旋转矩阵
     
a_x=(newaccel/target)*(x_1-x_0);
a_y=(newaccel/target)*(y_1-y_0);
a_z=(newaccel/target)*(z_1-z_0);
omega_x_d=(newaccel/target)*(Roll_start-Roll);
omega_y_d=(newaccel/target)*(Pitch_start-Pitch);
omega_z_d=(newaccel/target)*(Yaw_start-Yaw);
v_x=(currentvel/target)*(x_1-x_0);
v_y=(currentvel/target)*(y_1-y_0);
v_z=(currentvel/target)*(z_1-z_0);
omega_x=(currentvel/target)*(Roll_start-Roll);
omega_y=(currentvel/target)*(Pitch_start-Pitch);
omega_z=(currentvel/target)*(Yaw_start-Yaw);

    position(1,p) =x_now;   position(2,p)=y_now;   position(3,p)=z_now;   position(4,p)=Roll_now_degree ;   position(5,p)=Pitch_now_degree ;   position(6,p)=Yaw_now_degree;%位置参数
    Velocity(1,p) =(currentvel/target)*(x_1-x_0);  Velocity(2,p) =(currentvel/target)*(y_1-y_0); Velocity(3,p) =(currentvel/target)*(z_1-z_0); Velocity(4,p) =(currentvel/target)*(Roll_start-Roll); Velocity(5,p) =(currentvel/target)*(Pitch_start-Pitch); Velocity(6,p) =(currentvel/target)*(Yaw_start-Yaw);         %w位移速度与加速度位移数组
    accelrate(1,p)=(newaccel/target)*(x_1-x_0); accelrate(2,p)=(newaccel/target)*(y_1-y_0); accelrate(3,p)=(newaccel/target)*(z_1-z_0); accelrate(4,p)=(newaccel/target)*(Roll_start-Roll); accelrate(5,p)=(newaccel/target)*(Pitch_start-Pitch); accelrate(6,p)=(newaccel/target)*(Yaw_start-Yaw);
      
%-----------------------------------------计算八个铰点的位置矢量----------------------------------
     [T_Force]=Dynamics_eight_ropt(x_now,y_now,z_now,Pitch_now_degree,Yaw_now_degree,Roll_now_degree,a_x,a_y,a_z,omega_x_d,omega_y_d,omega_z_d,v_x,v_y,v_z,omega_x,omega_y,omega_z,F_ex,F_ey,F_ez,M_ex,M_ey,M_ez);
    [F_rea] =FP_method_noextf(x_now,y_now,z_now,Roll_now_degree,Pitch_now_degree,Yaw_now_degree);

% F_1=T_Force(1)+F_rea(1);F_3=T_Force(3)+F_rea(3);F_5=T_Force(5)+F_rea(5);F_7=T_Force(7)+F_rea(7);
% F_2=T_Force(2)+F_rea(2);F_4=T_Force(4)+F_rea(4);F_6=T_Force(6)+F_rea(6);F_8=T_Force(8)+F_rea(8);

% F_1=T_Force(1);F_3=T_Force(3);F_5=T_Force(5);F_7=T_Force(7);
% F_2=T_Force(2);F_4=T_Force(4);F_6=T_Force(6);F_8=T_Force(8);

F_1=F_rea(1);F_3=F_rea(3);F_5=F_rea(5);F_7=F_rea(7);
F_2=F_rea(2);F_4=F_rea(4);F_6=F_rea(6);F_8=F_rea(8);

F_rope=[F_1;F_2;F_3;F_4;F_5;F_6;F_7;F_8];
%TEST
[X_DD] = forward_Dynamics(x_now,y_now,z_now,Pitch_now_degree,Yaw_now_degree,Roll_now_degree,F_rea);





X_out_M(1,p)=F_1;X_out_M(2,p)=F_2;
X_out_M(3,p)=F_3;X_out_M(4,p)=F_4;
X_out_M(5,p)=F_5;X_out_M(6,p)=F_6;
X_out_M(7,p)=F_7;X_out_M(8,p)=F_8;

    p=p+1;
end

FIG1=(X_out_M(1,:));FIG2=(X_out_M(2,:));FIG3=(X_out_M(3,:));FIG4=(X_out_M(4,:));FIG5=(X_out_M(5,:));FIG6=(X_out_M(6,:));FIG7=(X_out_M(7,:));FIG8=(X_out_M(8,:));

hold on;
plot(FIG1);
plot(FIG2);
plot(FIG3);
plot(FIG4);
plot(FIG5);
plot(FIG6);
plot(FIG7);
plot(FIG8);