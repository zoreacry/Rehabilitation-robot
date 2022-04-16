clc
clear
con_1=zeros(6,45);
con_2=zeros(8,45);
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

x_1 = 0;                                       %设定末端点的位置
y_1 = 0;                                       %可以改动，意为：更改为末端点的位置
z_1 = 300;
%-------------------------------------------------------------------------------------
delta_x=[0;0;0;0;0;0];                        %Impedence control input x in wall
%-----------------------------------------------------------------------------------
target = sqrt((x_1 - x_0) ^ 2 + (y_1 - y_0) ^ 2 + (z_1 - z_0) ^ 2);           %计算所走位移
progress = 0;                                                                 %设定当前位移所走距离
currentvel = 0;                                                               %设定当前速度
time = 0;      %设定初始时刻
m=5;
T_s=0.01;
i=1 ;              %迭代次数
while   progress < target
    
 [position,Velocity,accelrate,progress]=Position_found(x_0,y_0,z_0,Roll,Pitch,Yaw,x_1,y_1,z_1,Roll_start,Pitch_start,Yaw_start,time,currentvel,progress,target);
  
 [Cho] = Impact_checking_main(progress);
 if Cho ==0
     
 break;
 
 end
 
 
end

% [F_rea_noextf] =FP_method_noextf(position(1),position(2),position(3),position(4),position(5),position(6));    %Static Force
% 
% [F_rea] =FP_method(position(1),position(2),position(3),position(4),position(5),position(6),m*accelrate(1),m*accelrate(2),m*accelrate(3),m*accelrate(4),m*accelrate(5),m*accelrate(6));%Moving Force

[F_wall]=Sim_Sec_wall(delta_x);  %need updated
[M,B,K]=I_C_Para();

dx_dd_start=accelrate;
dx_d_start=Velocity;
dx_d_start=position;

Jud_1=dx_d_start(2)+dx_d_start(3)+dx_d_start(4)+dx_d_start(5)+dx_d_start(6);         %判断域（不一定合理）
Jud_2=dx_dd_start(1)+dx_dd_start(2)+dx_dd_start(3)+dx_dd_start(4)+dx_dd_start(5)+dx_dd_start(6);

while     abs(Jud_1)>0.01 || abs(Jud_2)> 0.01
   %Force_area-------------------------------------------------------------------- 
  [F_rea_noextf] =FP_method_noextf(position(1),position(2),position(3),position(4),position(5),position(6));    %Static Force
  [F_rea] =FP_method(position(1),position(2),position(3),position(4),position(5),position(6),m*accelrate(1),m*accelrate(2),m*accelrate(3),m*accelrate(4),m*accelrate(5),m*accelrate(6));%Moving Force
  [F_XYZ] =Platform_F_noextf(position,F_rea);
  %-----------------------------------------------------------------------------------------------------------------
    
[Rope_Len]=kinematic_control(position);
    
[dx_dd_re,dx_d_re,dx_re]=Impedance_control_main(M,B,K,T_s,position,Velocity,accelrate,delta_x);

delta_x=dx_d_start-dx_re;

accelrate = dx_dd_re ;
Velocity  = dx_d_re  ;
position  = dx_re;
         
                    Jud_1=dx_d_re(2)+dx_d_re(3)+dx_d_re(4)+dx_d_re(5)+dx_d_re(6);       
                    Jud_2=dx_dd_re(1)+dx_dd_re(2)+dx_dd_re(3)+dx_dd_re(4)+dx_dd_re(5)+dx_dd_re(6); 

     con_1(:,i)=    position;     %exhaust!
     con_2(:,i)=    F_rea;
i=i+1;



[delta_Len]=kinematic_control_by_Ic(Rope_Len,position);


end
