function []=Fap_mix_con()
%----------------------------------------------------------------------------
%choose the Selection_matrix 's model
% "1" stand for position control and "0" stand for Force control
s=[1,1,0,0,0,0];                                                                   %choose key-point
s_inv=[0,0,1,0,0,0];

[S]=Selection_matrix(s);
[S_inv]=Selection_matrix(s_inv);
%这里调用外力变化模拟仿真函数
%----------------------------------------------------------------------------
%perpare a turly track;
%----------------------------------------初始位置的姿态------------------------------------
x_0 = 0;                                
y_0 = 0;                                      
z_0 = 0;
Roll = 0;                           
Pitch = 0;                               
Yaw = 0;
%---------------------------------------其他设定------------------------------------
Roll_start = 0;                       
Pitch_start = 0;                          
Yaw_start = 0;
x_1 = 150;                                       
y_1 = 350;                                       
z_1 = 0;
%-------------------------------------
target = sqrt((x_1 - x_0) ^ 2 + (y_1 - y_0) ^ 2 + (z_1 - z_0) ^ 2);           %计算所走位移
progress = 0;                                                                 %设定当前位移所走距离
currentvel = 0;                                                               %设定当前速度
time = 0;      %设定初始时刻
m=5;
T_s=0.01;
i=1 ;              %迭代次数
%---------------------------------------------------------------------------------------
%Position Control
while   progress < target
    
[position,Velocity,accelrate,progress] = pos_cont(x_0,y_0,z_0,Roll,Pitch,Yaw,x_1,y_1,z_1,Roll_start,Pitch_start,Yaw_start,time,currentvel,progress,target);
 
 
Position_P=S*position;                                                          %Position_control,need updated
Velocity_P=S*Velocity;
accelrate_P=S*accelrate;


%-----------------------------------------------------------------------------------------
%Force control
x_now=position(1);y_now=position(2);z_now=position(3);Roll=position(4);Pitch=position(5);Yaw=position(6);
F_x=m*accelrate(1);F_y=m*accelrate(2);F_z=m*accelrate(3);F_R=m*accelrate(4);F_P=m*accelrate(5);F_Y=m*accelrate(6);
  %  need to change the F_x,F_y,F_z~~~~~
  %then add a admitance model to reduce the error of Force,and exchange it
  %to position error.
%---------------------------------------------------------------------
 
%这里引用每个时间点内的力变化值
%Function here
  %----------------力阻抗值控制，对于六维力--------------------------------------
[Output] = Z_axis_ext_f_cha(i);

F_F=S_inv*Output';                                                                        %Create the external Z-axis Force
position_F=S_inv*position;  
Velocity_F=S_inv*Velocity;
accelrate_F=S_inv*accelrate;   




 [M,B,K]=I_C_PF_Para();


[dx_dd_re,dx_d_re,dx_re]=Impedance_control_PF_contr(M,B,K,T_s,position_F,Velocity_F,accelrate_F,i,S_inv);

%--------------------------------输入Δ力输出位移捏，最速减少力----------------------------------------------------


position=position+dx_re;
Velocity=Velocity+dx_d_re;
accelrate=accelrate+dx_dd_re;


[F_rea] =FP_method(x_now,y_now,z_now,Roll,Pitch,Yaw,F_x,F_y+m*accelrate(3),F_z,F_R,F_P,F_Y);            %The force of cable
[F_rea_norm] =FP_method(x_now,y_now,z_now,Roll,Pitch,Yaw,F_x,F_y,F_z,F_R,F_P,F_Y);



i=i+1;
end


