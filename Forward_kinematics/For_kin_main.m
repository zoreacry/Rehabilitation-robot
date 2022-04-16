clc
clear
%input eight rope lengths 在这里输入你的绳长
Len_now=[0;0;0;0;0;0;0;0];
[x_now,y_now,z_now,Roll_now_degree,Pitch_now_degree,Yaw_now_degree]=Forward_kinematics(Len_now);
%这里要输出当前的位姿