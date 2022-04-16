function [X_out]=dynamics_main(x,y,z,Roll_now_degree,Pitch_now_degree,Yaw_now_degree,a_x,a_y,a_z,omega_x_d,omega_y_d,omega_z_d,v_x,v_y,v_z,omega_x,omega_y,omega_z,F_ex,F_ey,F_ez,M_ex,M_ey,M_ez)
%您需要输入：   1.当前位姿
%              2.当前速度
%              3.当前加速度
%              4.当前外力
%              5.当前外力矩
%              6.当前动平台质量
%               若无请置0
%---------------------------------------------------------------------------------------------------                                                                   %m/s^2
m_p=10;                                                 %运动平台质量
%------------------------------------------------------------------------------------------------------

[X_out]=Dynamics_least_square_function(x,y,z,Roll_now_degree,Pitch_now_degree, Yaw_now_degree, a_x,a_y,a_z,omega_x_d,omega_y_d,omega_z_d,v_x,v_y,v_z,omega_x,omega_y,omega_z,F_ex,F_ey,F_ez,M_ex,M_ey,M_ez,m_p);

