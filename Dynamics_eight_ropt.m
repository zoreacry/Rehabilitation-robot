clc
clear
%-------------------------------------------------------------------------------------------------
%您需要输入：   1.当前位姿
%              2.当前速度
%              3.当前加速度
%              4.当前外力
%              5.当前外力矩
%              6.当前动平台质量
%               若无请置0
%---------------------------------------------------------------------------------------------------
g=[0 0 9.8]';                                                                      %m/s^2
syms m_p;                                                                          %动平台质量（kg）
%-----------------------------
syms t_1 t_2 t_3 t_4 t_5 t_6 t_7 t_8                                               %定义绳索力标量
syms x y z Pitch_now_degree  Roll_now_degree Yaw_now_degree                        %定义位姿[x,y,z,a,b,r]
syms v_x v_y v_z omega_x omega_y omega_z                                          %定义速度[vx,vy,vz,omega_x omega_y omega z]
syms a_x a_y a_z omega_x_d omega_y_d omega_z_d                                    %定义加速度
omega=[omega_x omega_y omega_z]';                                                  %动平台角速度
omega_d=[omega_x_d omega_y_d omega_z_d]';                                         %动平台角加速度
X_d  =[v_x v_y v_z]';                                                             %动平台线速度(m/s^2)
X_dd =[a_x a_y a_z]';                                                             %动平台线加速度
%---------------------------------------------------------
syms F_ex F_ey F_ez                                                                %数据输入外力
syms M_ex M_ey M_ez                                                                 %输入外力矩
F_e=[F_ex,F_ey,F_ez]';                                                               %外部力(N)
M_e=[M_ex,M_ey,M_ez]';                                                               %外力矩
%-----------------------------------------------------------------
TransM = [cosd(Pitch_now_degree) * cosd(Yaw_now_degree), sind(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree) - cosd(Roll_now_degree) * sind(Yaw_now_degree), sind(Roll_now_degree) * sind(Yaw_now_degree) + cosd(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree);cosd(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * cosd(Yaw_now_degree) + sind(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree) - sind(Roll_now_degree) * cosd(Yaw_now_degree); - sind(Pitch_now_degree), sind(Roll_now_degree) * cosd(Pitch_now_degree), cosd(Roll_now_degree) * cosd(Pitch_now_degree)];%变换矩阵
%------------------------------------------------------------
x=5;y=6;z=15;Pitch_now_degree=20; Yaw_now_degree=15; Roll_now_degree=16;     
a_x=0;a_y=0;a_z=0;
omega_x_d=0.1;omega_y_d=0;omega_z_d=0.1;
v_x=2;v_y=5;v_z=8;
omega_x=1;omega_y=0;omega_z=0;
F_ex=0;F_ey=0;F_ez=0;
M_ex=0;M_ey=0;M_ez=0;
m_p=10;





%----------------------------------------------------------------------------
omega_cross=[0         -omega_z   omega_y;...                                        %Anti-symmetric matrix
             omega_z      0      -omega_x;...
             -omega_y    omega_x       0 ];
%---------------------------------------------
load_1 = [ - 100; - 100; - 100];                                                    %动平台铰接点位置(mm)
load_2 = [ - 100;100; - 100];
load_3 = [100;100; - 100];
load_4 = [100; - 100; - 100];
load_5 = [ - 100; - 100;100];
load_6 = [ - 100;100;100];
load_7 = [100;100;100];
load_8 = [100; - 100;100];

L1=[-1000,-1000,-1000]';                                                               %固定平台铰接点位置
L2=[-1000, 1000,-1000]';
L3=[ 1000, 1000,-1000]';
L4=[ 1000,-1000,-1000]';
L5=[-1000,-1000,1000]';
L6=[-1000,1000,1000]';
L7=[1000,1000,1000]';
L8=[1000,-1000,1000]';


T_1=TransM*load_1;
T_2=TransM*load_2;
T_3=TransM*load_3;
T_4=TransM*load_4;
T_5=TransM*load_5;
T_6=TransM*load_6;
T_7=TransM*load_7;
T_8=TransM*load_8;


%---------------------------------------
U_1=(TransM*load_1-L1)/sqrt((T_1(1)-L1(1))^2+(T_1(2)-L1(2))^2+(T_1(3)-L1(3))^2);
U_2=(TransM*load_2-L2)/sqrt((T_2(1)-L2(1))^2+(T_2(2)-L2(2))^2+(T_2(3)-L2(3))^2);
U_3=(TransM*load_3-L3)/sqrt((T_3(1)-L3(1))^2+(T_3(2)-L3(2))^2+(T_3(3)-L3(3))^2);
U_4=(TransM*load_4-L4)/sqrt((T_4(1)-L4(1))^2+(T_4(2)-L4(2))^2+(T_4(3)-L4(3))^2);
U_5=(TransM*load_5-L5)/sqrt((T_5(1)-L5(1))^2+(T_5(2)-L5(2))^2+(T_5(3)-L5(3))^2);
U_6=(TransM*load_6-L6)/sqrt((T_6(1)-L6(1))^2+(T_6(2)-L6(2))^2+(T_6(3)-L6(3))^2);
U_7=(TransM*load_7-L7)/sqrt((T_7(1)-L7(1))^2+(T_7(2)-L7(2))^2+(T_7(3)-L7(3))^2);
U_8=(TransM*load_8-L8)/sqrt((T_8(1)-L8(1))^2+(T_8(2)-L8(2))^2+(T_8(3)-L8(3))^2);
%------------------------------------------------------
U=[U_1,U_2,U_3,U_4,U_5,U_6,U_7,U_8];                                              %绳索单位向量:N
T=[t_1,t_2,t_3,t_4,t_5,t_6,t_7,t_8]';                                              %绳索拉力向量
%--------------------------
J=[U;cross(TransM*load_1,U_1) cross(TransM*load_2,U_2) cross(TransM*load_3,U_3) cross(TransM*load_4,U_4) cross(TransM*load_5,U_5) cross(TransM*load_6,U_6) cross(TransM*load_7,U_7) cross(TransM*load_8,U_8) ];
%--------------------------------------------------
I_pxx=m_p*(z^2+y^2);
I_pyy=m_p*(x^2+z^2);
I_pzz=m_p*(x^2+y^2);
I_p=[I_pxx 0     0;...
    0     I_pyy 0;...
    0     0     I_pzz];
I=TransM*I_p*TransM';                                             %惯性张量
I_vect=[1 0 0;...                                                  %单位矩阵
        0 1 0;...
        0 0 1];
%-----------------------------------
%牛顿欧拉法---------------------------------------------------------------------------
M=[m_p*I_vect zeros(3,3);...                                                    %6*6矩阵
    zeros(3,3)         I];
N=[zeros(3,3) zeros(3,3);...
    zeros(3,3) -I*omega_cross];
W_e=[F_e;M_e];                                                                 %外力向量
W_g=[m_p*g;0;0;0];                                                                  %重力向量
X_D=[X_d;omega];                                                               %速度向量
X_DD=[X_dd;omega_d];                                                            %加速度向量
%-----------------------------------------------------------------------------------------------------------


A=J*T;
B=(M*X_DD+N*X_D-W_e-W_g);
C=A-B;

J*T=(M*X_dd+N*X_D-W_e-W_g);%动力学方程
[X_out]=Dynamics_least_square_function(x,y,z,Pitch_now_degree, Yaw_now_degree, Roll_now_degree,a_x,a_y,a_z,omega_x_d,omega_y_d,omega_z_d,v_x,v_y,v_z,omega_x,omega_y,omega_z,F_ex,F_ey,F_ez,M_ex,M_ey,M_ez,m_p);

