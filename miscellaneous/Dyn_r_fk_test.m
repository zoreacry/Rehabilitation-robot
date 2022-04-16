clc
clear
tic
%This Function mainly be used to product the former for FK_DYNAMICS
%Written by Zoreacry
%===========================================================
syms x y z Roll_now_degree Pitch_now_degree Yaw_now_degree
syms omega_x omega_y omega_z
syms omega_x_d omega_y_d omega_z_d     
syms v_x v_y v_z
syms  a_x a_y a_z
syms F_ex F_ey F_ez
syms M_ex M_ey M_ez
syms m_p
syms F_1 F_2 F_3 F_4 F_5 F_6 F_7 F_8

% The above of unknown count need be edited
%============================================================
Pose=[x;y;z];
%---------------------------------------------------------------------------------------------------
g=[0 ;0 ;9.8];                                                                      %m/s^2
%-----------------------------

omega=[omega_x; omega_y; omega_z];                                                  %动平台角速度
omega_d=[omega_x_d ;omega_y_d ;omega_z_d];                                         %动平台角加速度
X_d  =[v_x ;v_y ;v_z];                                                             %动平台线速度(m/s^2)
X_dd =[a_x ;a_y ;a_z];                                                             %动平台线加速度
%---------------------------------------------------------
F_e=[F_ex;F_ey;F_ez];                                                               %外部力(N)
M_e=[M_ex;M_ey;M_ez];                                                               %外力矩
%-----------------------------------------------------------------
TransM = [cosd(Pitch_now_degree) * cosd(Yaw_now_degree), sind(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree) - cosd(Roll_now_degree) * sind(Yaw_now_degree), sind(Roll_now_degree) * sind(Yaw_now_degree) + cosd(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree);cosd(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * cosd(Yaw_now_degree) + sind(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree) - sind(Roll_now_degree) * cosd(Yaw_now_degree); - sind(Pitch_now_degree), sind(Roll_now_degree) * cosd(Pitch_now_degree), cosd(Roll_now_degree) * cosd(Pitch_now_degree)];%变换矩阵
%----------------------------------------------------------------------------
omega_cross= [0                -omega_z   omega_y ;...                                        %Anti-symmetric matrix
                         omega_z      0              -omega_x;...
                       -omega_y      omega_x              0 ];
%---------------------------------------------
load_1 = [ - 200; - 150; - 100];   L1=[-1000;-1000;-1000];                                                     
load_2 = [ - 200;150; - 100];  L2=[-1000; 1000;-1000];
load_3 = [200;150; - 100];     L3=[ 1000; 1000;-1000];
load_4 = [200; - 150; - 100]; L4=[ 1000;-1000;-1000];
load_5 = [ - 200; - 150;100]; L5=[-1000;-1000;1000];
load_6 = [ - 200;150;100];    L6=[-1000;1000;1000];
load_7 = [200;150;100];        L7=[1000;1000;1000];
load_8 = [200; - 150;100];    L8=[1000;-1000;1000];


T_1=TransM*load_1+Pose;
T_2=TransM*load_2+Pose;
T_3=TransM*load_3+Pose;
T_4=TransM*load_4+Pose;
T_5=TransM*load_5+Pose;
T_6=TransM*load_6+Pose;
T_7=TransM*load_7+Pose;
T_8=TransM*load_8+Pose;


%---------------------------------------
U_1=(L1-T_1)/sqrt((T_1(1)-L1(1))^2+(T_1(2)-L1(2))^2+(T_1(3)-L1(3))^2);
U_2=(L2-T_2)/sqrt((T_2(1)-L2(1))^2+(T_2(2)-L2(2))^2+(T_2(3)-L2(3))^2);
U_3=(L3-T_3)/sqrt((T_3(1)-L3(1))^2+(T_3(2)-L3(2))^2+(T_3(3)-L3(3))^2);
U_4=(L4-T_4)/sqrt((T_4(1)-L4(1))^2+(T_4(2)-L4(2))^2+(T_4(3)-L4(3))^2);
U_5=(L5-T_5)/sqrt((T_5(1)-L5(1))^2+(T_5(2)-L5(2))^2+(T_5(3)-L5(3))^2);
U_6=(L6-T_6)/sqrt((T_6(1)-L6(1))^2+(T_6(2)-L6(2))^2+(T_6(3)-L6(3))^2);
U_7=(L7-T_7)/sqrt((T_7(1)-L7(1))^2+(T_7(2)-L7(2))^2+(T_7(3)-L7(3))^2);
U_8=(L8-T_8)/sqrt((T_8(1)-L8(1))^2+(T_8(2)-L8(2))^2+(T_8(3)-L8(3))^2);
%------------------------------------------------------
U=[U_1,U_2,U_3,U_4,U_5,U_6,U_7,U_8];                                              %绳索单位向量:N
%--------------------------
%  J=[U;cross(TransM*load_1,U_1) cross(TransM*load_2,U_2) cross(TransM*load_3,U_3) cross(TransM*load_4,U_4) cross(TransM*load_5,U_5) cross(TransM*load_6,U_6) cross(TransM*load_7,U_7) cross(TransM*load_8,U_8) ];
%  J=[U;cross(T_1,U_1) cross(T_2,U_2) cross(T_3,U_3) cross(T_4,U_4) cross(T_5,U_5) cross(T_6,U_6) cross(T_7,U_7) cross(T_8,U_8) ];
J=[        U_1                                  U_2                       U_3                    U_4                         U_5                    U_6                      U_7                          U_8
      cross(load_1,U_1), cross(load_2,U_2), cross(load_3,U_3), cross(load_4,U_4), cross(load_5,U_5), cross(load_6,U_6) ,cross(load_7,U_7) ,cross(load_8,U_8) ];

%=====================================================================================================
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
M=[m_p*I_vect zeros(3,3)                                               %6*6矩阵
    zeros(3,3)        I];

N=[zeros(3,3) zeros(3,3)
    zeros(3,3)   -I*omega_cross];

W_e=[F_e;M_e];                                                                 %外力向量

W_g=[m_p*g;0;0;0];                                                                  %重力向量

X_D=[X_d;omega];                                                               %速度向量

X_DD=[X_dd;omega_d];       

T_Force=[F_1 F_2 F_3 F_4 F_5 F_6 F_7 F_8]';

Total=J*T_Force-(W_e+W_g-(M*X_DD+N*X_D));



