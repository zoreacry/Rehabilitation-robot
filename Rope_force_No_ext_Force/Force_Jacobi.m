 function [J]=Force_Jacobi(x_now ,y_now ,z_now, Roll_now_degree_now  , Pitch_now_degree_now ,Yaw_now_degree_now  )   
%-----------------------------------------------------------------
P_now=[x_now;y_now;z_now];
TransM = [cosd(Pitch_now_degree_now) * cosd(Yaw_now_degree_now), sind(Roll_now_degree_now) * sind(Pitch_now_degree_now) * cosd(Yaw_now_degree_now) - cosd(Roll_now_degree_now) * sind(Yaw_now_degree_now), sind(Roll_now_degree_now) * sind(Yaw_now_degree_now) + cosd(Roll_now_degree_now) * sind(Pitch_now_degree_now) * cosd(Yaw_now_degree_now);...
          cosd(Pitch_now_degree_now) * sind(Yaw_now_degree_now), cosd(Roll_now_degree_now) * cosd(Yaw_now_degree_now) + sind(Roll_now_degree_now) * sind(Pitch_now_degree_now) * sind(Yaw_now_degree_now), cosd(Roll_now_degree_now) * sind(Pitch_now_degree_now) * sind(Yaw_now_degree_now) - sind(Roll_now_degree_now) * cosd(Yaw_now_degree_now);... 
          - sind(Pitch_now_degree_now), sind(Roll_now_degree_now) * cosd(Pitch_now_degree_now), cosd(Roll_now_degree_now) * cosd(Pitch_now_degree_now)];%变换矩阵
%---------------------------------------------
load_1 = [ - 100; - 100; - 100];                                                    %动平台铰接点位置(mm)
load_2 = [ - 100;100; - 100];
load_3 = [100;100; - 100];
load_4 = [100; - 100; - 100];
load_5 = [ - 100; - 100;100];
load_6 = [ - 100;100;100];
load_7 = [100;100;100];
load_8 = [100; - 100;100];

L_1=[-1000;-1000;-1000];                                                               %固定平台铰接点位置
L_2=[-1000; 1000;-1000];
L_3=[ 1000; 1000;-1000];
L_4=[ 1000;-1000;-1000];
L_5=[-1000;-1000;1000];
L_6=[-1000;1000;1000];
L_7=[1000;1000;1000];
L_8=[1000;-1000;1000];




T_1=TransM*load_1;
T_2=TransM*load_2;
T_3=TransM*load_3;
T_4=TransM*load_4;
T_5=TransM*load_5;
T_6=TransM*load_6;
T_7=TransM*load_7;
T_8=TransM*load_8;


%---------------------------------------
U_1=(T_1-L_1)/(sqrt((T_1(1)-L_1(1))^2+(T_1(2)-L_1(2))^2+(T_1(3)-L_1(3))^2));
U_2=(T_2-L_2)/(sqrt((T_2(1)-L_2(1))^2+(T_2(2)-L_2(2))^2+(T_2(3)-L_2(3))^2));
U_3=(T_3-L_3)/(sqrt((T_3(1)-L_3(1))^2+(T_3(2)-L_3(2))^2+(T_3(3)-L_3(3))^2));
U_4=(T_4-L_4)/(sqrt((T_4(1)-L_4(1))^2+(T_4(2)-L_4(2))^2+(T_4(3)-L_4(3))^2));
U_5=(T_5-L_5)/(sqrt((T_5(1)-L_5(1))^2+(T_5(2)-L_5(2))^2+(T_5(3)-L_5(3))^2));
U_6=(T_6-L_6)/(sqrt((T_6(1)-L_6(1))^2+(T_6(2)-L_6(2))^2+(T_6(3)-L_6(3))^2));
U_7=(T_7-L_7)/(sqrt((T_7(1)-L_7(1))^2+(T_7(2)-L_7(2))^2+(T_7(3)-L_7(3))^2));
U_8=(T_8-L_8)/(sqrt((T_8(1)-L_8(1))^2+(T_8(2)-L_8(2))^2+(T_8(3)-L_8(3))^2));
%------------------------------------------------------
U=[U_1,U_2,U_3,U_4,U_5,U_6,U_7,U_8];                                              %绳索单位向量:N

%--------------------------
J=[U;cross(T_1,U_1), cross(T_2,U_2) ,cross(T_3,U_3) ,cross(T_4,U_4) ,cross(T_5,U_5) ,cross(T_6,U_6) ,cross(T_7,U_7) ,cross(T_8,U_8) ];
%--------------------------------------------------

 end

