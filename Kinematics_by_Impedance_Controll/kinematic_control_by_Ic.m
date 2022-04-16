function [delta_Len]=kinematic_control_by_Ic(Len_now,x_next)

x_now=x_next(1);
y_now=x_next(2);
z_now=x_next(3);
Roll_now_degree=x_next(4);
Pitch_now_degree=x_next(5);
Yaw_now_degree=x_next(6);
%---------------------------------------------------------------


P = [x_now;y_now;z_now];                                                       %动平台相对于静平台的实时位置
    
TransM = [cosd(Pitch_now_degree) * cosd(Yaw_now_degree), sind(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree) - cosd(Roll_now_degree) * sind(Yaw_now_degree), sind(Roll_now_degree) * sind(Yaw_now_degree) + cosd(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree);cosd(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * cosd(Yaw_now_degree) + sind(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree) - sind(Roll_now_degree) * cosd(Yaw_now_degree); - sind(Pitch_now_degree), sind(Roll_now_degree) * cosd(Pitch_now_degree), cosd(Roll_now_degree) * cosd(Pitch_now_degree)];
    %XYZ旋转矩阵
 %-----------------------------------------计算八个铰点的位置矢量----------------------------------
load_1 = [ - 100; - 100; - 100];
    now_1 = TransM * load_1 + P;
    x_1_1 = now_1(1, 1);
    y_1_1 = now_1(2, 1);
    z_1_1 = now_1(3, 1);
    
load_2 = [ - 100;100; - 100];
    now_2 = TransM * load_2 + P;
    x_2 = now_2(1, 1);
    y_2 = now_2(2, 1);
    z_2 = now_2(3, 1);
    
load_3 = [100;100; - 100];
    now_3 = TransM * load_3 + P;
    x_3 = now_3(1, 1);
    y_3 = now_3(2, 1);
    z_3 = now_3(3, 1);
    
load_4 = [100; - 100; - 100];
    now_4 = TransM * load_4 + P;
    x_4 = now_4(1, 1);
    y_4 = now_4(2, 1);
    z_4 = now_4(3, 1);
    
load_5 = [ - 100; - 100;100];
    now_5 = TransM * load_5 + P;
    x_5 = now_5(1, 1);
    y_5 = now_5(2, 1);
    z_5 = now_5(3, 1);
    
load_6 = [ - 100;100;100];
    now_6 = TransM * load_6 + P;
    x_6 = now_6(1, 1);
    y_6 = now_6(2, 1);
    z_6 = now_6(3, 1);
    
load_7 = [100;100;100];
    now_7 = TransM * load_7 + P;
    x_7 = now_7(1, 1);
    y_7 = now_7(2, 1);
    z_7 = now_7(3, 1);
    
load_8 = [100; - 100;100];
    now_8 = TransM * load_8 + P;
    x_8 = now_8(1, 1);
    y_8 = now_8(2, 1);
    z_8 = now_8(3, 1);

%-------------------------------------------------------------------------

 [L1_point,L2_point,L3_point,L4_point,L5_point,L6_point,L7_point,L8_point]=Stat_plat_para();
 
    Len_next(1)=sqrt((L1_point(1) - x_1_1 )^2+(L1_point(2) - y_1_1 )^2+(L1_point(3) - z_1_1 )^2);
    Len_next(2)=sqrt((L2_point(1) - x_2 )^2+(L2_point(2) - y_2 )^2+(L2_point(3) - z_2 )^2);
    Len_next(3)=sqrt((L3_point(1) - x_3 )^2+(L3_point(2) - y_3 )^2+(L3_point(3) - z_3 )^2);
    Len_next(4)=sqrt((L4_point(1) - x_4 )^2+(L4_point(2) - y_4 )^2+(L4_point(3) - z_4 )^2);
    Len_next(5)=sqrt((L5_point(1) - x_5 )^2+(L5_point(2) - y_5 )^2+(L5_point(3) - z_5 )^2);
    Len_next(6)=sqrt((L6_point(1) - x_6 )^2+(L6_point(2) - y_6 )^2+(L6_point(3) - z_6 )^2);
    Len_next(7)=sqrt((L7_point(1) - x_7 )^2+(L7_point(2) - y_7 )^2+(L7_point(3) - z_7 )^2);
    Len_next(8)=sqrt((L8_point(1) - x_8 )^2+(L8_point(2) - y_8 )^2+(L8_point(3) - z_8 )^2);
    
    
delta_Len=Len_next-Len_now;
    
end







