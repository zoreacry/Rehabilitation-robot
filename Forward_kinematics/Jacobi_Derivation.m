clc
clear
syms x_now y_now z_now Yaw_now_degree Roll_now_degree Pitch_now_degree
syms l1_fk_find                               
syms l2_fk_find
syms l3_fk_find
syms l4_fk_find
syms l5_fk_find                               
syms l6_fk_find
syms l7_fk_find
syms l8_fk_find
[Hin_point] =Motion_Plat_Para();
load_1 = Hin_point(:,1);
load_2 = Hin_point(:,2);
load_3 = Hin_point(:,3);
load_4 = Hin_point(:,4);
load_5 = Hin_point(:,5);
load_6 = Hin_point(:,6);
load_7 = Hin_point(:,7);
load_8 = Hin_point(:,8);

TransM = [cosd(Pitch_now_degree) * cosd(Yaw_now_degree), sind(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree) - cosd(Roll_now_degree) * sind(Yaw_now_degree), sind(Roll_now_degree) * sind(Yaw_now_degree) + cosd(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree);cosd(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * cosd(Yaw_now_degree) + sind(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree) - sind(Roll_now_degree) * cosd(Yaw_now_degree); - sind(Pitch_now_degree), sind(Roll_now_degree) * cosd(Pitch_now_degree), cosd(Roll_now_degree) * cosd(Pitch_now_degree)];
[L1_point,L2_point,L3_point,L4_point,L5_point,L6_point,L7_point,L8_point]=Stat_plat_para();
    A_1_fk=L1_point;                                                                    %用向量表示出绳点
    A_2_fk=L2_point;
    A_3_fk=L3_point;
    A_4_fk=L4_point;
    A_5_fk=L5_point;
    A_6_fk=L6_point;
    A_7_fk=L7_point;
    A_8_fk=L8_point;
    
    P = [x_now;y_now;z_now];

    P_1_fk=TransM * load_1 + P;                                %计算某事件点上绳索铰接点在大坐标系下的位姿
    P_2_fk=TransM * load_2 + P;
    P_3_fk=TransM * load_3 + P;
    P_4_fk=TransM * load_4 + P;
    P_5_fk=TransM * load_5 + P;                                %计算某事件点上绳索铰接点在大坐标系下的位姿
    P_6_fk=TransM * load_6 + P;
    P_7_fk=TransM * load_7 + P;
    P_8_fk=TransM * load_8 + P;

    L1_fk= A_1_fk- P_1_fk;                                     %用向量描述绳长向量
    L2_fk= A_2_fk- P_2_fk;
    L3_fk= A_3_fk- P_3_fk;
    L4_fk= A_4_fk- P_4_fk;
    L5_fk= A_5_fk- P_5_fk;                               
    L6_fk= A_6_fk- P_6_fk;
    L7_fk= A_7_fk- P_7_fk;
    L8_fk= A_8_fk- P_8_fk;
    
    l1_fk=sqrt(L1_fk'*L1_fk);                                    %计算||L||
    l2_fk=sqrt(L2_fk'*L2_fk);
    l3_fk=sqrt(L3_fk'*L3_fk);
    l4_fk=sqrt(L4_fk'*L4_fk);
    l5_fk=sqrt(L5_fk'*L5_fk);                                   
    l6_fk=sqrt(L6_fk'*L6_fk);
    l7_fk=sqrt(L7_fk'*L7_fk);
    l8_fk=sqrt(L8_fk'*L8_fk);
    
    F_1_fk=l1_fk^2-l1_fk_find^2;                                 %计算绳长误差
    F_2_fk=l2_fk^2-l2_fk_find^2;
    F_3_fk=l3_fk^2-l3_fk_find^2;
    F_4_fk=l4_fk^2-l4_fk_find^2;
    F_5_fk=l5_fk^2-l5_fk_find^2;                               
    F_6_fk=l6_fk^2-l6_fk_find^2;
    F_7_fk=l7_fk^2-l7_fk_find^2;
    F_8_fk=l8_fk^2-l8_fk_find^2;

    
J_1=[diff(F_1_fk,x_now,1) diff(F_1_fk,y_now,1)     diff(F_1_fk,z_now,1)  diff(F_1_fk,Roll_now_degree,1) diff(F_1_fk,Pitch_now_degree ,1) diff(F_1_fk,Yaw_now_degree,1)];
J_2=[diff(F_2_fk,x_now,1) diff(F_2_fk,y_now,1)     diff(F_2_fk,z_now,1)  diff(F_2_fk,Roll_now_degree,1) diff(F_2_fk,Pitch_now_degree ,1) diff(F_2_fk,Yaw_now_degree,1)];
J_3=[diff(F_3_fk,x_now,1) diff(F_3_fk,y_now,1)     diff(F_3_fk,z_now,1)  diff(F_3_fk,Roll_now_degree,1) diff(F_3_fk,Pitch_now_degree ,1) diff(F_3_fk,Yaw_now_degree,1)];
J_4=[diff(F_4_fk,x_now,1) diff(F_4_fk,y_now,1)     diff(F_4_fk,z_now,1)  diff(F_4_fk,Roll_now_degree,1) diff(F_4_fk,Pitch_now_degree ,1) diff(F_4_fk,Yaw_now_degree,1)];
J_5=[diff(F_5_fk,x_now,1) diff(F_5_fk,y_now,1)     diff(F_5_fk,z_now,1)  diff(F_5_fk,Roll_now_degree,1) diff(F_5_fk,Pitch_now_degree ,1) diff(F_5_fk,Yaw_now_degree,1)];
J_6=[diff(F_6_fk,x_now,1) diff(F_6_fk,y_now,1)     diff(F_6_fk,z_now,1)  diff(F_6_fk,Roll_now_degree,1) diff(F_6_fk,Pitch_now_degree ,1) diff(F_6_fk,Yaw_now_degree,1)];
J_7=[diff(F_7_fk,x_now,1) diff(F_7_fk,y_now,1)     diff(F_7_fk,z_now,1)  diff(F_7_fk,Roll_now_degree,1) diff(F_7_fk,Pitch_now_degree ,1) diff(F_7_fk,Yaw_now_degree,1)];
J_8=[diff(F_8_fk,x_now,1) diff(F_8_fk,y_now,1)     diff(F_8_fk,z_now,1)  diff(F_8_fk,Roll_now_degree,1) diff(F_8_fk,Pitch_now_degree ,1) diff(F_8_fk,Yaw_now_degree,1)];

J=[J_1;J_2;J_3;J_4;J_5;J_6;J_7;J_8];
F_com=[F_1_fk;F_2_fk;F_3_fk;F_4_fk;F_5_fk;F_6_fk;F_7_fk;F_8_fk];

%(代码具体解释可见：rope_six_dof_fk.m)