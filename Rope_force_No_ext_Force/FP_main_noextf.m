function [F_rea]=FP_main_noextf(x_now,y_now,z_now,Roll_now_degree_now,Pitch_now_degree_now,Yaw_now_degree_now)
%-----------------------------------------------------------------
%set
F_max=[2000;2000;2000;2000 ;2000;2000 ;2000;2000];                          %N
F_min=[5;5; 5;5;  5;5;  5;5];                                               %N
g=9.8;
m=5;                                                                        %kg
%-------------------------------------------------------------
pose=[x_now;y_now;z_now;Roll_now_degree_now;Pitch_now_degree_now;Yaw_now_degree_now];
[J]=Force_Jacobi(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6)); 
W=[0;0 ;m*g;0 ;0 ;0];

%----------------------------------------------------------
F_ini=0.5*(F_max+F_min);
[F_rea]= FP_limitation_factor(J,W,F_ini);
end