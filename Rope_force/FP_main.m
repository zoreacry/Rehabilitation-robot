function [F_rea]=FP_main(x_now,y_now,z_now,Roll_now_degree_now,Pitch_now_degree_now,Yaw_now_degree_now,F_x,F_y,F_z,F_roll,F_pitch,F_yaw)
%-----------------------------------------------------------------
%set
F_max=[2000;2000;2000;2000 ;2000;2000 ;2000;2000];    %N
F_min=[5;5; 5;5;  5;5;  5;5];      %N
g=9.8;
m=5;%kg
%-------------------------------------------------------------
pose=[x_now;y_now;z_now;Roll_now_degree_now;Pitch_now_degree_now;Yaw_now_degree_now];
[J]=Force_Jacobi(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6)); 
W=[F_x;F_y ;m*g+F_z;F_roll ;F_pitch ;F_yaw];

%----------------------------------------------------------
F_ini=0.5*(F_max+F_min);
[F_rea]= FP_limitation_factor(J,W,F_ini);
end