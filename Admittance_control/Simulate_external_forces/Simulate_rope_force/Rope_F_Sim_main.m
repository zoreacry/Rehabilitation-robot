function [Force_xyz,F_rea] = Rope_F_Sim_main(x_now,y_now,z_now,Roll_now_degree,Pitch_now_degree,Yaw_now_degree,t,dx_dd_act,dx_d_act)


position=[x_now,y_now,z_now,Roll_now_degree,Pitch_now_degree,Yaw_now_degree];

%----------------------------------------------------------------------------------------------------------------------
%                 exteral force shown
 [F_ex,F_ey,F_ez,M_ex,M_ey,M_ez]=ext_F_sim_Admit_diric(t);

  [T_Force]=Dynamics_eight_ropt(x_now,y_now,z_now,Pitch_now_degree,Yaw_now_degree,Roll_now_degree,dx_dd_act(1),dx_dd_act(2),dx_dd_act(3),dx_dd_act(4),dx_dd_act(5),dx_dd_act(6),dx_d_act(1),dx_d_act(2),dx_d_act(3),dx_d_act(4),dx_d_act(5),dx_d_act(6),F_ex,F_ey,F_ez,M_ex,M_ey,M_ez);
 [F_rea] =FP_method_noextf(x_now,y_now,z_now,Roll_now_degree,Pitch_now_degree,Yaw_now_degree);
[Force_xyz_ext]=Platform_F_main_noextf(position,T_Force);

Total_F=T_Force+F_rea;
%-------------------------------------------------------------------------------------------------------------------
[Force_rea]=Platform_F_main_noextf(position,F_rea);


    Force_xyz=Force_xyz_ext+Force_rea;
    
end




