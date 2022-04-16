function [dx_dd_re,dx_d_re,dx_re]=Impedance_control_main(M,B,K,T_s,dx_re,dx_d_re,dx_dd_re,delta_x)

%---------------------------------------------------------
% 在碰到一些物体时，通过此种方法可以迅速停止当前运动，保护机器或操作员
%修改：
%1.M\B\K
%2.修改F_ext  应与运动平台传感器力同
%-------------------------------------设定环境模拟力---------------------------------------------
[F_wall]=Sim_Sec_wall(delta_x);  %need updated
F_ext=F_wall;                                            %弹簧力(受外力/墙)
%-----------------------------------------------------------------
%F_ext=M*dx_dd+B*dx_d+K*dx
    
   dx_dd_act= inv(M)*(F_ext-B*dx_d_re-K*dx_re);
   dx_d_act = (T_s/2)*(dx_dd_act+dx_dd_re)+dx_d_re;
   dx_act   = (T_s/2)*(dx_d_act+dx_d_re)+dx_re;                         %need updated
   
       dx_dd_re = dx_dd_act;
       dx_d_re = dx_d_act;
       dx_re = dx_act;
   
   
   
   
end