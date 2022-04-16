function [dx_dd_re,dx_d_re,dx_re]=Impedance_control_PF_contr(M,B,K,T_s,dx_re,dx_d_re,dx_dd_re,i,S_inv)

%---------------------------------------------------------
%修改：
%1.M\B\K
%2.修改F_ext  应与运动平台传感器力同
%-------------------------------------设定环境模拟力---------------------------------------------
[Output] = Z_axis_ext_f_cha(i);
F_ext=S_inv*Output'; 
                                                           %弹簧力(受外力/墙)
%-----------------------------------------------------------------
   dx_dd_act= inv(M)*(F_ext-B*dx_d_re-K*dx_re);
   dx_d_act = (T_s/2)*(dx_dd_act+dx_dd_re)+dx_d_re;
   dx_act   = (T_s/2)*(dx_d_act+dx_d_re)+dx_re;                             %need updated
   
       dx_dd_re = dx_dd_act;
       dx_d_re = dx_d_act;
       dx_re = dx_act;
   
       
       
   
   
   
end

