function [X_out]=Dynamics_least_square_function(x,y,z,Roll_now_degree,Pitch_now_degree, Yaw_now_degree,a_x,a_y,a_z,omega_x_d,omega_y_d,omega_z_d,v_x,v_y,v_z,omega_x,omega_y,omega_z,F_ex,F_ey,F_ez,M_ex,M_ey,M_ez,m_p)
x0=[0 0 0 0 0 0 0 0];
%options=optimset('Algorithm','Levenberg-Marquardt','OutputFcn', @LMoutfun,'MaxIter',1000,'MaxFunEvals',10000,'TolCon',1e-6,'TolFun',1e-6,'TolX',1e-6);
options=optimset('Algorithm','levenberg-marquardt');
[x]=fsolve(@(x0)Dynamics_least_square_main(x0,x,y,z,Roll_now_degree,Pitch_now_degree, Yaw_now_degree, a_x,a_y,a_z,omega_x_d,omega_y_d,omega_z_d,v_x,v_y,v_z,omega_x,omega_y,omega_z,F_ex,F_ey,F_ez,M_ex,M_ey,M_ez,m_p),x0,options);


X_out=x;

end