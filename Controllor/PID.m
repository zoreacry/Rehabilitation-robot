function []=PID_mypid(X_exp,V_exp,A_exp,X_now,V_now,A_now)

%------------------------------------------------------------------------------------------------------------
x_exp=X_exp(1);  y_exp=X_exp(2);  z_exp=X_exp(3);  Roll_exp=X_exp(4);  Pitch_exp=X_exp(5);  Yaw_exp=  X_exp(6);
x_v_exp=V_exp(1);y_v_exp=V_exp(2);z_v_exp=V_exp(3);Roll_v_exp=V_exp(4);Pitch_v_exp=V_exp(5);Yaw_v_exp=V_exp(6);
x_a_exp=A_exp(1);y_a_exp=A_exp(2);z_a_exp=A_exp(3);Roll_a_exp=A_exp(4);Pitch_a_exp=A_exp(5);Yaw_a_exp=A_exp(6);
%------------------------------------------------------------------------------------------------------------------
x_now=X_now(1);  y_now=X_now(2);  z_now=X_now(3);  Roll_now=X_now(4);  Pitch_now=X_now(5);  Yaw_now=  X_now(6);
x_v_now=V_now(1);y_v_now=V_now(2);z_v_now=V_now(3);Roll_v_now=V_now(4);Pitch_v_now=V_now(5);Yaw_v_now=V_now(6);
x_a_now=A_now(1);y_a_now=A_now(2);z_a_now=A_now(3);Roll_a_now=A_now(4);Pitch_a_now=A_now(5);Yaw_a_now=A_now(6);
%--------------------------------------------------------------------------------------------------------------------------

K_p=[1,0,0,0,0,0
    0,1,0,0,0,0
    0,0,1,0,0,0
    0,0,0,1,0,0
    0,0,0,0,1,0
    0,0,0,0,0,1];
K_i=[1,0,0,0,0,0
    0,1,0,0,0,0
    0,0,1,0,0,0
    0,0,0,1,0,0
    0,0,0,0,1,0
    0,0,0,0,0,1];
K_d=[1,0,0,0,0,0
    0,1,0,0,0,0
    0,0,1,0,0,0
    0,0,0,1,0,0
    0,0,0,0,1,0
    0,0,0,0,0,1];


    
  ek = setpoint - point;           %得到当前误差
    
  uk = (Kp*(ek - ek_1)+ Ki*ek + Kd*(ek - 2*ek_1 + ek_2));  
   
  ek_2 = ek_1;    
  ek_1 = ek;     
end

end