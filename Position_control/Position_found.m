 function [position,Velocity,accelrate,progress,time,currentvel,newaccel]=Position_found(x_0,y_0,z_0,Roll,Pitch,Yaw,x_1,y_1,z_1,Roll_start,Pitch_start,Yaw_start,time,currentvel,progress,target)

%===========================================================%

[cycle_time,maxaccel,feed_override,maxvel,reqvel] = Acc_A_dec_para();

[progress, currentvel,newaccel] = tcRunCycle(progress, reqvel, currentvel, cycle_time  , target  , maxaccel, feed_override, maxvel);       
   time = time + cycle_time;   
   
    
    x_now = progress / target * (x_1 - x_0) + x_0;                         
    y_now = progress / target * (y_1 - y_0) + y_0;
    z_now = progress / target * (z_1 - z_0) + z_0;
    
    Roll_now_degree = (progress / target) * (Roll_start - Roll) + Roll;                 
    Pitch_now_degree = (progress / target) * (Pitch_start - Pitch) + Pitch;
    Yaw_now_degree = (progress / target) * (Yaw_start - Yaw) + Yaw;
    
                 
%------------------------------------------------------------------------------
position(1) =x_now;position(2)=y_now;   position(3)=z_now; position(4)=Roll_now_degree;   position(5)=Pitch_now_degree;   position(6)=Yaw_now_degree;                                            

Velocity(1) =(currentvel/target)*(x_1-x_0);Velocity(2) =(currentvel/target)*(y_1-y_0); Velocity(3) =(currentvel/target)*(z_1-z_0); Velocity(4) =(currentvel/target)*(Roll_start-Roll); Velocity(5) =(currentvel/target)*(Pitch_start-Pitch); Velocity(6) =(currentvel/target)*(Yaw_start-Yaw);           

accelrate(1)=(newaccel/target)*(x_1-x_0);
accelrate(2)=(newaccel/target)*(y_1-y_0);
accelrate(3)=(newaccel/target)*(z_1-z_0);
accelrate(4)=(newaccel/target)*(Roll_start-Roll);
accelrate(5)=(newaccel/target)*(Pitch_start-Pitch); 
accelrate(6)=(newaccel/target)*(Yaw_start-Yaw);

end



