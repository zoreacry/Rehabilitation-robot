function [position,Velocity,accelrate,progress] = pos_cont(x_0,y_0,z_0,Roll,Pitch,Yaw,x_1,y_1,z_1,Roll_start,Pitch_start,Yaw_start,time,currentvel,progress,target)

%=======================================以下都可以进行相应改动==========================%

cycle_time = 0.01;                                            %设定步长时间

maxaccel = 5;                                                 %设定最大加速度

feed_override = 1.2;                                          %设定倍率（进行瞬调）

maxvel = 10000;                                                %设定机床规定最大速度

reqvel = 150;                                                  %设定此用户给定的速度


%--------------------------------------------------进行每次画图-------------------------------


[progress, currentvel,newaccel] = tcRunCycle(progress, reqvel, currentvel,cycle_time,target,maxaccel, feed_override, maxvel);         %调用速度规划函数
    
    x_now = progress / target * (x_1 - x_0) + x_0;                                 %计算当前位置
    y_now = progress / target * (y_1 - y_0) + y_0;
    z_now = progress / target * (z_1 - z_0) + z_0;
    
    Roll_now_degree = (progress / target) * (Roll_start - Roll) + Roll;                    %计算当前角度
    Pitch_now_degree = (progress / target) * (Pitch_start - Pitch) + Pitch;
    Yaw_now_degree = (progress / target) * (Yaw_start - Yaw) + Yaw;

position =zeros(6,1);

position(1) =x_now;
position(2)=y_now;   
position(3)=z_now;   
position(4)=Roll_now_degree;   
position(5)=Pitch_now_degree;   
position(6)=Yaw_now_degree;                                                    %位置参数 

Velocity =zeros(6,1);

Velocity(1) =(currentvel/target)*(x_1-x_0);
Velocity(2) =(currentvel/target)*(y_1-y_0); 
Velocity(3) =(currentvel/target)*(z_1-z_0); 
Velocity(4) =(currentvel/target)*(Roll_start-Roll); 
Velocity(5) =(currentvel/target)*(Pitch_start-Pitch); 
Velocity(6) =(currentvel/target)*(Yaw_start-Yaw);                          %w位移速度与加速度位移数组

accelrate =zeros(6,1);

accelrate(1)=(newaccel/target)*(x_1-x_0);
accelrate(2)=(newaccel/target)*(y_1-y_0);
accelrate(3)=(newaccel/target)*(z_1-z_0);
accelrate(4)=(newaccel/target)*(Roll_start-Roll);
accelrate(5)=(newaccel/target)*(Pitch_start-Pitch); 
accelrate(6)=(newaccel/target)*(Yaw_start-Yaw);

end



