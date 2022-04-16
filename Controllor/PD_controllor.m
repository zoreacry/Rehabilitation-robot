% function [error]=PD_controllor(x)
clc
clear

i=1;
%PID system parameters
%Incremental PID
% KP=1;
% KI=10;
% KD=1;
Con_1=zeros(1,20);
%--------------------------------------------------------------------
Speed=0;
SetSpeed = 0.2;
ActualSpeed = 0.2;
Err = 0.2;
Err_last = 0;
Kp = 0.02;
Ki = 0.15;
Kd = 0.02;
Voltage = 0;
Integral = 0;
%----------------------------------------
while abs(ActualSpeed)>0.00001
SetSpeed = Speed;
Err = SetSpeed - ActualSpeed;
Integral =Integral+ Err;
Voltage = Kp * Err + Ki * Integral + Kd *(Err - Err_last);
Err_last = Err;
ActualSpeed =Voltage * 1.0; 
	
    
    Con_1(i)=ActualSpeed;
    i=i+1;
 
end

plot(Con_1);
% end