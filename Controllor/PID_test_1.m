clc
clear

Input=10;
Output=0;
Setpoint=0;
errSum=0;
lastErr=0;
error=1;
i=1;

   kp = 0.1;
   ki = 1;
   kd = 1;
%--------------------------------
nowTime=0.1;
lastTime=0;
%---------------------------------


while     abs( error)>0.0001
    
timeChange = nowTime - lastTime;

error = Setpoint - Input;
errSum = errSum + ( error * timeChange );
dErr = ( error - lastErr) / timeChange;
  
Output = kp * error + ki * errSum + kd * dErr;
  
lastErr =  error;
lastTime = now;




i=i+1;
end
