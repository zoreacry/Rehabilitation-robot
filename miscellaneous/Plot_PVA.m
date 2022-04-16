function [POSITION,VELOCITY,ACCELRATE] = Plot_PVA(position,Velocity,Accelrate,i)
%Plot_PVA    plot the Position & Velocity & Accelrate
%  when it moving.......


POSITION(1,i)=position(1); POSITION(2,i)=position(2); POSITION(3,i)=position(3); POSITION(4,i)=position(4); POSITION(5,i)=position(5); POSITION(6,i)=position(6);
VELOCITY(1,i)=Velocity(1); VELOCITY(2,i)=Velocity(2); VELOCITY(3,i)=Velocity(3); VELOCITY(4,i)=Velocity(4); VELOCITY(5,i)=Velocity(5); VELOCITY(6,i)=Velocity(6);
ACCELRATE(1,i)=Accelrate(1); ACCELRATE(2,i)=Accelrate(2); ACCELRATE(3,i)=Accelrate(3); ACCELRATE(4,i)=Accelrate(4); ACCELRATE(5,i)=Accelrate(5); ACCELRATE(6,i)=Accelrate(6);


end


