%The main function focus on the force/position control in impendant control
%-
%-
%-
%-
clc
clear
%---------------------------------------------------------------------------------------
POSITION=zeros(6,20);
VELOCITY=zeros(6,20);
ACCELRATE=zeros(6,20);

%------------------------------------------------------------
x_0 = 0;                                
y_0 = 0;                              
z_0 = 0;

%---------------------------------------------------

Roll = 0;                          
Pitch = 0;                                   
Yaw = 0;

%----------------------------------------------------------

Roll_start = 0;           
Pitch_start = 0;                              
Yaw_start = 0;

x_1 = 450;                               
y_1 = 300;                                 
z_1 = 650;
%-------------------------------------------------------------------------------------
delta_x=[0;0;0;0;0;0];                        %Impedence control input x in wall
%-----------------------------------------------------------------------------------
target = sqrt((x_1 - x_0) ^ 2 + (y_1 - y_0) ^ 2 + (z_1 - z_0) ^ 2);         
progress = 0;                                                             
currentvel = 0;                                                      
time = 0;        
m=5;
T_s=0.01;
i=1 ;         
%--------------------------------------------------------------------------------------
while   progress < target
    
 [position,Velocity,Accelrate,progress,time,currentvel,newaccel]=Position_found(x_0,y_0,z_0,Roll,Pitch,Yaw,x_1,y_1,z_1,Roll_start,Pitch_start,Yaw_start,time,currentvel,progress,target);
  
 [Cho] = Impact_checking_main(progress);
 
 i=i+1;
 
 
POSITION(1,i)=position(1); POSITION(2,i)=position(2); POSITION(3,i)=position(3); POSITION(4,i)=position(4); POSITION(5,i)=position(5); POSITION(6,i)=position(6);
VELOCITY(1,i)=Velocity(1); VELOCITY(2,i)=Velocity(2); VELOCITY(3,i)=Velocity(3); VELOCITY(4,i)=Velocity(4); VELOCITY(5,i)=Velocity(5); VELOCITY(6,i)=Velocity(6);
ACCELRATE(1,i)=Accelrate(1); ACCELRATE(2,i)=Accelrate(2); ACCELRATE(3,i)=Accelrate(3); ACCELRATE(4,i)=Accelrate(4); ACCELRATE(5,i)=Accelrate(5); ACCELRATE(6,i)=Accelrate(6);
 
 if Cho ==0
     
     
 break;
 
 end

  
 
end

figure(1)
hold on;
plot(POSITION(1,:));plot(POSITION(2,:));plot(POSITION(3,:));plot(POSITION(4,:));plot(POSITION(5,:));plot(POSITION(6,:));

figure(2)
hold on;
plot(VELOCITY(1,:));plot(VELOCITY(2,:));plot(VELOCITY(3,:));plot(VELOCITY(4,:));plot(VELOCITY(5,:));plot(VELOCITY(6,:));

figure(3)
hold on;
plot(ACCELRATE(1,:));plot(ACCELRATE(2,:));plot(ACCELRATE(3,:));plot(ACCELRATE(4,:));plot(ACCELRATE(5,:));plot(ACCELRATE(6,:));