clc
clear
%-------------------------------------------------
%屈伸运动
%原点为（0，0）
%杆一长350mm、杆二长300mm
%角1：ang_1 ; 角2：ang_2
%--------------------------------------------------
%圆：圆心（0，200，0） 半径（100）
i=1;
% axis([-1000,1000,-1000,1000]);
L_1=350;
L_2=300;
Con_Pos=zeros(2,20);
r = 100;%半径
a = 0;%圆心横坐标
b = 300;%圆心纵坐标
for theta = 0:pi/20:2*pi %角度[0,2*pi] 

x = a+r*cos(theta);
y = b+r*sin(theta);



    C2=(x.^2+y.^2-L_1.^2-L_2.^2)/(2*L_1*L_2);%计算C2角
    Th2=acos(C2);%获得th2角度
    K1=L_1+L_2*cos(Th2);%计算K1
    K2=L_2*sin(Th2);%计算K2
    Th1=atan2(y,x)-atan2(K2,K1);%使用K1\K2,计算Th1角
    %当Th2变化时，C2,S2也变动，K1\K2变化，Th1也跟随变动。


Th_1_deg=Th1*180/pi;
Th_2_deg=Th2*180/pi;

Con_Pos(1,i)=Th_1_deg;
Con_Pos(2,i)=Th_2_deg-90;
i=i+1;

end
hold on
grid on
plot(Con_Pos(1,:));
plot(Con_Pos(2,:));

    xlabel('time/(s)');
    ylabel('Angle Change /(°)');
    title('Flexion And Extension Movement');
