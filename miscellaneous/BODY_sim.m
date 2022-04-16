clc
clear
%body workspace plot
%----------------------------------------------

i=1;

Con_Pos=zeros(3,20);
Con_1=zeros(1,20);

O_pos_x=300;
O_pos_y=250;
O_pos_z=0;

L_1=350;
L_2=300;

%-----------------------------------------------
for th_1=0:10:80
    for th_2=-40:10:70
        for th_3=-40:10:70
            for th_4=0:10:135
                for th_5=0

% syms O_pos_x O_pos_y O_pos_z
% 
% syms th_1 th_2 th_3 th_4 th_5
% 
% syms L_1 L_2

% th_1=th1*pi/180;
% th_2=th2*pi/180;
% th_3=th3*pi/180;
% th_4=th4*pi/180;
% th_5=th5*pi/180;



T_1=[0 -1    0  O_pos_x ;...
         1  0     0  O_pos_y ;...
         0  0     1  O_pos_z ;...
         0  0     0         1];
T_2=[cosd(th_1) -sind(th_1)    0  0 ;...
         0  0 -1 0 ;...
         sind(th_1)  cosd(th_1)     0  0 ;...
         0    0    0       1];
T_3=[cosd(th_2) -sind(th_2)    0  0 ;...
         0  0  1  0 ;...
         -sind(th_2)  -cosd(th_2)  0  0 ;... 
         0    0    0       1] ;
T_4=[cosd(th_3) -sind(th_3)    0  0 ;...
         0  0      1   -L_1 ;...
         -sind(th_3)  -cosd(th_3)  0  0;... 
         0    0    0    1];  
T_5=[cosd(th_4) -sind(th_4)    0  0;...
         0  0  1  0 ;...
         -sind(th_4)  -cosd(th_4)  0  0;... 
         0  0  0  1];  
T_6=[cosd(th_5) -sind(th_5)    0  -L_2 ;...
         sind(th_5)  cosd(th_5)     0     0 ;...
         0                    0               1     0 ;...
         0                    0               0     1];  
     
     T_fin=T_1*T_2*T_3*T_4*T_5*T_6;
     
     x_now=T_fin(1,4);
     y_now=T_fin(2,4);
     z_now=T_fin(3,4);
     
   Con_1(1,i)= T_fin(3,3); 
 Con_Pos(1,i)=x_now;    
 Con_Pos(2,i)=y_now;    
 Con_Pos(3,i)=z_now;    
    i=i+1;
    
    if abs(x_now)<  15 || abs(y_now)<  15 || abs(z_now)<  15 
    break;
    end
                end
            end
        end
    end
end

     scatter3(Con_Pos(1,:),Con_Pos(2,:),Con_Pos(3,:),'r.');
    view(45,45);
    xlabel('X-axis /(mm)');
    ylabel('Y-axis /(mm)');
    zlabel('Z-axis /(mm)');
    title('Workplace of leg');
grid on;
    