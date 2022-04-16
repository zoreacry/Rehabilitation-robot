clc
clear;
%close all
tic
U=1200;
W=150;
L=250;
H=100;
i=1;
%---------------------------------平台基本尺寸（可以进行改动）---------------------------------
Col=zeros(8,20);
for x_now=-U:10:U
  for y_now=-U:10:U
      for z_now=500


L1_point_x = - U;                                             
L1_point_y = - U;
L1_point_z = - U;
Point_1=[L1_point_x;L1_point_y;L1_point_z];

L2_point_x = - U;                
L2_point_y = U;
L2_point_z = - U;
Point_2=[L1_point_x;L2_point_y;L2_point_z];

L3_point_x = U;                                          
L3_point_y = U;
L3_point_z = - U;
Point_3=[L3_point_x;L3_point_y;L3_point_z];

L4_point_x = U;                                          
L4_point_y = - U;
L4_point_z = - U;
Point_4=[L4_point_x;L4_point_y;L4_point_z];

L5_point_x = - U;                                            
L5_point_y = - U;
L5_point_z = U;
Point_5=[L5_point_x;L5_point_y;L5_point_z];


L6_point_x = - U;                                 
L6_point_y = U;
L6_point_z = U;
Point_6=[L6_point_x;L6_point_y;L6_point_z];

L7_point_x = U;                                        
L7_point_y = U;
L7_point_z = U;
Point_7=[L7_point_x;L7_point_y;L7_point_z];

L8_point_x = U;                                      
L8_point_y = - U;
L8_point_z = U;
Point_8=[L8_point_x;L8_point_y;L8_point_z];


Roll_now_degree=0;
Pitch_now_degree=0;
Yaw_now_degree=0;

    P = [x_now;y_now;z_now];                                                       %动平台相对于静平台的实时位置
    
    TransM = [cosd(Pitch_now_degree) * cosd(Yaw_now_degree), sind(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree) - cosd(Roll_now_degree) * sind(Yaw_now_degree), sind(Roll_now_degree) * sind(Yaw_now_degree) + cosd(Roll_now_degree) * sind(Pitch_now_degree) * cosd(Yaw_now_degree);cosd(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * cosd(Yaw_now_degree) + sind(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree), cosd(Roll_now_degree) * sind(Pitch_now_degree) * sind(Yaw_now_degree) - sind(Roll_now_degree) * cosd(Yaw_now_degree); - sind(Pitch_now_degree), sind(Roll_now_degree) * cosd(Pitch_now_degree), cosd(Roll_now_degree) * cosd(Pitch_now_degree)];
    %XYZ旋转矩阵
    %-----------------------------------------计算八个铰点的位置矢量----------------------------------
    
    
% load_1 = [ - 200; - 150; - 100];                                                   
% load_2 = [ - 200;150; - 100];  
% load_3 = [200;150; - 100];     
% load_4 = [200; - 150; - 100]; 
% load_5 = [ - 200; - 150;100];
% load_6 = [ - 200;150;100]; 
% load_7 = [200;150;100];        
% load_8 = [200; - 150;100];  
    
load_1 = [ - W; - L; - H];                                                   
load_2 = [ - W;L; - H];  
load_3 = [W;L; - H];     
load_4 = [W; - L; - H]; 
load_5 = [ - W; - L;H];
load_6 = [ - W;L;H]; 
load_7 = [W;L;H];        
load_8 = [W; - L;H];  


    now_1 = TransM * load_1 + P;
    x_1_1 = now_1(1, 1);
    y_1_1 = now_1(2, 1);
    z_1_1 = now_1(3, 1);
    

    now_2 = TransM * load_2 + P;
    x_2 = now_2(1, 1);
    y_2 = now_2(2, 1);
    z_2 = now_2(3, 1);
    

    now_3 = TransM * load_3 + P;
    x_3 = now_3(1, 1);
    y_3 = now_3(2, 1);
    z_3 = now_3(3, 1);
    

    now_4 = TransM * load_4 + P;
    x_4 = now_4(1, 1);
    y_4 = now_4(2, 1);
    z_4 = now_4(3, 1);
    

    now_5 = TransM * load_5 + P;
    x_5 = now_5(1, 1);
    y_5 = now_5(2, 1);
    z_5 = now_5(3, 1);
    

    now_6 = TransM * load_6 + P;
    x_6 = now_6(1, 1);
    y_6 = now_6(2, 1);
    z_6 = now_6(3, 1);
    

    now_7 = TransM * load_7 + P;
    x_7 = now_7(1, 1);
    y_7 = now_7(2, 1);
    z_7 = now_7(3, 1);
    

    now_8 = TransM * load_8 + P;
    x_8 = now_8(1, 1);
    y_8 = now_8(2, 1);
    z_8 = now_8(3, 1);
    
   
%=================================================
    
Line_One=Point_1-now_1;
Line_Two=Point_2-now_2;
Line_Thr= Point_3-now_3;
Line_Fou=Point_4-now_4;
Line_Fiv=  Point_5-now_5;
Line_Six= Point_6-now_6;
Line_Sev=Point_7-now_7;
Line_Eig= Point_8-now_8;
                                
    
Theta_1=atan2(Line_One(1),Line_One(2));
Theta_deg_1=Theta_1*180/pi;

Theta_2=atan2(Line_Two(1),Line_Two(2));
Theta_deg_2=Theta_2*180/pi;

Theta_3=atan2(Line_Thr(1),Line_Thr(2));
Theta_deg_3=Theta_3*180/pi;

Theta_4=atan2(Line_Fou(1),Line_Fou(2));
Theta_deg_4=Theta_4*180/pi;

Theta_5=atan2(Line_Fiv(1),Line_Fiv(2));
Theta_deg_5=Theta_5*180/pi;

Theta_6=atan2(Line_Six(1),Line_Six(2));
Theta_deg_6=Theta_6*180/pi;

Theta_7=atan2(Line_Sev(1),Line_Sev(2));
Theta_deg_7=Theta_7*180/pi;

Theta_8=atan2(Line_Eig(1),Line_Eig(2));
Theta_deg_8=Theta_8*180/pi;


if  Theta_deg_3>38.65 &&Theta_deg_4<180-(38.65)
Col(1,i)=x_now;
Col(2,i)=y_now;
end

i=i+1;
      end
  end
end

hold on;
rectangle('Position',[ 300,-250, 200 400],'edgecolor','k','facecolor','g','linewidth',1.8) 

hold on;
rectangle('Position',[ -200,150, 600 100],'edgecolor','k','facecolor','g','linewidth',1.8) 

axis([-1000 1000 -1000 1000])
plot(Col(1,:),Col(2,:),'o');
xlabel('X-axis /(mm)');
ylabel('Y-axis /(mm)');
zlabel('Point Point');
grid on;

