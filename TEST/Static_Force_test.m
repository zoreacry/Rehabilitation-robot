clc
clear
i=1;
Col=zeros(8,20);
for x_now=-1000:40:1000
  for y_now=-1000:40:1000
      for z_now=-1000:40:1000

 Roll_now_degree_now=0;
Pitch_now_degree_now=0;
 Yaw_now_degree_now=0;
%---------------------------------------------------------------
%set
F_max=[2000;2000;2000;2000 ;2000;2000 ;2000;2000];                          %N
F_min=[5;5; 5;5;  5;5;  5;5];                                               %N
g=9.8;
m=10;                                                                        %kg
%-------------------------------------------------------------
pose=[x_now;y_now;z_now;Roll_now_degree_now;Pitch_now_degree_now;Yaw_now_degree_now];
[J]=Force_Jacobi(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6)); 
W=[0;0 ;m*g;0 ;0 ;0];

%----------------------------------------------------------
F_ini=0.5*(F_max+F_min);
[F_rea]= FP_limitation_factor(J,W,F_ini);


if F_rea(1)>0 && F_rea(2)>0 && F_rea(3)>0 && F_rea(4)>0 && F_rea(5)>0 && F_rea(6)>0 && F_rea(7)>0 && F_rea(8)>0

Col(1,i)=x_now;
Col(2,i)=y_now;
Col(3,i)=z_now;

end

i=i+1;
end
  end
end

data = rand(116725,3);

     scatter3(Col(1,:),Col(2,:),Col(3,:),[],data(:,3));
    view(45,45);
    xlabel('X-axis /(mm)');
    ylabel('Y-axis /(mm)');
    zlabel('Z-axis /(mm)');
    title('Workplace of Static Force');
grid on;
    
