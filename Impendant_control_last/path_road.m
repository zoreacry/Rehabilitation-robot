clc
clear
%---------------------------------轨迹规划-----------------------------------------
%画出一条曲线
R=200;
Com=ones(6,200);
for i=1:360
    x=R*cosd(i);        
    y=R*sind(i);       
       Com(1,i)=x;
       Com(2,i)=y;
end

for j=1:360
   z=0.1*j;
       Com(3,j)=z;
end

for k=1:360
   a=1*k;
   b=0.08*k;
   c=0.07*k;
     Com(4,k)=a;
     Com(5,k)=b;
     Com(6,k)=c;
    
end
%==================================================================================
%计算速度加速度







%======================================================================================
%计算受力
%动力学输入输出力









 [Con_1]=
 
 
 
 
 
 
 
 
 
 
 