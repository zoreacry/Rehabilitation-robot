clc
clear
num = xlsread('C:\Users\13061\Desktop\workspace_data');
t=   num(7,1:9261);

y1=num(1,:);
y2=num(2,:);
y3=num(3,:);


scatter3(y1,y2,y3);
