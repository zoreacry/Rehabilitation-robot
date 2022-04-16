clc
clear
i=1;
for theta =0:pi/180:2*pi
%  for theta=0:pi/180:2*pi



r = 20;%半径
a = 10;%圆心横坐标
b = 15;%圆心纵坐标
x = a+r*cos(theta);
y = b+r*sin(theta);
con_1(i)=x;
con_2(i)=y;
i=i+1;
end
plot(con_1,con_2,'-bo',  'LineWidth',2.5,'MarkerIndices',1:5:length(con_1),    'LineWidth',2,...
    'MarkerEdgeColor','r',...
    'MarkerFaceColor',[.49 1 .63],...
    'MarkerSize',5);

axis([-80,80,-80,80]);
grid on;
