function dy = myfun(t,y)
%UNTITLED2 此处显示有关此函数的摘要
%   此处显示详细说明
dy=zeros(2,1);
dy(1)=y(2);
dy(2)=-20*y(1)-4*y(2)-10*t;
end

