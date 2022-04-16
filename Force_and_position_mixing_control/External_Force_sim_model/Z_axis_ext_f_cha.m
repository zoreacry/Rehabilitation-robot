function [Output] = Z_axis_ext_f_cha(t)
%UNTITLED 此处显示有关此函数的摘要
%   此处显示详细说明

     
Output(1)=10*diric(t,5); %updated                     
Output(2)=10*diric(t,8);    
Output(3)=10*diric(t,10);
Output(4)=0;
Output(5)=0;
Output(6)=0;








end

