function [M,B,K]=I_C_Para()

M=[0.1,0,0,0,0,0;...
   0,0.01,0,0,0,0;...
   0,0,0.1,0,0,0;...
   0,0,0,1,0,0;...
   0,0,0,0,1,0;...
   0,0,0,0,0,1];
B=[1,0,0,0,0,0;...
   0,1,0,0,0,0;...
   0,0,1,0,0,0;...
   0,0,0,1,0,0;...
   0,0,0,0,2,0;...
   0,0,0,0,0,4];
K=[1,0,0,0,0,0;...
   0,1,0,0,0,0;...
   0,0,1,0,0,0;...
   0,0,0,1,0,0;...
   0,0,0,0,1,0;...
   0,0,0,0,0,1];
end