function [S]=Selection_matrix(s)
%Select control mode direction matrix in force-position hybrid control

S=[ s(1),0,0,0,0,0;
    0,s(2),0,0,0,0;
    0,0,s(3),0,0,0;
    0,0,0,s(4),0,0;
    0,0,0,0,s(5),0;
    0,0,0,0,0,s(6)];




end
