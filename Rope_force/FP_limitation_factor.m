function [F]= FP_limitation_factor(J,W,F_ini)
%   PIN=J'*inv(J*J');


F=F_ini-(pinv(J)*(W+J*F_ini));


% PIN_inv=PIN'* (pinv(PIN*PIN'));
% 
% 
% PPK=PIN_inv*(F_ini-F)-J*F_ini;
% 
% F_test=F_ini-(PIN*(PPK+J*F_ini));
end