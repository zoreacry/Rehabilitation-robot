function [F]= FP_limitation_factor(J,W,F_ini)
PIN=J'*pinv(J*J');
% F=F_ini-lsqminnorm(J,(W+J*F_ini));
F=F_ini-PIN*(W+J*F_ini);
end