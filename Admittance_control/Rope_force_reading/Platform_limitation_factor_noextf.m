function [W]= Platform_limitation_factor_noextf(J,F_rope)
%consult the educational paper
%--------------------------------------------------
F_max=[2000;2000;2000;2000 ;2000;2000 ;2000;2000];                          %N
F_min=[5;5; 5;5;  5;5;  5;5];                                               %N                                                  
%----------------------------------------------------------
% F_ini=0.5*(F_max+F_min);

% PIN=J'*pinv(J*J');  

% PIN_inv=PIN'* pinv(PIN*PIN');
% 
% W=PIN_inv*(F_ini-F_rope)-J*F_ini;


 W=-J*F_rope;

end