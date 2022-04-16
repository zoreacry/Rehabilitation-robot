function [W]=Platform_F_main_noextf(position,F_rope)


%-------------------------------------------------------------

[J]=Force_Jacobi(position(1),position(2),position(3),position(4),position(5),position(6)); 
%----------------------------------------------------------
[W]= Platform_limitation_factor_noextf(J,F_rope);
end