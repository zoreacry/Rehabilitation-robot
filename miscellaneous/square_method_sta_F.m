function [W]=square_method_sta_F(position)
%ready to delete
%moving on.....................

syms F_1 F_2 F_3 F_4 F_5 F_6 F_7 F_8 
F_rope=[F_1 F_2 F_3 F_4 F_5 F_6 F_7 F_8 ]' ;

[J]=Force_Jacobi(position(1),position(2),position(3),position(4),position(5),position(6)); % Jacobian Function


W=-J*F_rope;

end