function [Cho] = Impact_checking_main(tot_dis)
%Impact_checking,When it was been activated,will start to use impendant
%control(like protect wall like a ball

limit_dis=400 ;                                            %mm


if  tot_dis < limit_dis
    
    
Cho=1;

else

Cho=0;


end

