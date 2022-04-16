function [cycle_time,maxaccel,feed_override,maxvel,reqvel] = Acc_A_dec_para()

cycle_time = 0.01;                                            %设定步长时间

maxaccel = 100;                                                 %设定加速度

feed_override = 1.2;                                          %设定倍率（进行瞬调）

maxvel = 10000;                                                %设定机床规定最大速度

reqvel = 150;                                                  %设定此用户给定的速度                                   

end

