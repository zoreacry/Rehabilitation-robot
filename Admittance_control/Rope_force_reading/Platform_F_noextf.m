 function [F_rea] =Platform_F_noextf(position,Rope_Force)
% main function
F_rope=Rope_Force;

[F_rea]=Platform_F_main_noextf(position,F_rope);
 end
