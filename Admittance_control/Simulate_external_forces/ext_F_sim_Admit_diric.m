function [F_sim_x,F_sim_y,F_sim_z,F_sim_R,F_sim_P,F_sim_Y]=ext_F_sim_Admit_diric(t)
 %-----------------------------------------------------------------------------
%  i=1;                                %running  times
%  F_sim_x=zeros(1,4000);
%  F_sim_y=zeros(1,4000);
%  F_sim_z=zeros(1,4000);
%  F_sim_R=zeros(1,4000);
%  F_sim_P=zeros(1,4000);
%  F_sim_Y=zeros(1,4000);
%  
 
 
 
%---------------------------------------------------------
     
 F_sim_x=10*diric(t,5); %updated                     
 F_sim_y=10*diric(t,6);    
 F_sim_z=10*diric(t,7);
 F_sim_R=0;
 F_sim_P=0;
 F_sim_Y=0;
 
%------------------------------------------------------------------------------- 

 
end

