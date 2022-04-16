clc
clear
%-----------------------------------------------
%Force workspace
%
Con_1=zeros(6,30);
i=1;
for   x=-1000:100:1000
    for   y=-1000:100:1000
        for   z=-1000:100:1000
            for   Roll=-45:9:45
                for   Pitch=-45:9:45
                    for   Yaw=-45:9:45
                        
                 Position=[x,y,z,Roll,Pitch,Yaw]';
                 F_x=0;F_y=0;F_z=0;F_R=0;F_P=0;F_Y=0;
                 [F_rea] =FP_method(Position(1),Position(2),Position(3),Position(4),Position(5),Position(6),F_x,F_y,F_z,F_R,F_P,F_Y);           
                 if F_rea(1)>0 && F_rea(2)>0 && F_rea(3)>0 && F_rea(4)>0 && F_rea(5)>0 && F_rea(6)>0 && F_rea(7)>0 && F_rea(8)>0 
                 
                  Con_1(:,i)=   Position;
                  i=i+1;
                 
                 end
                    end
                end
            end
        end
    end
end

    