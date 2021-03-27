function [modified_trajec,new_first,right] = avoid_collision(first,waypoint_lead,min_dis, min_init,turn_right, mod)
%Function for avoiding collision between two leaders during their
%trajectories
%used in program : Two leader follower with union
modified_trajec=[];
 switch first
     case 1

                 modified_trajec(1)=waypoint_lead(1)+mod(1);
                 modified_trajec(2)=waypoint_lead(2)+mod(2);
                 
                 new_first=2;
                 right=0;
                
            
     case 2
                    if min_dis<=min_init
                     modified_trajec(1)=waypoint_lead(1)-mod(1);
                      modified_trajec(2)=waypoint_lead(2)-mod(2);
                     new_first=3;
                    right=0;
           
                    else
                   modified_trajec(1)=waypoint_lead(1)+mod(1);
                     modified_trajec(2)=waypoint_lead(2)-mod(2);
                     new_first=3;
                    right=1;
                    end
     case 3
               if turn_right==0
                    modified_trajec(1)=waypoint_lead(1)-mod(1);
                      modified_trajec(2)=waypoint_lead(2)-mod(2);
                      right=0;
                      new_first=3;
                elseif turn_right==1
                    modified_trajec(1)=waypoint_lead(1)+mod(1);
                      modified_trajec(2)=waypoint_lead(2)-mod(2);
                      right=0;
                      new_first=3;
               end
                    
              
                 
                    
 end 
end

