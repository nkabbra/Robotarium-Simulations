%Two leader follower with union

%Master ATSI : Multi Agent System Course

%Group 3: Nadine KABBARA
%Leonardo FELIPE TOSO
%Lara JABER
%Soha KANSO
%Fadhlallah BOUDEHANE

%Two leaders each try to follow their own trajectories while deploying 
%an obstacle avoidance learning algorithm and avoiding accidents between
%each other
%two leaders finally join into one single formation with a single leader
%and a common trajectory
%% Robots and robotarium

% Number of robots
N = 8;

% initial poses

initial_positions=1*[-10 -13 -16 -13.5 -12 -10.6 -13.5 -12.5  ;0 1 -1 0.5 1 0 4 5 ;0.4,0.4,0.4,0.4 0.4 0.4 0.4 0.3]; 

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

     
%% Number of iterations in experiment
% Select the number of iterations for the experiment.
iterations = 10000;
% Sampling time (not to be modified)
Ts = 0.033;



%% ----- Plot tools (to be removed for experiments)
% Plot tools
rPlot = RobotariumPlots(N, iterations, Ts);

 M=eye(N,N);
 L=zeros(N,N);
 %L(2:N,1:N)=-M(1:N-1,1:N);
 % communication graph with two leaders each having a seperate trajectory
L=[ 4  0  0  0  -1 -1  -1  -1;
   -1  2 -1  0  0  0  0  0;
    0 -1  2 -1  0  0  0  0;
    0  0 -1  1  0  0  0  0;
    -1  -1  -1 -1  4  0  0  0;
    0  0  0  0 -1  2 -1  0;
    0  0  0  0  0 -1  2 -1;
    0  0  0  0  0  0  -1 1;
     
    ];
%communication graph after combining two leaders into one single
%leader with one trajectory
L_united=[  1  0  0  0  -1  0  0  0;
           0  2   0  0 -1  -1  0  0;
            0 0 2 0  0  -1  -1 0;
            0  0  0  2  0  0 -1  -1;
            -1  -1  0  0  2  0  0  0;
            0  -1  -1  0  0  2 0  0;
            0  0  -1  -1  0 0  2 0;
            0  0  0  0  0  0  0  0;
      

    ];
%Initialize velocity vector
dxi = zeros(2, N);

%States for leaders
%state of leader 1
state = 1;
%state of leader 2
state_lead2=1;
%state of new leader after combination
state_united=1;

% These are gains for our formation control algorithm
formation_control_gain = 12;
%desired distance between robots
desired_distance = 0.3;


%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 1.2);
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate2();
% Single-integrator position controller
leader_controller = create_si_position_controller('XVelocityGain', 1.2, 'YVelocityGain', 1.2, 'VelocityMagnitudeLimit', 0.1);




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Trajectory initialization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%trajectory points for leader one
waypoints=[];
%trajectory points for leader two
waypoints_lead2=[];
i=0;
% leader one follows cosine trajectory and leader two follows cosine
% trajectory dephased by angle pi
for x=-10:0.1:3.5
    
  
    if x>=3
        waypoints_lead2=[waypoints_lead2;[x+0.2 0]];
    
    else
    waypoints_lead2=[waypoints_lead2;[x 0.4*cos(x+pi)]];
    
    end
    if x>=2.1
        waypoints=[waypoints;[x -1.2]]
    else
         waypoints=[waypoints;[x 1.3*cos(x)]];
    end
    i=i+1;
end

% final trajectory of leader one and leader two just before combination is
% a line
for x=3.6:0.1:5
   i=i+1
    waypoints=[waypoints;[x -1.2]]
   waypoints_lead2=[waypoints_lead2;[x+0.2 0]];
    i=i+1;
  
end
%trajectory points for new leader after combination
waypoints_united=[];
for i=120:-1:1
    waypoints_united=[waypoints_united;[waypoints(i,1) waypoints(i,2)]];
end

waypoints_united=waypoints_united';
waypoints=waypoints';
waypoints_lead2=waypoints_lead2';
plot(waypoints(1,:),waypoints(2,:),'-','LineWidth',1)
plot(waypoints_lead2(1,:),waypoints_lead2(2,:),'-','LineWidth',1)

%Obstacle Positions
test=[-3 0.396;0.4 -0.3684]';
plot(test(1,:),test(2,:),'o','LineWidth',2);

%minimum distance for reaching trajectory point
close_enough = 0.03;



% initialzing first point in trajectory for position control 
 waypoint = waypoints(:, state);
 waypoint_lead2 = waypoints_lead2(:, state_lead2);
 waypoint_united = waypoints_united(:, state_united);
 right=1;
 move=0;
 first=1;
 first_ld=1;
 % flag for combination of leaders one and two
 united=0;
 %desired distance between robots
 desired_distance=0.3;
 min_init=0.1;
 ct=[0 0];
 count=0;
 move=0;
 k=1.1;
 turn_right(1)=0;
 for i=1:N
 %indicates current step in learning algorithm
 %first_foll=1 saves current distance of follower from obstacle
 %first_foll=2 applies collision avoidance algorithm based on learning
 first_foll(i)=1;
 %flag for obstacle avoidance learning algorithm
up(i)=1;
 end
 
 
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Main Program %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
for t = 1:iterations
 % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
   
    
     %% Algorithm
    
for i = 1:N
        
        %Zero velocity and get the topological neighbors of agent i
        dxi(:, i) = [0 ; 0];
        
        neighbors = topological_neighbors(L, i);
        
   % control algorithm for followers of leader one (index one) and leader two (index five ) before combination     
   if  x(1,5)<4.8 && united==0
       
       %for followers of leader two, we want to learn control algorithm with
       %obstacle avoidance
       %We assume that followers have no prior knowledge of obstacle
       %positions
       
       %calculate minimum distance from followers to obstacle
       if i>5
             for m=1:length(test)
                  s(m)=norm(x(1:2, i) - test(:,m));
             end
                minn=min(s);
          
             %no near obstacle   
             if (minn>0.2)
                 first_foll(i)=1;
                  up(i)=1;
                 
             end
             
             %obstacle learning algorithm
             
             if (minn<=0.2)
               
                 
                %first try to move up and save original distance away
                %from obstacle
               if first_foll(i)==1
                    
                ct=[0.065 up(i)*0.5];
               
                min_dis_foll(i)=minn;
                first_foll(i)=2;
                
            %if moving  upwards was beneficial (i.e it avoided collision with obstacle
            % then continue moving upwards,else  
             %move downwards
               else
                    if first_foll(i)==2
                   
                        ct=[0.065 up(i)*0.5];
                        %flip direction of motion to avoid obstacle if
                        %chosen direction was not correct (the robot hence iteratively learns the best direction to choose 
                        %to avoid the obstacle)
                        if(sign(minn-min_dis_foll(i))<0 &&up(i)==1)
                            up(i)=-1;
                        elseif(sign(minn-min_dis_foll(i))<0 &&up(i)==-1)
                            up(i)=1;
                        end
                        first_foll(i)=1;
                   end
                
               end
             end
          
       end
        %apply learned algorithm
                 for j = neighbors
                if x(1,5)>1 && i>=6
                    desired_distance=desired_distance+0.01;
                else
                    desired_distance=0.3;
                    
                end
                if i<5
                    ct=[0 0];
                   
                end
                %choose appropriate gains for learned algorithm 
                if x(1,5)>0.55
                    k=1.1;
                else 
                    k=4.1;
                end
                if x(1,5)>3 && i>5
                    desired_distance=0.44;
                end
                    
             dxi(:, i) = dxi(:, i) + ...
                    formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*k*(x(1:2, j) - x(1:2, i)+ct');    

                 end
  
                   
        %control algorithm for leader five with a learning algorithm that avoids accidents with
        %leader one and its followers
        %Note that we assume the leaders have prior knowledge of obstacle
        %positons from sensor data
        if i==5
          
            %calculate distance of leader two from leader one and its
            %followers
             for j=neighbors
                 s(j)=norm(x(1:2, 5) - x(1:2,j));
             end
             
             % take minimum distance and test if lower than minimum then
             % apply learning algorithm to avoid accident 
             min_dis=min(s);
             
%              waypoint_lead2 = waypoints_lead2(:, state_lead2) ;
                n=norm(x(1:2, 5) - waypoint_lead2);
                if(n < close_enough)
                    state_lead2 = state_lead2+1;
                    if(state_lead2==length(waypoints_lead2)+1)
                        state_lead2=1;
                    end
                   
                
                   waypoint_lead2 = waypoints_lead2(:, state_lead2) ;


                end 
            
                % in case no possibility for accident with other
                % leader,test for possibilty of obstacle collision
             if min_dis>0.35
                 dis=[];
                for k=1:length(test)
                    dis(k)=norm(x(1:2, i) - test(:,k));
                end

                   mins=min(dis);
                   %in case of possible obstacle collision, move leader two
                   %according to prior knowledge of obstacles position
                   if mins<0.4 && count==0

                    if  x(1,5)<-1 
                      waypoint_lead2(2)=waypoint_lead2(2)+0.4;
                      move=1;
                      count=1;
                      
                    elseif x(1,5)>=0 
                        
                    waypoint_lead2(2)=waypoint_lead2(2)-0.4;
                    move=1;
                    count=1;
                   
                 
                    end
                    %translate next 5 trajectory points of leader two upwards or
                    %downwards to avoid obstacle collision

                   elseif move==1
%              
                     if count<=6 
                         if x(1,5)<-1
                      waypoint_lead2(2)=waypoints_lead2(2, state_lead2) +0.2;
                         elseif x(1,5)>=0
                      waypoint_lead2(2)=waypoints_lead2(2, state_lead2) -0.2;
                         end
                         count=count+1;
                         if count ==6
                             move=0;
                             count=0;
                         end

                     end
                    elseif mins>0.4
                    waypoint_lead2 = waypoints_lead2(:, state_lead2) ;
                    end
                     
                dxi(:, 5) = leader_controller(x(1:2, 5), waypoint_lead2);
             
                first=1;
                
          

            
%%%%%%%%%%%%%%%%%% in case possibilty for accident, 
%%%%%%%%%%%%%%%%% Stop motion of robots of second leader to avoid accident
%%%%%%%%%%%%%%%%% with robots of leader one
             else
            dxi(:, 5) = 0;
             end

             
         %apply control algorithm for leader one
        elseif i==1
              if x(1,1)<4.6
                   n=norm(x(1:2, 1) - waypoints(:, state));
                    if(n < close_enough)
                        state = state+1;
                        if(state==length(waypoints)+1)
                            state=1;
                        end
                       waypoint = waypoints(:, state) ;
                       

                    end
             
              end 

            dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);

            
        end
   
       
         
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%               
      %combination of leader one and two and control algorithm for new leader       
   elseif x(1,5)>=4.8 || united==1
                 if x(1,8)>2.6
  
                        formation_control_gain=5;
                        v=0.6;
                 elseif x(1,8)<=2.6
  
                       formation_control_gain=10;
                       v=1.2;
                 end
                si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', v);
                leader_controller = create_si_position_controller('XVelocityGain', v, 'YVelocityGain', v, 'VelocityMagnitudeLimit', 0.1);

               
             
                 desired_distance = 0.3;
                 L=L_united;
                 united=1;
                 
                 dxi(:, 8) = leader_controller(x(1:2, 8), waypoint_united);
                   n=norm(x(1:2, 8) - waypoints_united(:, state_united));
                    if(n < close_enough)
                        state_united = state_united+1;
                        if(state_united==length(waypoints_united)+1)
                            state_united=1;
                        end
                       waypoint_united = waypoints_united(:, state_united) ;
                       

                    end
                     
                if i ~=8 
                  
                 for j = neighbors
 

                    dxi(:, i) = dxi(:, i) + ...
                        formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*1.1*(x(1:2, j) - x(1:2, i));    

                 end
            
            
            
                end
              
   end
end
   

  
            
                 
        
    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    to_thresh(1) = 0;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi, x);
%     dxu = uni_barrier_cert(dxu, x,[-3 0.396;0.4 -0.3684]');
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!    
    r.step();
    
end

% ----- plots (to be removed for experiments)
%rPlot.plotStates();
%rPlot.plotInputs();
%rPlot.plotTrajectories();  % trajectories

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();
