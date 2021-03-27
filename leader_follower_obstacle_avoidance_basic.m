%*Leader follower obstacle avoidance basic
%Master ATSI : Multi Agent System Course

%Group 3: Nadine KABBARA
%Leonardo FELIPE TOSO
%Lara JABER
%Soha KANSO
%Fadhlallah BOUDEHANE


%basic program with basic learning algorithm, which was advanced into  the avoid_collision function
%and a more advanced obstacle avoidance learning algorithm in : Two leader
%follower with union
%% Robots and robotarium

% Number of robots
N = 4;

% initial poses

%initial_positions = generate_initial_conditions(N, 'Width', 1, 'Height', 1, 'Spacing', 0.3);
initial_positions=2*[1 1.3 1.6 1.3  ;-2 -3.5 -2 -4.5  ;0.4,0.1,0.1,0.4];

r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

     
%% Number of iterations in experiment
% Select the number of iterations for the experiment.
iterations = 7200;
% Sampling time (not to be modified)
Ts = 0.033



%% ----- Plot tools (to be removed for experiments)
% Plot tools
rPlot = RobotariumPlots(N, iterations, Ts);

 M=eye(N,N);
 L=zeros(N,N);
 %L(2:N,1:N)=-M(1:N-1,1:N);
L=[ 0  0  0  0;
   -1  2 -1  0;
    0 -1  2 -1;
    0  0 -1  1]

%Initialize velocity vector
dxi = zeros(2, N);

%State for leader
state = 1;

% These are gains for our formation control algorithm
formation_control_gain = 12;
desired_distance = 0.3;
%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate();
% Single-integrator position controller
leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.08);

%waypoints = [-1 0.8; 0 0.8; 0 1.8; 1 1.8;1 2.8;2 2.8;2 3.8;3 3.8]';
waypoints=[1 0;0.5 0.5;0 1;-0.5 0.5;-1 0;-0.5 -0.5;0 -1;0.5 -0.5]';
close_enough = 0.03;


%% Positions de référence initiale pour les robots
width_rect=0.3;
obstacle=[1.14 -0.54];
% Color Vector for Plotting
% Note the Robotarium MATLAB instance runs in a docker container which will 
% produce the same rng value every time unless seeded by the user.
CM = ['r'];

%Marker, font, and line sizes
marker_size_goal = 0.5;
font_size = 0.5;

test=[1.5 -2.3;1.2 -1.05;1.1 -0.5;0.1 0.5]';
%test_track_works=[1.5 -2.3;1.2 -1.05;1.1 -0.5;0.1 0.5]';
plot(test(1,:),test(2,:),'o','LineWidth',4)
 waypoint = waypoints(:, state);
 right=1;
 move=0;
 first(1)=0;
 turn_right(1)=0;
 for i=2:N
 first(i)=1;
 turn_right(i)=0;
 end
 minz=zeros(N,iterations);
for t = 1:iterations
 % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
     
     %% Algorithm
    
    for i = 2:N
        
        %Zero velocity and get the topological neighbors of agent i
        dxi(:, i) = [0 ; 0];
        
        neighbors = topological_neighbors(L, i);
        %calculate distance to each obstacle
        for m=1:length(test)
               s(m)=norm(x(1:2, i) - test(:,m));
        end
   %find nearest obstacle
             minn=min(s);
             minz(i,t)=minn;
                
             if (minn>0.5)
                 first(i)=1;
                 turn_right(i)=0;
                 
             end
             if (minn<=0.35)
                   %first try to move left and save original distance away
                %from obstacle
                if first(i)==1
                ct=[-0.004 0.004];
                min_dis(i)=minn;
                first(i)=2;
                
            %if moving to the left was beneficial (i.e it avoided obstacle
            %collision) then continue moving left 
             %move right in unbeneficial case
                elseif first(i)==2
                    if minn<=min_dis(i)
                    ct=[(0.05/(min_dis(i)+0.07)) 0.004 ];
                    turn_right(i)=1;
                    dir(i,t)=1;
                    first(i)=3;
                   
                    else
                    ct=[-(0.04/(min_dis(i)+0.07)) 0.004];

                    turn_right(i)=0;
                    first(i)=3;
                    dir(i,t)=0;
                    end
                    %continue moving left or right to avoid obstacle
                elseif first(i)==3 && turn_right(i)==0
                    ct=[-(0.025/(min_dis(i)+0.05)) 0.05];

                elseif first(i)==3 && turn_right(i)==1
                    ct=[(0.025/(min_dis(i)+0.05)) 0.05];                        
                end 
                
             end
        for j = neighbors
            %reinitialization after passing obstacle
          
             %control algorithm in case no near obstacle 
            if (minn> 0.35)
              dxi(:, i) = dxi(:, i) + ...
                formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*4.1*(x(1:2, j) - x(1:2, i));
             ct=[];
%             min_dis(i)=0;
            
%in case of near obstacle, use learning algorithm in order to avoid
%obstacle
            else
              
                %control algorithm after learning best movement direction
                 dxi(:, i) = dxi(:, i) + ...
                formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*4.1*(x(1:2, j) - x(1:2, i)+ct');
            
            end
        end
        
    end
    
    %% Make the leader travel between waypoints
    
   
    %we assume that the leader has a prior knowledge of the positon of the
    %obstacles and hence decides to turn right or left appropriately to
    %avoid the obstacle
   
   for i=1:length(test)
    s(i)=norm(x(1:2, 1) - test(:,i));
   end
   
   mins=min(s);

    if mins>0.5
        right=mod(right,2)+1;
    end 
    if (mins< 0.25)
        if right==2
        waypoint(1,1)=waypoint(1,1)+0.4;
        else 
        waypoint(1,1)=waypoint(1,1)-0.4; 
        end
      

    else
        waypoint = waypoints(:, state) ;
     
       
    end
    
    
       dxi(:, 1) = leader_controller(x(1:2, 1), waypoint);
       n=norm(x(1:2, 1) - waypoint);
            if(n < close_enough)
                state = state+1;
                if(state==length(waypoints)+1)
                    state=1;
                end
               waypoint = waypoints(:, state) ;
               mins=10;
               
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
   % dxu = uni_barrier_cert(dxu, x);
    
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