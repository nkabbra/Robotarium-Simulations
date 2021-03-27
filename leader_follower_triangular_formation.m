%% Leader-follower with triangular formation
%Master ATSI : Multi Agent System Course

%Group 3: Nadine KABBARA
%Leonardo FELIPE TOSO
%Lara JABER
%Soha KANSO
%Fadhlallah BOUDEHANE

%robots form a triangular formation and follow a leader in its trajectory
%while keeping the formation
%% Set up the Robotarium object

N = 6;
%initial_positions = generate_initial_conditions(N, 'Width', 1, 'Height', 1, 'Spacing', 0.3);
initial_positions = [ -0.75, 0, -1, 0, -1,0; ...
0.6, 0.8, 0, 0, -0.5, -0.5 ; ...
0, 0, 0, 0, 0, 0];
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

%% Experiment Constants

%Run the simulation for a specific number of iterations
iterations = 6000;
% Sampling time (not to be modified)
Ts = 0.033;
%% Create the desired Laplacian

%Graph laplacian
% 
L= [0  0  0  0  0  0 ; ... 
    -1  3  0 -1  0  -1; ... 
    -1  0  3  -1  0 -1 ; ... 
     0  -1  -1  3 -1 0 ; ... 
    -1 0  0 -1  3  -1 ; ... 
     0  -1 -1 0 -1  3];
 A= [0  0  0  0  0  0 ; ... 
    -1  0  0 -1  0  -1; ... 
    -1  0  0  -1  0 -1 ; ... 
     0  -1  -1  0 -1 0 ; ... 
    -1 0  0 -1  0  -1 ; ... 
     0  -1 -1 0 -1  0];
% The desired inter-agent distance for the formation
d = 0.4; 

% Pre-compute diagonal values for the rectangular formation
ddiag = d*sqrt(3);


         
 weights = [ 0 0 0 0 0 0; ... 
             d 0 0 d 0 ddiag; ... 
             d 0 0 ddiag 0 d; ... 
             0 d ddiag 0 d 0; ... 
             ddiag 0 0 d 0 d; ... 
             0 ddiag d 0 d 0];
         
%Initialize velocity vector
dxi = zeros(2, N);

%State for leader
state = 1;

% These are gains for our formation control algorithm
formation_control_gain = 0.3;



%% Grab tools we need to convert from single-integrator to unicycle dynamics
% 
 [~, uni_to_si_states] = create_si_to_uni_mapping();
%   si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion();
  si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.5, 'AngularVelocityLimit', pi/2);
% Single-integrator position controller
uni_barrier_cert= create_uni_barrier_certificate_with_boundary();
waypoints =0.9* [-1 0.8;-1 -0.8; 1 -0.8; 1 0.8]';
close_enough = 0.05;
v=0.8;
leader_controller = create_si_position_controller('XVelocityGain', v, 'YVelocityGain', v, 'VelocityMagnitudeLimit', 0.1);

for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    % Convert to SI states
    xsi = uni_to_si_states(x);
    
    
    %% Algorithm
    if t<300
        state=1;
         formation_control_gain = 2;
    end
    if t>=300 && t<1000
        state=1;
           formation_control_gain = 5;
    end
    if t>=1000
         formation_control_gain = 10;

        
    end
    
         for i = 2:N
        
        %Zero velocity and get the topological neighbors of agent i
        dxi(:, i) = [0 ; 0];
        
        neighbors = topological_neighbors(L, i);
        
            for j =  neighbors
                 dxi(:, i) = dxi(:, i) + ...
                formation_control_gain*(norm(xsi(:, i) - xsi(:, j))^2 - weights(i, j)^2) ... 
                *(xsi(:, j) - xsi(:, i));
            end
         end
  
    %% Make the leader travel between waypoints
    
    waypoint = waypoints(:, state);
    
           
        
            dxi(:, 1) = leader_controller(xsi(:, 1), waypoint);
            if(norm(xsi(:, 1) - waypoint) < close_enough)
                state = state+1;
                if(state==5)
                    state=1;
                end
            end
        
            
  
    
        
    %% Avoid actuator errors
    
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Use barrier certificate and convert to unicycle dynamics
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert(dxu, x);
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:N, dxu);
    
    %Iterate experiment
    r.step();
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();