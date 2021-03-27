% ************************************************************************
%            CentraleSupelec 3A - Mention Control Engineering - 2021
%  				  "Control architectures of complex systems"
% ************************************************************************

% Nadine KABBARA 
%robots form hexagon formation with loss of communication with one of
%robots
%% Robots and robotarium

% Number of robots
N = 6;

% initial poses
initial_poses = [ -1, 0, 1, 1, 0,-1; ...
0.8, 0.8, 0.8, -0.8, -0.8, -0.8 ; ...
0, 0, 0, 0, 0, 0];
% Robotarium
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_poses);


%% Communication graph
% adjacency matrix
A= ones(N,N)-eye(N);
A_init = ones(N,N)-eye(N);
% [L, lambda2] = calc_laplace(A)
A_out=[0 1 0 1 1 1;
       1 0 0 1 1 1;
       0 0 0 0 0 0;
       1 1 0 0 1 1;
       1 1 0 1 0 1;
       1 1 0 1 1 0
       ];
%% Number of iterations in experiment
% Select the number of iterations for the experiment.
iterations = 4000;
% Sampling time (not to be modified)
Ts = 0.033;

scaleFactor = 0.2;
r12Ref = [-2 ; -2];
r13Ref = [-4 ; -2];
r14Ref = [-6 ; 0];
r15Ref = [-4 ; 2];
r16Ref = [-2 ; 2];
r1xRef = scaleFactor*[ [0;0], r12Ref, r13Ref, r14Ref, r15Ref, r16Ref ];

r10Ref = [-3; 0];
r20Ref = [-1; 2];
r30Ref = [1; 2];
r40Ref = [3; 0];
r50Ref = [1; -2];
r60Ref = [-1; -2];
rx0Ref = scaleFactor*[ r10Ref, r20Ref, r30Ref, r40Ref, r50Ref, r60Ref];

x0 = [0; 0];
A=A_init;
%% ----- Plot tools (to be removed for experiments)
% Plot tools
 rPlot = RobotariumPlots(N, iterations, Ts);


%% Control gain
kf = 0.01; %1.0; % better behavior for small velocities

kt = 0.03;
%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Gain for the diffeomorphism transformation between single-integrator and
% unicycle dynamics
[~, uni_to_si_states] = create_si_to_uni_mapping();
si_to_uni_dyn = create_si_to_uni_dynamics_with_backwards_motion();

% Barrier certificates
%si_barrier_cert_boundary = create_si_barrier_certificate_with_boundary();
uni_barrier_cert_boundary = create_uni_barrier_certificate_with_boundary();

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dxi = zeros(2, N);
out=0;
%Iterate for the previously specified number of iterations
for t = 1:iterations
    
    %% Retrieve poses of the robots 
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    
    % Convert to SI states
    xsi = uni_to_si_states(x);
    
    % ----- add to data for plots (to be removed for experiments)
    rPlot.addStateData(x, t);
    
    if t>100&& t<1500
       A=A_out;
       out=1;
    else
          A=A_init;
          out=0;
    end
%     
    %% Control algorithm
    
    % ----- for plots (to be removed for experiments)
    U = zeros(2,N);
    
    % control of each robot
    for i = 1:N
        
        % initialize control input (for sum)
        ui = [0 ; 0];
       
       
      
        for j=1 :N
              rijRef = r1xRef(:,j)-r1xRef(:,i);
             ui = ui - kf * A(i,j)*( xsi(:,i) - xsi(:,j) - rijRef );
             
        end
        if  out==0 || i~=3
         ui = ui - kt*( (xsi(:,i)-x0) - rx0Ref(:,i));
        end
        % ----- for plots (to be removed for experiments)
        U(:,i) = ui;
        
        dxi(:, i) = ui;
    end
    
    % ----- add to data for plots (to be removed for experiments)
    rPlot.addInputData(U, t);
    
    
    %% Avoid actuator errors
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    
    %% Map SI to Uni dynamics and utilize barrier certificates
    % convert single integrator dynamics to unicycle dynamics
    dxu = si_to_uni_dyn(dxi, x);
    % enforce constraints (collision avoidance, boundaries)
    dxu = uni_barrier_cert_boundary(dxu, x);


      
    %% Send velocities to agents
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    % Send the previously set velocities to the agents.  This function must be called!    
    r.step();
    
    
end


% ----- plots (to be removed for experiments)
rPlot.plotStates();
rPlot.plotInputs();
rPlot.plotTrajectories();

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();