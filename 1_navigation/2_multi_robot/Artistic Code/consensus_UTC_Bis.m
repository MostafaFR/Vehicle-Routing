%% Consensus with a static, undirected topology
% Sean Wilson
% 07/2019
% Demonstrates a theoretical example of the consensus algorithm.  Lacks 
% implementation considerations such as obstacle avoidance, keeping robots 
% inside the workspace, and robot velocity thresholding, which you must 
% include. If you submit this example to be run on the Robotarium it will
% be rejected.

N = 8;
Xi = [0 1.75 2 1.75 0 -1.75 -2 -1.75]/2.5;
Yi = [2 1.75 0 -1.75 -2 -1.75 0 1.75]/2.5;
Oi = [-pi/2 -3*pi/4 -pi 3*pi/4  pi/2 pi/3   0   -pi/4];   
init = [Xi; Yi; Oi]
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', init);

%% Experiment constants

% Generate a cyclic graph Laplacian from our handy utilities.  For this
% algorithm, any connected graph will yield consensus


Db = eye(8);
Ab=[0 0 0 0 0 0 0 1;
    1 0 0 0 0 0 0 0;
    0 1 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0;
    0 0 0 1 0 0 0 0;
    0 0 0 0 1 0 0 0;
    0 0 0 0 0 1 0 0;
    0 0 0 0 0 0 1 0];
L = Db - Ab

%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Gain for the diffeomorphism transformation between single-integrator and
% unicycle dynamics
[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping();

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 1000;

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dxi = zeros(2, N);
X = zeros(1000,3,8);
g = 1;
%Iterate for the previously specified number of iterations
for t = 1:iterations
%     if mod(t, 60) == 0
%         L = L'
%         g = -g;
%     end
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    X(t,:,:) = x;
    
    % Convert to SI states
    xi = uni_to_si_states(x);
    
    %% Algorithm
    
    for i = 1:N
        
        % Initialize velocity to zero for each agent.  This allows us to sum
        %over agent i's neighbors
        dxi(:, i) = [0 ; 0];
        
        % Get the topological neighbors of agent i based on the graph
        %Laplacian L
        neighbors = topological_neighbors(L, i);
        
        % Iterate through agent i's neighbors
        for j = neighbors
            
            % For each neighbor, calculate appropriate consensus term and
            %add it to the total velocity
            dxi(:, i) = dxi(:, i) + (xi(:, j) - g* xi(:, i));
        end        
    end
    
    %% Map to unicycle dynamics    
    % Transform the single-integrator to unicycle dynamics using the the
    % transformation we created earlier
    dxu = si_to_uni_dyn(dxi, x);
    
    %% Send velocities to agents
    
    % Set velocities of agents 1,...,N
    r.set_velocities(1:N, dxu);
    
    % Send the previously set velocities to the agents.  This function must be called!    
    r.step();
end

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();
for t= 1:8
    plot( X(:,1,t), X(:,2,t) )
end
