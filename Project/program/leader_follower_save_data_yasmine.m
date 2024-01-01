close all;
% Same script as leader_follower_with_plotting.m but with additional data saving. Two
% data sets will be saved, one saving the distance between connected robots
% through time, and another with the distance between the leader and goal
% location when the goal is "reached".

% Sean Wilson
% 07/2019

%% Experiment Constants

%Run the simulation for a specific number of iterations
iterations = 6000;

%% Set up the Robotarium object

N = 5;
initial_positions = generate_initial_conditions(N, 'Width', 1, 'Height', 1, 'Spacing', 0.3);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

%% Create the desired Laplacian

L = zeros(N, N);
% DIAMOND TOPOLOGY
followers = -completeGL(N-2); % we don't take into account Robot 5 in the Laplacian graph to keep the current formation
% L(2:N - 1, 2:N - 1) = followers; % ^
% L(2, 1) = -1;
% L(2, 2) = L(2, 2) + 1;

% % we add connections from robot 5 to robots 3 & 4 (followers 2 & 3)
% L(5, 3) = -1;
% L(5, 4) = -1;
% L(5, 5) = L(5, 5) + 2;  % Degree of Robot 5

L = [1 -1 0 0 0;
-1 3 -1 -1 0;
0 -1 3 -1 -1;
0 -1 -1 3 -1;
0 0 -1 -1 2];


                                % IN-LINE TOPOLOGY
% L(2, 1) = -1; % for first follower
% L(2, 2) = L(2, 2) + 1;
% for i = 3:N % for each other followers
%     L(i, i-1) = -1;   % Connection to the robot in front
%     L(i-1, i) = -1;   % Connection to the robot behind
%     L(i, i) = 2;      % Degree of each follower robot
% end

                                % FRONT TOPOLOGY
% for i = 3:N-1 % for each other followers
%     L(i, 1) = -1;
%     L(i, i-1) = -1;   % Connection to the robot in front
%     L(i, i+1) = -1;   % Connection to the robot behind
%     L(i, i) = 3;      % Degree of each follower robot
% 
% end
% L(2, 1) = -1;
% L(2, 3) = -1;
% L(2, 2) = 2;
% 
% L(5,4) = -1;
% L(5, 1) = -1;
% L(5, 5) = 2;

                                % SQUARE TOPOLOGY
% L(1:3, 1:3) = -completeGL(3);
% L(4:5, 4:5) = -completeGL(2);
% for i = 4:5
%     L(i, i - 2) = -1;
%     L(i - 2, i) = 1;
%     L(i, i) = L(i, i) + 1;
% end
% L(4, 3) = -1;
% L(4, 4) = L(4, 4) + 1;
% L(5, 2) = -1;
% L(5, 5) = L(5, 5) + 1;


%Initialize velocity vector
dxi = zeros(2, N);

%State for leader
state = 1;

% These are gains for our formation control algorithm
formation_control_gain = 10;
desired_distance = 0.2;

%% Grab tools we need to convert from single-integrator to unicycle dynamics

% Single-integrator -> unicycle dynamics mapping
si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 0.8);
% Single-integrator barrier certificates
uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
% Single-integrator position controller
leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.1);

waypoints = [-1 0.8; -1 -0.8; 1 -0.8; 1 0.8]';

% Calculate a virtual waypoint between last first to constraint path
% inside borders
first_waypoint = waypoints(:,1);
last_waypoint = waypoints(:,end);
virtual_waypoint = (first_waypoint + last_waypoint) / 2;

% Insert the virtual waypoint into the waypoints list
% and make the waypoint list a loop for smoother path
modified_waypoints = [waypoints, first_waypoint];
number_of_waypoints = size(modified_waypoints, 2);

% Interpolation
interp_factor = 10; % Increase for more intermediate points
interp_points = linspace(1, number_of_waypoints, interp_factor*number_of_waypoints);
x_smooth_path = interp1(modified_waypoints(1, :), interp_points, 'spline');
y_smooth_path = interp1(modified_waypoints(2, :), interp_points, 'spline');
smooth_path = [x_smooth_path; y_smooth_path];
smooth_path_index = 1;

% Obstacle between waypoints 2 & 3
obstacles = [] ; %zeros(3, 1); % (x, y, radius)

obstacle_position = (waypoints(:,2) + waypoints(:,3)) / 2;
obstacle_radius = 0.2;
obstacles(:,1) = [obstacle_position(1), obstacle_position(2), obstacle_radius];

obstacle_position = (waypoints(:,4) + waypoints(:,1)) / 2;
obstacle_radius = 0.2;
obstacles(:,2) = [obstacle_position(1), obstacle_position(2), obstacle_radius];


base_close_enough = 0.2;

%% Plotting Setup

% Color Vector for Plotting
% Note the Robotarium MATLAB instance runs in a docker container which will 
% produce the same rng value every time unless seeded by the user.
CM = ['k','b','r','g'];

%Marker, font, and line sizes
marker_size_goal = determine_marker_size(r, 0.20);
font_size = determine_font_size(r, 0.05);
line_width = 5;

% Create goal text and markers.
for i = 1:length(waypoints)
    % Text with goal identification
    goal_caption = sprintf('G%d', i);
    % Plot colored square for goal location.
    g(i) = plot(waypoints(1,i), waypoints(2,i),'s','MarkerSize',marker_size_goal,'LineWidth',line_width,'Color',CM(i));
    % Plot the goal identification text inside the goal location
    goal_labels{i} = text(waypoints(1,i)-0.05, waypoints(2,i), goal_caption, 'FontSize', font_size, 'FontWeight', 'bold');
end

% Plot graph connections
%Need location of robots
x=r.get_poses();

% Follower connections to each other
[rows, cols] = find(abs(L) == 1);

%Only considering half due to symmetric nature
for k = 1:length(rows)
   lf(k) = line([x(1,rows(k)), x(1,cols(k))],[x(2,rows(k)), x(2,cols(k))], 'LineWidth', line_width, 'Color', 'b'); 
end


% Leader connection assuming only connection between first and second
% robot.
ll = line([x(1,1), x(1,2)],[x(2,1), x(2,2)], 'LineWidth', line_width, 'Color', 'r'); 

% Follower plot setup
for j = 1:N-1    
    % Text for robot identification
    follower_caption{j} = sprintf('Follower Robot %d', j);
    % Plot the robot label text 
    follower_labels{j} = text(500, 500, follower_caption{j}, 'FontSize', font_size, 'FontWeight', 'bold');
end

%Leader plot setup
leader_label = text(500, 500, 'Leader Robot', 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');


% Initialize variables for formation shape control plot
previous_total_distance = 0;
current_total_distance = 0;
modulo = 200;
distance_changes_for_plot = [];
iterations_array_for_plot = [];
plot_iterations = 0;

% Obstacle drawing
for i = 1:size(obstacles, 2) % Iterate over the number of obstacles (columns)
    viscircles(obstacles(1:2, i)', obstacles(3, i), 'Color', 'r');
end


% Sliding-window live plot
iteration_sliding_window = 10;
plot_figure = figure; % Open a new figure window
previous_plot = plot(0);
xlabel('Iteration');
ylabel('Live change in Total Distance');
title('Live change in Total Distance Over Iterations');
hold on;


%% Data Saving Setup

%Preallocate what we can.
robot_distance = zeros(5,iterations); % 4 distances and time
goal_distance = []; % Cannot preallocate this as we do not know how many
                   % times the goal will be reached.
start_time = tic;

r.step();

   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%INIT LOCALISATION%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%pose initiale
pose = r.get_poses();
% pose - instant - robot
odometer (:,1,:)=pose(:,:) ; % Valeurs obtenues par la localisation 
expected_odometer(:,1,:) = odometer (:,1,:) ; % Valeurs réelles

Qu= [0.5*10^-4 0; 0 0.5*10^-4]*0; %covariance associée aux bruits du vecteur d'entrée
Q= [0.6*10^-5 0 0;0 0.4*10^-5 0;0 0 10^-5]; %covariance du bruit de modèle 
podo= [1 0 0; 0 1 0;0 0 0.5]; %valeur initiale de la matrice de covariance P

tfault1= [500:1000,4500:4900];
tfault2= [500:1000,4500:4900];
tfault3= [200:480,2000:2500];
tfault4= [3100:3600,4000:4400];

sigmax = zeros (iterations, 1);
sigmay = zeros (iterations, 1);

R=10^-5; %covariance du bruit de mesure
r.step();
% % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%FIN INIT%%%%%%%%%%%%%%

delays = [];

for t = 2:iterations
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%Localisation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    pose = r.get_poses();
    expected_odometer(:,t,:) = pose(:,:);
    velocities = r.get_velocities()+normrnd(0,0.001);

    for i=1:N
        % Prédiction
        % omega(i) = 0.033*r.get_velocities(2,i); %time_step = 0.033
        % delta(i) = 0.033*r.get_velocities(1,i);
        A = [cos(odometer(3,t-1,i)) 0; sin(odometer(3,t-1,i)) 0; 0 1];
        odometer(:,t,i)=odometer(:,t-1,i) + A*[0.033*velocities(1,i) ; 0.033*velocities(2,i)]; % delta_k, omega_k

        FF = [1 0 -0.033*velocities(1,i)*sin(odometer(3,t-1,i)) ; 0 1 0.033*velocities(1,i)*cos(odometer(3,t-1,i)); 0 0 1]; %delta_k
        podo = FF*podo*FF' + Q; %     

        X = odometer(:,t,i); % vecteur d'état prédit
        Yp = inv(podo); % matrice informationnelle
        yp = Yp*X; % vecteur informationnel

        % Update
        % Calcul des distances entre les amers et les coordonnées réelles du robot à l'instant t 
        d1=sqrt((expected_odometer(1,t,i)-waypoints(1,1)).^2 + (expected_odometer(2,t,i)-waypoints(2,1)).^2);
        d2=sqrt((expected_odometer(1,t,i)-waypoints(1,2)).^2 + (expected_odometer(2,t,i)-waypoints(2,2)).^2);
        d3=sqrt((expected_odometer(1,t,i)-waypoints(1,3)).^2 + (expected_odometer(2,t,i)-waypoints(2,3)).^2);
        d4=sqrt((expected_odometer(1,t,i)-waypoints(1,4)).^2 + (expected_odometer(2,t,i)-waypoints(2,4)).^2);
        % On crée le vecteur z en ajoutant un bruit blanc pour simuler le capteur LIDAR --> ne pas mettre + que 0.001, sinon plus de "bonne correction
        z = [d1+normrnd(0,0.001);d2+normrnd(0,0.001);d3+normrnd(0,0.001);d4+normrnd(0,0.001)];

        %injection de défauts
        if find(tfault1==t)
               z(1)=z(1)+0.05;
        end
        if find(tfault2==t)
               z(2)=z(2)+0.05;
        end

        if find(tfault3==t)
               z(3)=z(3)+0.05;
        end 

        if find(tfault4==t)
               z(4)=z(4)+0.05;
        end

        %initialisation des gains informationnels
        sgI=zeros(3,3);
        sgi=zeros(3,1);
        gI=[];
        gi=[];
        th_part=chi2inv(0.9,3);

        %% Etape de correction du filtre informationnel
        n = length(waypoints);

        for j = 1:n
            zestime = sqrt((X(1) - waypoints(1, j)) ^ 2 + (X(2) - waypoints(2, j)) ^ 2);
            H = [(X(1) - waypoints(1, j)) / zestime, (X(2) - waypoints(2, j)) / zestime, 0]; % jacobienne
            gI{j} = H' * inv(R) * H; % contribution informationnelle associée à la mesure z(i)
            sgI = sgI + gI{j};
            nu = z(j) - zestime;
            gi{j} = H' * inv(R) * [nu + H * X]; % contribution informtionnelle associée à la mesure z(i)
            sgi = sgi + gi{j};

            r_isol{t}(j) = diagnostic(gI{j}, gi{j}, yp, Yp, X); %résidu pour l'isolation des défauts
        end

        %% detection de défaut
        [r_global(t),detect]=detection(sgi,sgI,yp,Yp,X); 

        % si défaut détecté=> exclusion des mesures erronées

        if detect
            for i=1:n
                if r_isol{t}(i)>th_part  
                    sgi=sgi-gi{i};
                    sgI=sgI-gI{i};
                    fprintf('exclusion de défauts numéro %d\n',i)
                end
            end
        end

        %% correction
        Yu=Yp+sgI; % matrice informationnelle mise à jour
        yu=yp+sgi; % vecteur informationnel mis à jour

        podo=inv(Yu); % Matrice de covariance mise à jour 
        odometer(:, t,i) = podo*yu; % Vecteur d'état mis à jour à l'instant t 

        if i ==1
            sigmax(t)=sqrt(podo(1,1)); 
            sigmay(t)=sqrt(podo(2,2)); 
        end
    end
    x=odometer(:,t,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%FIN Localisation%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
    
    % % Retrieve the most recent poses from the Robotarium.  The time delay is
    % % approximately 0.033 seconds
    % x = r.get_poses();

    % if mod(t, modulo) == 0
    %     for i = 1:N-1
    %         for j = i+1:N
    %             delay = calculate_communication_delay([dxi(1:2, i), dxi(1:2, j)], obstacles);
    %             % fprintf('%f\t%f\t%f\t%f\n', t, i, j, delay);
    %             delays = [delays, [t, i, j, delay]];
    %         end
    %     end
    % end
    
    %% Algorithm
    
    for i = 2:N
        
        %Zero velocity and get the topological neighbors of agent i
        dxi(:, i) = [0 ; 0];
        
        neighbors = topological_neighbors(L, i);
        
        
        nearest_obstacle = [9999; 9999; 9999];
        for k = 1:size(obstacles, 2)
            if norm(obstacles(1:2, k) - x(1:2, i)) < norm(nearest_obstacle(1:2) - x(1:2, 1))
                nearest_obstacle = obstacles(:,k);
            end
        end

        avoidance_pose = x(1:2, 1) - nearest_obstacle(1:2);
        distance_to_obstacle = norm(avoidance_pose);

        for j = neighbors
            dxi(:, i) = dxi(:, i) + ...
                            formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*(x(1:2, j) - x(1:2, i));
        end

        if (~isequaln(nearest_obstacle, [9999; 9999; 9999]))
            dxi(:, i) = dxi(:, i) + (nearest_obstacle(3)/distance_to_obstacle)^2 * avoidance_pose * 0.3;
        end
        
    end

    nearest_obstacle = [9999; 9999; 9999];
    for k=1:size(obstacles, 2)
        if norm(obstacles(1:2, k) - x(1:2, 1)) < norm(nearest_obstacle(1:2) - x(1:2, 1))
            nearest_obstacle = obstacles(:,k);
        end
    end

    avoidance_pose = x(1:2, 1) - nearest_obstacle(1:2);
    distance_to_obstacle = norm(avoidance_pose);
    fprintf('%f\n', distance_to_obstacle);
    %% Make the leader travel along the smooth path
    current_goal = smooth_path(:, smooth_path_index);

    close_enough_multiplier = 1;

    % Update leader's velocity based on the smooth path
    dxi(:, 1) = leader_controller(x(1:2, 1), current_goal);
    if (~isequaln(nearest_obstacle, [9999, 9999, 9999]))
        dxi(:, 1) = dxi(:, 1) + (nearest_obstacle(3)/distance_to_obstacle)^2 * avoidance_pose * 0.3;
        close_enough_multiplier = 1 + (nearest_obstacle(3) / max(distance_to_obstacle, nearest_obstacle(3))); % need to be increased if obstacle is big
    end
    close_enough = base_close_enough * close_enough_multiplier;
    
    % Check if leader is close enough to current goal, if so, move to the next
    if(norm(x(1:2, 1) - current_goal) < close_enough)
        smooth_path_index = smooth_path_index + 1;
        if smooth_path_index > size(smooth_path, 2)
            smooth_path_index = 1; % Loop back to the start
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
    % dxu = uni_barrier_cert(dxu, x);
    
    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:N, dxu);
    
    %% Update Plot Handles
    
    %Update position of labels for followers
    for q = 1:N-1
        follower_labels{q}.Position = x(1:2, q+1) + [-0.15;0.15];    
    end
    
    %Update position of graph connection lines
    for m = 1:length(rows)
        lf(m).XData = [x(1,rows(m)), x(1,cols(m))];
        lf(m).YData = [x(2,rows(m)), x(2,cols(m))];
    end
    
    
    %Update position of label and graph connection for leader
    leader_label.Position = x(1:2, 1) + [-0.15;0.15];
    ll.XData = [x(1,1), x(1,2)];
    ll.YData = [x(2,1), x(2,2)];
    
    % Resize Marker Sizes (In case user changes simulated figure window
    % size, this is unnecessary in submission as the figure window 
    % does not change size).

    marker_size_goal = num2cell(ones(1,length(waypoints))*determine_marker_size(r, 0.20));
    [g.MarkerSize] = marker_size_goal{:};
    font_size = determine_font_size(r, 0.05);
    leader_label.FontSize = font_size;
    
    for n = 1:N
        % Have to update font in loop for some conversion reasons.
        % Again this is unnecessary when submitting as the figure
        % window does not change size when deployed on the Robotarium.
        follower_labels{n}.FontSize = font_size;
        goal_labels{n}.FontSize = font_size;
    end
    
    %% Compute data to be saved and store in matrix.
    % Distances between connected robots.
    robot_distance(1,t) = norm([x(1:2,1) - x(1:2,2)],2);
    robot_distance(5,t) = toc(start_time);
    for b = 1:length(rows)/2+1
        robot_distance(b+1,t) = norm([x(1:2,rows(b)) - x(1:2,cols(b))],2);   
    end
    
    if(norm(x(1:2, 1) - current_goal) < close_enough)
        goal_distance = [goal_distance [norm(x(1:2, 1) - current_goal);toc(start_time)]];
    end

    %% Formation shape control
    for i = 1:N
        for j = i+1:N
            current_total_distance = current_total_distance + norm(x(1:2, i) - x(1:2, j));
        end
    end
    
    % Every $modulo iterations, print the difference in total distance
    if mod(t, modulo) == 0
        plot_iterations = plot_iterations + 1;
        distance_change = abs(current_total_distance - previous_total_distance);
        distance_changes_for_plot(plot_iterations) = distance_change;
        iterations_array_for_plot(plot_iterations) = t;
      
        delete(previous_plot);
        plot_starting_iteration = max(1, plot_iterations - iteration_sliding_window);
        previous_plot = plot(iterations_array_for_plot(plot_starting_iteration:plot_iterations), ...
            distance_changes_for_plot(plot_starting_iteration:plot_iterations), '-*b');

        drawnow;

        % fprintf('Iteration %d: Change in Total Distance = %f\n', t, distance_change);
        
        previous_total_distance = current_total_distance;
        current_total_distance = 0;
    end
    
    %Iterate experiment
    r.step();
end

hold off;

% Plot the changes in total distance after the simulation
figure; % Open a new figure window
plot(iterations_array_for_plot, distance_changes_for_plot, '-*b');
xlabel('Iteration');
ylabel('Change in Total Distance');
title('Change in Total Distance Over Iterations');

% Save the data
save('DistanceData.mat', 'robot_distance');
save('GoalData.mat', 'goal_distance');

% We can call this function to debug our experiment!  Fix all the errors
% before submitting to maximize the chance that your experiment runs
% successfully.
r.debug();

%% Helper Functions

% Marker Size Helper Function to scale size with figure window
% Input: robotarium instance, desired size of the marker in meters
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)

% Get the size of the robotarium figure window in pixels
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Points');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the robot size to the x-axis (the axis are
% normalized so you could do this with y and figure height as well).
marker_ratio = (marker_size_meters)/(robotarium_instance.boundaries(2) -...
    robotarium_instance.boundaries(1));

% Determine the marker size in points so it fits the window. cursize(3) is
% the width of the figure window in pixels. (the axis are
% normalized so you could do this with y and figure height as well).
marker_size = cursize(3) * marker_ratio;

end

% Font Size Helper Function to scale size with figure window
% Input: robotarium instance, desired height of the font in meters
function font_size = determine_font_size(robotarium_instance, font_height_meters)

% Get the size of the robotarium figure window in point units
curunits = get(robotarium_instance.figure_handle, 'Units');
set(robotarium_instance.figure_handle, 'Units', 'Points');
cursize = get(robotarium_instance.figure_handle, 'Position');
set(robotarium_instance.figure_handle, 'Units', curunits);

% Determine the ratio of the font height to the y-axis
font_ratio = (font_height_meters)/(robotarium_instance.boundaries(4) -...
    robotarium_instance.boundaries(3));

% Determine the font size in points so it fits the window. cursize(4) is
% the hight of the figure window in points.
font_size = cursize(4) * font_ratio;

end