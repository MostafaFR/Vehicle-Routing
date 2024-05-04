% Same script as leader_follower_with_plotting.m but with additional data saving. Two
% data sets will be saved, one saving the distance between connected robots
% through time, and another with the distance between the leader and goal
% location when the goal is "reached".

% ABOU GHALIA - TAESCH
% 12/2023

%% Experiment Constants
%Run the simulation for a specific number of iterations
iterations = 5000; % set to 5000 for final experiments

%% Set up the Robotarium object
N = 4; % TBC --> 5 robots + diamond shape
initial_positions = generate_initial_conditions(N, 'Width', 1, 'Height', 1, 'Spacing', 0.3);
r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions);

%% Create the desired Laplacian
%Graph laplacian
followers = -completeGL(N-1);
L = zeros(N, N);
L(2:N, 2:N) = followers;
L(2, 2) = L(2, 2) + 1;
L(2, 1) = -1;

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

waypoints = [-0.8 0.8; -1 -0.8; 1 -0.8; 1 0.8]'; % positions waypoints
close_enough = 0.2;

%% Plotting Setup

% Color Vector for Plotting
% Note the Robotarium MATLAB instance runs in a docker container which will 
% produce the same rng value every time unless seeded by the user.
CM = ['k','b','r','g'];

%Marker, font, and line sizes
marker_size_goal = determine_marker_size(r, 0.20);
font_size = determine_font_size(r, 0.05);
line_width = 3;

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
[rows, cols] = find(L == 1);

%Only considering half due to symmetric nature
for k = 1:length(rows)/2+1
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

%% Data Saving Setup

%Preallocate what we can.
robot_distance = zeros(5,iterations); % 4 distances and time
goal_distance = []; % Cannot preallocate this as we do not know how many
                   % times the goal will be reached.
start_time = tic;

r.step();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INIT LOC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%pose initiale
pose = r.get_poses();
% pose - instant - robot
odometer(:,1,:)=pose(:,:); % Valeurs obtenues par la localisation
expected_odometer(:,1,:) = odometer(:,1,:); % Valeurs réelles

Qu=[0.5*10^-4 0; 0 0.5*10^-4]*0; %covariance associée aux bruits du vecteur d'entrée
Q=[0.6*10^-5 0 0;0 0.4*10^-5 0;0 0 10^-5]; %covariance du bruit de modèle 
podo=[1 0 0 ; 0 1 0 ;0 0 0.5]; %valeur initiale de la matrice de covariance P

tfault1=[20:50,200:260];
tfault2=[20:50,80:100,340:400];
tfault3=[150:180,370:450];
tfault4=[400:500,1700:2500];

sigmax = zeros(iterations,1);
sigmay = zeros(iterations,1);

R=10^-5; %covariance du bruit de mesure
r.step();

%% Communication QoS Constants
%communication_status = 'perfect'; % Change to 'obstacle' or 'broken_link' as needed
communication_delay_no_obstacle = 0.1; % Example delay for perfect communication
obstacle_delay = 0.5; % Example delay for communication with an obstacle
communication_timeout = 3.0; % Communication timeout in seconds
communication_range = 0.2; % Example communication range, adjust as needed
bypass_speed = 1.0; % Example speed at which the robot bypasses the obstacle

%communication_delay = calculate_communication_delay(communication_status, obstacle_delay, bypass_speed, communication_timeout);
%disp(['Communication Delay: ' num2str(communication_delay) ' seconds']);

% integration tableau communication status
communication_status = cell(1, iterations);

% Define percentages
perfect_percentage = 0.9;
obstacle_percentage = 0.07;
broken_link_percentage = 0.03;

% Generate communication statuses based on percentages
communication_status(1:round(perfect_percentage * iterations)) = repmat({'perfect'}, 1, round(perfect_percentage * iterations));
communication_status(end + 1:end + round(obstacle_percentage * iterations)) = repmat({'obstacle'}, 1, round(obstacle_percentage * iterations));
communication_status(end + 1:end + round(broken_link_percentage * iterations)) = repmat({'broken_link'}, 1, round(broken_link_percentage * iterations));

% Fill the remaining with 'perfect'
communication_status(end + 1:end + (iterations - length(communication_status))) = repmat({'perfect'}, 1, (iterations - length(communication_status)));

% Shuffle the array to randomize the distribution
communication_status = communication_status(randperm(iterations));

% Initialize delay_data array to store the communication delays
delay_data = zeros(1, iterations);

% Initialize broken_count to keep track of consecutive broken_link iterations
broken_count = 0;

% Initialize broken_yes_tableau
broken_yes_tableau = cell(1, iterations);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% INIT LOC %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for t = 2:iterations
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOCALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    pose = r.get_poses();
    expected_odometer(:,t,:) = pose(:,:);
    velocities = r.get_velocities() + normrnd(0,0.1); 
    
    %% ############ START Communication QoS Evaluation ############
    
    % Communication QoS Evaluation
    if is_perfect_communication(x, communication_range)
        communication_delay = communication_delay_no_obstacle;
        % Implement reliable communication handling
        handle_reliable_communication();
    else
        % Example: Communication with obstacle
        communication_delay = obstacle_delay;
        % Implement obstacle communication handling
        handle_obstacle_communication();
    end

    % Update the delay and packet loss data
    delay_data(t) = communication_delay;
    packet_loss_data(t) = calculate_packet_loss(true, communication_timeout);

    % Check for consecutive broken_link iterations
    if strcmp(communication_status{t}, 'broken_link')
        broken_count = broken_count + 1;
        if broken_count >= 90
            %disp('Connection Lost');
        end
    else
        broken_count = 0;
    end

    % Fill broken_yes_tableau
    broken_yes_tableau{t} = communication_status{t};
    
    % ############ END Communication QoS Evaluation ############
    
    for i = 1:N
        % Prédiction
        A = [cos(odometer(3,t-1,i)) 0; sin(odometer(3,t-1,i)) 0; 0 1];
        odometer(:,t,i)=odometer(:,t-1,i) + A*[r.time_step*velocities(1,i) ; r.time_step*velocities(2,i)]; % delta_k, omega_k

        FF = [1 0 -r.time_step*velocities(1,i)*sin(odometer(3,t-1,i)) ; 0 1 r.time_step*velocities(1,i)*cos(odometer(3,t-1,i)); 0 0 1]; %delta_k
        podo = FF*podo*FF' + Q; %     

        X = odometer(:,t,i); % vecteur d'état prédit
        Y = inv(podo); % matrice informationnelle
        y = Y*X; % vecteur informationnel
        Yp=inv(podo); %matrice informationnelle
        yp=Yp*X; %vecteur informationnel

        % Update
        % Calcul des distances entre les amers et les coordonnées réelles du
        % robot à l'instant t 
        d1=sqrt((expected_odometer(1,t,i)-waypoints(1,1)).^2 + (expected_odometer(2,t,i)-waypoints(2,1)).^2);
        d2=sqrt((expected_odometer(1,t,i)-waypoints(1,2)).^2 + (expected_odometer(2,t,i)-waypoints(2,2)).^2);
        d3=sqrt((expected_odometer(1,t,i)-waypoints(1,3)).^2 + (expected_odometer(2,t,i)-waypoints(2,3)).^2);
        d4=sqrt((expected_odometer(1,t,i)-waypoints(1,4)).^2 + (expected_odometer(2,t,i)-waypoints(2,4)).^2);
        % On crée le vecteur z en ajoutant un bruit blanc pour simuler le
        % capteur LIDAR --> ne pas mettre + que 0.001, sinon plus de "bonne
        % correction$
        z = [d1+normrnd(0,0.001);d2+normrnd(0,0.001);d3+normrnd(0,0.001);d4+normrnd(0,0.001)];

        % Injection de défauts sur les 4 capteurs de distance sur un intervalle donné
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

        % initialisation des gains informationnels
        % Sans détection de défauts
        gI=zeros(3,3);
        gi=zeros(3,1);
        % Avec détection de défauts
        sgI=zeros(3,3);
        sgi=zeros(3,1);
        gI=[];
        gi=[];
        th_part=chi2inv(0.9,3);

        %% detection défaut - choix
        for j=1:4
            % zestime=sqrt((X(1)-waypoints(1,j))^2+(X(2)-waypoints(2,j))^2);
            % H=[(X(1)-waypoints(1,j))/zestime, (X(2)-waypoints(2,j))/zestime,0]; % jacobienne
            % gI = gI+H'*inv(R)*H; % contribution informationnelle associée à la matrice
            % nu=z(j)-zestime;
            % gi=gi+H'inv(R)(nu+H*X); % contribution informationnelle associée au vecteur
            
            %%%%%%%%%%%%%%%%%%%%%%%% DETECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            zestime=sqrt((X(1)-waypoints(1,j))^2+(X(2)-waypoints(2,j))^2);
            H=[(X(1)-waypoints(1,j))/zestime, (X(2)-waypoints(2,j))/zestime,0]; % jacobienne
            gI{j}=H'*inv(R)*H; % contribution informationnelle associée à la mesure z(i)
            sgI=sgI+gI{j};
            nu=z(j)-zestime;
            gi{j}=H'*inv(R)*[nu+H*X]; % contribution informtionnelle associée à la mesure z(i)
            sgi=sgi+gi{j};

            r_isol{t}(j)=diagnostic(gI{j},gi{j},yp,Yp,X); %résidu pour l'isolation des défauts  

        end
        
        %% Mise à jour finale sans détection de défauts
        % Y=Y+gI; % Mise à jour matrice informationnelle
        % y=y+gi; % Mise à jour du vecteur informationnel
        
        % podo=inv(Y); % Mise à jour de la matrice de covariance
        % odometer(:,t,i)=podo*y; % Mise à jour du vecteur d'état à l'instant t
        
        %% Mise à jour finale avec détection de défauts
        %%%%%%%%%%%%%%%%%%%%%%% detection de défaut %%%%%%%%%%%%%%%%%%%%%
        [r_global(t),detect]=detection(sgi,sgI,yp,Yp,X); 

        % si défaut détecté=> exclusion des mesures erronées
        if detect
            disp(n)
            for k=1:N
                if r_isol{t}(k)>th_part  
                    sgi=sgi-gi{k};
                    sgI=sgI-gI{k};
                    %fprintf('exclusion de défauts numéro %d\n',k)
                end
            end
        end
        
        %% correction phase
        Yu=Yp+sgI; % matrice informationnelle mise à jour
        yu=yp+sgi; % vecteur informationnel mis à jour

        podo=inv(Yu); % matrice de covariance mise à jour
        odometer(:,t,i)=podo*yu; % vecteur d'état mis à jour à l'instant t
        % ################## FIN detection de défaut ##################

        if i == 1 % On base notre étude uniquement sur le leader
            sigmax(t) = sqrt(podo(1,1));
            sigmay(t) = sqrt(podo(2,2));
        end  
    end 

    x = odometer(:,t,:); % on assigne à x la prédiction finale de la localisation
    
    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOCALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Algorithm
    for i = 2:N
        
        %Zero velocity and get the topological neighbors of agent i
        dxi(:, i) = [0 ; 0];
        
        neighbors = topological_neighbors(L, i);
        
        for j = neighbors
            dxi(:, i) = dxi(:, i) + ...
                formation_control_gain*(norm(x(1:2, j) - x(1:2, i))^2 -  desired_distance^2)*(x(1:2, j) - x(1:2, i));
        end
    end
      
    %% Logique de navigation du leader
    waypoint = waypoints(:, state); % Sélection du waypoint actuel
    next_state = mod(state, size(waypoints, 2)) + 1; % Passage à l'état suivant
    next_waypoint = waypoints(:, next_state); % Prochain waypoint

    angle_to_waypoint = atan2(waypoint(2) - x(2, 1), waypoint(1) - x(1, 1)); % Angle vers le waypoint actuel
    distance_to_waypoint = norm(x(1:2, 1) - waypoint); % Distance au waypoint actuel

    if distance_to_waypoint < close_enough
        % Calcul de l'angle vers le waypoint suivant
        angle_to_target = atan2(next_waypoint(2) - x(2, 1), next_waypoint(1) - x(1, 1));
    else
        % Calcul de l'angle vers le waypoint actuel
        angle_to_target = atan2(waypoint(2) - x(2, 1), waypoint(1) - x(1, 1));
    end

    % Facteur de contrôle pour ajuster la vitesse angulaire
    omega_gain = 1.3; % Vous pouvez ajuster ce facteur selon vos besoins

    % Calcul de la vitesse angulaire en fonction de l'angle vers la cible
    omega = omega_gain * wrapToPi(angle_to_target - x(3, 1));

    % if (t > 1)
    %     variation_angle_leader = abs(x(3, 1)) - leader_angular_speeds(t - 1);
    %     V = abs(0.2 * cos(angle_to_target - x(3, 1)));
    %     fprintf('Vitesse linéaire du leader : %f\n', V')
    % end

    V = 2; % Vitesse linéaire du leader
    V = min(V, r.max_linear_velocity); % Limite de la vitesse linéaire
    dxu(:, 1) = [V; omega]; % Vecteur des vitesses linéaire et angulaire du leader

    if distance_to_waypoint < close_enough
        state = next_state; % Passage à l'état suivant si le waypoint actuel est atteint
    end

    %% Avoid actuator errors
    % To avoid errors, we need to threshold dxi
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4*r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold*dxi(:, to_thresh)./norms(to_thresh);
    
    %% Appliquer les certificats de barrière et la transformation en dynamique unicycle
    dxu(:, 2:N) = si_to_uni_dyn(dxi(:, 2:N), x(:, 2:N));
    dxu = uni_barrier_cert(dxu, x);

    %% Send velocities to agents
    
    %Set velocities
    r.set_velocities(1:N, dxu);
    
    %% Update Plot Handles
    %Update position of labels for followers
    for q = 1:N-1
        follower_labels{q}.Position = x(1:2, q+1) + [-0.15;0.15];    
    end
    
    %Update position of graph connection lines
    for m = 1:length(rows)/2+1
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
    
    if(norm(x(1:2, 1) - waypoint) < close_enough)
        goal_distance = [goal_distance [norm(x(1:2, 1) - waypoint);toc(start_time)]];
    end
    
    %Iterate experiment
    r.step();
end

% Save the data
save('DistanceData.mat', 'robot_distance');
save('GoalData.mat', 'goal_distance');


%% ##################### EVALUATION LOCALISATION #####################
% On s'intéresse sur cette partie seulement à l'évaluation de la
% localisation du robot leader
figure(2) % Trajectoire réelle et trajectoire prédite
plot(odometer(1,:,1), odometer(2,:,1), 'b');
hold on % pour dessiner sur le même graphe
plot(expected_odometer(1,:,1), expected_odometer(2,:,1), 'r');
plot(waypoints(1,1),waypoints(2,1),'*g');
plot(waypoints(1,2),waypoints(2,2),'*g');
plot(waypoints(1,3),waypoints(2,3),'*g');
plot(waypoints(1,4),waypoints(2,4),'*g');

erx = expected_odometer(1,:,1)-odometer(1,:,1);
ery = expected_odometer(2,:,1)-odometer(2,:,1);
er = sqrt(erx.^2+ery.^2);

figure(3) % Plot de l'erreur globale au fil du temps
plot(er);
er=mean(er);
%disp(er)

% sdlkgnkdjgndg

% Assuming 'iterations', 'sigmax', 'sigmay', 'erx', and 'ery' are defined

% Create a new figure with two subplots
figure(4);

% Subplot for X errors
subplot(2,1,1);
plot(3*sigmax(2:iterations), 'r', 'LineWidth', 1.5);
hold on;
plot(erx, 'b', 'LineWidth', 1.5);
hold on;
plot(-3*sigmax(2:iterations), 'r', 'LineWidth', 1.5);
title('Errors in X with 99.7% Confidence Interval');
legend('3*sigma', 'X Errors', '-3*sigma');
grid on;

% Subplot for Y errors
subplot(2,1,2);
plot(3*sigmay(2:iterations), 'r', 'LineWidth', 1.5);
hold on;
plot(ery, 'b', 'LineWidth', 1.5);
hold on;
plot(-3*sigmay(2:iterations), 'r', 'LineWidth', 1.5);
title('Errors in Y with 99.7% Confidence Interval');
legend('3*sigma', 'Y Errors', '-3*sigma');
grid on;

% Adjust layout for better appearance
sgtitle('Combined Errors in X and Y');


% figure(4) % Erreurs en X et la région critique à 99.7%
% plot(3*sigmax(2:iterations), 'r')
% hold on
% plot(erx,'b');
% hold on
% plot(-3*sigmax(2:iterations), 'r')
% 
% figure(5) % Erreurs en Y et la région critique à 99.7%
% plot(3*sigmay(2:iterations), 'r')
% hold on
% plot(ery,'b');
% hold on
% plot(-3*sigmay(2:iterations), 'r')

% affiche les residus isolants
figure(6);
r1 = []; r2 = []; r3 = []; r4 = [];

for i = 2:iterations
    r1 = [r1 r_isol{i}(1)];
    r2 = [r2 r_isol{i}(2)];
    r3 = [r3 r_isol{i}(3)];
    r4 = [r4 r_isol{i}(4)];
end

plot(r1, 'r');
hold on;
plot(r2, 'b');
hold on;
plot(r3, 'g');
hold on;
plot(r4, 'k');
hold on;
line([1 iterations], [th_part th_part], 'linewidth', 1, 'color', 'g');

% affiche le residu global
figure(7);
plot(r_global);
title('Residu global');
xlabel('Temps (s)');
ylabel('Residu global');
grid on;



%% Plot Communication
% Plot and analyze packet loss rate
% figure(6);
% plot(1:iterations, packet_loss_data, 'b-', 'LineWidth', 2, 'Marker', 'o', 'MarkerSize', 5);
% title('Packet Loss Rate Over Time');
% xlabel('Time (iterations)');
% ylabel('Packet Loss Rate');
% ylim([0, communication_timeout]); % Set y-axis limit for better readability
% grid on; % Add grid lines
% legend('Packet Loss Rate');

%%%%%%%%%%%%%%%%%%%%%%%%% EVALUATION LOCALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%% COMMUNICATION Helper Function
function perfect_communication = is_perfect_communication(x, communication_range)
    % Check if communication is perfect
    % For example, check if the robots are within communication range
    
    % Assuming x is the matrix of robot positions and communication_range is the communication range
    distances = pdist(x(1:2, :)', 'euclidean');
    perfect_communication = all(distances <= communication_range);
end

function handle_reliable_communication()
    % Implement reliable communication handling logic
    % For example, log successful communication or take appropriate actions
    disp('Communication is reliable. Log successful communication or take actions.');
end

function handle_obstacle_communication()
    % Implement obstacle communication handling logic
    % For example, log communication delay due to obstacle or take appropriate actions
    %disp('Communication delay due to obstacle. Log the delay or take actions.');
end

function packet_loss = calculate_packet_loss(successful_communication, timeout_threshold)
    % Simulate packet loss based on successful communication and timeout threshold
    % For example, introduce a probability of packet loss or simulate based on conditions
    
    % Assuming successful_communication is a logical indicating successful communication
    % and timeout_threshold is the maximum time allowed for a communication attempt
    
    if successful_communication
        % Simulate packet loss with a certain probability (e.g., 10% chance of loss)
        probability_of_loss = 0.05;
        if rand() <= probability_of_loss
            packet_loss = timeout_threshold; % Packet loss
        else
            packet_loss = 0; % No packet loss
        end
    else
        packet_loss = timeout_threshold; % Communication timeout
    end
end

function communication_delay = calculate_communication_delay(communication_status, obstacle_delay, bypass_speed, communication_timeout)
    % Calculate communication delay based on different scenarios

    switch communication_status
        case 'perfect'
            communication_delay = 0; % No delay for perfect communication
        case 'obstacle'
            % Delay due to obstacle, depends on how fast the robot will bypass the obstacle
            communication_delay = obstacle_delay / bypass_speed;
        case 'broken_link'
            % Communication link is broken
            % If the communication is not restored after communication_timeout seconds, display a message
            if communication_timeout > 0
                communication_delay = communication_timeout;
                fprintf('Cannot re-establish connection. Communication link broken for %d seconds.\n', communication_timeout);
            else
                communication_delay = 0; % Communication timeout is 0, no delay
            end
        otherwise
            error('Invalid communication status. Use ''perfect'', ''obstacle'', or ''broken_link''.');
    end
end

