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
