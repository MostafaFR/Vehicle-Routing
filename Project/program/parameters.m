function [iterations, plot_live, N, r, d, dxi, state, formation_control_gain, si_to_uni_dyn, uni_barrier_cert, uni_to_si_states, waypoints, obstacles, close_enough, list_omega, list_V, leader_speeds, leader_angular_speeds, deriv_leader_speeds, deriv_leader_angular_speeds, robot_distance, goal_distance, line_width] = parameters()
    %% Constantes de l'expérience
    iterations = 1000; % Nombre d'itérations de l'expérience
    plot_live = 0; % Afficher les graphiques en direct (0 pour désactiver, 1 pour activer)

    %% Mise en place de l'objet Robotarium
    N = 5; % Nombre de robots
    initial_positions = generate_initial_conditions(N, 'Width', 2, 'Height', 2, 'Spacing', 0.6); % Génération des positions initiales aléatoires
    r = Robotarium('NumberOfRobots', N, 'ShowFigure', true, 'InitialConditions', initial_positions); % Création de l'objet Robotarium avec les paramètres spécifiés

    %% Initialisation des variables
    desired_distance = 0.3; % Distance désirée entre les robots
    d = desired_distance;

    line_width = 5;

    robot_distance = zeros(5, iterations); % Pour stocker les distances entre les robots et le temps
    goal_distance = []; % Pour stocker les distances aux objectifs lorsque atteints

    dxi = zeros(2, N); % Initialisation du vecteur de vélocités
    state = 1; % État pour le leader
    formation_control_gain = 20; % Paramètres de contrôle de la formation

    %% Configuration des outils pour la dynamique unicycle

    % Transformation de la dynamique single-integrator en unicycle
    si_to_uni_dyn = create_si_to_uni_dynamics('LinearVelocityGain', 1.5);
    % Certificats de barrière single-integrator
    uni_barrier_cert = create_uni_barrier_certificate_with_boundary();
    % Transformation de la dynamique unicycle en single-integrator
    [~, uni_to_si_states] = create_si_to_uni_mapping();

    % Waypoints pour la navigation du leader
    waypoints = [-1 0.8; -1 -0.8; 1 -0.8; 1 0.8]';
    obstacles = [-1 0; 0 -0.8; 1 0; 0 0.8]';
    close_enough = 0.2; % Seuil de proximité aux waypoints

    % Initialisation des tableaux pour stocker les vitesses/acceleration linéaire et angulaire du leader
    list_omega = [];
    list_V = [];
    leader_speeds = [];
    leader_angular_speeds = [];
    deriv_leader_speeds = [];
    deriv_leader_angular_speeds = [];
end
