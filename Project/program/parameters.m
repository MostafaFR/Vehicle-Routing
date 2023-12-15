function [initial_positions, x, R, odometer, expected_odometer, Qu, Q, podo, tfault1, tfault2, tfault3, tfault4, sigmax, sigmay, iterations, plot_live, N, r, d, dxi, state, formation_control_gain, si_to_uni_dyn, uni_barrier_cert, leader_controller, uni_to_si_states, waypoints, obstacles, close_enough, list_omega, list_V, leader_speeds, leader_angular_speeds, deriv_leader_speeds, deriv_leader_angular_speeds, robot_distance, goal_distance, line_width] = parameters()
    clear;     % Efface toutes les variables de l'espace de travail
    close all; % Ferme toutes les fenêtres de figure
    clc; % Nettoie la fenêtre de commande
    %% Constantes de l'expérience
    iterations = 5000; % Nombre d'itérations de l'expérience
    plot_live = 0; % Afficher les graphiques en direct (0 pour désactiver, 1 pour activer)

    %% Mise en place de l'objet Robotarium
    N = 5; % Nombre de robots
    
    initial_positions = generate_initial_conditions(N, 'Width', 2, 'Height', 2, 'Spacing', 0.6); % Génération des positions initiales aléatoires
    % position du robot leader aléatoire tout en haut
    initial_positions(:, 1) = [0.5 1.3 3]';
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
    % Single-integrator position controller
    leader_controller = create_si_position_controller('XVelocityGain', 0.8, 'YVelocityGain', 0.8, 'VelocityMagnitudeLimit', 0.1);
    % Transformation de la dynamique unicycle en single-integrator
    [~, uni_to_si_states] = create_si_to_uni_mapping();

    % Waypoints pour la navigation du leader
    waypoints = [-1 0.8; -1 -0.8; 1 -0.8; 1 0.8]';
    obstacles = [-1.4 0; 0 -0.8; 1 0; 0 0.8]';
    close_enough = 0.2; % Seuil de proximité aux waypoints

    % Initialisation des tableaux pour stocker les vitesses/acceleration linéaire et angulaire du leader
    list_omega = [];
    list_V = [];
    leader_speeds = [];
    leader_angular_speeds = [];
    deriv_leader_speeds = [];
    deriv_leader_angular_speeds = [];

    %% LOCALISATION
    x = r.get_poses();
    odometer(:, 1, :) = x(:, :); % Valeurs obtenues par la localisation
    expected_odometer(:, 1, :) = odometer(:, 1, :); % Valeurs réelles

    Qu = [0.5 * 10 ^ -4 0; 0 0.5 * 10 ^ -4] * 0; %covariance associée aux bruits du vecteur d'entrée
    Q = [0.6 * 10 ^ -5 0 0; 0 0.4 * 10 ^ -5 0; 0 0 10 ^ -5]; %covariance du bruit de modèle
    podo = [1 0 0; 0 1 0; 0 0 0.5]; %valeur initiale de la matrice de covariance P

    tfault1 = [20:50, 200:260];
    tfault2 = [20:50, 80:100, 340:400];
    tfault3 = [150:180, 370:450];
    tfault4 = [400:500, 1700:2500];

    sigmax = zeros(iterations, 1);
    sigmay = zeros(iterations, 1);

    R = 10 ^ -5; %covariance du bruit de mesure
end
