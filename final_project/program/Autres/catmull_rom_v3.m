[iterations, plot_live, N, r, d, dxi, state, formation_control_gain, si_to_uni_dyn, uni_barrier_cert, uni_to_si_states, waypoints, obstacles, close_enough, list_omega, list_V, leader_speeds, leader_angular_speeds, deriv_leader_speeds, deriv_leader_angular_speeds, robot_distance, goal_distance, line_width] = parameters();

[L_diamond, weights_diamond, L_line, weights_line] = laplacian_matrices(d);

% Configuration de la représentation graphique des connexions entre les robots
x = r.get_poses(); % Obtention des positions des robots

[L, weights, rows, cols, lf, ll] = set_laplacian(L_diamond, weights_diamond, x, line_width);

[font_size, g, goal_labels, follower_caption, follower_labels, leader_label, angular_velocity_arrows] = Affichage(r, waypoints, obstacles, N, line_width, x);
start_time = tic;

if (plot_live == 1)
    [plot1, plot2] = subplots(r);
end

% -----------------------
% Définition des points de contrôle pour la courbe de Catmull-Rom
% -----------------------
numPoints = 100; % Nombre de points intermédiaires
curvePoints = []; % Initialisation des points de la courbe
waypoint_index = 1; % Index pour suivre la courbe Catmull-Rom

initial_points = waypoints(:, 1:2)'; % Utiliser les waypoints comme points de contrôle
initial_points = [x(1:2, 1)'; initial_points]; % Ajouter la position du leader comme point de contrôle
initial_points = [initial_points(1, :); initial_points; initial_points(end, :)]; % Ajouter des points supplémentaires

curvePoints = calculateCatmullRomCurve(initial_points, numPoints); % Calculer la courbe de Catmull-Rom

% Tracez la courbe de Catmull-Rom
curve_x = curvePoints(:, 1);
curve_y = curvePoints(:, 2);
curve_line = plot(curve_x, curve_y, 'm-', 'LineWidth', 2); % Tracez la courbe en magenta
hold on;

r.step();
% COMMUNICATION DE LA FORMATION EN BLE 5.0
time_iteration = []
debit_theorique = 2; % en Mbps
portee_max = 10; % en m

for t = 1:iterations

    current_time_iteration = tic;
    % Changement de formation en ligne au cours du temps
    if (t == 1000)
        L = L_line;
        weights = weights_line;
        % Supprimer les connexions existantes
        delete(lf);
        % Créer de nouvelles connexions entre les robots suiveurs
        [rows, cols] = find(L == -1);

        for k = 1:length(rows)
            lf(k) = line([x(1, rows(k)), x(1, cols(k))], [x(2, rows(k)), x(2, cols(k))], 'LineWidth', line_width, 'Color', 'b');
        end

    end

    % Changement de formation en diamant au cours du temps
    if (t == 2000)
        L = L_diamond;
        weights = weights_diamond;
        % Supprimer les connexions existantes
        delete(lf);
        % Créer de nouvelles connexions entre les robots suiveurs
        [rows, cols] = find(L == -1);

        for k = 1:length(rows)
            lf(k) = line([x(1, rows(k)), x(1, cols(k))], [x(2, rows(k)), x(2, cols(k))], 'LineWidth', line_width, 'Color', 'b');
        end

    end

    %% Mise à jour des positions des robots
    x = r.get_poses(); % Récupération des poses les plus récentes du Robotarium
    xsi = uni_to_si_states(x); % Conversion en états single-integrator

    %% Algorithme de contrôle pour la formation
    for i = 2:N
        dxi(:, i) = [0; 0]; % Réinitialisation de la vitesse du robot i

        neighbors = topological_neighbors(L, i); % Obtenir les voisins du robot i

        for j = neighbors
            dxi(:, i) = dxi(:, i) + formation_control_gain * (norm(xsi(:, i) - xsi(:, j)) ^ 2 - weights(i, j) ^ 2) * (xsi(:, j) - xsi(:, i));
        end

    end

    %% Logique de navigation du leader
    waypoint = waypoints(:, state); % Sélection du waypoint actuel
    next_state = mod(state, size(waypoints, 2)) + 1; % Passage à l'état suivant
    next_waypoint = waypoints(:, next_state); % Prochain waypoint
    distance_to_waypoint = norm(x(1:2, 1) - waypoint); % Distance au waypoint actuel

    % Mise à jour de la position du leader
    target_point = curvePoints(waypoint_index, :)';
    dxi(:, 1) = leader_controller(x(1:2, 1), target_point);

    if norm(x(1:2, 1) - target_point) < 0.2
        waypoint_index = waypoint_index + 1;

        if waypoint_index > size(curvePoints, 1)
            waypoint_index = 1; % Recommencer à suivre la courbe
        else
            % Supprimer la partie de la courbe parcourue
            if (waypoint_index > 10)
                curvePoints_maj = curvePoints(waypoint_index -10:end, :); % Conserve uniquement les points non parcourus
                set(curve_line, 'XData', curvePoints_maj(:, 1), 'YData', curvePoints_maj(:, 2)); % Mise à jour de la courbe
            end

        end

    end

    if distance_to_waypoint < close_enough

        % Sélectionner les trois prochains waypoints
        next_indices = state:min(state + 2, size(waypoints, 2));

        if length(next_indices) < 3
            next_indices = [next_indices, 1:(3 - length(next_indices))];
        end

        % Recalculer la courbe de Catmull-Rom avec les nouveaux waypoints
        control_points = [x(1:2, 1)'; waypoints(:, next_indices(1))'; waypoints(:, next_indices(2))'; waypoints(:, next_indices(3))'];
        control_points = [control_points; control_points(1, :)]; % Boucler sur le premier point pour continuité
        curvePoints = calculateCatmullRomCurve(control_points, numPoints); % Recalcul de la courbe
        waypoint_index = 1; % Réinitialisation de l'index de la courbe
        set(curve_line, 'XData', curvePoints(:, 1), 'YData', curvePoints(:, 2)); % Mise à jour de la courbe

        state = next_state; % Passage à l'état suivant si le waypoint actuel est atteint
    end

    %% Éviter les erreurs de l'actionneur
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4 * r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold * dxi(:, to_thresh) ./ norms(to_thresh);

    %% Appliquer les certificats de barrière et la transformation en dynamique unicycle
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert(dxu, x);

    list_omega = [list_omega; dxu(2, 1)]; % Stockage de la vitesse angulaire du leader
    list_V = [list_V; dxu(1, 1)]; % Stockage de la vitesse linéaire du leader
    leader_speeds(t) = dxu(1, 1); % Vitesse linéaire du leader
    leader_angular_speeds(t) = abs(x(3, 1)); % Vitesse angulaire du leader

    %% Envoyer les vitesses aux robots
    r.set_velocities(1:N, dxu);

    %% Mise à jour des éléments graphiques
    for q = 1:N - 1
        follower_labels{q}.Position = x(1:2, q + 1) + [-0.15; 0.15];
    end

    % Connexions entre les robots suiveurs
    for m = 1:length(rows)
        lf(m).XData = [x(1, rows(m)), x(1, cols(m))];
        lf(m).YData = [x(2, rows(m)), x(2, cols(m))];
    end

    % Connexion du leader
    leader_label.Position = x(1:2, 1) + [-0.15; 0.15];
    ll.XData = [x(1, 1), x(1, 2)];
    ll.YData = [x(2, 1), x(2, 2)];
    marker_size_goal = num2cell(ones(1, length(waypoints)) * determine_marker_size(r, 0.20));
    [g.MarkerSize] = marker_size_goal{:};
    font_size = determine_font_size(r, 0.05);
    leader_label.FontSize = font_size;

    if (plot_live == 1)
        % Mise à jour des subplots en temps réel
        set(plot1, 'XData', 1:length(list_omega), 'YData', list_omega);
        set(plot2, 'XData', 1:length(list_V), 'YData', list_V);
        drawnow; % Redessiner la figure
    end

    % Texte pour les objectifs
    for n = 1:N
        follower_labels{n}.FontSize = font_size;
        goal_labels{n}.FontSize = font_size;
    end

    % Flèches de vitesse angulaire
    for i = 1:N
        arrow_length = 0.2;
        arrow_dx = arrow_length * cos(x(3, i) + dxu(2, i));
        arrow_dy = arrow_length * sin(x(3, i) + dxu(2, i));
        set(angular_velocity_arrows(i), 'XData', x(1, i), 'YData', x(2, i), 'UData', arrow_dx, 'VData', arrow_dy);
    end

    %% Calculer les données à sauvegarder et les stocker dans la matrice
    robot_distance(1, t) = norm([x(1:2, 1) - x(1:2, 2)], 2);
    robot_distance(5, t) = toc(start_time);

    % Calculer les distances entre les robots
    for b = 1:length(rows)
        robot_distance(b + 1, t) = norm([x(1:2, rows(b)) - x(1:2, cols(b))], 2);
    end

    % Calculer les distances aux objectifs
    if (distance_to_waypoint < close_enough)
        goal_distance = [goal_distance [norm(x(1:2, 1) - waypoint); toc(start_time)]];
    end

    r.step();
    time_iteration = [time_iteration toc(current_time_iteration)];
end

disp(time_iteration)

for t = 2:iterations
    deriv_leader_speeds(t) = leader_speeds(t) - leader_speeds(t - 1);
    deriv_leader_angular_speeds(t) = leader_angular_speeds(t) - leader_angular_speeds(t - 1);
end

if (plot_live == 0)
    %% Graphiques pour omega et V dans des subplots
    figure;
    subplot(2, 1, 1);
    plot(2:iterations, deriv_leader_speeds);
    title('Dérivée de la Vitesse Linéaire du Leader');
    xlabel('Itération');
    ylabel('Dérivée de la Vitesse');

    subplot(2, 1, 2); % Deuxième subplot pour V
    plot(list_V);
    title('V');
    xlabel('Temps');
    ylabel('Vitesse');
    ylim([0, r.max_linear_velocity]); % Échelle pour V

    figure;
    subplot(2, 1, 1); % Premier subplot pour omega
    plot(list_omega);
    title('Omega');
    xlabel('Temps');
    ylabel('Omega');
    ylim([0, pi]); % Définir les limites de l'axe des y à [-pi, pi]
    yticks([0, pi / 2, pi]); % Définir les marques spécifiques sur l'axe des y
    yticklabels({'0', '\pi/2', '\pi'}); % Étiqueter les marques de l'axe des y

    subplot(2, 1, 2);
    plot(2:iterations, deriv_leader_angular_speeds);
    title('Dérivée de la Vitesse Angulaire du Leader');
    xlabel('Itération');
    ylabel('Dérivée de la Vitesse Angulaire');

end

% Sauvegarde des données
save('DonneesDistance.mat', 'robot_distance');
save('DonneesObjectif.mat', 'goal_distance');

% Fonction de débogage pour l'expérience
r.debug();

%% Fonctions auxiliaires

% Fonction auxiliaire pour déterminer la taille du marqueur en fonction de la fenêtre de la figure
function marker_size = determine_marker_size(robotarium_instance, marker_size_meters)
    curunits = get(robotarium_instance.figure_handle, 'Units');
    set(robotarium_instance.figure_handle, 'Units', 'Points');
    cursize = get(robotarium_instance.figure_handle, 'Position');
    set(robotarium_instance.figure_handle, 'Units', curunits);
    marker_ratio = (marker_size_meters) / (robotarium_instance.boundaries(2) - ...
        robotarium_instance.boundaries(1));
    marker_size = cursize(3) * marker_ratio;
end

% Fonction auxiliaire pour déterminer la taille de la police en fonction de la fenêtre de la figure
function font_size = determine_font_size(robotarium_instance, font_height_meters)
    curunits = get(robotarium_instance.figure_handle, 'Units');
    set(robotarium_instance.figure_handle, 'Units', 'Points');
    cursize = get(robotarium_instance.figure_handle, 'Position');
    set(robotarium_instance.figure_handle, 'Units', curunits);
    font_ratio = (font_height_meters) / (robotarium_instance.boundaries(4) - ...
        robotarium_instance.boundaries(3));
    font_size = cursize(4) * font_ratio;
end

function [font_size, g, goal_labels, follower_caption, follower_labels, leader_label, angular_velocity_arrows] = Affichage(r, waypoints, obstacles, N, line_width, x)
    %% Configuration de l'affichage
    CM = ['k', 'b', 'r', 'g']; % Assurez-vous que cette ligne contient assez de couleurs pour tous vos waypoints

    % Tailles des marqueurs, des polices et des lignes
    marker_size_goal = determine_marker_size(r, 0.20);
    font_size = determine_font_size(r, 0.05);

    % Configuration de l'affichage du leader
    leader_label = text(500, 500, 'Robot Leader', 'FontSize', font_size, 'FontWeight', 'bold', 'Color', 'r');

    % Assurez-vous que le tableau CM a suffisamment de couleurs pour le nombre de waypoints
    num_waypoints = size(waypoints, 2);

    if length(CM) < num_waypoints
        CM = repmat(CM, 1, ceil(num_waypoints / length(CM)));
    end

    % Initialisation du tableau g pour les marqueurs des waypoints
    g = gobjects(1, num_waypoints);

    % Création des marqueurs et des textes pour les objectifs
    for i = 1:num_waypoints
        goal_caption = sprintf('G%d', i); % Texte d'identification de l'objectif
        g(i) = plot(waypoints(1, i), waypoints(2, i), 's', 'MarkerSize', marker_size_goal, 'LineWidth', line_width, 'Color', CM(i)); % Carré coloré pour l'objectif
        plot(obstacles(1, :), obstacles(2, :), 'o', 'LineWidth', 1)
        goal_labels{i} = text(waypoints(1, i) - 0.05, waypoints(2, i), goal_caption, 'FontSize', font_size, 'FontWeight', 'bold'); % Texte dans l'objectif
    end

    % Configuration de l'affichage des robots suiveurs
    for j = 1:N - 1
        follower_caption{j} = sprintf('RS %d', j); % Texte d'identification du robot suiveur
        follower_labels{j} = text(500, 500, follower_caption{j}, 'FontSize', font_size, 'FontWeight', 'bold');
    end

    angular_velocity_arrows = set_angular_velocity_arrows(N, x);
end

function [L, weights, rows, cols, lf, ll] = set_laplacian(L_diamond, weights_diamond, x, line_width);
    L = L_diamond; % Initialisation de la matrice Laplacienne (formation en diamant)
    weights = weights_diamond; % Initialisation des poids pour les distances

    % Connexions entre les robots suiveurs
    [rows, cols] = find(L == -1);

    for k = 1:length(rows)
        lf(k) = line([x(1, rows(k)), x(1, cols(k))], [x(2, rows(k)), x(2, cols(k))], 'LineWidth', line_width, 'Color', 'b');
    end

    % Connexion du leader (supposée seulement entre le premier et le deuxième robot)
    ll = line([x(1, 1), x(1, 2)], [x(2, 1), x(2, 2)], 'LineWidth', line_width, 'Color', 'r');
end

function [plot1, plot2] = subplots(r)
    % Création de la figure et des subplots
    figure;
    subplot(2, 1, 1);
    plot1 = plot(NaN, NaN); % Création d'un objet de ligne vide
    title('Omega');
    xlabel('Temps');
    ylabel('Omega');
    ylim([0, pi]); % Définir les limites de l'axe des y à [-pi, pi]
    yticks([0, pi / 2, pi]); % Définir les marques spécifiques sur l'axe des y
    yticklabels({'0', '\pi/2', '\pi'}); % Étiqueter les marques de l'axe des y

    subplot(2, 1, 2);
    plot2 = plot(NaN, NaN); % Création d'un objet de ligne vide
    title('V');
    xlabel('Temps');
    ylabel('Vitesse');
    ylim([0, r.max_linear_velocity]);

end

function angular_velocity_arrows = set_angular_velocity_arrows(N, x)
    angular_velocity_arrows = gobjects(1, N); % Tableau d'objets graphiques pour les flèches de vitesse angulaire

    for i = 1:N
        angular_velocity_arrows(i) = quiver(x(1, i), x(2, i), 0, 0, 'MaxHeadSize', 0.5, 'Color', 'm');
    end

end

function curvePoints = calculateCatmullRomCurve(points, numPoints)
    curvePoints = [];

    for i = 1:length(points) - 3
        P0 = points(i, :);
        P1 = points(i + 1, :);
        P2 = points(i + 2, :);
        P3 = points(i + 3, :);

        for t = 0:1 / numPoints:1
            T = [1, t, t ^ 2, t ^ 3];
            M = [0, 1, 0, 0;
                 -1, 0, 1, 0;
                 2, -2, 1, -1;
                 -1, 1, -1, 1];
            G = [P0; P1; P2; P3];
            pt = T * M * G;
            curvePoints = [curvePoints; pt];
        end

    end

end
