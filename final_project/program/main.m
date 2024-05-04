[initial_positions, x, R, odometer, expected_odometer, Qu, Q, podo, tfault1, tfault2, tfault3, tfault4, sigmax, sigmay, iterations, plot_live, N, r, d, dxi, state, formation_control_gain, si_to_uni_dyn, uni_barrier_cert, leader_controller, uni_to_si_states, waypoints, obstacles, close_enough, list_omega, list_V, leader_speeds, leader_angular_speeds, deriv_leader_speeds, deriv_leader_angular_speeds, robot_distance, goal_distance, line_width] = parameters()

[L_diamond, weights_diamond, L_line, weights_line] = laplacian_matrices(d);

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

control_points = waypoints(:, 1:2)'; % Utiliser les waypoints comme points de contrôle
control_points = [x(1:2, 1)'; control_points]; % Ajouter la position du leader comme point de contrôle
control_points = [control_points(1, :); control_points; control_points(end, :)]; % Ajouter des points supplémentaires

curvePoints = calculateCatmullRomCurve(control_points, numPoints); % Calculer la courbe de Catmull-Rom

% Tracez la courbe de Catmull-Rom
curve_x = curvePoints(:, 1);
curve_y = curvePoints(:, 2);
curve_line = plot(curve_x, curve_y, 'g', 'LineWidth', 2); % Tracez la courbe en magenta
hold on;

% -----------------------
% Initialisation des variables Communication

debit_bluetooth = 2; % en Mbit/s
poid_donnee = 0.25; % en Mbit
portee_max = 10; % en m
temps_transmission = poid_donnee / debit_bluetooth; % en s
energie = zeros(2, iterations); % vecteur d'énergie
% time_iteration
packet_loss_data = zeros(1, iterations); % vecteur de perte de paquets
probabilite_perte_paquet = 0.05; % 5 % de probabilité de perte de paquet

r.step();

for t = 2:iterations

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% COMMUNICATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    time_current_iteration = tic;
    energie(t) = 0.7 * (1 - t / iterations) + 0.3;

    % Simulation de la perte de paquet
    if rand() <= probabilite_perte_paquet
        packet_loss_data(t) = 1; % Perte de paquet
    else
        packet_loss_data(t) = 0; % Transmission réussie
    end

    % LOCALISATION
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOCALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    x = r.get_poses();
    expected_odometer(:, t, :) = x(:, :);
    velocities = r.get_velocities() + normrnd(0, 0.001);

    for i = 1:N
        % Prédiction
        A = [cos(odometer(3, t - 1, i)) 0; sin(odometer(3, t - 1, i)) 0; 0 1];
        odometer(:, t, i) = odometer(:, t - 1, i) + A * [r.time_step * velocities(1, i); r.time_step * velocities(2, i)]; % delta_k, omega_k

        FF = [1 0 -r.time_step * velocities(1, i) * sin(odometer(3, t - 1, i)); 0 1 r.time_step * velocities(1, i) * cos(odometer(3, t - 1, i)); 0 0 1]; %delta_k
        podo = FF * podo * FF' + Q; %

        X = odometer(:, t, i); % vecteur d'état prédit
        Y = inv(podo); % matrice informationnelle
        y = Y * X; % vecteur informationnel
        Yp = inv(podo); %matrice informationnelle
        yp = Yp * X; %vecteur informationnel

        % Update
        % Calcul des distances entre les amers et les coordonnées réelles du
        % robot à l'instant t
        d1 = sqrt((expected_odometer(1, t, i) - waypoints(1, 1)) .^ 2 + (expected_odometer(2, t, i) - waypoints(2, 1)) .^ 2);
        d2 = sqrt((expected_odometer(1, t, i) - waypoints(1, 2)) .^ 2 + (expected_odometer(2, t, i) - waypoints(2, 2)) .^ 2);
        d3 = sqrt((expected_odometer(1, t, i) - waypoints(1, 3)) .^ 2 + (expected_odometer(2, t, i) - waypoints(2, 3)) .^ 2);
        d4 = sqrt((expected_odometer(1, t, i) - waypoints(1, 4)) .^ 2 + (expected_odometer(2, t, i) - waypoints(2, 4)) .^ 2);
        % On crée le vecteur z en ajoutant un bruit blanc pour simuler le
        % capteur LIDAR --> ne pas mettre + que 0.001, sinon plus de "bonne
        % correction$
        z = [d1 + normrnd(0, 0.001); d2 + normrnd(0, 0.001); d3 + normrnd(0, 0.001); d4 + normrnd(0, 0.001)];

        % Injection de défauts sur les 4 capteurs de distance sur un intervalle donné
        if find(tfault1 == t)
            z(1) = z(1) + 0.05;
        end

        if find(tfault2 == t)
            z(2) = z(2) + 0.05;
        end

        if find(tfault3 == t)
            z(3) = z(3) + 0.05;
        end

        if find(tfault4 == t)
            z(4) = z(4) + 0.05;
        end

        % initialisation des gains informationnels
        % Sans détection de défauts
        gI = zeros(3, 3);
        gi = zeros(3, 1);
        % Avec détection de défauts
        sgI = zeros(3, 3);
        sgi = zeros(3, 1);
        gI = [];
        gi = [];
        th_part = chi2inv(0.9, 3);

        n = length(waypoints);
        %% detection défaut - choix
        for j = 1:n
            % zestime=sqrt((X(1)-waypoints(1,j))^2+(X(2)-waypoints(2,j))^2);
            % H=[(X(1)-waypoints(1,j))/zestime, (X(2)-waypoints(2,j))/zestime,0]; % jacobienne
            % gI = gI+H'*inv(R)*H; % contribution informationnelle associée à la matrice
            % nu=z(j)-zestime;
            % gi=gi+H'inv(R)(nu+H*X); % contribution informationnelle associée au vecteur

            %%%%%%%%%%%%%%%%%%%%%%%% DETECTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            zestime = sqrt((X(1) - waypoints(1, j)) ^ 2 + (X(2) - waypoints(2, j)) ^ 2);
            H = [(X(1) - waypoints(1, j)) / zestime, (X(2) - waypoints(2, j)) / zestime, 0]; % jacobienne
            gI{j} = H' * inv(R) * H; % contribution informationnelle associée à la mesure z(i)
            sgI = sgI + gI{j};
            nu = z(j) - zestime;
            gi{j} = H' * inv(R) * [nu + H * X]; % contribution informtionnelle associée à la mesure z(i)
            sgi = sgi + gi{j};

            r_isol{t}(j) = diagnostic(gI{j}, gi{j}, yp, Yp, X); %résidu pour l'isolation des défauts

        end

        %% Mise à jour finale sans détection de défauts
        % Y=Y+gI; % Mise à jour matrice informationnelle
        % y=y+gi; % Mise à jour du vecteur informationnel

        % podo=inv(Y); % Mise à jour de la matrice de covariance
        % odometer(:,t,i)=podo*y; % Mise à jour du vecteur d'état à l'instant t

        %% Mise à jour finale avec détection de défauts
        %%%%%%%%%%%%%%%%%%%%%%% detection de défaut %%%%%%%%%%%%%%%%%%%%%
        [r_global(t), detect] = detection(sgi, sgI, yp, Yp, X);

        % si défaut détecté=> exclusion des mesures erronées
        if detect

            for k = 1:n

                if r_isol{t}(k) > th_part
                    sgi = sgi - gi{k};
                    sgI = sgI - gI{k};
                end

            end

        end

        %% correction phase
        Yu = Yp + sgI; % matrice informationnelle mise à jour
        yu = yp + sgi; % vecteur informationnel mis à jour

        podo = inv(Yu); % matrice de covariance mise à jour
        odometer(:, t, i) = podo * yu; % vecteur d'état mis à jour à l'instant t
        % ################## FIN detection de défaut ##################

        if i == 1 % On base notre étude uniquement sur le leader
            sigmax(t) = sqrt(podo(1, 1));
            sigmay(t) = sqrt(podo(2, 2));
        end

    end

    x_odo = odometer(:, t, :); % on assigne à x_odo la prédiction finale de la localisation

    %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% LOCALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Changement de formation en ligne au cours du temps
    if (t == 1000)
        L = L_line;
        weights = weights_line;
        % Supprimer les connexions existantes
        delete(lf);
        delete(ll);
        % Créer de nouvelles connexions entre les robots suiveurs
        [rows, cols] = find(L == -1);

        for k = 1:length(rows)
            lf(k) = line([x(1, rows(k)), x(1, cols(k))], [x(2, rows(k)), x(2, cols(k))], 'LineWidth', line_width, 'Color', 'b');
        end

        % Connexion du leader (supposée seulement entre le premier et le deuxième robot)
        ll = line([x(1, 1), x(1, 2)], [x(2, 1), x(2, 2)], 'LineWidth', line_width, 'Color', 'r');

    end

    % Changement de formation en diamant au cours du temps
    if (t == 2000)
        L = L_diamond;
        weights = weights_diamond;
        % Supprimer les connexions existantes
        delete(lf);
        delete(ll);
        % Créer de nouvelles connexions entre les robots suiveurs
        [rows, cols] = find(L == -1);

        for k = 1:length(rows)
            lf(k) = line([x(1, rows(k)), x(1, cols(k))], [x(2, rows(k)), x(2, cols(k))], 'LineWidth', line_width, 'Color', 'b');
        end

        % Connexion du leader (supposée seulement entre le premier et le deuxième robot)
        ll = line([x(1, 1), x(1, 2)], [x(2, 1), x(2, 2)], 'LineWidth', line_width, 'Color', 'r');

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
        savedPoints = curvePoints(end - 200:end - 199, :);
        % Sélectionner les trois prochains waypoints
        next_indices = state:min(state + 2, size(waypoints, 2));

        if length(next_indices) < 3
            next_indices = [next_indices, 1:(3 - length(next_indices))];
        end

        % Recalculer la courbe de Catmull-Rom avec les nouveaux waypoints
        control_points = [savedPoints; waypoints(:, next_indices(1))'; waypoints(:, next_indices(2))'; waypoints(:, next_indices(3))'];
        control_points = [control_points; control_points(1, :)]; % Boucler sur le premier point pour continuité
        curvePoints = calculateCatmullRomCurve(control_points, numPoints); % Recalcul de la courbe
        % Supprimer tous les points de la courbe jusqu'a waypoints(:, next_indices(1))'
        curvePoints = curvePoints(waypoint_index:end, :);
        waypoint_index = 1; % Réinitialisation de l'index de la courbe
        set(curve_line, 'XData', curvePoints(:, 1), 'YData', curvePoints(:, 2)); % Mise à jour de la courbe

        state = next_state; % Passage à l'état suivant si le waypoint actuel est atteint
    end

    %% Mise à jour des positions des robots
    xsi = uni_to_si_states(x); % Conversion en états single-integrator

    % %% Algorithme de contrôle pour la formation
    % for i = 2:N
    %     dxi(:, i) = [0; 0]; % Réinitialisation de la vitesse du robot i

    %     neighbors = topological_neighbors(L, i); % Obtenir les voisins du robot i

    %     for j = neighbors
    %         dxi(:, i) = dxi(:, i) + formation_control_gain * (norm(xsi(:, i) - xsi(:, j)) ^ 2 - weights(i, j) ^ 2) * (xsi(:, j) - xsi(:, i));
    %     end

    % end
    % Update robots' states and control inputs
    for i = 2:N
        robot_pos = x(1:2, i); % Current position of robot i
        robot_angle = x(3, i); % Current orientation of robot i

        [d_obs, d_obs_dot, nearest_obstacle_idx] = compute_obstacle_distance_and_derivative(robot_pos, obstacles);

        V = lyapunov_function(d_obs);
        V_dot = lyapunov_derivative(d_obs, d_obs_dot);

        if d_obs < close_enough
            % Robot is approaching an obstacle, steer away from it
            % Determine the direction to turn based on obstacle position
            obstacle_pos = obstacles(:, nearest_obstacle_idx); % Get position of nearest obstacle
            angle_to_obstacle = atan2(obstacle_pos(2) - robot_pos(2), obstacle_pos(1) - robot_pos(1));
            turn_direction = sign(sin(robot_angle - angle_to_obstacle)); % Determine turn direction

            avoidance_speed = 0.5; % Set a speed for avoidance
            avoidance_turn_rate = 0.3; % Set a turn rate for avoidance

            % Update control input to steer away from the obstacle
            dxi(:, i) = [cos(avoidance_turn_rate); sin(avoidance_turn_rate)] * avoidance_speed;
        else

            if i > 1
                dxi(:, i) = [0; 0]; % Réinitialisation de la vitesse du robot i

                neighbors = topological_neighbors(L, i); % Obtenir les voisins du robot i

                for j = neighbors
                    dxi(:, i) = dxi(:, i) + formation_control_gain * (norm(xsi(:, i) - xsi(:, j)) ^ 2 - weights(i, j) ^ 2) * (xsi(:, j) - xsi(:, i));
                end

            end

        end

    end

    %% Éviter les erreurs de l'actionneur
    norms = arrayfun(@(x) norm(dxi(:, x)), 1:N);
    threshold = 3/4 * r.max_linear_velocity;
    to_thresh = norms > threshold;
    dxi(:, to_thresh) = threshold * dxi(:, to_thresh) ./ norms(to_thresh);

    %% Appliquer les certificats de barrière et la transformation en dynamique unicycle
    dxu = si_to_uni_dyn(dxi, x);
    dxu = uni_barrier_cert(dxu, x);

    list_omega = [list_omega; abs(x(3, 1))]; % Stockage de la vitesse angulaire du leader
    list_V = [list_V; dxu(1, 1)]; % Stockage de la vitesse linéaire du leader
    leader_speeds(t) = dxu(1, 1); % Vitesse linéaire du leader
    leader_angular_speeds(t) = abs(x(3, 1)); % Vitesse angulaire du leader
    % Robot 1 et 2
    robot_d12(t) = norm([x(1:2, 1) - x(1:2, 2)], 2);
    % Robot 2 et 3
    robot_d23(t) = norm([x(1:2, 2) - x(1:2, 3)], 2);
    % Robot 2 et 4
    robot_d24(t) = norm([x(1:2, 2) - x(1:2, 4)], 2);
    % Robot 3 et 4
    robot_d34(t) = norm([x(1:2, 3) - x(1:2, 4)], 2);
    % Robot 3 et 5
    robot_d35(t) = norm([x(1:2, 3) - x(1:2, 5)], 2);
    % Robot 4 et 5
    robot_d45(t) = norm([x(1:2, 4) - x(1:2, 5)], 2);

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
    arrow_length = 0.2;
    arrow_dx = arrow_length * cos(x(3, 1) + dxu(2, 1));
    arrow_dy = arrow_length * sin(x(3, 1) + dxu(2, 1));
    set(angular_velocity_arrows(1), 'XData', x(1, 1), 'YData', x(2, 1), 'UData', arrow_dx, 'VData', arrow_dy);

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

    time_iteration(t) = toc(time_current_iteration);

    r.step();
end

for t = 2:iterations
    deriv_leader_speeds(t) = leader_speeds(t) - leader_speeds(t - 1);
    deriv_leader_angular_speeds(t) = leader_angular_speeds(t) - leader_angular_speeds(t - 1);
end

if (plot_live == 0)
    %% Graphiques pour omega et V dans des subplots
    figure;
    subplot(2, 1, 1);
    plot(2:iterations, list_V);
    title('Vitesses Linéaires du Leader V');
    xlabel('Temps');
    ylabel('Vitesse');
    ylim([0, r.max_linear_velocity]); % Échelle pour V

    subplot(2, 1, 2); % Deuxième subplot pour V
    plot(1:iterations, deriv_leader_speeds);
    title('Dérivée de la Vitesse Linéaire du Leader');
    xlabel('Itération');
    ylabel('Dérivée de la Vitesse');

    figure;
    subplot(2, 1, 1); % Premier subplot pour omega
    plot(2:iterations, list_omega);
    disp(list_omega);
    title('Angulaire du Leader Omega');
    xlabel('Temps');
    ylabel('Omega');
    ylim([0, pi]); % Définir les limites de l'axe des y à [0, pi]
    yticks([0, pi / 2, pi]); % Définir les marques spécifiques sur l'axe des y
    yticklabels({'0', '\pi/2', '\pi'}); % Étiqueter les marques de l'axe des y

    subplot(2, 1, 2);
    plot(1:iterations, deriv_leader_angular_speeds);
    title('Dérivée de Omega');
    xlabel('Itération');
    ylabel('Dérivée de la Vitesse Angulaire');

end

% Plot les distances de robot robot_d entre eux dans des subplot
figure;
subplot(2, 3, 1);
plot(robot_d12);
title('Distance entre le robot 1 et le robot 2');
xlabel('Temps');
ylabel('Distance');
ylim([0, 1.5]);

subplot(2, 3, 2);
plot(robot_d23);
title('Distance entre le robot 2 et le robot 3');
xlabel('Temps');
ylabel('Distance');
ylim([0, 1.5]);

subplot(2, 3, 3);
plot(robot_d24);
title('Distance entre le robot 2 et le robot 4');
xlabel('Temps');
ylabel('Distance');
ylim([0, 1.5]);

subplot(2, 3, 4);
plot(robot_d34);
title('Distance entre le robot 3 et le robot 4');
xlabel('Temps');
ylabel('Distance');
ylim([0, 1.5]);

subplot(2, 3, 5);
plot(robot_d35);
title('Distance entre le robot 3 et le robot 5');
xlabel('Temps');
ylabel('Distance');
ylim([0, 1.5]);

subplot(2, 3, 6);
plot(robot_d45);
title('Distance entre le robot 4 et le robot 5');
xlabel('Temps');
ylabel('Distance');
ylim([0, 1.5]);

% Temps de simulation au fur et à mesure des itérations
figure;
plot(time_iteration);
title('Temps de simulation au fur et à mesure des itérations');
xlabel('Itération');
ylabel('Temps (s)');
grid on;

% Plot du temps de transmission des données entre robot
figure;
subplot(2, 3, 1);
plot(robot_d12 * 10 / portee_max * temps_transmission * energie(t));
title('Temps de transmission entre le robot 1 et le robot 2');
xlabel('Temps');
ylabel('Temps (s)');
ylim([0, 0.25]);

subplot(2, 3, 2);
plot(robot_d23 * 10 / portee_max * temps_transmission * energie(t));
title('Temps de transmission entre le robot 2 et le robot 3');
xlabel('Temps');
ylabel('Temps (s)');
ylim([0, 0.25]);

subplot(2, 3, 3);
plot(robot_d24 * 10 / portee_max * temps_transmission * energie(t));
title('Temps de transmission entre le robot 2 et le robot 4');
xlabel('Temps');
ylabel('Temps (s)');
ylim([0, 0.25]);

subplot(2, 3, 4);
plot(robot_d34 * 10 / portee_max * temps_transmission * energie(t));
title('Temps de transmission entre le robot 3 et le robot 4');
xlabel('Temps');
ylabel('Temps (s)');
ylim([0, 0.25]);

subplot(2, 3, 5);
plot(robot_d35 * 10 / portee_max * temps_transmission * energie(t));
title('Temps de transmission entre le robot 3 et le robot 5');
xlabel('Temps');
ylabel('Temps (s)');
ylim([0, 0.25]);

subplot(2, 3, 6);
plot(robot_d45 * 10 / portee_max * temps_transmission * energie(t));
title('Temps de transmission entre le robot 4 et le robot 5');
xlabel('Temps');
ylabel('Temps (s)');
ylim([0, 0.25]);

% Création du graphique de la perte de paquets
figure;
plot(packet_loss_data);
title('Taux de Perte de Paquets au Fil du Temps');
xlabel('Temps (itérations)');
ylabel('Taux de Perte de Paquets');
ylim([0, 1]); % Définir les limites de l'axe des y pour une meilleure lisibilité
grid on; % Ajouter des lignes de grille
legend('Taux de Perte de Paquets');

%% ##################### EVALUATION LOCALISATION #####################
% On s'intéresse sur cette partie seulement à l'évaluation de la
% localisation du robot leader
figure; % Trajectoire réelle et trajectoire prédite
plot(odometer(1, :, 1), odometer(2, :, 1), 'b');
hold on % pour dessiner sur le même graphe
plot(expected_odometer(1, :, 1), expected_odometer(2, :, 1), 'r');
plot(waypoints(1, 1), waypoints(2, 1), '*g');
plot(waypoints(1, 2), waypoints(2, 2), '*g');
plot(waypoints(1, 3), waypoints(2, 3), '*g');
plot(waypoints(1, 4), waypoints(2, 4), '*g');

erx = expected_odometer(1, :, 1) - odometer(1, :, 1);
ery = expected_odometer(2, :, 1) - odometer(2, :, 1);
er = sqrt(erx .^ 2 + ery .^ 2);

figure; % Plot de l'erreur globale au fil du temps
plot(er);
er = mean(er);

% sdlkgnkdjgndg

% Assuming 'iterations', 'sigmax', 'sigmay', 'erx', and 'ery' are defined

% Create a new figure with two subplots
figure;

% Subplot for X errors
subplot(2, 1, 1);
plot(3 * sigmax(2:iterations), 'r', 'LineWidth', 1.5);
hold on;
plot(erx, 'b', 'LineWidth', 1.5);
hold on;
plot(-3 * sigmax(2:iterations), 'r', 'LineWidth', 1.5);
title('Errors in X with 99.7% Confidence Interval');
legend('3*sigma', 'X Errors', '-3*sigma');
grid on;

% Subplot for Y errors
subplot(2, 1, 2);
plot(3 * sigmay(2:iterations), 'r', 'LineWidth', 1.5);
hold on;
plot(ery, 'b', 'LineWidth', 1.5);
hold on;
plot(-3 * sigmay(2:iterations), 'r', 'LineWidth', 1.5);
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
figure;
r1 = []; r2 = []; r3 = []; r4 = [];

for i = 2:iterations
    r1(i) = r_isol{i}(1);
    r2(i) = r_isol{i}(2);
    r3(i) = r_isol{i}(3);
    r4(i) = r_isol{i}(4);
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
figure;
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

function V = lyapunov_function(d_obs)
    V = 1 / (d_obs ^ 2);
end

function V_dot = lyapunov_derivative(d_obs, d_obs_dot)
    V_dot = -2 * d_obs_dot / (d_obs ^ 3);
end

function [d_obs, d_obs_dot, nearest_obstacle_idx] = compute_obstacle_distance_and_derivative(robot_pos, obstacles)
    % Compute the distance to the nearest obstacle and its rate of change
    % Return the index of the nearest obstacle as well

    % Placeholder logic for obstacle distance and its derivative
    distances = sqrt(sum((obstacles - robot_pos) .^ 2, 1));
    [d_obs, nearest_obstacle_idx] = min(distances);

    % Placeholder for the derivative of distance
    d_obs_dot = -0.01; % Example value, replace with actual computation
end
