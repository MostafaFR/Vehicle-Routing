% Définition des points de contrôle
P0 = [0, 0]; % Premier point de contrôle
P1 = [1, 1]; % Deuxième point de contrôle
P2 = [-1, 1]; % Troisième point de contrôle

% Calcul du point de contrôle ajusté pour que la courbe passe par P1
t = 0.5; % Supposons que P1 est atteint à t = 0.5 pour la simplicité
P1_adj = (P1 - (1-t)^2 * P0 - t^2 * P2) / (2*(1-t)*t);

% Nombre de points sur la courbe
n = 100;

% Initialisation des vecteurs pour les coordonnées de la courbe
Bx = zeros(1, n);
By = zeros(1, n);

% Calcul des points sur la courbe de Bézier
for i = 1:n
    t = (i-1)/(n-1);
    Bx(i) = (1-t)^2 * P0(1) + 2*(1-t)*t * P1_adj(1) + t^2 * P2(1);
    By(i) = (1-t)^2 * P0(2) + 2*(1-t)*t * P1_adj(2) + t^2 * P2(2);
end

% Tracé de la courbe
figure;
plot(Bx, By, 'b-', 'LineWidth', 2);
hold on;

% Tracé des points de contrôle et des points par lesquels la courbe passe
plot([P0(1), P1_adj(1), P2(1)], [P0(2), P1_adj(2), P2(2)], 'ro--');
plot(P1(1), P1(2), 'gx', 'MarkerSize', 10, 'LineWidth', 2);

% Configuration du graphique
title('Courbe de Bézier quadratique ajustée');
xlabel('X');
ylabel('Y');
grid on;
hold off;