% Définition des points de contrôle
points = [0,0;-1, 0.8; -1, -0.8; 1, -0.8; 1, 0.8; -1, 0.8];

% Ajouter des points supplémentaires aux extrémités pour gérer les extrémités
points = [points(1, :); points; points(end, :)];

% Nombre de points intermédiaires à calculer entre chaque paire de points
numPoints = 500;

% Initialisation des vecteurs pour les coordonnées de la co4urbe
xx = [];
yy = [];

% Calcul de la courbe de Catmull-Rom
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
        xx = [xx pt(1)];
        yy = [yy pt(2)];
    end

end

% Tracé de la courbe
plot(xx, yy, 'b-', 'LineWidth', 2);
hold on;

% Tracé des points de contrôle
plot(points(:, 1), points(:, 2), 'ro-');

% Configuration du graphique
title('Courbe de Catmull-Rom');
xlabel('X');
ylabel('Y');
grid on;
hold off;
