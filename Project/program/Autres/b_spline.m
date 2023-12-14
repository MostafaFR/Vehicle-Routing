% Définition des points de contrôle
points = [0, 0; 1, 1; -1, 1];

% Ajouter des points supplémentaires pour la fin et le début
% pour forcer la courbe à passer par les points de contrôle originaux
points = [points(1,:); points; points(end,:)];

% Séparation des coordonnées X et Y
x = points(:, 1);
y = points(:, 2);

% Paramètre pour le spline
t = 1:length(points);

% Interpolation spline
tt = linspace(1, length(points), 100);
xx = spline(t, x, tt);
yy = spline(t, y, tt);

% Tracé de la courbe
figure;
plot(xx, yy, 'b-', 'LineWidth', 2);
hold on;

% Tracé des points de contrôle
plot(x, y, 'ro-');

% Configuration du graphique
title('Courbe B-Spline');
xlabel('X');
ylabel('Y');
grid on;
hold off;
