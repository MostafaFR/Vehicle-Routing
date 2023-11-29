clear all

load('dqg.mat')
load('dqd.mat')
load('Xtruth.mat')
load('Ytruth.mat')

l1 = [0.5 1.4];
l2 = [2 0.6];
l3 = [3 1];
l = [l1; l2; l3]; %les coordones des 3 amers

r = (70/2) * 10 ^ -3; %rayon des roues droites et gauches
e = 230 * 10 ^ -3; %voie (distance entre les 2 roues)

%pose initiale
odometer(1, 1) = 0;
odometer(2, 1) = 0.5;
odometer(3, 1) = 0;

Qu = [0.5 * 10 ^ -4 0; 0 0.5 * 10 ^ -4]; %covariance associe aux bruits du vecteur d'entre
Q = [0.6 * 10 ^ -5 0 0; 0 0.4 * 10 ^ -5 0; 0 0 10 ^ -5]; %covariance du bruit de modle
podo = [1 0 0; 0 1 0; 0 0 0.5]; %valeur initiale de la matrice de covariance P

R = 10 ^ -1; %covariance du bruit de mesure ( adapter la valeur)

% Definir X(k/k-1)
X = [0 0.5 0]'; %valeur initiale de l'�tat

% boucle pour delta_k
% delta_k = r * (dqg(1,:) + dqd(1,:)) / 2;
% omega_k = r * (dqg(1,:) - dqd(1,:)) / e;

% boucle pour delta_k
for t = 2:260
    delta_k(t - 1) = r * (dqg(t) + dqd(t)) / 2;
    omega_k(t - 1) = r * (dqd(t) - dqg(t)) / e;

    A = [cos(odometer(3, t - 1)) 0; sin(odometer(3, t - 1)) 0; 0 1]; %matrice A
    odometer(:, t) = odometer(:, t - 1) + A * [delta_k(t - 1); omega_k(t - 1)]; %calcul de odometer(k/k-1)

    % Jacobienne de A
    % B=[cos(odometer(3,t-1)) 0; sin(odometer(3,t-1)) 0; 0 1]; %matrice B % Jacobienne
    FF = [1 0 -delta_k(t - 1) * sin(odometer(3, t - 1)); 0 1 delta_k(t - 1) * cos(odometer(3, t - 1)); 0 0 1]; %matrice F
    % podo= FF*podo*FF'+B*Qu*B' + Q; %calcul de P(k/k-1)
    podo = FF * podo * FF' + Q; %calcul de P(k/k-1)
    X = odometer(:, t); %calcul de X(k/k-1)
    Y = inv(podo); %calcul de Y(k/k-1)
    y = Y * X; %calcul de y(k/k-1)

    % calculer les distances d1,d2,d3 aux 3 amers à chaque instant
    d1(t) = sqrt((Xtruth(t) - l1(1)) .^ 2 + (Ytruth(t) - l1(2)) .^ 2);
    d2(t) = sqrt((Xtruth(t) - l2(1)) .^ 2 + (Ytruth(t) - l2(2)) .^ 2);
    d3(t) = sqrt((Xtruth(t) - l3(1)) .^ 2 + (Ytruth(t) - l3(2)) .^ 2);

    z = [d1(t) + normrnd(0, 0.001); d2(t) + normrnd(0, 0.001); d3(t) + normrnd(0, 0.001)]

    % DEBUT FILTRE INFORMATIONNEL
    %initialisation des gains informationnels
    gI = zeros(3, 3);
    gi = zeros (3, 1);

    for i = 1:3
        zestime = sqrt((X(1) - l(i, 1)) ^ 2 + (X(2) - l(i, 2)) ^ 2);
        H = [(X(1) - l(i, 1)) / zestime, (X(2) - l(i, 2)) / zestime, 0]; % jacobienne associe a la mesure

        gI = gI + H' * inv(R) * H; % contribution infomationnelle associe a la matrice infomationnelle
        nu = z(i) - zestime;
        gi = gi + H' * inv(R) * [nu + H * X]; % contribution infomationnelle associe au vecteur infomationnel
    end

    Y = Y + gI; %matrice informationnell mise à jour
    y = y + gi; %vecteur informationnel mis à jour

    podo = inv(Y); % matrice de covariance mise a jour
    odometer(:, t) = podo * y; % vecteur d'etat mis a jour a l'instant t
    % FIN FILTRE INFORMATIONNEL
    
    sigma_x(t) = sqrt(podo(1, 1));
    sigma_y(t) = sqrt(podo(2, 2));
    sigma_theta(t) = sqrt(podo(3, 3));
end

% calcul de l'erreur sur x, y et theta
erreur_x = Xtruth' - odometer(1, :);
erreur_y = Ytruth' - odometer(2, :);
erreur = sqrt(erreur_x .^ 2 + erreur_y .^ 2);

% Tracé de la trajectoire estimée vs trajectoire réelle
figure;
plot(odometer(1, :), odometer(2, :), 'r');
hold on;
plot(Xtruth, Ytruth, 'b');
title('Trajectoire estimée vs trajectoire réelle');
xlabel('x (m)');
ylabel('y (m)');
% Obstacles l1, l2 et l3
plot(l1(1), l1(2), 'k*');
plot(l2(1), l2(2), 'k*');
plot(l3(1), l3(2), 'k*');
legend('Trajectoire estimée', 'Trajectoire réelle', 'Obstacles');
grid on;

% Tracé de l'erreur sur x et y avec ±3σ
figure;
subplot(3, 1, 1);
plot(erreur, 'r');
hold on;
plot(3 * sigma_x(2:260), 'b');
plot(-3 * sigma_x(2:260), 'b');
title('Erreur avec ±3σ');
xlabel('Temps (s)');
ylabel('Erreur (m)');
legend('Erreur', '3σ', '-3σ');
grid on;

subplot(3, 1, 2);
plot(erreur_x, 'r');
hold on;
plot(3 * sigma_x(2:260), 'b');
plot(-3 * sigma_x(2:260), 'b');
title('Erreur sur x avec ±3σ');
xlabel('Temps (s)');
ylabel('Erreur (m)');
legend('Erreur sur x', '3σ', '-3σ');
grid on;

subplot(3, 1, 3);
plot(erreur_y, 'r');
hold on;
plot(3 * sigma_y(2:260), 'b');
plot(-3 * sigma_y(2:260), 'b');
title('Erreur sur y avec ±3σ');
xlabel('Temps (s)');
ylabel('Erreur (m)');
legend('Erreur sur y', '3σ', '-3σ');
grid on;
