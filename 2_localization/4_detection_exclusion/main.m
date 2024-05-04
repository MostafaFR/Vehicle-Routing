clear all

load('dqg.mat')
load('dqd.mat')
load('Xtruth.mat')
load('Ytruth.mat')

l1 = [0.5 1.4];
l2 = [2 0.6];
l3 = [3 1];
l4 = [3.5 1.6];
l5 = [0.1 0.1];
l = [l1; l2; l3; l4; l5]; %les coordones des 3 amers

r = (70/2) * 10 ^ -3; %rayon des roues droites et gauches
e = 230 * 10 ^ -3; %voie (distance entre les 2 roues)

%pose initiale
odometer(1, 1) = 0;
odometer(2, 1) = 0.5;
odometer(3, 1) = 0;

Qu = [0.5 * 10 ^ -4 0; 0 0.5 * 10 ^ -4] * 0; %covariance associe aux bruits du vecteur d'entre
Q = [0.6 * 10 ^ -5 0 0; 0 0.4 * 10 ^ -5 0; 0 0 10 ^ -5]; %covariance du bruit de modle
podo = [1 0 0; 0 1 0; 0 0 0.5]; %valeur initiale de la matrice de covariance P

R = 10 ^ -5; %covariance du bruit de mesure ( adapter la valeur)

% Definir X(k/k-1)
X = [0 0.5 0]'; %valeur initiale de l'�tat

tfault1 = [20:50];
tfault2 = [20:50, 80:100];
tfault5 = [150:180];

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
    Yp = inv(podo); %calcul de Y(k/k-1)
    yp = Yp * X; %calcul de y(k/k-1)

    % calculer les distances d1,d2,d3 aux 3 amers à chaque instant
    d1(t) = sqrt((Xtruth(t) - l1(1)) .^ 2 + (Ytruth(t) - l1(2)) .^ 2);
    d2(t) = sqrt((Xtruth(t) - l2(1)) .^ 2 + (Ytruth(t) - l2(2)) .^ 2);
    d3(t) = sqrt((Xtruth(t) - l3(1)) .^ 2 + (Ytruth(t) - l3(2)) .^ 2);
    d4(t) = sqrt((Xtruth(t) - l4(1)) .^ 2 + (Ytruth(t) - l4(2)) .^ 2);
    d5(t) = sqrt((Xtruth(t) - l5(1)) .^ 2 + (Ytruth(t) - l5(2)) .^ 2);

    z = [d1(t) + normrnd(0, 0.001); d2(t) + normrnd(0, 0.001); d3(t) + normrnd(0, 0.001); d4(t) + normrnd(0, 0.001); d5(t) + normrnd(0, 0.001)]; %calcul de z(k)

    %injection de d�fauts
    if find(tfault1 == t)
        z(1) = z(1) + 0.05;
    end

    if find(tfault2 == t)
        z(2) = z(2) + 0.05;
    end

    if find(tfault5 == t)
        z(5) = z(5) + 0.05;
    end

    % DEBUT FILTRE INFORMATIONNEL
    %initialisation des gains informationnels
    sgI = zeros(3, 3); % matrice informationnelle de correction
    sgi = zeros(3, 1); % vecteur informationnel de correction
    gI = []; % matrice informationnelle de correction
    gi = []; % vecteur informationnel de correction
    th_part = chi2inv(0.9, 3); % seuil de d�tection de d�faut

    n = 5; % nombre de mesures

    for i = 1:n
        zestime = sqrt((X(1) - l(i, 1)) ^ 2 + (X(2) - l(i, 2)) ^ 2);
        H = [(X(1) - l(i, 1)) / zestime, (X(2) - l(i, 2)) / zestime, 0]; % jacobienne associe a la mesure

        gI{i} = H' * inv(R) * H; % contribution infomationnelle associe a la mesure
        sgI = sgI + gI{i}; % somme des matrices informationnelles
        nu = z(i) - zestime;
        gi{i} = H' * inv(R) * [nu + H * X]; % contribution informationnelle associe a la mesure
        sgi = sgi + gi{i}; % somme des vecteurs informationnels

        r_isol{t}(i) = diagnostic(gI{i}, gi{i}, yp, Yp, X); % calcul du residu isolant
    end

    % detection de défaut
    [r_global(t), detect] = detection(sgi, sgI, yp, Yp, X); % calcul du residu global

    % si defaut detecté => exclusion de la mesure erronée

    if detect

        for i = 1:n

            if r_isol{t}(i) > th_part
                sgi = sgi - gi{i}; % vecteur informationnel mis à jour
                sgI = sgI - gI{i}; % matrice informationnelle mise à jour
                fprintf('defaut detecte a l instant %d sur la mesure %d\n', t, i);
            end

        end

    end

    Yu = Yp + sgI; % matrice de covariance mise a jour
    yu = yp + sgi; % vecteur d'etat mis a jour a l'instant t

    podo = inv(Yu); % matrice de covariance mise a jour
    odometer(:, t) = podo * yu; % vecteur d'�tat mis � jour � l'instant t
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
plot(l4(1), l4(2), 'k*');
plot(l5(1), l5(2), 'k*');
% legend('Trajectoire estimée', 'Trajectoire réelle', 'Obstacles');
grid on;

% Tracé de l'erreur sur x et y avec ±3σ
figure;
subplot(5, 1, 1);
plot(erreur, 'r');
hold on;
plot(3 * sigma_x(2:260), 'b');
plot(-3 * sigma_x(2:260), 'b');
title('Erreur avec ±3σ');
xlabel('Temps (s)');
ylabel('Erreur (m)');
legend('Erreur', '3σ', '-3σ');
grid on;

subplot(5, 1, 2);
plot(erreur_x, 'r');
hold on;
plot(3 * sigma_x(2:260), 'b');
plot(-3 * sigma_x(2:260), 'b');
title('Erreur sur x avec ±3σ');
xlabel('Temps (s)');
ylabel('Erreur (m)');
legend('Erreur sur x', '3σ', '-3σ');
grid on;

subplot(5, 1, 3);
plot(erreur_y, 'r');
hold on;
plot(3 * sigma_y(2:260), 'b');
plot(-3 * sigma_y(2:260), 'b');
title('Erreur sur y avec ±3σ');
xlabel('Temps (s)');
ylabel('Erreur (m)');
legend('Erreur sur y', '3σ', '-3σ');
grid on;

% affiche les residus isolants
subplot(5, 1, 4);
r1 = []; r2 = []; r3 = []; r4 = []; r5 = [];

for i = 2:260
    r1 = [r1 r_isol{i}(1)];
    r2 = [r2 r_isol{i}(2)];
    r3 = [r3 r_isol{i}(3)];
    r4 = [r4 r_isol{i}(4)];
    r5 = [r5 r_isol{i}(5)];
end

plot(r1, 'r');
hold on;
plot(r2, 'b');
hold on;
plot(r3, 'g');
hold on;
plot(r4, 'k');
hold on;
plot(r5, 'm');
line([1 259], [th_part th_part], 'linewidth', 1, 'color', 'g');

% affiche le residu global
subplot(5, 1, 5);
plot(r_global);
title('Residu global');
xlabel('Temps (s)');
ylabel('Residu global');
grid on;
